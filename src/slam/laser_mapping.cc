// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <ceres/ceres.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <cmath>
#include <queue>
#include <thread>
#include <vector>

#include "common/common.h"
#include "common/rigid_transform.h"
#include "common/tic_toc.h"
#include "common/type_conversion.h"
#include "slam/scan_matching/lidar_factor.h"

namespace {

int frameCount = 0;

// 格网原点的x轴网格偏移数
int laserCloudCenWidth = 10;
// 格网原点的y轴网格偏移数
int laserCloudCenHeight = 10;
// 格网原点的z轴网格偏移数
int laserCloudCenDepth = 5;
// 全局点云的x轴网格数
constexpr int laserCloudWidth = 21;
// 全局点云的y轴网格数
constexpr int laserCloudHeight = 21;
// 全局点云的z轴网格数
constexpr int laserCloudDepth = 11;

// 全局点云的网格总数
constexpr int laserCloudNum =
    laserCloudWidth * laserCloudHeight * laserCloudDepth;

int laserCloudValidInd[125];
int laserCloudSurroundInd[125];

// input: from odom
PointCloudPtr laserCloudCornerLast(new PointCloud);
PointCloudPtr laserCloudSurfLast(new PointCloud);

// ouput: all visualble cube points
PointCloudPtr laserCloudSurround(new PointCloud);

// surround points in map to build tree
PointCloudPtr laserCloudCornerFromMap(new PointCloud);
PointCloudPtr laserCloudSurfFromMap(new PointCloud);

// input & output: points in one frame. local --> global
PointCloudPtr laserCloudFullRes(new PointCloud);

// points in every cube
PointCloudPtr laserCloudCornerArray[laserCloudNum];
PointCloudPtr laserCloudSurfArray[laserCloudNum];

// kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(
    new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(
    new pcl::KdTreeFLANN<PointType>());

// Transformation from scan to odom's world frame
Rigid3d pose_odom_scan2world;

// Transformation from scan to map's world frame
Rigid3d pose_map_scan2world;

// Transformation between odom's world and map's world frame
Rigid3d pose_odom2map;

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;

pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterSurf;

std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;

PointType pointOri, pointSel;

ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes,
    pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath;

nav_msgs::Path laserAfterMappedPath;

}  // namespace

// set initial guess
void transformAssociateToMap() {
  pose_map_scan2world = pose_odom2map * pose_odom_scan2world;
}

void transformUpdate() {
  pose_odom2map = pose_map_scan2world * pose_odom_scan2world.inverse();
}

void pointAssociateToMap(const PointType &pi, PointType &po) {
  Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
  Eigen::Vector3d point_w = pose_map_scan2world.rotation() * point_curr +
                            pose_map_scan2world.translation();
  po.x = point_w.x();
  po.y = point_w.y();
  po.z = point_w.z();
  po.intensity = pi.intensity;
}

void laserCloudCornerLastHandler(
    const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2) {
  cornerLastBuf.push(laserCloudCornerLast2);
}

void laserCloudSurfLastHandler(
    const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2) {
  surfLastBuf.push(laserCloudSurfLast2);
}

void laserCloudFullResHandler(
    const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2) {
  fullResBuf.push(laserCloudFullRes2);
}

// receive odomtry
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry) {
  odometryBuf.push(laserOdometry);

  // high frequence publish
  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.child_frame_id = "/aft_mapped";
  odomAftMapped.header.frame_id = "/camera_init";
  odomAftMapped.header.stamp = laserOdometry->header.stamp;
  odomAftMapped.pose = ToRos(pose_odom2map * FromRos(laserOdometry->pose));
  pubOdomAftMappedHighFrec.publish(odomAftMapped);
}

void process() {
  while (ros::ok()) {
    while (!cornerLastBuf.empty() && !surfLastBuf.empty() &&
           !fullResBuf.empty() && !odometryBuf.empty()) {
      // sync odometryBuf to cornerLastBuf
      while (!odometryBuf.empty() &&
             odometryBuf.front()->header.stamp.toSec() <
                 cornerLastBuf.front()->header.stamp.toSec())
        odometryBuf.pop();
      if (odometryBuf.empty()) {
        break;
      }

      // sync surfLastBuf to cornerLastBuf
      while (!surfLastBuf.empty() &&
             surfLastBuf.front()->header.stamp.toSec() <
                 cornerLastBuf.front()->header.stamp.toSec())
        surfLastBuf.pop();
      if (surfLastBuf.empty()) {
        break;
      }

      // sync fullResBuf to cornerLastBuf
      while (!fullResBuf.empty() &&
             fullResBuf.front()->header.stamp.toSec() <
                 cornerLastBuf.front()->header.stamp.toSec())
        fullResBuf.pop();
      if (fullResBuf.empty()) {
        break;
      }

      double timeLaserCloudCornerLast =
          cornerLastBuf.front()->header.stamp.toSec();
      double timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();
      double timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();
      double timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();

      if (timeLaserCloudCornerLast != timeLaserOdometry ||
          timeLaserCloudSurfLast != timeLaserOdometry ||
          timeLaserCloudFullRes != timeLaserOdometry) {
        LOG(WARNING) << "[MAP] unsync message: "
                     << "corner=" << timeLaserCloudCornerLast
                     << ", surf=" << timeLaserCloudSurfLast
                     << ", full=" << timeLaserCloudFullRes
                     << ", odom=" << timeLaserOdometry;
        break;
      }

      pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
      cornerLastBuf.pop();

      pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
      surfLastBuf.pop();

      pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
      fullResBuf.pop();

      pose_odom_scan2world = FromRos(odometryBuf.front()->pose);
      odometryBuf.pop();

      while (!cornerLastBuf.empty()) {
        cornerLastBuf.pop();
        LOG(WARNING)
            << "[MAP] drop lidar frame in mapping for real time performance";
      }

      TicToc t_whole;

      transformAssociateToMap();

      TicToc t_shift;
      int centerCubeI =
          int((pose_map_scan2world.translation().x() + 25.0) / 50.0) +
          laserCloudCenWidth;
      int centerCubeJ =
          int((pose_map_scan2world.translation().y() + 25.0) / 50.0) +
          laserCloudCenHeight;
      int centerCubeK =
          int((pose_map_scan2world.translation().z() + 25.0) / 50.0) +
          laserCloudCenDepth;

      if (pose_map_scan2world.translation().x() + 25.0 < 0) centerCubeI--;
      if (pose_map_scan2world.translation().y() + 25.0 < 0) centerCubeJ--;
      if (pose_map_scan2world.translation().z() + 25.0 < 0) centerCubeK--;

      // 调整 centerCube* 和 laserCloudCen*
      {
        while (centerCubeI < 3) {
          for (int j = 0; j < laserCloudHeight; j++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int i = laserCloudWidth - 1;
              PointCloudPtr laserCloudCubeCornerPointer =
                  laserCloudCornerArray[i + laserCloudWidth * j +
                                        laserCloudWidth * laserCloudHeight * k];
              PointCloudPtr laserCloudCubeSurfPointer =
                  laserCloudSurfArray[i + laserCloudWidth * j +
                                      laserCloudWidth * laserCloudHeight * k];
              for (; i >= 1; i--) {
                laserCloudCornerArray[i + laserCloudWidth * j +
                                      laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCornerArray[i - 1 + laserCloudWidth * j +
                                          laserCloudWidth * laserCloudHeight *
                                              k];
                laserCloudSurfArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k] =
                    laserCloudSurfArray[i - 1 + laserCloudWidth * j +
                                        laserCloudWidth * laserCloudHeight * k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeI++;
          laserCloudCenWidth++;
        }

        while (centerCubeI >= laserCloudWidth - 3) {
          for (int j = 0; j < laserCloudHeight; j++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int i = 0;
              PointCloudPtr laserCloudCubeCornerPointer =
                  laserCloudCornerArray[i + laserCloudWidth * j +
                                        laserCloudWidth * laserCloudHeight * k];
              PointCloudPtr laserCloudCubeSurfPointer =
                  laserCloudSurfArray[i + laserCloudWidth * j +
                                      laserCloudWidth * laserCloudHeight * k];
              for (; i < laserCloudWidth - 1; i++) {
                laserCloudCornerArray[i + laserCloudWidth * j +
                                      laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCornerArray[i + 1 + laserCloudWidth * j +
                                          laserCloudWidth * laserCloudHeight *
                                              k];
                laserCloudSurfArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k] =
                    laserCloudSurfArray[i + 1 + laserCloudWidth * j +
                                        laserCloudWidth * laserCloudHeight * k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeI--;
          laserCloudCenWidth--;
        }

        while (centerCubeJ < 3) {
          for (int i = 0; i < laserCloudWidth; i++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int j = laserCloudHeight - 1;
              PointCloudPtr laserCloudCubeCornerPointer =
                  laserCloudCornerArray[i + laserCloudWidth * j +
                                        laserCloudWidth * laserCloudHeight * k];
              PointCloudPtr laserCloudCubeSurfPointer =
                  laserCloudSurfArray[i + laserCloudWidth * j +
                                      laserCloudWidth * laserCloudHeight * k];
              for (; j >= 1; j--) {
                laserCloudCornerArray[i + laserCloudWidth * j +
                                      laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCornerArray[i + laserCloudWidth * (j - 1) +
                                          laserCloudWidth * laserCloudHeight *
                                              k];
                laserCloudSurfArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k] =
                    laserCloudSurfArray[i + laserCloudWidth * (j - 1) +
                                        laserCloudWidth * laserCloudHeight * k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeJ++;
          laserCloudCenHeight++;
        }

        while (centerCubeJ >= laserCloudHeight - 3) {
          for (int i = 0; i < laserCloudWidth; i++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int j = 0;
              PointCloudPtr laserCloudCubeCornerPointer =
                  laserCloudCornerArray[i + laserCloudWidth * j +
                                        laserCloudWidth * laserCloudHeight * k];
              PointCloudPtr laserCloudCubeSurfPointer =
                  laserCloudSurfArray[i + laserCloudWidth * j +
                                      laserCloudWidth * laserCloudHeight * k];
              for (; j < laserCloudHeight - 1; j++) {
                laserCloudCornerArray[i + laserCloudWidth * j +
                                      laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCornerArray[i + laserCloudWidth * (j + 1) +
                                          laserCloudWidth * laserCloudHeight *
                                              k];
                laserCloudSurfArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k] =
                    laserCloudSurfArray[i + laserCloudWidth * (j + 1) +
                                        laserCloudWidth * laserCloudHeight * k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeJ--;
          laserCloudCenHeight--;
        }

        while (centerCubeK < 3) {
          for (int i = 0; i < laserCloudWidth; i++) {
            for (int j = 0; j < laserCloudHeight; j++) {
              int k = laserCloudDepth - 1;
              PointCloudPtr laserCloudCubeCornerPointer =
                  laserCloudCornerArray[i + laserCloudWidth * j +
                                        laserCloudWidth * laserCloudHeight * k];
              PointCloudPtr laserCloudCubeSurfPointer =
                  laserCloudSurfArray[i + laserCloudWidth * j +
                                      laserCloudWidth * laserCloudHeight * k];
              for (; k >= 1; k--) {
                laserCloudCornerArray[i + laserCloudWidth * j +
                                      laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCornerArray[i + laserCloudWidth * j +
                                          laserCloudWidth * laserCloudHeight *
                                              (k - 1)];
                laserCloudSurfArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k] =
                    laserCloudSurfArray[i + laserCloudWidth * j +
                                        laserCloudWidth * laserCloudHeight *
                                            (k - 1)];
              }
              laserCloudCornerArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeK++;
          laserCloudCenDepth++;
        }

        while (centerCubeK >= laserCloudDepth - 3) {
          for (int i = 0; i < laserCloudWidth; i++) {
            for (int j = 0; j < laserCloudHeight; j++) {
              int k = 0;
              PointCloudPtr laserCloudCubeCornerPointer =
                  laserCloudCornerArray[i + laserCloudWidth * j +
                                        laserCloudWidth * laserCloudHeight * k];
              PointCloudPtr laserCloudCubeSurfPointer =
                  laserCloudSurfArray[i + laserCloudWidth * j +
                                      laserCloudWidth * laserCloudHeight * k];
              for (; k < laserCloudDepth - 1; k++) {
                laserCloudCornerArray[i + laserCloudWidth * j +
                                      laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCornerArray[i + laserCloudWidth * j +
                                          laserCloudWidth * laserCloudHeight *
                                              (k + 1)];
                laserCloudSurfArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k] =
                    laserCloudSurfArray[i + laserCloudWidth * j +
                                        laserCloudWidth * laserCloudHeight *
                                            (k + 1)];
              }
              laserCloudCornerArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeK--;
          laserCloudCenDepth--;
        }
      }

      int laserCloudValidNum = 0;
      int laserCloudSurroundNum = 0;
      // 获取周围点云所在的网格索引（250 x 250 x 150 米范围内）
      for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
        for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
          for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++) {
            if (i >= 0 && i < laserCloudWidth && j >= 0 &&
                j < laserCloudHeight && k >= 0 && k < laserCloudDepth) {
              laserCloudValidInd[laserCloudValidNum] =
                  i + laserCloudWidth * j +
                  laserCloudWidth * laserCloudHeight * k;
              laserCloudValidNum++;
              laserCloudSurroundInd[laserCloudSurroundNum] =
                  i + laserCloudWidth * j +
                  laserCloudWidth * laserCloudHeight * k;
              laserCloudSurroundNum++;
            }
          }
        }
      }

      // 获取周围点云（250 x 250 x 150 米范围内）
      laserCloudCornerFromMap->clear();
      laserCloudSurfFromMap->clear();
      for (int i = 0; i < laserCloudValidNum; i++) {
        *laserCloudCornerFromMap +=
            *laserCloudCornerArray[laserCloudValidInd[i]];
        *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
      }
      int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
      int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

      PointCloudPtr laserCloudCornerStack(new PointCloud);
      downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
      downSizeFilterCorner.filter(*laserCloudCornerStack);
      int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

      PointCloudPtr laserCloudSurfStack(new PointCloud);
      downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
      downSizeFilterSurf.filter(*laserCloudSurfStack);
      int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

      LOG_STEP_TIME("MAP", "prepare", t_shift.toc());
      LOG(INFO) << "[MAP]"
                << " corner=" << laserCloudCornerFromMapNum
                << ", surf=" << laserCloudSurfFromMapNum;
      if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50) {
        TicToc t_opt;
        TicToc t_tree;
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
        LOG_STEP_TIME("MAP", "build tree", t_tree.toc());

        for (int iterCount = 0; iterCount < 2; iterCount++) {
          // ceres::LossFunction *loss_function = NULL;
          ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
          ceres::LocalParameterization *q_parameterization =
              new ceres::EigenQuaternionParameterization();
          ceres::Problem::Options problem_options;

          ceres::Problem problem(problem_options);
          problem.AddParameterBlock(
              pose_map_scan2world.rotation().coeffs().data(), 4,
              q_parameterization);
          problem.AddParameterBlock(pose_map_scan2world.translation().data(),
                                    3);

          TicToc t_data;
          int corner_num = 0;

          for (int i = 0; i < laserCloudCornerStackNum; i++) {
            pointOri = laserCloudCornerStack->points[i];
            //            double sqrtDis = pointOri.x * pointOri.x + pointOri.y
            //            * pointOri.y +
            //                             pointOri.z * pointOri.z;
            pointAssociateToMap(pointOri, pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                                pointSearchSqDis);

            if (pointSearchSqDis[4] < 1.0) {
              TicToc t;
              Eigen::Matrix<double, 3, 5> matA0;
              for (int j = 0; j < 5; j++) {
                matA0.col(j) =
                    laserCloudCornerFromMap->points[pointSearchInd[j]]
                        .getVector3fMap()
                        .cast<double>();
              }
              Eigen::Vector3d center = matA0.rowwise().mean();
              matA0 = matA0.colwise() - center;
              Eigen::Matrix3d covMat = matA0 * matA0.transpose();

              Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

              // if is indeed line feature
              // note Eigen library sort eigenvalues in increasing order
              Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
              Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
              if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
                const Eigen::Vector3d &point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function =
                    LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                problem.AddResidualBlock(
                    cost_function, loss_function,
                    pose_map_scan2world.rotation().coeffs().data(),
                    pose_map_scan2world.translation().data());
                corner_num++;
              }
            }
            /*
            else if (pointSearchSqDis[4] < 0.01 * sqrtDis) {
              Eigen::Vector3d center(0, 0, 0);
              for (int j = 0; j < 5; j++) {
                Eigen::Vector3d tmp =
                    laserCloudCornerFromMap->points[pointSearchInd[j]]
                        .getVector3fMap()
                        .cast<double>();
                center += tmp;
              }
              center = center / 5.0;
              Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
              ceres::CostFunction *cost_function =
                  LidarDistanceFactor::Create(curr_point, center);
              problem.AddResidualBlock(cost_function, loss_function,
                                       pose_map_scan2world.rotation().coeffs().data(),
                                       pose_map_scan2world.translation().data());
            }
             */
          }

          int surf_num = 0;
          for (int i = 0; i < laserCloudSurfStackNum; i++) {
            pointOri = laserCloudSurfStack->points[i];
            // double sqrtDis = pointOri.x * pointOri.x + pointOri.y *
            // pointOri.y + pointOri.z * pointOri.z;
            pointAssociateToMap(pointOri, pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                              pointSearchSqDis);

            if (pointSearchSqDis[4] < 1.0) {
              Eigen::Matrix<double, 5, 3> matA0;
              Eigen::Matrix<double, 5, 1> matB0 =
                  -1 * Eigen::Matrix<double, 5, 1>::Ones();
              for (int j = 0; j < 5; j++) {
                matA0.row(j) = laserCloudSurfFromMap->points[pointSearchInd[j]]
                                   .getVector3fMap()
                                   .cast<double>();
              }
              // 平面到原点的垂直向量
              Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
              // 平面到原点的垂直距离
              double negative_OA_dot_norm = 1 / norm.norm();
              norm.normalize();

              // Here n(pa, pb, pc) is unit norm of plane
              bool planeValid = true;
              for (int j = 0; j < 5; j++) {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(
                        norm(0) *
                            laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                        norm(1) *
                            laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                        norm(2) *
                            laserCloudSurfFromMap->points[pointSearchInd[j]].z +
                        negative_OA_dot_norm) > 0.2) {
                  planeValid = false;
                  break;
                }
              }
              Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
              if (planeValid) {
                ceres::CostFunction *cost_function =
                    LidarPlaneNormFactor::Create(curr_point, norm,
                                                 negative_OA_dot_norm);
                problem.AddResidualBlock(
                    cost_function, loss_function,
                    pose_map_scan2world.rotation().coeffs().data(),
                    pose_map_scan2world.translation().data());
                surf_num++;
              }
            }
            /*
            else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
            {
                    Eigen::Vector3d center(0, 0, 0);
                    for (int j = 0; j < 5; j++)
                    {
                            Eigen::Vector3d
            tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
                                                                    laserCloudSurfFromMap->points[pointSearchInd[j]].y,
                                                                    laserCloudSurfFromMap->points[pointSearchInd[j]].z);
                            center = center + tmp;
                    }
                    center = center / 5.0;
                    Eigen::Vector3d curr_point(pointOri.x, pointOri.y,
            pointOri.z); ceres::CostFunction *cost_function =
            LidarDistanceFactor::Create(curr_point, center);
                    problem.AddResidualBlock(cost_function, loss_function,
            parameters, parameters + 4);
            }
            */
          }

          LOG_STEP_TIME("MAP", "Data association", t_data.toc());

          TicToc t_solver;
          ceres::Solver::Options options;
          options.linear_solver_type = ceres::DENSE_QR;
          options.max_num_iterations = 4;
          options.minimizer_progress_to_stdout = false;
          options.check_gradients = false;
          options.gradient_check_relative_precision = 1e-4;
          ceres::Solver::Summary summary;
          ceres::Solve(options, &problem, &summary);
          LOG_STEP_TIME("MAP", "Solver time", t_solver.toc());
        }
        LOG_STEP_TIME("MAP", "Optimization twice", t_opt.toc());
      } else {
        LOG(WARNING) << "[MAP] time Map corner and surf num are not enough";
      }
      transformUpdate();

      TicToc t_add;
      for (int i = 0; i < laserCloudCornerStackNum; i++) {
        pointAssociateToMap(laserCloudCornerStack->points[i], pointSel);

        int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
        int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
        int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

        if (pointSel.x + 25.0 < 0) cubeI--;
        if (pointSel.y + 25.0 < 0) cubeJ--;
        if (pointSel.z + 25.0 < 0) cubeK--;

        if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 &&
            cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth) {
          int cubeInd = cubeI + laserCloudWidth * cubeJ +
                        laserCloudWidth * laserCloudHeight * cubeK;
          laserCloudCornerArray[cubeInd]->push_back(pointSel);
        }
      }

      for (int i = 0; i < laserCloudSurfStackNum; i++) {
        pointAssociateToMap(laserCloudSurfStack->points[i], pointSel);

        int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
        int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
        int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

        if (pointSel.x + 25.0 < 0) cubeI--;
        if (pointSel.y + 25.0 < 0) cubeJ--;
        if (pointSel.z + 25.0 < 0) cubeK--;

        if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 &&
            cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth) {
          int cubeInd = cubeI + laserCloudWidth * cubeJ +
                        laserCloudWidth * laserCloudHeight * cubeK;
          laserCloudSurfArray[cubeInd]->push_back(pointSel);
        }
      }
      LOG_STEP_TIME("MAP", "add points", t_add.toc());

      TicToc t_filter;
      for (int i = 0; i < laserCloudValidNum; i++) {
        int ind = laserCloudValidInd[i];

        PointCloudPtr tmpCorner(new PointCloud);
        downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
        downSizeFilterCorner.filter(*tmpCorner);
        laserCloudCornerArray[ind] = tmpCorner;

        PointCloudPtr tmpSurf(new PointCloud);
        downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
        downSizeFilterSurf.filter(*tmpSurf);
        laserCloudSurfArray[ind] = tmpSurf;
      }
      LOG_STEP_TIME("MAP", "filter time", t_filter.toc());

      // publish surround map for every 5 frame
      if (frameCount % 5 == 0) {
        laserCloudSurround->clear();
        for (int i = 0; i < laserCloudSurroundNum; i++) {
          int ind = laserCloudSurroundInd[i];
          *laserCloudSurround += *laserCloudCornerArray[ind];
          *laserCloudSurround += *laserCloudSurfArray[ind];
        }

        sensor_msgs::PointCloud2 laserCloudSurround3;
        pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
        laserCloudSurround3.header.stamp =
            ros::Time().fromSec(timeLaserOdometry);
        laserCloudSurround3.header.frame_id = "/camera_init";
        pubLaserCloudSurround.publish(laserCloudSurround3);
      }

      if (frameCount % 20 == 0) {
        PointCloud laserCloudMap;
        for (int i = 0; i < 4851; i++) {
          laserCloudMap += *laserCloudCornerArray[i];
          laserCloudMap += *laserCloudSurfArray[i];
        }
        sensor_msgs::PointCloud2 laserCloudMsg;
        pcl::toROSMsg(laserCloudMap, laserCloudMsg);
        laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        laserCloudMsg.header.frame_id = "/camera_init";
        pubLaserCloudMap.publish(laserCloudMsg);
      }

      int laserCloudFullResNum = laserCloudFullRes->points.size();
      for (int i = 0; i < laserCloudFullResNum; i++) {
        pointAssociateToMap(laserCloudFullRes->points[i],
                            laserCloudFullRes->points[i]);
      }

      sensor_msgs::PointCloud2 laserCloudFullRes3;
      pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
      laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
      laserCloudFullRes3.header.frame_id = "/camera_init";
      pubLaserCloudFullRes.publish(laserCloudFullRes3);

      LOG_STEP_TIME("MAP", "whole mapping", t_whole.toc());

      nav_msgs::Odometry odomAftMapped;
      odomAftMapped.header.frame_id = "/camera_init";
      odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
      odomAftMapped.child_frame_id = "/aft_mapped";
      odomAftMapped.pose = ToRos(pose_map_scan2world);
      pubOdomAftMapped.publish(odomAftMapped);

      geometry_msgs::PoseStamped laserAfterMappedPose;
      laserAfterMappedPose.header = odomAftMapped.header;
      laserAfterMappedPose.pose = odomAftMapped.pose.pose;
      laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
      laserAfterMappedPath.header.frame_id = "/camera_init";
      laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
      pubLaserAfterMappedPath.publish(laserAfterMappedPath);

      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin({pose_map_scan2world.translation().x(),
                           pose_map_scan2world.translation().y(),
                           pose_map_scan2world.translation().z()});
      transform.setRotation({pose_map_scan2world.rotation().x(),
                             pose_map_scan2world.rotation().y(),
                             pose_map_scan2world.rotation().z(),
                             pose_map_scan2world.rotation().w()});
      br.sendTransform(tf::StampedTransform(transform,
                                            odomAftMapped.header.stamp,
                                            "/camera_init", "/aft_mapped"));

      frameCount++;
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  float lineRes = 0;
  float planeRes = 0;
  nh.param<float>("mapping_line_resolution", lineRes, 0.4);
  nh.param<float>("mapping_plane_resolution", planeRes, 0.8);
  LOG(INFO) << "[MAP]"
            << " line resolution " << lineRes << " plane resolution "
            << planeRes;
  downSizeFilterCorner.setLeafSize(lineRes, lineRes, lineRes);
  downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

  ros::Subscriber subLaserCloudCornerLast =
      nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100,
                                             laserCloudCornerLastHandler);

  ros::Subscriber subLaserCloudSurfLast =
      nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100,
                                             laserCloudSurfLastHandler);

  ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>(
      "/laser_odom_to_init", 100, laserOdometryHandler);

  ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_cloud_3", 100, laserCloudFullResHandler);

  pubLaserCloudSurround =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);

  pubLaserCloudMap =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);

  pubLaserCloudFullRes =
      nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);

  pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);

  pubOdomAftMappedHighFrec =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);

  pubLaserAfterMappedPath =
      nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

  for (int i = 0; i < laserCloudNum; i++) {
    laserCloudCornerArray[i].reset(new PointCloud);
    laserCloudSurfArray[i].reset(new PointCloud);
  }

  std::thread mapping_process{process};

  ros::spin();

  return 0;
}