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

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <queue>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"

#define DISTORTION 0

namespace {

constexpr double kScanPeriod = 0.1;
constexpr double kDistanceSqThreshold = 25;
constexpr double kNearByScan = 2.5;

int g_skip_frame_num = 5;
bool g_is_system_inited = false;

// less sharp 点构造的 kdtree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast(
    new pcl::KdTreeFLANN<PointType>());
// less flat 点构造的 kdtree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast(
    new pcl::KdTreeFLANN<PointType>());

pcl::PointCloud<PointType>::Ptr cloud_corner_sharp(
    new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cloud_corner_less_sharp(
    new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cloud_surf_flat(
    new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cloud_surf_less_flat(
    new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr cloud_corner_last(
    new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cloud_surf_last(
    new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(
    new pcl::PointCloud<PointType>());

// Rotation from scan to world
Eigen::Quaterniond g_r_scan2world(1, 0, 0, 0);
// Translation from scan to world
Eigen::Vector3d g_t_scan2world(0, 0, 0);

// Rotation from current scan to previous scan
Eigen::Quaterniond g_r_curr2last(1, 0, 0, 0);
// Translation from current scan to previous scan
Eigen::Vector3d g_t_curr2last(0, 0, 0);

// 消息队列
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;

// undistort lidar point
void TransformToStart(const PointType &pi, PointType &po) {
  // interpolation ratio
  double s;
  if (DISTORTION)
    s = (pi.intensity - int(pi.intensity)) / kScanPeriod;
  else
    s = 1.0;
  // s = 1;
  Eigen::Quaterniond q_point_last =
      Eigen::Quaterniond::Identity().slerp(s, g_r_curr2last);
  Eigen::Vector3d t_point_last = s * g_t_curr2last;
  Eigen::Vector3d point(pi.x, pi.y, pi.z);
  Eigen::Vector3d un_point = q_point_last * point + t_point_last;

  po.x = un_point.x();
  po.y = un_point.y();
  po.z = un_point.z();
  po.intensity = pi.intensity;
}

// transform all lidar points to the start of the next frame
void TransformToEnd(const PointType &pi, PointType &po) {
  // undistort point first
  PointType un_point_tmp;
  TransformToStart(pi, un_point_tmp);

  Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
  Eigen::Vector3d point_end =
      g_r_curr2last.inverse() * (un_point - g_t_curr2last);

  po.x = point_end.x();
  po.y = point_end.y();
  po.z = point_end.z();

  // Remove distortion time info
  po.intensity = int(pi.intensity);
}

void HandleCloudSharpMsg(
    const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp) {
  cornerSharpBuf.push(cornerPointsSharp);
}

void HandleCloudLessSharpMsg(
    const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp) {
  cornerLessSharpBuf.push(cornerPointsLessSharp);
}

void HandleCloudFlatMsg(
    const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat) {
  surfFlatBuf.push(surfPointsFlat);
}

void HandleCloudLessFlatMsg(
    const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat) {
  surfLessFlatBuf.push(surfPointsLessFlat);
}

// receive all point cloud
void HandleCloudFullMsg(
    const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes) {
  fullPointsBuf.push(laserCloudFullRes);
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle nh;

  nh.param<int>("mapping_skip_frame", g_skip_frame_num, 2);
  LOG(INFO) << "Mapping every " << g_skip_frame_num << " frames";

  ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>(
      "/laser_cloud_sharp", 100, HandleCloudSharpMsg);
  ros::Subscriber subCornerPointsLessSharp =
      nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100,
                                             HandleCloudLessSharpMsg);
  ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>(
      "/laser_cloud_flat", 100, HandleCloudFlatMsg);
  ros::Subscriber subSurfPointsLessFlat =
      nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100,
                                             HandleCloudLessFlatMsg);
  ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_cloud_2", 100, HandleCloudFullMsg);

  ros::Publisher cloud_corner_last_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
  ros::Publisher cloud_surf_last_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
  ros::Publisher cloud_full_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);
  ros::Publisher laser_odom_publisher =
      nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
  ros::Publisher laser_path_publisher =
      nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

  nav_msgs::Path laserPath;

  int frameCount = 0;
  ros::Rate rate(100);

  while (ros::ok()) {
    ros::spinOnce();

    if (cornerSharpBuf.empty() || cornerLessSharpBuf.empty() ||
        surfFlatBuf.empty() || surfLessFlatBuf.empty() ||
        fullPointsBuf.empty()) {
      rate.sleep();
      continue;
    }

    /**
     * @brief 获取消息
     *
     */
    double timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
    double timeCornerPointsLessSharp =
        cornerLessSharpBuf.front()->header.stamp.toSec();
    double timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
    double timeSurfPointsLessFlat =
        surfLessFlatBuf.front()->header.stamp.toSec();
    double timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();

    // 消息必须同步
    if (timeCornerPointsSharp != timeLaserCloudFullRes ||
        timeCornerPointsLessSharp != timeLaserCloudFullRes ||
        timeSurfPointsFlat != timeLaserCloudFullRes ||
        timeSurfPointsLessFlat != timeLaserCloudFullRes) {
      LOG(FATAL) << "Unsync message!";
    }

    pcl::fromROSMsg(*cornerSharpBuf.front(), *cloud_corner_sharp);
    cornerSharpBuf.pop();

    pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cloud_corner_less_sharp);
    cornerLessSharpBuf.pop();

    pcl::fromROSMsg(*surfFlatBuf.front(), *cloud_surf_flat);
    surfFlatBuf.pop();

    pcl::fromROSMsg(*surfLessFlatBuf.front(), *cloud_surf_less_flat);
    surfLessFlatBuf.pop();

    pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
    fullPointsBuf.pop();

    TicToc t_whole;
    // initializing
    if (!g_is_system_inited) {
      g_is_system_inited = true;
      LOG(INFO) << "Initailizing ...";
    } else {
      TicToc t_opt;

      for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
        int corner_correspondence = 0, plane_correspondence = 0;

        // ceres::LossFunction *loss_function = NULL;
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_parameterization =
            new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(g_r_curr2last.coeffs().data(), 4,
                                  q_parameterization);
        problem.AddParameterBlock(g_t_curr2last.data(), 3);

        PointType pointSel;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        TicToc t_data;
        // find correspondence for corner features
        for (int i = 0; i < cloud_corner_sharp->size(); ++i) {
          TransformToStart(cloud_corner_sharp->points[i], pointSel);
          kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd,
                                           pointSearchSqDis);

          int closestPointInd = -1, minPointInd2 = -1;
          if (pointSearchSqDis[0] < kDistanceSqThreshold) {
            closestPointInd = pointSearchInd[0];
            int closestPointScanID =
                int(cloud_corner_last->points[closestPointInd].intensity);

            double minPointSqDis2 = kDistanceSqThreshold;
            // search in the direction of increasing scan line
            for (int j = closestPointInd + 1;
                 j < (int)cloud_corner_last->size(); ++j) {
              // if in the same scan line, continue
              if (int(cloud_corner_last->points[j].intensity) <=
                  closestPointScanID)
                continue;

              // if not in nearby scans, end the loop
              if (int(cloud_corner_last->points[j].intensity) >
                  (closestPointScanID + kNearByScan))
                break;

              double pointSqDis =
                  (cloud_corner_last->points[j].x - pointSel.x) *
                      (cloud_corner_last->points[j].x - pointSel.x) +
                  (cloud_corner_last->points[j].y - pointSel.y) *
                      (cloud_corner_last->points[j].y - pointSel.y) +
                  (cloud_corner_last->points[j].z - pointSel.z) *
                      (cloud_corner_last->points[j].z - pointSel.z);

              if (pointSqDis < minPointSqDis2) {
                // find nearer point
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
              }
            }

            // search in the direction of decreasing scan line
            for (int j = closestPointInd - 1; j >= 0; --j) {
              // if in the same scan line, continue
              if (int(cloud_corner_last->points[j].intensity) >=
                  closestPointScanID)
                continue;

              // if not in nearby scans, end the loop
              if (int(cloud_corner_last->points[j].intensity) <
                  (closestPointScanID - kNearByScan))
                break;

              double pointSqDis =
                  (cloud_corner_last->points[j].x - pointSel.x) *
                      (cloud_corner_last->points[j].x - pointSel.x) +
                  (cloud_corner_last->points[j].y - pointSel.y) *
                      (cloud_corner_last->points[j].y - pointSel.y) +
                  (cloud_corner_last->points[j].z - pointSel.z) *
                      (cloud_corner_last->points[j].z - pointSel.z);

              if (pointSqDis < minPointSqDis2) {
                // find nearer point
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
              }
            }
          }
          if (minPointInd2 >=
              0)  // both closestPointInd and minPointInd2 is valid
          {
            Eigen::Vector3d curr_point(cloud_corner_sharp->points[i].x,
                                       cloud_corner_sharp->points[i].y,
                                       cloud_corner_sharp->points[i].z);
            Eigen::Vector3d last_point_a(
                cloud_corner_last->points[closestPointInd].x,
                cloud_corner_last->points[closestPointInd].y,
                cloud_corner_last->points[closestPointInd].z);
            Eigen::Vector3d last_point_b(
                cloud_corner_last->points[minPointInd2].x,
                cloud_corner_last->points[minPointInd2].y,
                cloud_corner_last->points[minPointInd2].z);

            double s;
            if (DISTORTION)
              s = (cloud_corner_sharp->points[i].intensity -
                   int(cloud_corner_sharp->points[i].intensity)) /
                  kScanPeriod;
            else
              s = 1.0;
            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(
                curr_point, last_point_a, last_point_b, s);
            problem.AddResidualBlock(cost_function, loss_function,
                                     g_r_curr2last.coeffs().data(),
                                     g_t_curr2last.data());
            corner_correspondence++;
          }
        }

        // find correspondence for plane features
        for (int i = 0; i < cloud_surf_flat->size(); ++i) {
          TransformToStart(cloud_surf_flat->points[i], pointSel);
          kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd,
                                         pointSearchSqDis);

          int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
          if (pointSearchSqDis[0] < kDistanceSqThreshold) {
            closestPointInd = pointSearchInd[0];

            // get closest point's scan ID
            int closestPointScanID =
                int(cloud_surf_last->points[closestPointInd].intensity);
            double minPointSqDis2 = kDistanceSqThreshold,
                   minPointSqDis3 = kDistanceSqThreshold;

            // search in the direction of increasing scan line
            for (size_t j = closestPointInd + 1; j < cloud_surf_last->size();
                 ++j) {
              // if not in nearby scans, end the loop
              if (int(cloud_surf_last->points[j].intensity) >
                  (closestPointScanID + kNearByScan))
                break;

              double pointSqDis =
                  (cloud_surf_last->points[j].x - pointSel.x) *
                      (cloud_surf_last->points[j].x - pointSel.x) +
                  (cloud_surf_last->points[j].y - pointSel.y) *
                      (cloud_surf_last->points[j].y - pointSel.y) +
                  (cloud_surf_last->points[j].z - pointSel.z) *
                      (cloud_surf_last->points[j].z - pointSel.z);

              // if in the same or lower scan line
              if (int(cloud_surf_last->points[j].intensity) <=
                      closestPointScanID &&
                  pointSqDis < minPointSqDis2) {
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
              }
              // if in the higher scan line
              else if (int(cloud_surf_last->points[j].intensity) >
                           closestPointScanID &&
                       pointSqDis < minPointSqDis3) {
                minPointSqDis3 = pointSqDis;
                minPointInd3 = j;
              }
            }

            // search in the direction of decreasing scan line
            for (int j = closestPointInd - 1; j >= 0; --j) {
              // if not in nearby scans, end the loop
              if (int(cloud_surf_last->points[j].intensity) <
                  (closestPointScanID - kNearByScan))
                break;

              double pointSqDis =
                  (cloud_surf_last->points[j].x - pointSel.x) *
                      (cloud_surf_last->points[j].x - pointSel.x) +
                  (cloud_surf_last->points[j].y - pointSel.y) *
                      (cloud_surf_last->points[j].y - pointSel.y) +
                  (cloud_surf_last->points[j].z - pointSel.z) *
                      (cloud_surf_last->points[j].z - pointSel.z);

              // if in the same or higher scan line
              if (int(cloud_surf_last->points[j].intensity) >=
                      closestPointScanID &&
                  pointSqDis < minPointSqDis2) {
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
              } else if (int(cloud_surf_last->points[j].intensity) <
                             closestPointScanID &&
                         pointSqDis < minPointSqDis3) {
                // find nearer point
                minPointSqDis3 = pointSqDis;
                minPointInd3 = j;
              }
            }

            if (minPointInd2 >= 0 && minPointInd3 >= 0) {
              Eigen::Vector3d curr_point(cloud_surf_flat->points[i].x,
                                         cloud_surf_flat->points[i].y,
                                         cloud_surf_flat->points[i].z);
              Eigen::Vector3d last_point_a(
                  cloud_surf_last->points[closestPointInd].x,
                  cloud_surf_last->points[closestPointInd].y,
                  cloud_surf_last->points[closestPointInd].z);
              Eigen::Vector3d last_point_b(
                  cloud_surf_last->points[minPointInd2].x,
                  cloud_surf_last->points[minPointInd2].y,
                  cloud_surf_last->points[minPointInd2].z);
              Eigen::Vector3d last_point_c(
                  cloud_surf_last->points[minPointInd3].x,
                  cloud_surf_last->points[minPointInd3].y,
                  cloud_surf_last->points[minPointInd3].z);

              double s;
              if (DISTORTION)
                s = (cloud_surf_flat->points[i].intensity -
                     int(cloud_surf_flat->points[i].intensity)) /
                    kScanPeriod;
              else
                s = 1.0;
              ceres::CostFunction *cost_function = LidarPlaneFactor::Create(
                  curr_point, last_point_a, last_point_b, last_point_c, s);
              problem.AddResidualBlock(cost_function, loss_function,
                                       g_r_curr2last.coeffs().data(),
                                       g_t_curr2last.data());
              plane_correspondence++;
            }
          }
        }

        LOG_STEP_TIME("data association", t_data.toc());

        if ((corner_correspondence + plane_correspondence) < 10) {
          LOG(WARNING) << "coner_correspondance=" << corner_correspondence
                       << ", plane_correspondence=" << plane_correspondence;
          LOG(WARNING) << "less correspondence! "
                          "*************************************************\n";
        }

        TicToc t_solver;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        LOG_STEP_TIME("solver time", t_solver.toc());
      }
      LOG_STEP_TIME("optimization twice", t_opt.toc());

      LOG(WARNING) << "odometry_delta: " << g_t_curr2last.transpose();
      LOG(WARNING) << "odometry_curr: " << g_t_scan2world.transpose();
      g_t_scan2world = g_t_scan2world + g_r_scan2world * g_t_curr2last;
      g_r_scan2world = g_r_scan2world * g_r_curr2last;
    }

    TicToc t_pub;

    // publish odometry
    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "/camera_init";
    laserOdometry.child_frame_id = "/laser_odom";
    laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    laserOdometry.pose.pose.orientation.x = g_r_scan2world.x();
    laserOdometry.pose.pose.orientation.y = g_r_scan2world.y();
    laserOdometry.pose.pose.orientation.z = g_r_scan2world.z();
    laserOdometry.pose.pose.orientation.w = g_r_scan2world.w();
    laserOdometry.pose.pose.position.x = g_t_scan2world.x();
    laserOdometry.pose.pose.position.y = g_t_scan2world.y();
    laserOdometry.pose.pose.position.z = g_t_scan2world.z();
    laser_odom_publisher.publish(laserOdometry);

    geometry_msgs::PoseStamped laserPose;
    laserPose.header = laserOdometry.header;
    laserPose.pose = laserOdometry.pose.pose;
    laserPath.header.stamp = laserOdometry.header.stamp;
    laserPath.poses.push_back(laserPose);
    laserPath.header.frame_id = "/camera_init";
    laser_path_publisher.publish(laserPath);

    // transform corner features and plane features to the scan end point
    if (0) {
      int cornerPointsLessSharpNum = cloud_corner_less_sharp->size();
      for (int i = 0; i < cornerPointsLessSharpNum; i++) {
        TransformToEnd(cloud_corner_less_sharp->points[i],
                       cloud_corner_less_sharp->points[i]);
      }

      int surfPointsLessFlatNum = cloud_surf_less_flat->size();
      for (int i = 0; i < surfPointsLessFlatNum; i++) {
        TransformToEnd(cloud_surf_less_flat->points[i],
                       cloud_surf_less_flat->points[i]);
      }

      int laserCloudFullResNum = laserCloudFullRes->size();
      for (int i = 0; i < laserCloudFullResNum; i++) {
        TransformToEnd(laserCloudFullRes->points[i],
                       laserCloudFullRes->points[i]);
      }
    }

    cloud_corner_last.swap(cloud_corner_less_sharp);
    cloud_surf_last.swap(cloud_surf_less_flat);

    kdtreeCornerLast->setInputCloud(cloud_corner_last);
    kdtreeSurfLast->setInputCloud(cloud_surf_last);

    if (frameCount % g_skip_frame_num == 0) {
      frameCount = 0;

      sensor_msgs::PointCloud2 laserCloudCornerLast2;
      pcl::toROSMsg(*cloud_corner_last, laserCloudCornerLast2);
      laserCloudCornerLast2.header.stamp =
          ros::Time().fromSec(timeSurfPointsLessFlat);
      laserCloudCornerLast2.header.frame_id = "/camera";
      cloud_corner_last_publisher.publish(laserCloudCornerLast2);

      sensor_msgs::PointCloud2 laserCloudSurfLast2;
      pcl::toROSMsg(*cloud_surf_last, laserCloudSurfLast2);
      laserCloudSurfLast2.header.stamp =
          ros::Time().fromSec(timeSurfPointsLessFlat);
      laserCloudSurfLast2.header.frame_id = "/camera";
      cloud_surf_last_publisher.publish(laserCloudSurfLast2);

      sensor_msgs::PointCloud2 laserCloudFullRes3;
      pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
      laserCloudFullRes3.header.stamp =
          ros::Time().fromSec(timeSurfPointsLessFlat);
      laserCloudFullRes3.header.frame_id = "/camera";
      cloud_full_publisher.publish(laserCloudFullRes3);
    }
    LOG_STEP_TIME("publication", t_pub.toc());
    LOG_STEP_TIME("whole laserOdometry", t_whole.toc());
    if (t_whole.toc() > 100) LOG(WARNING) << "odometry process over 100ms!!";

    frameCount++;
  }
  return 0;
}