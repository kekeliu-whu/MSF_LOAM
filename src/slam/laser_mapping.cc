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
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <queue>
#include <thread>
#include <vector>

#include "common/common.h"
#include "common/rigid_transform.h"
#include "common/tic_toc.h"
#include "common/timestamped_pointcloud.h"
#include "common/type_conversion.h"
#include "slam/hybrid_grid.h"
#include "slam/scan_matching/mapping_scan_matcher.h"

namespace {

int frameCount = 0;

HybridGrid hybrid_grid_map_corner(5.0);
HybridGrid hybrid_grid_map_surf(5.0);

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
      laserCloudCornerFromMap = hybrid_grid_map_corner.GetSurroundedCloud(
          laserCloudCornerLast, pose_map_scan2world);
      laserCloudSurfFromMap = hybrid_grid_map_surf.GetSurroundedCloud(
          laserCloudSurfLast, pose_map_scan2world);
      LOG_STEP_TIME("MAP", "Collect surround cloud", t_shift.toc());

      downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
      downSizeFilterCorner.filter(*laserCloudCornerLast);

      downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
      downSizeFilterSurf.filter(*laserCloudSurfLast);

      LOG(INFO) << "[MAP]"
                << " corner=" << laserCloudCornerFromMap->size()
                << ", surf=" << laserCloudSurfFromMap->size();
      if (laserCloudCornerFromMap->size() > 10 &&
          laserCloudSurfFromMap->size() > 50) {
        // TODO
        TimestampedPointCloud cloud_map, scan_curr;
        cloud_map.cloud_corner_less_sharp = laserCloudCornerFromMap;
        cloud_map.cloud_surf_less_flat = laserCloudSurfFromMap;
        scan_curr.cloud_corner_less_sharp = laserCloudCornerLast;
        scan_curr.cloud_surf_less_flat = laserCloudSurfLast;
        MappingScanMatcher::Match(cloud_map, scan_curr, &pose_map_scan2world);
      } else {
        LOG(WARNING) << "[MAP] time Map corner and surf num are not enough";
      }
      transformUpdate();

      TicToc t_add;

      for (size_t i = 0; i < laserCloudCornerLast->size(); i++) {
        laserCloudCornerLast->points[i] =
            pose_map_scan2world * laserCloudCornerLast->points[i];
      }
      hybrid_grid_map_corner.InsertScan(laserCloudCornerLast,
                                        downSizeFilterCorner);

      for (size_t i = 0; i < laserCloudSurfLast->size(); i++) {
        laserCloudSurfLast->points[i] =
            pose_map_scan2world * laserCloudSurfLast->points[i];
      }
      hybrid_grid_map_surf.InsertScan(laserCloudSurfLast, downSizeFilterSurf);

      LOG_STEP_TIME("MAP", "add points", t_add.toc());

      // publish surround map for every 5 frame
      if (frameCount % 5 == 0) {
        laserCloudSurround->clear();
        *laserCloudSurround += *laserCloudCornerFromMap;
        *laserCloudSurround += *laserCloudSurfFromMap;

        sensor_msgs::PointCloud2 laserCloudSurround3;
        pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
        laserCloudSurround3.header.stamp =
            ros::Time().fromSec(timeLaserOdometry);
        laserCloudSurround3.header.frame_id = "/camera_init";
        pubLaserCloudSurround.publish(laserCloudSurround3);
      }

      int laserCloudFullResNum = laserCloudFullRes->size();
      for (int i = 0; i < laserCloudFullResNum; i++) {
        laserCloudFullRes->points[i] =
            pose_map_scan2world * laserCloudFullRes->points[i];
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
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
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

  std::thread mapping_process{process};

  ros::spin();

  return 0;
}