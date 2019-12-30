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

#include <common/type_conversion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <queue>

#include "common/rigid_transform.h"
#include "common/tic_toc.h"
#include "slam/laser_mapping.h"
#include "slam/scan_matching/odometry_scan_matcher.h"

namespace {

std::shared_ptr<LaserMapping> laser_mapper;

int g_skip_frame_num = 5;
bool g_is_system_inited = false;

TimestampedPointCloud g_cloud_last;
TimestampedPointCloud g_cloud_curr;

// Transformation from scan to map
Rigid3d g_pose_scan2world;

// Transformation from current scan to previous scan
Rigid3d g_pose_curr2last;

// 消息队列
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;

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
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle nh;

  laser_mapper = std::make_shared<LaserMapping>(nh);

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

  int curr_frame_idx = 0;
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
      LOG(WARNING) << "[ODO] unsync message: "
                   << "sharp=" << timeCornerPointsSharp
                   << ", less sharp=" << timeCornerPointsLessSharp
                   << ", flat=" << timeSurfPointsFlat
                   << ", less flat=" << timeSurfPointsLessFlat
                   << ", full=" << timeLaserCloudFullRes;
      continue;
    }

    g_cloud_curr.timestamp = cornerSharpBuf.front()->header.stamp;

    pcl::fromROSMsg(*cornerSharpBuf.front(), *g_cloud_curr.cloud_corner_sharp);
    cornerSharpBuf.pop();

    pcl::fromROSMsg(*cornerLessSharpBuf.front(),
                    *g_cloud_curr.cloud_corner_less_sharp);
    cornerLessSharpBuf.pop();

    pcl::fromROSMsg(*surfFlatBuf.front(), *g_cloud_curr.cloud_surf_flat);
    surfFlatBuf.pop();

    pcl::fromROSMsg(*surfLessFlatBuf.front(),
                    *g_cloud_curr.cloud_surf_less_flat);
    surfLessFlatBuf.pop();

    pcl::fromROSMsg(*fullPointsBuf.front(), *g_cloud_curr.cloud_full_res);
    fullPointsBuf.pop();

    TicToc t_whole;
    // initializing
    if (!g_is_system_inited) {
      g_is_system_inited = true;
      LOG(INFO) << "[ODO] Initializing ...";
    } else {
      OdometryScanMatcher::Match(g_cloud_last, g_cloud_curr, &g_pose_curr2last);

      LOG(INFO) << "[ODO] odometry_delta: " << g_pose_curr2last;
      LOG(INFO) << "[ODO] odometry_curr: " << g_pose_scan2world;
      g_pose_scan2world = g_pose_scan2world * g_pose_curr2last;
    }

    TicToc t_pub;

    // publish odometry
    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "/camera_init";
    laserOdometry.child_frame_id = "/laser_odom";
    laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    laserOdometry.pose = ToRos(g_pose_scan2world);
    laser_odom_publisher.publish(laserOdometry);

    geometry_msgs::PoseStamped laserPose;
    laserPose.header = laserOdometry.header;
    laserPose.pose = laserOdometry.pose.pose;
    laserPath.header.stamp = laserOdometry.header.stamp;
    laserPath.poses.push_back(laserPose);
    laserPath.header.frame_id = "/camera_init";
    laser_path_publisher.publish(laserPath);

    std::swap(g_cloud_last, g_cloud_curr);

    if (curr_frame_idx % g_skip_frame_num == 0) {
      sensor_msgs::PointCloud2 laserCloudCornerLast2;
      pcl::toROSMsg(*g_cloud_last.cloud_corner_less_sharp,
                    laserCloudCornerLast2);
      laserCloudCornerLast2.header.stamp =
          ros::Time().fromSec(timeSurfPointsLessFlat);
      laserCloudCornerLast2.header.frame_id = "/camera";
      cloud_corner_last_publisher.publish(laserCloudCornerLast2);

      sensor_msgs::PointCloud2 laserCloudSurfLast2;
      pcl::toROSMsg(*g_cloud_curr.cloud_surf_less_flat, laserCloudSurfLast2);
      laserCloudSurfLast2.header.stamp =
          ros::Time().fromSec(timeSurfPointsLessFlat);
      laserCloudSurfLast2.header.frame_id = "/camera";
      cloud_surf_last_publisher.publish(laserCloudSurfLast2);

      sensor_msgs::PointCloud2 laserCloudFullRes3;
      pcl::toROSMsg(*g_cloud_curr.cloud_full_res, laserCloudFullRes3);
      laserCloudFullRes3.header.stamp =
          ros::Time().fromSec(timeSurfPointsLessFlat);
      laserCloudFullRes3.header.frame_id = "/camera";
      cloud_full_publisher.publish(laserCloudFullRes3);
    }

    g_cloud_curr.odom_pose = g_pose_scan2world;
    laser_mapper->AddLaserOdometryResult(g_cloud_curr);
    LOG_STEP_TIME("ODO", "publication", t_pub.toc());
    LOG_STEP_TIME("ODO", "whole laserOdometry", t_whole.toc());
    if (t_whole.toc() > 100) LOG(WARNING) << "odometry process over 100ms!!";

    curr_frame_idx++;
  }
  return 0;
}