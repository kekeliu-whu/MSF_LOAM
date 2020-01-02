// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk
//           Keke Liu
//           kekliu.priv@gmail.com

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
#include "slam/laser_odometry.h"
#include "slam/scan_matching/odometry_scan_matcher.h"

LaserOdometry::LaserOdometry(ros::NodeHandle& nh)
    : laser_mapper_(std::make_shared<LaserMapping>(nh)),
      is_system_inited_(false),
      curr_frame_idx_(0) {
  laser_odom_publisher_ =
      nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
  laser_path_publisher_ = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);
}

void LaserOdometry::AddLaserScan(const TimestampedPointCloud& scan) {
  /**
   * @brief 获取消息
   *
   */
  scan_curr_ = scan;

  TicToc t_whole;
  // initializing
  if (!is_system_inited_) {
    is_system_inited_ = true;
    LOG(INFO) << "[ODO] Initializing ...";
  } else {
    OdometryScanMatcher::Match(scan_last_, scan_curr_, &pose_curr2last_);

    LOG(INFO) << "[ODO] odometry_delta: " << pose_curr2last_;
    LOG(INFO) << "[ODO] odometry_curr: " << pose_scan2world_;
    pose_scan2world_ = pose_scan2world_ * pose_curr2last_;
  }

  TicToc t_pub;

  // publish odometry
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = "/camera_init";
  laserOdometry.child_frame_id = "/laser_odom";
  laserOdometry.header.stamp = scan_curr_.timestamp;
  laserOdometry.pose = ToRos(pose_scan2world_);
  laser_odom_publisher_.publish(laserOdometry);

  geometry_msgs::PoseStamped laserPose;
  laserPose.header = laserOdometry.header;
  laserPose.pose = laserOdometry.pose.pose;
  laser_path_.header.stamp = laserOdometry.header.stamp;
  laser_path_.poses.push_back(laserPose);
  laser_path_.header.frame_id = "/camera_init";
  laser_path_publisher_.publish(laser_path_);

  scan_curr_.odom_pose = pose_scan2world_;
  laser_mapper_->AddLaserOdometryResult(scan_curr_);

  std::swap(scan_last_, scan_curr_);

  LOG_STEP_TIME("ODO", "publication", t_pub.toc());
  LOG_STEP_TIME("ODO", "whole laserOdometry", t_whole.toc());
  if (t_whole.toc() > 100) LOG(WARNING) << "odometry process over 100ms!!";

  curr_frame_idx_++;
}