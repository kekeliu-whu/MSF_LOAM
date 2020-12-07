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

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <slam/msg_conversion.h>
#include <queue>

#include "common/rigid_transform.h"
#include "common/tic_toc.h"
#include "slam/local/laser_mapping.h"
#include "slam/local/laser_odometry.h"
#include "slam/local/scan_matching/odometry_scan_matcher.h"

LaserOdometry::LaserOdometry(bool is_offline_mode)
    : laser_mapper_handler_(std::make_shared<LaserMapping>(is_offline_mode)),
      scan_matcher_(std::make_unique<OdometryScanMatcher>()) {
  // NodeHandle uses reference counting internally,
  // thus a local variable can be created here
  ros::NodeHandle nh;
  LOG(INFO) << "LaserOdometry initializing ...";
  laser_odom_publisher_ =
      nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
  laser_path_publisher_ = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);
}

LaserOdometry::~LaserOdometry() { LOG(INFO) << "LaserOdometry finished."; }

void LaserOdometry::AddLaserScan(TimestampedPointCloud scan_curr) {
  auto rotation = AdvanceImuTracker(scan_curr.timestamp);
  if (rotation) {
    scan_curr.imu_rotation = *rotation;
  }

  TicToc t_whole;
  // initializing
  if (scan_last_.cloud_full_res->empty()) {
    LOG(INFO) << "[ODO] Initializing ...";
  } else {
    // pose_curr2last_.rotation() =
    // scan_last_.imu_rotation * scan_curr.imu_rotation.inverse();
    scan_matcher_->Match(scan_last_, scan_curr, &pose_curr2last_);

    LOG(INFO) << "[ODO] odometry_delta: " << pose_curr2last_;
    LOG(INFO) << "[ODO] odometry_curr: " << pose_scan2world_;
    pose_scan2world_ = pose_scan2world_ * pose_curr2last_;
  }

  // publish odometry
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = "camera_init";
  laserOdometry.child_frame_id = "laser_odom";
  laserOdometry.header.stamp = ToRos(scan_curr.timestamp);
  laserOdometry.pose = ToRos(pose_scan2world_);
  laser_odom_publisher_.publish(laserOdometry);

  geometry_msgs::PoseStamped laserPose;
  laserPose.header = laserOdometry.header;
  laserPose.pose = laserOdometry.pose.pose;
  laser_path_.header.stamp = laserOdometry.header.stamp;
  laser_path_.poses.push_back(laserPose);
  laser_path_.header.frame_id = "camera_init";
  laser_path_publisher_.publish(laser_path_);

  scan_curr.odom_pose = pose_scan2world_;
  laser_mapper_handler_->AddLaserOdometryResult(scan_curr);

  scan_last_ = scan_curr;

  LOG_STEP_TIME("ODO", "Whole LaserOdometry", t_whole.toc());
  LOG_IF_EVERY_N(WARNING, t_whole.toc() > 100, 10)
      << "Odometry process over 100ms!!";
}

void LaserOdometry::AddImu(const ImuData &imu_data) {
  // estimate rotation_delta
  if (!imu_tracker_) {
    LOG(INFO) << "Initializing imu tracker ...";
    imu_tracker_.reset(new ImuTracker(10, imu_data.time));
    imu_tracker_->AddImuObservation(imu_data);
    imu_tracker_->Advance(imu_data.time);
  }
  CHECK(imu_queue_.empty() || imu_data.time > imu_queue_.back().time);
  imu_queue_.push(imu_data);
  laser_mapper_handler_->AddImu(imu_data);
}

std::unique_ptr<Quaternion<double>> LaserOdometry::AdvanceImuTracker(
    const Time &time) {
  if (!imu_tracker_ || time < imu_tracker_->time()) return nullptr;
  while (!imu_queue_.empty() && imu_queue_.front().time <= time) {
    imu_tracker_->AddImuObservation(imu_queue_.front());
    imu_tracker_->Advance(imu_queue_.front().time);
    imu_queue_.pop();
  }
  imu_tracker_->Advance(time);
  std::unique_ptr<Quaternion<double>> r(new Quaternion<double>);
  *r = imu_tracker_->orientation();
  return r;
}

void LaserOdometry::AddOdom(const OdometryData &odom_data) {
  laser_mapper_handler_->AddOdom(odom_data);
}
