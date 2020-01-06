//
// Created by whu on 12/24/19.
//

#include <common/tic_toc.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/init.h>
#include <tf/transform_broadcaster.h>

#include "common/type_conversion.h"
#include "laser_mapping.h"
#include "slam/scan_matching/mapping_scan_matcher.h"

LaserMapping::LaserMapping(bool is_offline_mode, ros::NodeHandle &nh)
    : frame_idx_cur_(0),
      hybrid_grid_map_corner_(3.0),
      hybrid_grid_map_surf_(3.0),
      is_offline_mode_(is_offline_mode) {
  LOG(INFO) << "LaserMapping initializing ...";
  // get leaf size
  float line_res = 0;
  float plane_res = 0;
  LOG_IF(WARNING, !nh.param<float>("mapping_line_resolution", line_res, 0.2))
      << "Use default mapping_line_resolution: 0.2";
  LOG_IF(WARNING, !nh.param<float>("mapping_plane_resolution", plane_res, 0.4))
      << "Use default mapping_plane_resolution: 0.4";
  LOG(INFO) << "[MAP]"
            << " line resolution " << line_res << " plane resolution "
            << plane_res;
  downsize_filter_corner_.setLeafSize(line_res, line_res, line_res);
  downsize_filter_surf_.setLeafSize(plane_res, plane_res, plane_res);
  // set publishers
  pubLaserCloudSurround =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);

  pubLaserCloudFullRes =
      nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);

  pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);

  pubOdomAftMappedHighFrec =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);

  pubLaserAfterMappedPath =
      nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);
  // RUN
  thread_ = std::thread([this] { this->Run(); });
}

LaserMapping::~LaserMapping() {
  // todo
  thread_.join();
  LOG(INFO) << "LaserMapping finished.";
}

void LaserMapping::AddLaserOdometryResult(
    const LaserOdometryResultType &laser_odometry_result) {
  std::unique_lock<std::mutex> ul(mutex_);
  odometry_result_queue_.push(laser_odometry_result);
  cv_.notify_one();
  // todo
  // publish odom tf
  // high frequence publish
  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.child_frame_id = "/aft_mapped";
  odomAftMapped.header.frame_id = "/camera_init";
  odomAftMapped.header.stamp = laser_odometry_result.timestamp;
  odomAftMapped.pose = ToRos(pose_odom2map_ * laser_odometry_result.odom_pose);
  pubOdomAftMappedHighFrec.publish(odomAftMapped);
}

void LaserMapping::Run() {
  while (ros::ok()) {
    LaserOdometryResultType odom_result;
    {
      std::unique_lock<std::mutex> ul(mutex_);
      // Try to get new messages in 50 ms, return false if failed
      bool is_msg_recv = cv_.wait_for(
          ul, std::chrono::milliseconds(50),
          [this] { return !this->odometry_result_queue_.empty(); });
      if (!is_msg_recv) continue;
      odom_result = odometry_result_queue_.front();
      odometry_result_queue_.pop();
      if (!is_offline_mode_) {
        while (!odometry_result_queue_.empty()) {
          LOG(WARNING)
              << "[MAP] drop lidar frame in mapping for real time performance";
          odometry_result_queue_.pop();
        }
      }
    }

    // scan match
    // input: from odom
    PointCloudConstPtr laserCloudCornerLast =
        odom_result.cloud_corner_less_sharp;
    PointCloudConstPtr laserCloudSurfLast = odom_result.cloud_surf_less_flat;
    PointCloudConstPtr laserCloudFullRes = odom_result.cloud_full_res;

    pose_odom_scan2world_ = odom_result.odom_pose;

    TicToc t_whole;

    transformAssociateToMap();

    TicToc t_shift;
    PointCloudPtr laserCloudCornerFromMap =
        hybrid_grid_map_corner_.GetSurroundedCloud(laserCloudCornerLast,
                                                   pose_map_scan2world_);
    PointCloudPtr laserCloudSurfFromMap =
        hybrid_grid_map_surf_.GetSurroundedCloud(laserCloudSurfLast,
                                                 pose_map_scan2world_);
    LOG_STEP_TIME("MAP", "Collect surround cloud", t_shift.toc());

    PointCloudPtr laserCloudCornerLastStack(new PointCloud);
    downsize_filter_corner_.setInputCloud(laserCloudCornerLast);
    downsize_filter_corner_.filter(*laserCloudCornerLastStack);

    PointCloudPtr laserCloudSurfLastStack(new PointCloud);
    downsize_filter_surf_.setInputCloud(laserCloudSurfLast);
    downsize_filter_surf_.filter(*laserCloudSurfLastStack);

    LOG(INFO) << "[MAP]"
              << " corner=" << laserCloudCornerFromMap->size()
              << ", surf=" << laserCloudSurfFromMap->size();
    if (laserCloudCornerFromMap->size() > 10 &&
        laserCloudSurfFromMap->size() > 50) {
      TimestampedPointCloud cloud_map, scan_curr;
      cloud_map.cloud_corner_less_sharp = laserCloudCornerFromMap;
      cloud_map.cloud_surf_less_flat = laserCloudSurfFromMap;
      scan_curr.cloud_corner_less_sharp = laserCloudCornerLastStack;
      scan_curr.cloud_surf_less_flat = laserCloudSurfLastStack;
      MappingScanMatcher::Match(cloud_map, scan_curr, &pose_map_scan2world_);
    } else {
      LOG(WARNING) << "[MAP] time Map corner and surf num are not enough";
    }
    transformUpdate();

    TicToc t_add;

    hybrid_grid_map_corner_.InsertScan(
        TransformPointCloud(laserCloudCornerLastStack, pose_map_scan2world_),
        downsize_filter_corner_);

    hybrid_grid_map_surf_.InsertScan(
        TransformPointCloud(laserCloudSurfLastStack, pose_map_scan2world_),
        downsize_filter_surf_);

    LOG_STEP_TIME("MAP", "add points", t_add.toc());

    // publish surround map for every 5 frame
    if (frame_idx_cur_ % 5 == 0) {
      PointCloudPtr laserCloudSurround(new PointCloud);
      *laserCloudSurround += *laserCloudCornerFromMap;
      *laserCloudSurround += *laserCloudSurfFromMap;

      sensor_msgs::PointCloud2 laserCloudSurround3;
      pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
      laserCloudSurround3.header.stamp = odom_result.timestamp;
      laserCloudSurround3.header.frame_id = "/camera_init";
      pubLaserCloudSurround.publish(laserCloudSurround3);
    }

    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*TransformPointCloud(laserCloudFullRes, pose_map_scan2world_),
                  laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = odom_result.timestamp;
    laserCloudFullRes3.header.frame_id = "/camera_init";
    pubLaserCloudFullRes.publish(laserCloudFullRes3);

    LOG_STEP_TIME("MAP", "whole mapping", t_whole.toc());

    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "/camera_init";
    odomAftMapped.header.stamp = odom_result.timestamp;
    odomAftMapped.child_frame_id = "/aft_mapped";
    odomAftMapped.pose = ToRos(pose_map_scan2world_);
    pubOdomAftMapped.publish(odomAftMapped);

    geometry_msgs::PoseStamped laserAfterMappedPose;
    laserAfterMappedPose.header = odomAftMapped.header;
    laserAfterMappedPose.pose = odomAftMapped.pose.pose;
    laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
    laserAfterMappedPath.header.frame_id = "/camera_init";
    laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
    pubLaserAfterMappedPath.publish(laserAfterMappedPath);

    tf::Transform transform;
    transform.setOrigin({pose_map_scan2world_.translation().x(),
                         pose_map_scan2world_.translation().y(),
                         pose_map_scan2world_.translation().z()});
    transform.setRotation({pose_map_scan2world_.rotation().x(),
                           pose_map_scan2world_.rotation().y(),
                           pose_map_scan2world_.rotation().z(),
                           pose_map_scan2world_.rotation().w()});
    transform_broadcaster_.sendTransform(tf::StampedTransform(
        transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));

    frame_idx_cur_++;
  }
}
