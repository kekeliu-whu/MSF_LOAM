//
// Created by whu on 1/2/20.
//

#ifndef ALOAM_VELODYNE_LASER_ODOMETRY_H
#define ALOAM_VELODYNE_LASER_ODOMETRY_H

#include <ros/node_handle.h>

#include "common/timestamped_pointcloud.h"
#include "slam/laser_mapping.h"

class LaserOdometry {
 public:
  explicit LaserOdometry(ros::NodeHandle &nh);

  void AddLaserScan(const TimestampedPointCloud &scan);

 private:
  std::shared_ptr<LaserMapping> laser_mapper_;

  bool is_system_inited_;

  TimestampedPointCloud scan_last_;
  TimestampedPointCloud scan_curr_;

  // Transformation from scan to map
  Rigid3d pose_scan2world_;
  // Transformation from current scan to previous scan
  Rigid3d pose_curr2last_;

  ros::Publisher laser_odom_publisher_;
  ros::Publisher laser_path_publisher_;

  nav_msgs::Path laser_path_;

  int curr_frame_idx_;
};

#endif  // ALOAM_VELODYNE_LASER_ODOMETRY_H
