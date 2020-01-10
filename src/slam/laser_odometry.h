//
// Created by whu on 1/2/20.
//

#ifndef MSF_LOAM_VELODYNE_LASER_ODOMETRY_H
#define MSF_LOAM_VELODYNE_LASER_ODOMETRY_H

#include <ros/node_handle.h>

#include "common/timestamped_pointcloud.h"
#include "slam/laser_mapping.h"

class LaserOdometry {
 public:
  explicit LaserOdometry(bool is_offline_mode);

  ~LaserOdometry();

  void AddLaserScan(TimestampedPointCloud scan_curr);

 private:
  std::shared_ptr<LaserMapping> laser_mapper_handler_;

  TimestampedPointCloud scan_last_;

  // Transformation from scan to map
  Rigid3d pose_scan2world_;
  // Transformation from current scan to previous scan
  Rigid3d pose_curr2last_;

  ros::Publisher laser_odom_publisher_;
  ros::Publisher laser_path_publisher_;

  nav_msgs::Path laser_path_;
};

#endif  // MSF_LOAM_VELODYNE_LASER_ODOMETRY_H
