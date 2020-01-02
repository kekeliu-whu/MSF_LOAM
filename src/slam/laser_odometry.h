//
// Created by whu on 1/2/20.
//

#ifndef ALOAM_VELODYNE_LASER_ODOMETRY_H
#define ALOAM_VELODYNE_LASER_ODOMETRY_H

#include <ros/node_handle.h>

#include "common/timestamped_pointcloud.h"

class LaserOdometry {
 public:
  explicit LaserOdometry(ros::NodeHandle &nh);

  void AddLaserScan(const TimestampedPointCloud &scan);
};

#endif  // ALOAM_VELODYNE_LASER_ODOMETRY_H
