//
// Created by whu on 12/7/19.
//

#ifndef LOAM_VELODYNE_TIMESTAMPED_POINTCLOUD_H
#define LOAM_VELODYNE_TIMESTAMPED_POINTCLOUD_H

#include <pcl/point_cloud.h>
#include <Eigen/Eigen>

#include <sensor_msgs/PointCloud2.h>

#include "common/common.h"
#include "common/rigid_transform.h"

struct TimestampedPointCloud {
  ros::Time timestamp;
  pcl::PointCloud<PointType>::Ptr cloud_full;
  pcl::PointCloud<PointType>::Ptr cloud_corner_sharp;
  pcl::PointCloud<PointType>::Ptr cloud_corner_less_sharp;
  pcl::PointCloud<PointType>::Ptr cloud_surf_flat;
  pcl::PointCloud<PointType>::Ptr cloud_surf_less_flat;

  TimestampedPointCloud()
      : cloud_full(new pcl::PointCloud<PointType>),
        cloud_corner_sharp(new pcl::PointCloud<PointType>),
        cloud_corner_less_sharp(new pcl::PointCloud<PointType>),
        cloud_surf_flat(new pcl::PointCloud<PointType>),
        cloud_surf_less_flat(new pcl::PointCloud<PointType>) {}
};

#endif  // ALOAM_VELODYNE_TIME_STAMPED_POINT_CLOUD_H
