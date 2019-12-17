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
  std::string frame_id;
  Rigid3d global_posse;
  PointCloudPtr cloud_full_res;
  PointCloudPtr cloud_corner_sharp;
  PointCloudPtr cloud_corner_less_sharp;
  PointCloudPtr cloud_surf_flat;
  PointCloudPtr cloud_surf_less_flat;

  TimestampedPointCloud()
      : cloud_full_res(new PointCloud),
        cloud_corner_sharp(new PointCloud),
        cloud_corner_less_sharp(new PointCloud),
        cloud_surf_flat(new PointCloud),
        cloud_surf_less_flat(new PointCloud) {}
};

#endif  // ALOAM_VELODYNE_TIME_STAMPED_POINT_CLOUD_H
