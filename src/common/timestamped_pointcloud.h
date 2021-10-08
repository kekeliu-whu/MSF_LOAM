#ifndef LOAM_VELODYNE_TIMESTAMPED_POINTCLOUD_H
#define LOAM_VELODYNE_TIMESTAMPED_POINTCLOUD_H

#include <pcl/point_cloud.h>
#include <Eigen/Eigen>

#include "common/common.h"
#include "common/rigid_transform.h"
#include "common/time_def.h"

template <typename T>
struct TimestampedPointCloud {
  using PointCloudType    = pcl::PointCloud<T>;
  using PointCloudTypePtr = typename PointCloudType::Ptr;

  Time timestamp;
  std::string frame_id;
  Rigid3d odom_pose;
  Rigid3d map_pose;
  Quaternion<double> imu_rotation;

  PointCloudTypePtr cloud_full_res;
  PointCloudTypePtr cloud_corner_sharp;
  PointCloudTypePtr cloud_corner_less_sharp;
  PointCloudTypePtr cloud_surf_flat;
  PointCloudTypePtr cloud_surf_less_flat;

  TimestampedPointCloud()
      : imu_rotation(Quaterniond::Identity()),
        cloud_full_res(new PointCloudType),
        cloud_corner_sharp(new PointCloudType),
        cloud_corner_less_sharp(new PointCloudType),
        cloud_surf_flat(new PointCloudType),
        cloud_surf_less_flat(new PointCloudType) {}
};

struct OdometryData {
  Time timestamp;
  Rigid3d odom;
  double error;
};

#endif  // MSF_LOAM_VELODYNE_TIME_STAMPED_POINT_CLOUD_H
