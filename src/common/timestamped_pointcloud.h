#ifndef LOAM_VELODYNE_TIMESTAMPED_POINTCLOUD_H
#define LOAM_VELODYNE_TIMESTAMPED_POINTCLOUD_H

#include <pcl/point_cloud.h>
#include <Eigen/Eigen>

#include "common/common.h"
#include "common/rigid_transform.h"
#include "common/time_def.h"

struct TimestampedPointCloud {
  Time timestamp;
  std::string frame_id;
  Rigid3d odom_pose;
  Rigid3d map_pose;

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

inline PointCloudPtr TransformPointCloud(const PointCloudConstPtr &cloud_in,
                                         const Rigid3d &pose) {
  PointCloudPtr cloud_out(new PointCloud);
  cloud_out->resize(cloud_in->size());
  for (size_t i = 0; i < cloud_in->size(); ++i)
    (*cloud_out)[i] = pose * (*cloud_in)[i];
  return cloud_out;
}

#endif  // MSF_LOAM_VELODYNE_TIME_STAMPED_POINT_CLOUD_H
