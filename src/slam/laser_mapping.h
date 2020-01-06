//
// Created by whu on 12/24/19.
//

#ifndef ALOAM_VELODYNE_LASER_MAPPING_H
#define ALOAM_VELODYNE_LASER_MAPPING_H

#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>
#include <condition_variable>
#include <queue>
#include <thread>

#include "common/timestamped_pointcloud.h"
#include "hybrid_grid.h"

using LaserOdometryResultType = TimestampedPointCloud;

class LaserMapping {
 public:
  LaserMapping(bool is_offline_mode, ros::NodeHandle &nh);

  ~LaserMapping();

  void AddLaserOdometryResult(
      const LaserOdometryResultType &laser_odometry_result);

 private:
  void Run();

  // set initial guess for pose
  inline void transformAssociateToMap() {
    pose_map_scan2world_ = pose_odom2map_ * pose_odom_scan2world_;
  }

  inline void transformUpdate() {
    pose_odom2map_ = pose_map_scan2world_ * pose_odom_scan2world_.inverse();
  }

 private:
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;

  std::queue<LaserOdometryResultType> odometry_result_queue_;
  int frame_idx_cur_;

  HybridGrid hybrid_grid_map_corner_;
  HybridGrid hybrid_grid_map_surf_;

  // Transformation from scan to odom's world frame
  Rigid3d pose_odom_scan2world_;
  // Transformation from scan to map's world frame
  Rigid3d pose_map_scan2world_;
  // Transformation between odom's world and map's world frame
  Rigid3d pose_odom2map_;

  pcl::VoxelGrid<PointType> downsize_filter_corner_;
  pcl::VoxelGrid<PointType> downsize_filter_surf_;

  ros::Publisher pubLaserCloudSurround, pubLaserCloudFullRes, pubOdomAftMapped,
      pubOdomAftMappedHighFrec, pubLaserAfterMappedPath;

  nav_msgs::Path laserAfterMappedPath;

  bool is_offline_mode_;

  tf::TransformBroadcaster transform_broadcaster_;
};

#endif  // ALOAM_VELODYNE_LASER_MAPPING_H
