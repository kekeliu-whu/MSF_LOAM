#ifndef MSF_LOAM_VELODYNE_LASER_MAPPING_H
#define MSF_LOAM_VELODYNE_LASER_MAPPING_H

#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>
#include <condition_variable>
#include <queue>
#include <thread>

#include "common/timestamped_pointcloud.h"
#include "slam/gps_fusion/gps_fusion.h"
#include "slam/hybrid_grid.h"

using LaserOdometryResultType = TimestampedPointCloud;

class LaserMapping {
 public:
  explicit LaserMapping(bool is_offline_mode);

  ~LaserMapping();

  void AddLaserOdometryResult(
      const LaserOdometryResultType &laser_odometry_result);

 private:
  void Run();

  void PublishScan(const TimestampedPointCloud &scan);

  // set initial guess for pose
  void transformAssociateToMap() {
    pose_map_scan2world_ = pose_odom2map_ * pose_odom_scan2world_;
  }

  void transformUpdate() {
    pose_odom2map_ = pose_map_scan2world_ * pose_odom_scan2world_.inverse();
  }

 private:
  std::shared_ptr<GpsFusion> gps_fusion_handler_;

  int frame_idx_cur_;

  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;

  std::queue<LaserOdometryResultType> odometry_result_queue_;

  HybridGrid hybrid_grid_map_corner_;
  HybridGrid hybrid_grid_map_surf_;

  pcl::VoxelGrid<PointType> downsize_filter_corner_;
  pcl::VoxelGrid<PointType> downsize_filter_surf_;

  // Transformation from scan to odom's world frame
  Rigid3d pose_odom_scan2world_;
  // Transformation from scan to map's world frame
  Rigid3d pose_map_scan2world_;
  // Transformation between odom's world and map's world frame
  Rigid3d pose_odom2map_;

  /**
   * @brief ROS
   *
   */

  ros::Publisher cloud_scan_publisher_;
  ros::Publisher cloud_corner_publisher_;
  ros::Publisher cloud_corner_less_publisher_;
  ros::Publisher cloud_surf_publisher_;
  ros::Publisher cloud_surf_less_publisher_;

  ros::Publisher cloud_surround_publisher_;
  ros::Publisher aftmapped_odom_publisher_;
  ros::Publisher aftmapped_odom_highfrec_publisher_;
  ros::Publisher aftmapped_path_publisher_;

  nav_msgs::Path aftmapped_path_;

  tf::TransformBroadcaster transform_broadcaster_;
};

#endif  // MSF_LOAM_VELODYNE_LASER_MAPPING_H
