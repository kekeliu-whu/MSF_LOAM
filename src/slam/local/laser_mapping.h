#ifndef MSF_LOAM_VELODYNE_LASER_MAPPING_H
#define MSF_LOAM_VELODYNE_LASER_MAPPING_H

#include <absl/synchronization/mutex.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>
#include <boost/optional/optional.hpp>
#include <condition_variable>
#include <queue>
#include <thread>

#include "common/common.h"
#include "common/timestamped_pointcloud.h"
#include "msg.pb.h"
#include "slam/estimator/estimator.h"
#include "slam/gps_fusion/gps_fusion.h"
#include "slam/imu_fusion/types.h"
#include "slam/local/scan_matching/mapping_scan_matcher.h"
#include "slam/map/hybrid_grid.h"

class LaserMapping {
 public:
  explicit LaserMapping(bool is_offline_mode);

  ~LaserMapping();

  void AddLaserOdometryResult(
      const LaserOdometryResultType &laser_odometry_result);

  void AddImu(const ImuData &imu_data);

  void AddOdom(const OdometryData &odom_data);

 private:
  static void UndistortScan(
      const LaserOdometryResultType &laser_odometry_result,
      const std::vector<ImuData> &queue,
      LaserOdometryResultType &laser_odometry_result_deskewed);

  void FilterLessFlatLessCornerFeature(const LaserOdometryResultType &odom_result, LaserOdometryResultType &odom_result_filtered);

  void MatchScan2Map(const LaserOdometryResultType &odom_result);

  void InsertScan2Map(const LaserOdometryResultType &odom_result);

  void Run();

  void PublishTrajectory(const LaserOdometryResultType &scan);

  void PublishScan(const LaserOdometryResultType &scan);

  // set initial guess for pose
  void TransformAssociateToMap() {
    pose_map_scan2world_ = pose_odom2map_ * pose_odom_scan2world_;
  }

  void TransformUpdate() {
    pose_odom2map_ = pose_map_scan2world_ * pose_odom_scan2world_.inverse();
  }

 private:
  std::shared_ptr<GpsFusion> gps_fusion_handler_;
  std::unique_ptr<MappingScanMatcher> scan_matcher_;

  int frame_idx_cur_;

  std::thread thread_;

  absl::Mutex mtx_odometry_result_queue_;
  std::queue<LaserOdometryResultType> odometry_result_queue_ ABSL_GUARDED_BY(mtx_odometry_result_queue_);
  boost::optional<LaserOdometryResultType> prev_odometry_result_;

  Estimator estimator;

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

  Vector3d velocity_;

  /**
   * @brief ROS
   *
   */

  ros::Publisher cloud_scan_publisher_;
  ros::Publisher cloud_scan_origin_publisher_;
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

  /**
   * IMU
   */
  absl::Mutex mtx_imu_buf_;
  std::vector<ImuData> imu_buf_ ABSL_GUARDED_BY(mtx_imu_buf_);

  bool is_offline_mode_;
  bool is_firstframe_;
  volatile bool should_exit_;

  proto::PbData pb_data_;
};

#endif  // MSF_LOAM_VELODYNE_LASER_MAPPING_H
