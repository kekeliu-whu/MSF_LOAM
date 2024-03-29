#ifndef MSF_LOAM_VELODYNE_LASER_ODOMETRY_H
#define MSF_LOAM_VELODYNE_LASER_ODOMETRY_H

#include <ros/node_handle.h>

#include "common/timestamped_pointcloud.h"
#include "laser_mapping.h"
#include "proto/config.pb.h"
#include "slam/imu_fusion/types.h"
#include "slam/local/scan_matching/odometry_scan_matcher.h"

class LaserOdometry {
 public:
  explicit LaserOdometry(bool is_offline_mode, proto::MsfLoamConfig config);

  ~LaserOdometry();

  void AddLaserScan(const TimestampedPointCloud<PointTypeOriginal> &scan_curr);

  void AddImu(const ImuData &imu_data);

  void AddOdom(const OdometryData &odom_data);

 private:
  void PublishTrajectory(const TimestampedPointCloud<PointTypeOriginal> &scan_curr);

 private:
  std::shared_ptr<LaserMapping> laser_mapper_handler_;
  std::unique_ptr<OdometryScanMatcher> scan_matcher_;

  TimestampedPointCloud<PointTypeOriginal> scan_last_;

  // Transformation from scan to map
  Rigid3d pose_scan2world_;
  // Transformation from current scan to previous scan
  Rigid3d pose_curr2last_;

  ros::Publisher laser_odom_publisher_;
  ros::Publisher laser_path_publisher_;

  nav_msgs::Path laser_path_;

  proto::MsfLoamConfig config_;
};

#endif  // MSF_LOAM_VELODYNE_LASER_ODOMETRY_H
