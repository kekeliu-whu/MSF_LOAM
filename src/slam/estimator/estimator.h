#pragma once

#include "common/timestamped_pointcloud.h"
#include "slam/imu_fusion/integration_base.h"
#include "slam/imu_fusion/types.h"
#include "slam/local/scan_matching/scan_matcher.h"

using LaserOdometryResultType = TimestampedPointCloud<PointTypeOriginal>;

struct RobotState {
  Time time;

  Vector3d p;
  Vector3d v;
  Quaterniond q;
  Vector3d bg;
  Vector3d ba;
  std::shared_ptr<IntegrationBase> imu_preintegration;
};

std::shared_ptr<IntegrationBase> BuildPreintegration(
    const std::vector<ImuData> &imu_buf,
    const Time &start_time,
    const Time &end_time = Time::max());

class Estimator {
 public:
  // todo kk do not use magic number here
  Estimator() : is_initialized_(false),
                gravity_(0, 0, 9.8055) {}

  virtual ~Estimator() = default;

  // Add lidar odometry result and next lidar time
  // 1. Collect imu preintegration measurements and add push it to obs_;
  // 2. Init velocity-gravity when obs_.size()<kInitByFirstScanNums.
  void AddData(
      const LaserOdometryResultType &prev_odom,
      const Time &curr_odom_time,
      const std::vector<ImuData> &imu_buf,
      const Vector3d &velocity);

 public:
  bool IsInitialized() const {
    return is_initialized_;
    // return false;
  }

  Vector3d GetGravityVector() const { return gravity_; }

  RobotState GetPrevState() const {
    // CHECK_GT(obs_.size(), 0);
    // todo kk
    return obs_.size() > 0 ? obs_.back() : RobotState{};
  }

 private:
  std::vector<RobotState> obs_;
  Vector3d gravity_;
  bool is_initialized_;

  const int kInitByFirstScanNums = 50;
};