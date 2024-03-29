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
  Estimator(const Vector3d &gravity) : is_initialized_(false),
                                       gravity_(gravity) {}

  virtual ~Estimator() = default;

  // Add lidar odometry result and next lidar time
  // 1. Collect imu preintegration measurements and add push it to states_;
  // 2. Init velocity-gravity when states_.size()<kInitByFirstScanNums.
  void AddData(
      const LaserOdometryResultType &prev_odom,
      const Vector3d &velocity,
      const std::vector<ImuData> &imu_buf);

 public:
  bool IsInitialized() const {
    // return false;
    return is_initialized_;
  }

  Vector3d GetGravityVector() const { return gravity_; }

  RobotState GetPrevState() const {
    // CHECK_GT(states_.size(), 0);
    // todo kk
    return states_.size() > 0 ? states_.back() : RobotState{};
  }

 public:
  static const int kInitByFirstScanNums = 50;

 private:
  std::vector<RobotState> states_;
  Vector3d gravity_;
  bool is_initialized_;
};