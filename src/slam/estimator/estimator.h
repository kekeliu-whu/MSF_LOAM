#pragma once

#include <ceres/ceres.h>
#include <fmt/format.h>
#include <fmt/ostream.h>

#include "common/common.h"
#include "common/rigid_transform.h"
#include "common/time.h"
#include "common/timestamped_pointcloud.h"
#include "slam/imu_fusion/integration_base.h"
#include "slam/imu_fusion/types.h"
#include "slam/local/scan_matching/scan_matcher.h"

using LaserOdometryResultType = TimestampedPointCloud<PointTypeOriginal>;

struct ObservationRigid {
  Time time;

  Rigid3d pose;
  Vector3d velocity;
  std::shared_ptr<IntegrationBase> imu_preintegration;
};

class Estimator {
 public:
  // todo kk do not use magic number here
  Estimator() : is_initialized_(false),
                gravity_(0, 0, 9.8055) {}

  virtual ~Estimator() = default;

  // Add lidar odometry result and next lidar time
  // 1. Collect imu preintegration measurements and add push it to obs_;
  // 2. Init velocity-gravity when obs_.size()<kInitByFirstScanNums;
  // 3. Update velocity when obs_.size()>=kInitByFirstScanNums.
  void AddData(
      const LaserOdometryResultType &prev_odom,
      const Time &curr_odom_time,
      const std::vector<ImuData> &imu_buf);

 public:
  bool IsInitialized() const { return is_initialized_; }

 private:
  std::vector<ObservationRigid> obs_;
  Vector3d gravity_;
  bool is_initialized_;

  const int kInitByFirstScanNums = 50;
};