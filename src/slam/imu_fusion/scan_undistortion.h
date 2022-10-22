#pragma once

#include "common/timestamped_pointcloud.h"
#include "slam/imu_fusion/integration_base.h"

Rigid3d GetDeltaQP(const std::shared_ptr<IntegrationBase> &preintegration, double dt);

class ScanUndistortionUtils {
 public:
  static void DoUndistort(
      const TimestampedPointCloud<PointTypeOriginal> &scan_in,
      const std::shared_ptr<IntegrationBase> &imu_int,
      TimestampedPointCloud<PointTypeOriginal> &scan_out);
};
