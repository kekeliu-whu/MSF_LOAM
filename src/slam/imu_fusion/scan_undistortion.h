#pragma once

#include "common/timestamped_pointcloud.h"
#include "slam/imu_fusion/integration_base.h"

class ScanUndistortion {
 public:
  static void UndistortScan(
      const TimestampedPointCloud<PointTypeOriginal> &scan_in,
      const IntegrationBase &imu_int,
      TimestampedPointCloud<PointTypeOriginal> &scan_out);
};
