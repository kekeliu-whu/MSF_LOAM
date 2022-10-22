//
// Created by kekeliu on 12/7/20.
//

#ifndef MSF_LOAM_VELODYNE_SCAN_MATCHER_H
#define MSF_LOAM_VELODYNE_SCAN_MATCHER_H

#include "common/timestamped_pointcloud.h"
#include "slam/imu_fusion/integration_base.h"

#include <ceres/ceres.h>

class ScanMatcher {
 public:
  // Reject outliers instead of using loss function,
  // because of possible result instability caused by outliers.
  static void RefineByRejectOutliersWithThreshold(ceres::Problem &problem, int residual_block_size, double threshold = 0.2);

  static void RefineByRejectOutliersWithFrac(ceres::Problem &problem, int residual_block_size, double frac);

  virtual ~ScanMatcher() = default;
};

#endif  // MSF_LOAM_VELODYNE_SCAN_MATCHER_H
