//
// Created by kekeliu on 12/7/20.
//

#ifndef MSF_LOAM_VELODYNE_SCAN_MATCHER_H
#define MSF_LOAM_VELODYNE_SCAN_MATCHER_H

#include "common/timestamped_pointcloud.h"

#include <ceres/ceres.h>

class ScanMatcher {
 public:
  virtual bool MatchScan2Scan(const TimestampedPointCloud<PointTypeOriginal> &cloud1,
                              const TimestampedPointCloud<PointTypeOriginal> &cloud2,
                              Rigid3d *pose_estimate_curr2last) = 0;

  virtual bool MatchScan2Map(const TimestampedPointCloud<PointType> &cloud1,
                             const TimestampedPointCloud<PointType> &cloud2,
                             Rigid3d *pose_estimate_map_scan2world) = 0;

  // Reject outliers instead of using loss function,
  // because of possible result instability caused by outliers.
  static void RefineByRejectOutliersWithThreshold(ceres::Problem &problem, int residual_block_size, double threshold = 0.2);

  static void RefineByRejectOutliersWithFrac(ceres::Problem &problem, int residual_block_size, double frac);

  virtual ~ScanMatcher() = default;
};

#endif  // MSF_LOAM_VELODYNE_SCAN_MATCHER_H
