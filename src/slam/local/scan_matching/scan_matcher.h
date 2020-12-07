//
// Created by kekeliu on 12/7/20.
//

#ifndef MSF_LOAM_VELODYNE_SCAN_MATCHER_H
#define MSF_LOAM_VELODYNE_SCAN_MATCHER_H

#include "common/timestamped_pointcloud.h"

#include <ceres/ceres.h>

class ScanMatcher {
 public:
  virtual bool Match(const TimestampedPointCloud &cloud1,
                     const TimestampedPointCloud &cloud2,
                     Rigid3d *pose_estimate_2to1) = 0;

  void RefinePoseByRejectOutliers(ceres::Problem &problem);

  virtual ~ScanMatcher() = default;
};

#endif  // MSF_LOAM_VELODYNE_SCAN_MATCHER_H
