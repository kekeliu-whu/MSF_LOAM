//
// Created by whu on 12/7/19.
//

#ifndef LOAM_VELODYNE_ODOMETRY_SCAN_MATCHER_H
#define LOAM_VELODYNE_ODOMETRY_SCAN_MATCHER_H

#include "common/timestamped_pointcloud.h"

class OdometryScanMatcher {
 public:
  static bool Match(const TimestampedPointCloud &cloud_last,
                    const TimestampedPointCloud &cloud_curr,
                    Rigid3d *pose_estimate_curr2last);
};

#endif  // LOAM_VELODYNE_ODOMETRY_SCAN_MATCHER_H
