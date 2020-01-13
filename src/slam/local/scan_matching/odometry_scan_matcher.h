#ifndef LOAM_VELODYNE_ODOMETRY_SCAN_MATCHER_H
#define LOAM_VELODYNE_ODOMETRY_SCAN_MATCHER_H

#include "common/timestamped_pointcloud.h"

class OdometryScanMatcher {
 public:
  static bool Match(const TimestampedPointCloud &scan_last,
                    const TimestampedPointCloud &scan_curr,
                    Rigid3d *pose_estimate_curr2last);
};

#endif  // LOAM_VELODYNE_ODOMETRY_SCAN_MATCHER_H
