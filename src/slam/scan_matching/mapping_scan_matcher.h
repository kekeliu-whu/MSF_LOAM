//
// Created by kekeliu on 12/17/19.
//

#ifndef ALOAM_VELODYNE_MAPPING_SCAN_MATCHER_H
#define ALOAM_VELODYNE_MAPPING_SCAN_MATCHER_H

#include "common/timestamped_pointcloud.h"
#include "slam/hybrid_grid.h"

class MappingScanMatcher {
 public:
  static bool Match(const TimestampedPointCloud &cloud_map,
                    const TimestampedPointCloud &scan_curr,
                    Rigid3d *pose_estimate_map_scan2world);
};

#endif  // ALOAM_VELODYNE_MAPPING_SCAN_MATCHER_H
