//
// Created by kekeliu on 12/17/19.
//

#ifndef MSF_LOAM_VELODYNE_MAPPING_SCAN_MATCHER_H
#define MSF_LOAM_VELODYNE_MAPPING_SCAN_MATCHER_H

#include "scan_matcher.h"

class MappingScanMatcher : public ScanMatcher {
 public:
  virtual bool Match(const TimestampedPointCloud &cloud_map,
                     const TimestampedPointCloud &scan_curr,
                     Rigid3d *pose_estimate_map_scan2world);
};

#endif  // MSF_LOAM_VELODYNE_MAPPING_SCAN_MATCHER_H
