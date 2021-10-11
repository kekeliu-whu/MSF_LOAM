//
// Created by kekeliu on 12/17/19.
//

#ifndef MSF_LOAM_VELODYNE_MAPPING_SCAN_MATCHER_H
#define MSF_LOAM_VELODYNE_MAPPING_SCAN_MATCHER_H

#include <cstdlib>
#include "scan_matcher.h"

class MappingScanMatcher : public ScanMatcher {
 public:
  virtual bool MatchScan2Scan(const TimestampedPointCloud<PointTypeOriginal> &cloud1,
                              const TimestampedPointCloud<PointTypeOriginal> &cloud2,
                              Rigid3d *pose_estimate_2to1) {
    exit(-1);  // todo use LOG(FATAL) here
  }

  virtual bool MatchScan2Map(const TimestampedPointCloud<PointType> &cloud1,
                             const TimestampedPointCloud<PointType> &cloud2,
                             Rigid3d *pose_estimate_map_scan2world);
};

#endif  // MSF_LOAM_VELODYNE_MAPPING_SCAN_MATCHER_H
