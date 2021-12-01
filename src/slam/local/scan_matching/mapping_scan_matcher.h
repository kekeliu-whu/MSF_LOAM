//
// Created by kekeliu on 12/17/19.
//

#ifndef MSF_LOAM_VELODYNE_MAPPING_SCAN_MATCHER_H
#define MSF_LOAM_VELODYNE_MAPPING_SCAN_MATCHER_H

#include "scan_matcher.h"
#include "slam/estimator/estimator.h"
#include "slam/imu_fusion/scan_undistortion.h"

class MappingScanMatcher : public ScanMatcher {
 public:
  bool MatchScan2Map(const TimestampedPointCloud<PointType> &cloud1,
                     const TimestampedPointCloud<PointType> &cloud2,
                     const bool is_initialized,
                     const std::shared_ptr<IntegrationBase> &preintegration,
                     const Vector3d &gravity_vector,
                     const RobotState &prev_state,
                     Rigid3d *pose_estimate_map_scan2world,
                     Vector3d *velocity);
};

#endif  // MSF_LOAM_VELODYNE_MAPPING_SCAN_MATCHER_H
