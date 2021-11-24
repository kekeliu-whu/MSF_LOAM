//
// Created by kekeliu on 12/17/19.
//

#ifndef MSF_LOAM_VELODYNE_MAPPING_SCAN_MATCHER_H
#define MSF_LOAM_VELODYNE_MAPPING_SCAN_MATCHER_H

#include "scan_matcher.h"
#include "slam/estimator/estimator.h"

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

inline std::tuple<Quaterniond, Vector3d> GetDeltaQP(const std::shared_ptr<IntegrationBase> &preintegration, double dt) {
  // CHECK_NE(preintegration.get(), nullptr);
  CHECK_GE(dt, 0);
  auto sum_dt_buf = preintegration->sum_dt_buf_;
  auto it         = std::upper_bound(sum_dt_buf.begin(), sum_dt_buf.end(), dt);
  std::size_t idx = (it == sum_dt_buf.end())
                        ? sum_dt_buf.size() - 2
                        : std::distance(sum_dt_buf.begin(), it) - 1;
  CHECK_GE(idx, 0);

  double s = (dt - sum_dt_buf[idx]) / (sum_dt_buf[idx + 1] - sum_dt_buf[idx]);

  auto q = preintegration->delta_q_buf_[idx].slerp(s, preintegration->delta_q_buf_[idx + 1]);
  auto p = (1 - s) * preintegration->delta_p_buf_[idx] + s * preintegration->delta_p_buf_[idx + 1];
  return {q, p};
}

#endif  // MSF_LOAM_VELODYNE_MAPPING_SCAN_MATCHER_H
