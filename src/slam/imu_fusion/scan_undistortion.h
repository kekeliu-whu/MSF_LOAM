#pragma once

#include "common/timestamped_pointcloud.h"
#include "slam/imu_fusion/integration_base.h"

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

class ScanUndistortionUtils {
 public:
  static void DoUndistort(
      const TimestampedPointCloud<PointTypeOriginal> &scan_in,
      const std::shared_ptr<IntegrationBase> &imu_int,
      TimestampedPointCloud<PointTypeOriginal> &scan_out);
};
