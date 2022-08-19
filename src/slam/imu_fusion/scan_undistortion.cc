#include "scan_undistortion.h"

namespace {

void UndistortScanInternal(
    const PointCloudOriginalPtr &cloud_in,
    const std::shared_ptr<IntegrationBase> &imu_integration,
    PointCloudOriginalPtr &cloud_out) {
  auto sum_dt_buf = imu_integration->sum_dt_buf_;

  for (auto &p : *cloud_in) {
    CHECK_GE(p.time, 0);
    auto delta_qp          = GetDeltaQP(imu_integration, p.time);
    auto p_out             = p;
    p_out.getVector3fMap() = delta_qp.rotation().cast<float>() * p.getVector3fMap();
    cloud_out->push_back(p_out);
  }
}

}  // namespace

Rigid3d GetDeltaQP(const std::shared_ptr<IntegrationBase> &preintegration, double dt) {
  // CHECK_NE(preintegration.get(), nullptr);
  auto &&sum_dt_buf = preintegration->sum_dt_buf_;

  CHECK(dt <= sum_dt_buf.back() && dt >= sum_dt_buf.front())
      << "dt = " << dt
      << ",within preintegration first dt = " << preintegration->sum_dt_buf_.front()
      << ",last dt = " << preintegration->sum_dt_buf_.back()
      << ",len(preintegration) = " << preintegration->sum_dt_buf_.size();

  // Find first iterator which satisfies "dt < *it" (and *{it-1} <= dt )
  auto it = std::upper_bound(sum_dt_buf.begin(), sum_dt_buf.end(), dt);

  std::size_t idx = std::distance(sum_dt_buf.begin(), it) - 1;

  double s = (dt - sum_dt_buf[idx]) / (sum_dt_buf[idx + 1] - sum_dt_buf[idx]);
  auto q   = preintegration->delta_q_buf_[idx].slerp(s, preintegration->delta_q_buf_[idx + 1]);
  auto p   = (1 - s) * preintegration->delta_p_buf_[idx] + s * preintegration->delta_p_buf_[idx + 1];

  return {p, q};
}

void ScanUndistortionUtils::DoUndistort(
    const TimestampedPointCloud<PointTypeOriginal> &scan_in,
    const std::shared_ptr<IntegrationBase> &imu_integration,
    TimestampedPointCloud<PointTypeOriginal> &scan_out) {
  auto scan_out_filtered = scan_in.CopyAllFieldsWithoudCloud();
  UndistortScanInternal(scan_in.cloud_full_res, imu_integration, scan_out_filtered.cloud_full_res);
  UndistortScanInternal(scan_in.cloud_corner_sharp, imu_integration, scan_out_filtered.cloud_corner_sharp);
  UndistortScanInternal(scan_in.cloud_corner_less_sharp, imu_integration, scan_out_filtered.cloud_corner_less_sharp);
  UndistortScanInternal(scan_in.cloud_surf_flat, imu_integration, scan_out_filtered.cloud_surf_flat);
  UndistortScanInternal(scan_in.cloud_surf_less_flat, imu_integration, scan_out_filtered.cloud_surf_less_flat);

  scan_out = scan_out_filtered;
}
