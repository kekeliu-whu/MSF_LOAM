#include "scan_undistortion.h"

namespace {

void UndistortScanInternal(
    const PointCloudOriginalPtr &cloud_in,
    const IntegrationBase &imu_integration,
    PointCloudOriginalPtr &cloud_out) {
  auto sum_dt_buf = imu_integration.sum_dt_buf_;

  for (auto &p : *cloud_in) {
    CHECK_GE(p.time, 0);
    auto it         = std::upper_bound(sum_dt_buf.begin(), sum_dt_buf.end(), p.time);
    std::size_t idx = (it == sum_dt_buf.end())
                          ? sum_dt_buf.size() - 2
                          : std::distance(sum_dt_buf.begin(), it) - 1;
    CHECK_GE(idx, 0);
    double scale           = (p.time - sum_dt_buf[idx]) / (sum_dt_buf[idx + 1] - sum_dt_buf[idx]);
    auto q                 = imu_integration.delta_q_buf_[idx].slerp(scale, imu_integration.delta_q_buf_[idx + 1]);
    auto p_out             = p;
    p_out.getVector3fMap() = q.cast<float>() * p.getVector3fMap();
    cloud_out->push_back(p_out);
  }
}

}  // namespace

void ScanUndistortion::UndistortScan(
    const TimestampedPointCloud<PointTypeOriginal> &scan_in,
    const IntegrationBase &imu_integration,
    TimestampedPointCloud<PointTypeOriginal> &scan_out) {
  UndistortScanInternal(scan_in.cloud_full_res, imu_integration, scan_out.cloud_full_res);
  UndistortScanInternal(scan_in.cloud_corner_sharp, imu_integration, scan_out.cloud_corner_sharp);
  UndistortScanInternal(scan_in.cloud_corner_less_sharp, imu_integration, scan_out.cloud_corner_less_sharp);
  UndistortScanInternal(scan_in.cloud_surf_flat, imu_integration, scan_out.cloud_surf_flat);
  UndistortScanInternal(scan_in.cloud_surf_less_flat, imu_integration, scan_out.cloud_surf_less_flat);
}
