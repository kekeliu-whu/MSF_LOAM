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
    auto q                 = std::get<0>(delta_qp);
    auto p_out             = p;
    p_out.getVector3fMap() = q.cast<float>() * p.getVector3fMap();
    cloud_out->push_back(p_out);
  }
}

}  // namespace

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
