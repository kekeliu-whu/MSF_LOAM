#ifndef LOAM_VELODYNE_ODOMETRY_SCAN_MATCHER_H
#define LOAM_VELODYNE_ODOMETRY_SCAN_MATCHER_H

#include "scan_matcher.h"

class OdometryScanMatcher : public ScanMatcher {
 public:
  virtual bool MatchScan2Scan(const TimestampedPointCloud<PointTypeOriginal> &cloud1,
                              const TimestampedPointCloud<PointTypeOriginal> &cloud2,
                              Rigid3d *pose_estimate_curr2last);

  virtual bool MatchScan2Map(const TimestampedPointCloud<PointType> &cloud1,
                             const TimestampedPointCloud<PointType> &cloud2,
                             Rigid3d *pose_estimate_map_scan2world) {
    exit(-1);  // todo use LOG(FATAL) here
  }
};

inline PointCloudPtr ToTypePointXYZI(const PointCloudOriginalConstPtr &pc_in) {
  PointCloudPtr pc_out(new PointCloud);
  pcl::copyPointCloud(*pc_in, *pc_out);
  return pc_out;
}

inline PointCloudPtr ToTypePointXYZI(const PointCloudOriginalPtr &pc_in) {
  PointCloudPtr pc_out(new PointCloud);
  pcl::copyPointCloud(*pc_in, *pc_out);
  return pc_out;
}

inline PointCloud ToTypePointXYZI(const PointCloudOriginal &pc_in) {
  PointCloud pc_out;
  pcl::copyPointCloud(pc_in, pc_out);
  return pc_out;
}

inline TimestampedPointCloud<PointType> ToTypePointXYZI(const TimestampedPointCloud<PointTypeOriginal> &tpc_in) {
  TimestampedPointCloud<PointType> tpc_out;
  tpc_out.imu_rotation            = tpc_in.imu_rotation;
  tpc_out.frame_id                = tpc_in.frame_id;
  tpc_out.timestamp               = tpc_in.timestamp;
  tpc_out.map_pose                = tpc_in.map_pose;
  tpc_out.odom_pose               = tpc_in.odom_pose;
  tpc_out.cloud_full_res          = ToTypePointXYZI(tpc_in.cloud_full_res);
  tpc_out.cloud_corner_sharp      = ToTypePointXYZI(tpc_in.cloud_corner_sharp);
  tpc_out.cloud_corner_less_sharp = ToTypePointXYZI(tpc_in.cloud_corner_less_sharp);
  tpc_out.cloud_surf_flat         = ToTypePointXYZI(tpc_in.cloud_surf_flat);
  tpc_out.cloud_surf_less_flat    = ToTypePointXYZI(tpc_in.cloud_surf_less_flat);
  return tpc_out;
}

#endif  // LOAM_VELODYNE_ODOMETRY_SCAN_MATCHER_H
