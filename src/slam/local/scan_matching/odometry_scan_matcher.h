#ifndef LOAM_VELODYNE_ODOMETRY_SCAN_MATCHER_H
#define LOAM_VELODYNE_ODOMETRY_SCAN_MATCHER_H

#include <pcl/common/io.h>

#include "scan_matcher.h"

class OdometryScanMatcher : public ScanMatcher {
 public:
  virtual bool MatchScan2Scan(const TimestampedPointCloud<PointTypeOriginal> &cloud1,
                              const TimestampedPointCloud<PointTypeOriginal> &cloud2,
                              Rigid3d *pose_estimate_curr2last);
};

inline PointCloudPtr ToPointType(const PointCloudOriginalConstPtr &pc_in) {
  PointCloudPtr pc_out(new PointCloud);
  pcl::copyPointCloud(*pc_in, *pc_out);
  return pc_out;
}

inline PointCloudPtr ToPointType(const PointCloudOriginalPtr &pc_in) {
  PointCloudPtr pc_out(new PointCloud);
  pcl::copyPointCloud(*pc_in, *pc_out);
  return pc_out;
}

inline PointCloud ToPointType(const PointCloudOriginal &pc_in) {
  PointCloud pc_out;
  pcl::copyPointCloud(pc_in, pc_out);
  return pc_out;
}

inline TimestampedPointCloud<PointType> ToPointType(const TimestampedPointCloud<PointTypeOriginal> &tpc_in) {
  TimestampedPointCloud<PointType> tpc_out;
  tpc_out.frame_id                = tpc_in.frame_id;
  tpc_out.time                    = tpc_in.time;
  tpc_out.map_pose                = tpc_in.map_pose;
  tpc_out.odom_pose               = tpc_in.odom_pose;
  tpc_out.cloud_full_res          = ToPointType(tpc_in.cloud_full_res);
  tpc_out.cloud_corner_sharp      = ToPointType(tpc_in.cloud_corner_sharp);
  tpc_out.cloud_corner_less_sharp = ToPointType(tpc_in.cloud_corner_less_sharp);
  tpc_out.cloud_surf_flat         = ToPointType(tpc_in.cloud_surf_flat);
  tpc_out.cloud_surf_less_flat    = ToPointType(tpc_in.cloud_surf_less_flat);
  return tpc_out;
}

#endif  // LOAM_VELODYNE_ODOMETRY_SCAN_MATCHER_H
