#ifndef MSF_LOAM_VELODYNE_GPS_FUSION_H
#define MSF_LOAM_VELODYNE_GPS_FUSION_H

#include <common/rigid_transform.h>
#include <vector>

#include "common/time_def.h"

struct FixedPoint {
  Time timestamp;
  Eigen::Vector3d translation;
};

struct LocalPose {
  Time timestamp;
  Rigid3d pose;
};

class GpsFusion {
 public:
  explicit GpsFusion();

  ~GpsFusion();

  void AddFixedPoint(const Time &time, const Eigen::Vector3d &t);

  void AddLocalPose(const Time &time, const Rigid3d &pose);

  void Optimize();

 private:
  std::vector<FixedPoint> fixed_points_;
  std::vector<LocalPose> local_poses_;
};

#endif  // MSF_LOAM_VELODYNE_GPS_FUSION_H
