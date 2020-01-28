#ifndef MSF_LOAM_VELODYNE_GRAPH_FACTOR_H
#define MSF_LOAM_VELODYNE_GRAPH_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "common/rigid_transform.h"
#include "slam/gps_fusion/gps_factor.h"

struct PoseGraphEdgeFactor : public RelativePoseFactor {
  PoseGraphEdgeFactor(const Rigid3d &pose_gps_ij, double sr, double st)
      : RelativePoseFactor(Rigid3d(), pose_gps_ij, sr, st) {}

  static ceres::CostFunction *Create(const Rigid3d &pose_gps_ij, double sr,
                                     double st) {
    return (new ceres::AutoDiffCostFunction<PoseGraphEdgeFactor, 6, 4, 3, 4, 3>(
        new PoseGraphEdgeFactor(pose_gps_ij, sr, st)));
  }
};

#endif  // MSF_LOAM_VELODYNE_GPS_FACTOR_H
