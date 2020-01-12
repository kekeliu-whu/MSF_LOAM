//
// Created by whu on 1/12/20.
//

#ifndef MSF_LOAM_VELODYNE_GPS_FACTOR_H
#define MSF_LOAM_VELODYNE_GPS_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "common/rigid_transform.h"

struct GpsFactor {
  GpsFactor(const Eigen::Vector3d &t_gps, const double st)
      : t_gps_(t_gps), st_(st) {}

  // TODO
  // time not aligned yet
  template <typename T>
  bool operator()(const T *t, T *residual) const {
    Eigen::Map<Vector<T>>{residual} =
        (Vector<T>(t) - t_gps_.cast<T>()) / T(st_);
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &t_gps,
                                     const double st) {
    return new ceres::AutoDiffCostFunction<GpsFactor, 3, 3>(
        new GpsFactor(t_gps, st));
  }

 private:
  Eigen::Vector3d t_gps_;
  double st_;
};

struct RelativePoseFactor {
  RelativePoseFactor(const Rigid3d &pose_gps_i, const Rigid3d &pose_gps_j,
                     double sr, double st)
      : pose_gps_ij_(pose_gps_i.inverse() * pose_gps_j), sr_(sr), st_(st) {}

  // TODO
  template <typename T>
  bool operator()(const T *r_i, const T *t_i, const T *r_j, const T *t_j,
                  T *residual) const {
    // Use {} instead of () to avoid be treated as a function declaration
    Rigid3<T> pose_i{Vector<T>(t_i), Quaternion<T>(r_i)};
    Rigid3<T> pose_j{Vector<T>(t_j), Quaternion<T>(r_j)};
    Rigid3<T> pose_ij = pose_i.inverse() * pose_j;
    Rigid3<T> pose_residual = pose_ij.inverse() * pose_gps_ij_.cast<T>();
    Eigen::Map<Vector<T>>{residual} = pose_residual.translation() / T(st_);
    Eigen::Map<Vector<T>>{residual + 3} =
        pose_residual.rotation().coeffs().template head<3>() / T(sr_);
    //    residual[0] = pose_residual.translation().x() / T(st_);
    //    residual[1] = pose_residual.translation().y() / T(st_);
    //    residual[2] = pose_residual.translation().z() / T(st_);
    //    residual[3] = pose_residual.rotation().x() / T(sr_);
    //    residual[4] = pose_residual.rotation().y() / T(sr_);
    //    residual[5] = pose_residual.rotation().z() / T(sr_);
    return true;
  }

  static ceres::CostFunction *Create(const Rigid3d &pose_gps_i,
                                     const Rigid3d &pose_gps_j, double sr,
                                     double st) {
    return (new ceres::AutoDiffCostFunction<RelativePoseFactor, 6, 4, 3, 4, 3>(
        new RelativePoseFactor(pose_gps_i, pose_gps_j, sr, st)));
  }

 private:
  Rigid3d pose_gps_ij_;
  double sr_;
  double st_;
};

#endif  // MSF_LOAM_VELODYNE_GPS_FACTOR_H
