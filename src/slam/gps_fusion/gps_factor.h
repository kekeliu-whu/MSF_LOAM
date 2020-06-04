#ifndef MSF_LOAM_VELODYNE_GPS_FACTOR_H
#define MSF_LOAM_VELODYNE_GPS_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "common/rigid_transform.h"

struct GpsFactor {
  GpsFactor(const Eigen::Vector3d &t_gps, double t, double st)
      : t_gps_(t_gps), t_(t), st_(st) {}

  template <typename T>
  bool operator()(const T *t_i, const T *t_j, T *residual) const {
    Vector<T> t = T(1 - t_) * Vector<T>(t_i) + T(t_) * Vector<T>(t_j);
    Eigen::Map<Vector<T>>{residual} = (t - t_gps_.cast<T>()) / T(st_);
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &t_gps, double t,
                                     double st) {
    return new ceres::AutoDiffCostFunction<GpsFactor, 3, 3, 3>(
        new GpsFactor(t_gps, t, st));
  }

 private:
  Eigen::Vector3d t_gps_;
  double t_;
  double st_;
};

struct RelativePoseFactor {
  RelativePoseFactor(const Rigid3d &pose_gps_i, const Rigid3d &pose_gps_j,
                     double sr, double st)
      : pose_gps_ij_(pose_gps_i.inverse() * pose_gps_j), sr_(sr), st_(st) {}

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
    return true;
  }

  static ceres::CostFunction *Create(const Rigid3d &pose_gps_i,
                                     const Rigid3d &pose_gps_j, double sr,
                                     double st) {
    return (new ceres::AutoDiffCostFunction<RelativePoseFactor, 6, 4, 3, 4, 3>(
        new RelativePoseFactor(pose_gps_i, pose_gps_j, sr, st)));
  }

 protected:
  Rigid3d pose_gps_ij_;
  double sr_;
  double st_;
};

#endif  // MSF_LOAM_VELODYNE_GPS_FACTOR_H
