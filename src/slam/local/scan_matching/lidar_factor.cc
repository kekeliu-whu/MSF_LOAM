//
// Created by kekeliu on 5/30/20.
//

#include "slam/local/scan_matching/lidar_factor.h"

template <typename T>
bool LidarEdgeFactor::operator()(const T *q, const T *t, T *residual) const {
  Eigen::Matrix<T, 3, 1> cp = curr_point_.cast<T>();
  Eigen::Matrix<T, 3, 1> lpa = last_point_a_.cast<T>();
  Eigen::Matrix<T, 3, 1> lpb = last_point_b_.cast<T>();

  Eigen::Quaternion<T> g_r_curr2last(q);
  Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
  g_r_curr2last = q_identity.slerp(T(s_), g_r_curr2last);
  Eigen::Matrix<T, 3, 1> g_t_curr2last{T(s_) * t[0], T(s_) * t[1],
                                       T(s_) * t[2]};

  Eigen::Matrix<T, 3, 1> lp;
  lp = g_r_curr2last * cp + g_t_curr2last;

  Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
  Eigen::Matrix<T, 3, 1> de = lpa - lpb;

  residual[0] = nu.norm() / de.norm();

  return true;
}

template bool LidarEdgeFactor::operator()<double>(double const *,
                                                  double const *,
                                                  double *) const;
template bool LidarEdgeFactor::operator()<ceres::Jet<double, 7> >(
    ceres::Jet<double, 7> const *, ceres::Jet<double, 7> const *,
    ceres::Jet<double, 7> *) const;

ceres::CostFunction *LidarEdgeFactor::Create(
    const Eigen::Vector3d &curr_point, const Eigen::Vector3d &last_point_a,
    const Eigen::Vector3d &last_point_b, const double s) {
  return new ceres::AutoDiffCostFunction<LidarEdgeFactor, 1, 4, 3>(
      new LidarEdgeFactor(curr_point, last_point_a, last_point_b, s));
}

template <typename T>
bool LidarPlaneFactor::operator()(const T *q, const T *t, T *residual) const {
  Eigen::Matrix<T, 3, 1> cp = curr_point_.cast<T>();
  Eigen::Matrix<T, 3, 1> lpi = last_plane_C_.cast<T>();
  Eigen::Matrix<T, 3, 1> ijk = last_plane_N_.cast<T>();

  Eigen::Quaternion<T> g_r_curr2last(q);
  Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
  g_r_curr2last = q_identity.slerp(T(s_), g_r_curr2last);
  Eigen::Matrix<T, 3, 1> g_t_curr2last{T(s_) * t[0], T(s_) * t[1],
                                       T(s_) * t[2]};

  Eigen::Matrix<T, 3, 1> lp;
  lp = g_r_curr2last * cp + g_t_curr2last;

  residual[0] = (lp - lpi).dot(ijk);

  return true;
}

template bool LidarPlaneFactor::operator()<double>(double const *,
                                                   double const *,
                                                   double *) const;
template bool LidarPlaneFactor::operator()<ceres::Jet<double, 7> >(
    ceres::Jet<double, 7> const *, ceres::Jet<double, 7> const *,
    ceres::Jet<double, 7> *) const;

ceres::CostFunction *LidarPlaneFactor::Create(
    const Eigen::Vector3d &curr_point, const Eigen::Vector3d &last_point_i,
    const Eigen::Vector3d &last_point_j, const Eigen::Vector3d &last_point_k,
    const double s) {
  auto last_plane_N =
      (last_point_i - last_point_j).cross(last_point_i - last_point_k);
  last_plane_N.normalize();
  auto last_plane_C = (last_point_i + last_point_j + last_point_k) / 3;
  return new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>(
      new LidarPlaneFactor(curr_point, last_plane_C, last_plane_N, s));
}

ceres::CostFunction *LidarPlaneFactor::Create(
    const Eigen::Vector3d &curr_point, const Eigen::Vector3d &last_plane_C,
    const Eigen::Vector3d &last_plane_N) {
  return new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>(
      new LidarPlaneFactor(curr_point, last_plane_C, last_plane_N, 1.0));
}
