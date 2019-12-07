// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

/**
 * @brief “点-线”距离最小化
 *
 * residual[0] = 面积 / 线段长
 *
 */
struct LidarEdgeFactor {
  LidarEdgeFactor(const Eigen::Vector3d &curr_point,
                  const Eigen::Vector3d &last_point_a,
                  const Eigen::Vector3d &last_point_b, double s)
      : curr_point_(curr_point),
        last_point_a_(last_point_a),
        last_point_b_(last_point_b),
        s_(s) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Matrix<T, 3, 1> cp = curr_point_.cast<T>();
    Eigen::Matrix<T, 3, 1> lpa = last_point_a_.cast<T>();
    Eigen::Matrix<T, 3, 1> lpb = last_point_b_.cast<T>();

    // Eigen::Quaternion<T> g_r_curr2last{q[3], T(s) * q[0], T(s) * q[1], T(s) *
    // q[2]};
    Eigen::Quaternion<T> g_r_curr2last(q);
    Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
    g_r_curr2last = q_identity.slerp(T(s_), g_r_curr2last);
    Eigen::Matrix<T, 3, 1> g_t_curr2last{T(s_) * t[0], T(s_) * t[1],
                                         T(s_) * t[2]};

    Eigen::Matrix<T, 3, 1> lp;
    lp = g_r_curr2last * cp + g_t_curr2last;

    Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
    Eigen::Matrix<T, 3, 1> de = lpa - lpb;

    residual[0] = nu.x() / de.norm();
    residual[1] = nu.y() / de.norm();
    residual[2] = nu.z() / de.norm();

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &curr_point,
                                     const Eigen::Vector3d &last_point_a,
                                     const Eigen::Vector3d &last_point_b,
                                     const double s) {
    return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 3, 4, 3>(
        new LidarEdgeFactor(curr_point, last_point_a, last_point_b, s)));
  }

 private:
  Eigen::Vector3d curr_point_, last_point_a_, last_point_b_;
  double s_;
};

/**
 * @brief “点-面”距离最小化
 *
 * 已知单点和三点构成的面
 * residual[0] = 棱向量 · 平面法向量
 *
 */
struct LidarPlaneFactor {
  LidarPlaneFactor(const Eigen::Vector3d &curr_point,
                   const Eigen::Vector3d &last_point_i,
                   const Eigen::Vector3d &last_point_j,
                   const Eigen::Vector3d &last_point_k, double s)
      : curr_point_(curr_point),
        last_point_i_(last_point_i),
        last_point_j_(last_point_j),
        last_point_k_(last_point_k),
        s_(s) {
    ijk_norm_ =
        (last_point_i - last_point_j).cross(last_point_i - last_point_k);
    ijk_norm_.normalize();
  }

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Matrix<T, 3, 1> cp = curr_point_.cast<T>();
    Eigen::Matrix<T, 3, 1> lpi = last_point_i_.cast<T>();
    Eigen::Matrix<T, 3, 1> ijk = ijk_norm_.cast<T>();

    // Eigen::Quaternion<T> g_r_curr2last{q[3], T(s) * q[0], T(s) * q[1], T(s) *
    // q[2]};
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

  static ceres::CostFunction *Create(const Eigen::Vector3d &curr_point,
                                     const Eigen::Vector3d &last_point_i,
                                     const Eigen::Vector3d &last_point_j,
                                     const Eigen::Vector3d &last_point_k,
                                     const double s) {
    return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>(
        new LidarPlaneFactor(curr_point, last_point_i, last_point_j,
                             last_point_k, s)));
  }

 private:
  Eigen::Vector3d curr_point_, last_point_i_, last_point_j_, last_point_k_;
  Eigen::Vector3d ijk_norm_;
  double s_;
};

/**
 * @brief “点-面”距离最小化
 *
 * 已知平面法向量
 *
 */
struct LidarPlaneNormFactor {
  LidarPlaneNormFactor(const Eigen::Vector3d &curr_point,
                       const Eigen::Vector3d &plane_unit_norm,
                       double negative_OA_dot_norm)
      : curr_point_(curr_point),
        plane_unit_norm_(plane_unit_norm),
        negative_OA_dot_norm_(negative_OA_dot_norm) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w_curr(q);
    Eigen::Matrix<T, 3, 1> t_w_curr(t);
    Eigen::Matrix<T, 3, 1> cp = curr_point_.cast<T>();
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;

    Eigen::Matrix<T, 3, 1> norm = plane_unit_norm_.cast<T>();
    residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm_);
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &curr_point,
                                     const Eigen::Vector3d &plane_unit_norm,
                                     const double negative_OA_dot_norm) {
    return (new ceres::AutoDiffCostFunction<LidarPlaneNormFactor, 1, 4, 3>(
        new LidarPlaneNormFactor(curr_point, plane_unit_norm,
                                 negative_OA_dot_norm)));
  }

 private:
  Eigen::Vector3d curr_point_;
  Eigen::Vector3d plane_unit_norm_;
  double negative_OA_dot_norm_;
};

/**
 * @brief “点-点”距离最小化（ICP）
 *
 */
struct LidarDistanceFactor {
  LidarDistanceFactor(const Eigen::Vector3d &curr_point,
                      const Eigen::Vector3d &closed_point)
      : curr_point_(curr_point), closed_point_(closed_point) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w_curr(q);
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp = curr_point_.cast<T>();
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;

    residual[0] = point_w.x() - T(closed_point_.x());
    residual[1] = point_w.y() - T(closed_point_.y());
    residual[2] = point_w.z() - T(closed_point_.z());
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d &curr_point,
                                     const Eigen::Vector3d &closed_point) {
    return (new ceres::AutoDiffCostFunction<LidarDistanceFactor, 3, 4, 3>(
        new LidarDistanceFactor(curr_point, closed_point)));
  }

 private:
  Eigen::Vector3d curr_point_;
  Eigen::Vector3d closed_point_;
};
