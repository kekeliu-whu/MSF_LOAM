// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>

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
  bool operator()(const T *q, const T *t, T *residual) const;

  static ceres::CostFunction *Create(const Eigen::Vector3d &curr_point,
                                     const Eigen::Vector3d &last_point_a,
                                     const Eigen::Vector3d &last_point_b,
                                     const double s);

 private:
  const Eigen::Vector3d curr_point_, last_point_a_, last_point_b_;
  const double s_;
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
                   const Eigen::Vector3d &last_plane_C,
                   const Eigen::Vector3d &last_plane_N, double s)
      : curr_point_(curr_point),
        last_plane_C_(last_plane_C),
        last_plane_N_(last_plane_N),
        s_(s) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const;

  static ceres::CostFunction *Create(const Eigen::Vector3d &curr_point,
                                     const Eigen::Vector3d &last_point_i,
                                     const Eigen::Vector3d &last_point_j,
                                     const Eigen::Vector3d &last_point_k,
                                     const double s);

  static ceres::CostFunction *Create(const Eigen::Vector3d &curr_point,
                                     const Eigen::Vector3d &last_plane_C,
                                     const Eigen::Vector3d &last_plane_N);

 private:
  const Eigen::Vector3d curr_point_;
  const Eigen::Vector3d last_plane_C_;
  const Eigen::Vector3d last_plane_N_;
  const double s_;
};
