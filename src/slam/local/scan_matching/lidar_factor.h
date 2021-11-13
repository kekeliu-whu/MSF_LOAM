// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "slam/imu_fusion/utility.h"

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

struct LidarEdgeFactorSE3 : ceres::SizedCostFunction<3, 7> {
 public:
  LidarEdgeFactorSE3(const Eigen::Vector3d &curr_point,
                     const Eigen::Vector3d &last_plane_C,
                     const Eigen::Vector3d &last_plane_N)
      : curr_point_(curr_point),
        last_plane_C_(last_plane_C),
        last_plane_N_(last_plane_N) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<Eigen::Matrix<double, 3, 1>> residual(residuals);

    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    residual = last_plane_N_.cross(Qi * curr_point_ + Pi - last_plane_C_);

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
        jacobian_pose_i.setZero();
        jacobian_pose_i.block<3, 3>(0, 0) = Utility::skewSymmetric(last_plane_N_);
        jacobian_pose_i.block<3, 3>(0, 3) = -Utility::skewSymmetric(last_plane_N_) * (Qi.toRotationMatrix() * Utility::skewSymmetric(curr_point_));
      }
    }

    return true;
  }

 private:
  const Eigen::Vector3d curr_point_;
  const Eigen::Vector3d last_plane_C_;
  const Eigen::Vector3d last_plane_N_;
};

struct LidarPlaneFactorSE3 : ceres::SizedCostFunction<1, 7> {
 public:
  LidarPlaneFactorSE3(const Eigen::Vector3d &curr_point,
                      const Eigen::Vector3d &last_plane_C,
                      const Eigen::Vector3d &last_plane_N)
      : curr_point_(curr_point),
        last_plane_C_(last_plane_C),
        last_plane_N_(last_plane_N) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<Eigen::Matrix<double, 1, 1>> residual(residuals);

    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    // todo
    residual.setConstant(last_plane_N_.dot((Qi * curr_point_ + Pi - last_plane_C_)));

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
        jacobian_pose_i.setZero();
        jacobian_pose_i.block<1, 3>(0, 0) = last_plane_N_;
        jacobian_pose_i.block<1, 3>(0, 3) = -last_plane_N_.transpose() * (Qi.toRotationMatrix() * Utility::skewSymmetric(curr_point_));
      }
    }

    return true;
  }

 private:
  const Eigen::Vector3d curr_point_;
  const Eigen::Vector3d last_plane_C_;
  const Eigen::Vector3d last_plane_N_;
};
