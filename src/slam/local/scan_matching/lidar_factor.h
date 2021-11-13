// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "slam/imu_fusion/utility.h"

struct LidarEdgeFactorSE3 : ceres::SizedCostFunction<3, 7> {
 public:
  LidarEdgeFactorSE3(const Eigen::Vector3d &curr_point,
                     const Eigen::Vector3d &last_line_C,
                     const Eigen::Vector3d &last_line_N)
      : curr_point_(curr_point),
        last_line_C_(last_line_C),
        last_line_N_(last_line_N) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

 private:
  const Eigen::Vector3d curr_point_;
  const Eigen::Vector3d last_line_C_;
  const Eigen::Vector3d last_line_N_;
};

struct LidarPlaneFactorSE3 : ceres::SizedCostFunction<1, 7> {
 public:
  LidarPlaneFactorSE3(const Eigen::Vector3d &curr_point,
                      const Eigen::Vector3d &last_plane_C,
                      const Eigen::Vector3d &last_plane_N)
      : curr_point_(curr_point),
        last_plane_C_(last_plane_C),
        last_plane_N_(last_plane_N) {}

  LidarPlaneFactorSE3(
      const Eigen::Vector3d &curr_point,
      const Eigen::Vector3d &last_point_i,
      const Eigen::Vector3d &last_point_j,
      const Eigen::Vector3d &last_point_k)
      : curr_point_(curr_point),
        last_plane_C_((last_point_i + last_point_j + last_point_k) / 3),
        last_plane_N_((last_point_i - last_point_j).cross(last_point_i - last_point_k).normalized()) {
  }

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

 private:
  const Eigen::Vector3d curr_point_;
  const Eigen::Vector3d last_plane_C_;
  const Eigen::Vector3d last_plane_N_;
};
