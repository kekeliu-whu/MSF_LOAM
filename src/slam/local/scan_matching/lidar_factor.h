// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ceres/ceres.h>

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

  static constexpr int kResidualNums = 3;
};

struct LidarEdgeFactorDeskewSE3 : ceres::SizedCostFunction<3, 7, 9> {
 public:
  LidarEdgeFactorDeskewSE3(const Eigen::Vector3d &curr_point,
                           const Eigen::Vector3d &last_line_C,
                           const Eigen::Vector3d &last_line_N,
                           const Eigen::Vector3d &delta_p,
                           const Eigen::Quaterniond &delta_q,
                           const double &dt,
                           const Eigen::Vector3d &G)
      : curr_point_(curr_point),
        last_line_C_(last_line_C),
        last_line_N_(last_line_N),
        delta_p_(delta_p),
        delta_q_(delta_q),
        dt_(dt),
        G_(G) {}

  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

 private:
  const Eigen::Vector3d curr_point_;
  const Eigen::Vector3d last_line_C_;
  const Eigen::Vector3d last_line_N_;

  const Eigen::Vector3d delta_p_;
  const Eigen::Quaterniond delta_q_;
  const double dt_;
  const Eigen::Vector3d G_;

  static constexpr int kResidualNums = 3;
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

  static constexpr int kResidualNums = 1;
};

struct LidarPlaneFactorDeskewSE3 : ceres::SizedCostFunction<1, 7, 9> {
 public:
  LidarPlaneFactorDeskewSE3(const Eigen::Vector3d &curr_point,
                            const Eigen::Vector3d &last_plane_C,
                            const Eigen::Vector3d &last_plane_N,
                            const Eigen::Vector3d &delta_p,
                            const Eigen::Quaterniond &delta_q,
                            const double &dt,
                            const Eigen::Vector3d &G)
      : curr_point_(curr_point),
        last_plane_C_(last_plane_C),
        last_plane_N_(last_plane_N),
        delta_p_(delta_p),
        delta_q_(delta_q),
        dt_(dt),
        G_(G) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

 private:
  const Eigen::Vector3d curr_point_;
  const Eigen::Vector3d last_plane_C_;
  const Eigen::Vector3d last_plane_N_;

  const Eigen::Vector3d delta_p_;
  const Eigen::Quaterniond delta_q_;
  const double dt_;
  const Eigen::Vector3d G_;

  static constexpr int kResidualNums = 1;
};
