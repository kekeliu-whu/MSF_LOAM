//
// Created by kekeliu on 5/30/20.
//

#include "slam/local/scan_matching/lidar_factor.h"

bool LidarEdgeFactorSE3::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  Eigen::Map<Eigen::Matrix<double, 3, 1>> residual(residuals);

  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
  residual = last_line_N_.cross(Qi * curr_point_ + Pi - last_line_C_);

  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
      jacobian_pose_i.setZero();
      jacobian_pose_i.block<3, 3>(0, 0) = Utility::skewSymmetric(last_line_N_);
      jacobian_pose_i.block<3, 3>(0, 3) = -Utility::skewSymmetric(last_line_N_) * (Qi.toRotationMatrix() * Utility::skewSymmetric(curr_point_));
    }
  }

  return true;
}

bool LidarPlaneFactorSE3::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
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
