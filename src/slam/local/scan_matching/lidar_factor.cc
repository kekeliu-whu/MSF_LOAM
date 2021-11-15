//
// Created by kekeliu on 5/30/20.
//

#include "slam/local/scan_matching/lidar_factor.h"

bool LidarEdgeFactorSE3::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  Eigen::Map<Eigen::Matrix<double, kResidualNums, 1>> residual(residuals);

  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
  residual = last_line_N_.cross(Qi * curr_point_ + Pi - last_line_C_);

  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, kResidualNums, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
      jacobian_pose_i.setZero();
      jacobian_pose_i.block<kResidualNums, 3>(0, 0) = Utility::skewSymmetric(last_line_N_);
      jacobian_pose_i.block<kResidualNums, 3>(0, 3) = -Utility::skewSymmetric(last_line_N_) * (Qi.toRotationMatrix() * Utility::skewSymmetric(curr_point_));
    }
  }

  return true;
}

bool LidarPlaneFactorSE3::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  Eigen::Map<Eigen::Matrix<double, kResidualNums, 1>> residual(residuals);

  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
  // todo
  residual.setConstant(last_plane_N_.dot((Qi * curr_point_ + Pi - last_plane_C_)));

  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, kResidualNums, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
      jacobian_pose_i.setZero();
      jacobian_pose_i.block<kResidualNums, 3>(0, 0) = last_plane_N_;
      jacobian_pose_i.block<kResidualNums, 3>(0, 3) = -last_plane_N_.transpose() * (Qi.toRotationMatrix() * Utility::skewSymmetric(curr_point_));
    }
  }

  return true;
}

bool LidarEdgeFactorDeskewSE3::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  Eigen::Map<Eigen::Matrix<double, kResidualNums, 1>> residual(residuals);

  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
  Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);

  residual = last_line_N_.cross(Qi * (delta_q_ * curr_point_ + delta_p_) + Vi * dt_ - 0.5 * G_ * dt_ * dt_ + Pi - last_line_C_);

  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, kResidualNums, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
      jacobian_pose_i.setZero();
      jacobian_pose_i.block<kResidualNums, 3>(0, 0) = Utility::skewSymmetric(last_line_N_);
      jacobian_pose_i.block<kResidualNums, 3>(0, 3) = -Utility::skewSymmetric(last_line_N_) *
                                                      (Qi.toRotationMatrix() * Utility::skewSymmetric(delta_q_ * curr_point_ + delta_p_));
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, kResidualNums, 3, Eigen::RowMajor>> jacobian_bias_i(jacobians[0]);
      jacobian_bias_i = Utility::skewSymmetric(last_line_N_) * dt_;
    }
  }

  return true;
}

bool LidarPlaneFactorDeskewSE3::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  Eigen::Map<Eigen::Matrix<double, kResidualNums, 1>> residual(residuals);

  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
  Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);

  residual(0, 0) = last_plane_N_.dot(Qi * (delta_q_ * curr_point_ + delta_p_) + Vi * dt_ - 0.5 * G_ * dt_ * dt_ + Pi - last_plane_C_);

  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, kResidualNums, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
      jacobian_pose_i.setZero();
      jacobian_pose_i.block<kResidualNums, 3>(0, 0) = last_plane_N_.transpose();
      jacobian_pose_i.block<kResidualNums, 3>(0, 3) = -last_plane_N_.transpose() *
                                                      (Qi.toRotationMatrix() * Utility::skewSymmetric(delta_q_ * curr_point_ + delta_p_));
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, kResidualNums, 3, Eigen::RowMajor>> jacobian_bias_i(jacobians[0]);
      jacobian_bias_i = last_plane_N_.transpose() * dt_;
    }
  }

  return true;
}
