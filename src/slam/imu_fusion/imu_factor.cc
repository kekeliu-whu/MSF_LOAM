
#include "slam/imu_fusion/imu_factor.h"

#include <glog/logging.h>
#include <ros/console.h>

bool IMUFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  // P Q V Ba Bg are state to be estimated
  // P=(X,Y,Z)
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  // Q=(x,y,z,w)
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
  Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

  Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

  Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
  Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
  Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

#if 0
        if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
            (Bgi - pre_integration->linearized_bg).norm() > 0.01)
        {
            pre_integration->repropagate(Bai, Bgi);
        }
#endif

  Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
  // eq44
  // calculate residual, update jacobian matrix and covariance
  residual = pre_integration_->evaluate(Pi, Qi, Vi, Bai, Bgi,
                                        Pj, Qj, Vj, Baj, Bgj);

  // ceres take identity matrix as covariance
  // so we have to use llt decomposition for covariance, and then multiply it to the residual and Jacobian matrix respectively
  Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration_->covariance_.inverse()).matrixL().transpose();
  // sqrt_info.setIdentity();
  sqrt_info = 0.001 * sqrt_info;
  residual = sqrt_info * residual;
  // LOG(WARNING) << "preintegration->cov=\n"
  //              << pre_integration_->covariance_;
  // LOG(WARNING) << "sqrt_info=\n"
  //              << sqrt_info;

  if (jacobians) {
    double sum_dt          = pre_integration_->sum_dt_;
    Eigen::Matrix3d dp_dba = pre_integration_->jacobian_.template block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = pre_integration_->jacobian_.template block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = pre_integration_->jacobian_.template block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = pre_integration_->jacobian_.template block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = pre_integration_->jacobian_.template block<3, 3>(O_V, O_BG);

    if (pre_integration_->jacobian_.maxCoeff() > 1e8 || pre_integration_->jacobian_.minCoeff() < -1e8) {
      ROS_WARN("numerical unstable in preintegration");
      //LOG(WARNING) << pre_integration->jacobian ;
      ///                ROS_BREAK();
    }

    // here we use corrected_delta_q instead of delta_q_
    Eigen::Quaterniond corrected_delta_q = pre_integration_->delta_q_ * Utility::deltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg_));

    // compute jacobians
    if (jacobians[0]) {
      // eq46
      Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
      jacobian_pose_i.setZero();

      jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
      jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));
      jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).topLeftCorner<3, 3>();

      jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

      jacobian_pose_i = sqrt_info * jacobian_pose_i;

      if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8) {
        ROS_WARN("numerical unstable in preintegration");
        //std::cout << sqrt_info << std::endl;
        //ROS_BREAK();
      }
    }
    if (jacobians[1]) {
      // eq47
      Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
      jacobian_speedbias_i.setZero();
      jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V)  = -Qi.inverse().toRotationMatrix() * sum_dt;
      jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
      jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

#if 0
            jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
      jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).topLeftCorner<3, 3>() * dq_dbg;
      // jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * pre_integration_->delta_q_).topLeftCorner<3, 3>() * dq_dbg;
#endif

      jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V)  = -Qi.inverse().toRotationMatrix();
      jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
      jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

      jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

      jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

      jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

      //ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
      //ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
    }
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
      jacobian_pose_j.setZero();

      jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

      // jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
      jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).topLeftCorner<3, 3>();

      jacobian_pose_j = sqrt_info * jacobian_pose_j;

      //ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
      //ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
      // LOG(WARNING) << "jacobian_pose_j=\n"
      //              << jacobian_pose_j;
    }
    if (jacobians[3]) {
      // eq49
      Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
      jacobian_speedbias_j.setZero();

      jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V)   = Qi.inverse().toRotationMatrix();
      jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();
      jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

      jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

      //ROS_ASSERT(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
      //ROS_ASSERT(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
      // LOG(WARNING) << "jacobian_speedbias_j=\n"
      //              << jacobian_speedbias_j;
    }
  }

  return true;
}
