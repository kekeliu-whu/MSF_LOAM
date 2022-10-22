
#include "slam/imu_fusion/integration_base.h"

IntegrationBase::IntegrationBase(const Eigen::Vector3d &acc0, const Eigen::Vector3d &gyr0,
                                 const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg)
    : acc0_{acc0},
      gyr0_{gyr0},
      linearized_acc_{acc0},
      linearized_gyr_{gyr0},
      linearized_ba_{linearized_ba},
      linearized_bg_{linearized_bg},
      jacobian_{Eigen::Matrix<double, 15, 15>::Identity()},
      covariance_{Eigen::Matrix<double, 15, 15>::Zero()},
      sum_dt_{0.0},
      delta_p_{Eigen::Vector3d::Zero()},
      delta_q_{Eigen::Quaterniond::Identity()},
      delta_v_{Eigen::Vector3d::Zero()} {
  noise_                     = Eigen::Matrix<double, 18, 18>::Zero();
  noise_.block<3, 3>(0, 0)   = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
  noise_.block<3, 3>(3, 3)   = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
  noise_.block<3, 3>(6, 6)   = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
  noise_.block<3, 3>(9, 9)   = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
  noise_.block<3, 3>(12, 12) = (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
  noise_.block<3, 3>(15, 15) = (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
  AddResultToBuf();
}

void IntegrationBase::push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr) {
  if (dt == 0) return;
  dt_buf_.push_back(dt);
  acc_buf_.push_back(acc);
  gyr_buf_.push_back(gyr);
  propagate(dt, acc, gyr);
}

void IntegrationBase::repropagate(const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg) {
  sum_dt_ = 0.0;
  acc0_   = linearized_acc_;
  gyr0_   = linearized_gyr_;
  delta_p_.setZero();
  delta_q_.setIdentity();
  delta_v_.setZero();
  linearized_ba_ = linearized_ba;
  linearized_bg_ = linearized_bg;
  jacobian_.setIdentity();
  covariance_.setZero();
  ClearResultBuf();
  AddResultToBuf();
  for (int i = 0; i < static_cast<int>(dt_buf_.size()); i++) propagate(dt_buf_[i], acc_buf_[i], gyr_buf_[i]);
}

// compute p,q,v,ba,bg,J,P
void IntegrationBase::midPointIntegration(const double &dt,
                                          const Eigen::Vector3d &acc0, const Eigen::Vector3d &gyr0,
                                          const Eigen::Vector3d &acc1, const Eigen::Vector3d &gyr1,
                                          const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                                          const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                                          Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                                          Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg,
                                          bool update_jacobian) {
  // ROS_INFO("midpoint integration");
  // un := \hat{}
  Eigen::Vector3d un_acc_0 = delta_q * (acc0 - linearized_ba);
  Eigen::Vector3d un_gyr   = 0.5 * (gyr0 + gyr1) - linearized_bg;
  // eq7.1
  result_delta_q           = delta_q * Eigen::Quaterniond(1, un_gyr(0) * dt / 2, un_gyr(1) * dt / 2, un_gyr(2) * dt / 2);
  Eigen::Vector3d un_acc_1 = result_delta_q * (acc1 - linearized_ba);
  Eigen::Vector3d un_acc   = 0.5 * (un_acc_0 + un_acc_1);
  // eq7.2
  result_delta_p = delta_p + delta_v * dt + 0.5 * un_acc * dt * dt;
  // eq7.3
  result_delta_v = delta_v + un_acc * dt;
  // bias remain constant between two consecutive frames
  result_linearized_ba = linearized_ba;
  result_linearized_bg = linearized_bg;

  if (update_jacobian) {
    Eigen::Vector3d w_x   = 0.5 * (gyr0 + gyr1) - linearized_bg;
    Eigen::Vector3d a_0_x = acc0 - linearized_ba;
    Eigen::Vector3d a_1_x = acc1 - linearized_ba;
    Eigen::Matrix3d R_w_x, R_a_0_x, R_a_1_x;

    R_w_x << 0, -w_x(2), w_x(1), w_x(2), 0, -w_x(0), -w_x(1), w_x(0), 0;
    R_a_0_x << 0, -a_0_x(2), a_0_x(1), a_0_x(2), 0, -a_0_x(0), -a_0_x(1), a_0_x(0), 0;
    R_a_1_x << 0, -a_1_x(2), a_1_x(1), a_1_x(2), 0, -a_1_x(0), -a_1_x(1), a_1_x(0), 0;

    // eq22
    Eigen::MatrixXd F     = Eigen::MatrixXd::Zero(15, 15);
    F.block<3, 3>(0, 0)   = Eigen::Matrix3d::Identity();
    F.block<3, 3>(0, 3)   = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * dt * dt + -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt * dt;
    F.block<3, 3>(0, 6)   = Eigen::MatrixXd::Identity(3, 3) * dt;
    F.block<3, 3>(0, 9)   = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * dt * dt;
    F.block<3, 3>(0, 12)  = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * dt * dt * -dt;
    F.block<3, 3>(3, 3)   = Eigen::Matrix3d::Identity() - R_w_x * dt;
    F.block<3, 3>(3, 12)  = -1.0 * Eigen::MatrixXd::Identity(3, 3) * dt;
    F.block<3, 3>(6, 3)   = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * dt + -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt;
    F.block<3, 3>(6, 6)   = Eigen::Matrix3d::Identity();
    F.block<3, 3>(6, 9)   = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * dt;
    F.block<3, 3>(6, 12)  = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * dt * -dt;
    F.block<3, 3>(9, 9)   = Eigen::Matrix3d::Identity();
    F.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();
    // cout<<"A"<<endl<<A<<endl;

    // eq22
    Eigen::MatrixXd V     = Eigen::MatrixXd::Zero(15, 18);
    V.block<3, 3>(0, 0)   = 0.25 * delta_q.toRotationMatrix() * dt * dt;
    V.block<3, 3>(0, 3)   = 0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x * dt * dt * 0.5 * dt;
    V.block<3, 3>(0, 6)   = 0.25 * result_delta_q.toRotationMatrix() * dt * dt;
    V.block<3, 3>(0, 9)   = V.block<3, 3>(0, 3);
    V.block<3, 3>(3, 3)   = 0.5 * Eigen::MatrixXd::Identity(3, 3) * dt;
    V.block<3, 3>(3, 9)   = 0.5 * Eigen::MatrixXd::Identity(3, 3) * dt;
    V.block<3, 3>(6, 0)   = 0.5 * delta_q.toRotationMatrix() * dt;
    V.block<3, 3>(6, 3)   = 0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * dt * 0.5 * dt;
    V.block<3, 3>(6, 6)   = 0.5 * result_delta_q.toRotationMatrix() * dt;
    V.block<3, 3>(6, 9)   = V.block<3, 3>(6, 3);
    V.block<3, 3>(9, 12)  = Eigen::MatrixXd::Identity(3, 3) * dt;
    V.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * dt;

    // eq19
    jacobian_ = F * jacobian_;
    // eq18
    covariance_ = F * covariance_ * F.transpose() + V * noise_ * V.transpose();
  }
}

void IntegrationBase::propagate(double dt, const Eigen::Vector3d &acc1, const Eigen::Vector3d &gyr1) {
  dt_   = dt;
  acc1_ = acc1;
  gyr1_ = gyr1;
  Eigen::Vector3d result_delta_p;
  Eigen::Quaterniond result_delta_q;
  Eigen::Vector3d result_delta_v;
  Eigen::Vector3d result_linearized_ba;
  Eigen::Vector3d result_linearized_bg;

  midPointIntegration(
      dt_, acc0_, gyr0_, acc1_, gyr1_,
      delta_p_, delta_q_, delta_v_, linearized_ba_, linearized_bg_,
      result_delta_p, result_delta_q, result_delta_v, result_linearized_ba, result_linearized_bg,
      true);

  delta_p_       = result_delta_p;
  delta_q_       = result_delta_q;
  delta_v_       = result_delta_v;
  linearized_ba_ = result_linearized_ba;
  linearized_bg_ = result_linearized_bg;
  delta_q_.normalize();
  sum_dt_ += dt_;
  acc0_ = acc1_;
  gyr0_ = gyr1_;

  AddResultToBuf();
}

Eigen::Matrix<double, 15, 1> IntegrationBase::evaluate(
    const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi,
    const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
    const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj,
    const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj) {
  Eigen::Matrix<double, 15, 1> residuals;

  Eigen::Matrix3d dp_dba = jacobian_.block<3, 3>(O_P, O_BA);
  Eigen::Matrix3d dp_dbg = jacobian_.block<3, 3>(O_P, O_BG);

  Eigen::Matrix3d dq_dbg = jacobian_.block<3, 3>(O_R, O_BG);

  Eigen::Matrix3d dv_dba = jacobian_.block<3, 3>(O_V, O_BA);
  Eigen::Matrix3d dv_dbg = jacobian_.block<3, 3>(O_V, O_BG);

  Eigen::Vector3d dba = Bai - linearized_ba_;
  Eigen::Vector3d dbg = Bgi - linearized_bg_;

  // eq20
  Eigen::Quaterniond corrected_delta_q = delta_q_ * Utility::deltaQ(dq_dbg * dbg);
  Eigen::Vector3d corrected_delta_v    = delta_v_ + dv_dba * dba + dv_dbg * dbg;
  Eigen::Vector3d corrected_delta_p    = delta_p_ + dp_dba * dba + dp_dbg * dbg;

  // eq44
  residuals.block<3, 1>(O_P, 0)  = Qi.inverse() * (0.5 * G * sum_dt_ * sum_dt_ + Pj - Pi - Vi * sum_dt_) - corrected_delta_p;
  residuals.block<3, 1>(O_R, 0)  = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
  residuals.block<3, 1>(O_V, 0)  = Qi.inverse() * (G * sum_dt_ + Vj - Vi) - corrected_delta_v;
  residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
  residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
  return residuals;
}

void IntegrationBase::AddResultToBuf() {
  sum_dt_buf_.push_back(sum_dt_);
  delta_p_buf_.push_back(delta_p_);
  delta_q_buf_.push_back(delta_q_);
  delta_v_buf_.push_back(delta_v_);
}

void IntegrationBase::ClearResultBuf() {
  sum_dt_buf_.clear();
  delta_p_buf_.clear();
  delta_q_buf_.clear();
  delta_v_buf_.clear();
}