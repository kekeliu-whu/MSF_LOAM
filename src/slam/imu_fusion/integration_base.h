#pragma once

#include "slam/imu_fusion/parameters.h"
#include "slam/imu_fusion/utility.h"

#include <ceres/ceres.h>

class IntegrationBase {
 public:
  IntegrationBase() = delete;
  IntegrationBase(const Eigen::Vector3d &acc0, const Eigen::Vector3d &gyr0,
                  const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg);

  void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);

  void repropagate(const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg);

  // compute p,q,v,ba,bg,J,P
  void midPointIntegration(const double &dt,
                           const Eigen::Vector3d &acc0, const Eigen::Vector3d &gyr0,
                           const Eigen::Vector3d &acc1, const Eigen::Vector3d &gyr1,
                           const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                           const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                           Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                           Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg,
                           bool update_jacobian);

  void propagate(double dt, const Eigen::Vector3d &acc1, const Eigen::Vector3d &gyr1);

  Eigen::Matrix<double, 15, 1> evaluate(
      const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi,
      const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
      const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj,
      const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj);

 private:
  void AddResultToBuf();

  void ClearResultBuf();

 public:
  // two adjacent IMU data for midPointIntegration()
  double dt_;
  Eigen::Vector3d acc0_, gyr0_;
  Eigen::Vector3d acc1_, gyr1_;

  // first acc/gyro value in a preintegration buffer
  const Eigen::Vector3d linearized_acc_, linearized_gyr_;
  // linearized IMU bias, initialized by previous value and remain const in the preintegration
  Eigen::Vector3d linearized_ba_, linearized_bg_;

  Eigen::Matrix<double, 15, 15> jacobian_, covariance_;

  double sum_dt_;

  // current preintegration value <p,q,v>
  Eigen::Vector3d delta_p_;
  Eigen::Quaterniond delta_q_;
  Eigen::Vector3d delta_v_;

  // IMU raw buffer for re-propagation
  std::vector<double> dt_buf_;
  std::vector<Eigen::Vector3d> acc_buf_;
  std::vector<Eigen::Vector3d> gyr_buf_;

  std::vector<double> sum_dt_buf_;
  std::vector<Eigen::Vector3d> delta_p_buf_;
  std::vector<Eigen::Quaterniond> delta_q_buf_;
  std::vector<Eigen::Vector3d> delta_v_buf_;

  // IMU noise, including noise and random-walk-noise
  Eigen::Matrix<double, 18, 18> noise_;
};
