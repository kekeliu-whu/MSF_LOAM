#include <ceres/ceres.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <algorithm>
#include <cstddef>

#include "common/common.h"
#include "slam/estimator/estimator.h"

namespace {

struct VelocityGravityInitFactor {
  VelocityGravityInitFactor(
      const Rigid3d &pose_i,
      const Rigid3d &pose_j,
      const double &dt,
      const Vector3d &delta_p_ij,
      const Vector3d &delta_v_ij)
      : pose_i_(pose_i),
        pose_j_(pose_j),
        dt_(dt),
        delta_p_ij_(delta_p_ij),
        delta_v_ij_(delta_v_ij) {}

  template <typename T>
  bool operator()(const T *_g, const T *_v_i, const T *_v_j, T *residual) const {
    auto q_i = pose_i_.rotation().cast<T>();
    auto q_j = pose_j_.rotation().cast<T>();
    auto p_i = pose_i_.translation().cast<T>();
    auto p_j = pose_j_.translation().cast<T>();
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> g{_g};
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> v_i{_v_i};
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> v_j{_v_j};
    Eigen::Map<Eigen::Matrix<T, 6, 1>> r{residual};
    r.template head<3>() = q_i.conjugate() * (p_i - p_j + v_i * T(dt_) - T(0.5) * g * T(dt_) * T(dt_)) + delta_p_ij_.cast<T>();
    r.template tail<3>() = q_i.conjugate() * (v_i - v_j - g * T(dt_)) + delta_v_ij_.cast<T>();
    return true;
  }

  static ceres::CostFunction *Create(
      const Rigid3d &pose_i,
      const Rigid3d &pose_j,
      double dt,
      const Vector3d &delta_p_ij,
      const Vector3d &delta_v_ij) {
    return new ceres::AutoDiffCostFunction<VelocityGravityInitFactor, 6, 3, 3, 3>(new VelocityGravityInitFactor(pose_i, pose_j, dt, delta_p_ij, delta_v_ij));
  }

 private:
  const Rigid3d pose_i_, pose_j_;
  double dt_;
  const Vector3d delta_p_ij_;
  const Vector3d delta_v_ij_;
};

}  // namespace

void Estimator::AddData(
    const LaserOdometryResultType &curr_odom,
    const Vector3d &velocity,
    const std::vector<ImuData> &imu_buf) {
  auto rs = RobotState{};
  rs.time = curr_odom.time;
  rs.p    = curr_odom.map_pose.translation();
  rs.v    = velocity;
  rs.q    = curr_odom.map_pose.rotation();
  if (states_.empty()) {
    states_.push_back(rs);
    return;
  }

  states_.back().imu_preintegration = BuildPreintegration(imu_buf, states_.back().time, curr_odom.time);
  states_.push_back(rs);

  // todo do not use magic number here
  if (states_.size() == kInitByFirstScanNums) {
    ceres::Problem problem;
    problem.AddParameterBlock(gravity_.data(), 3, new ceres::HomogeneousVectorParameterization(3));
    for (int i = 0; i < states_.size() - 1; ++i) {
      problem.AddResidualBlock(
          VelocityGravityInitFactor::Create(
              {states_[i].p, states_[i].q},
              {states_[i + 1].p, states_[i + 1].q},
              ToSeconds(states_[i + 1].time - states_[i].time),
              states_[i].imu_preintegration->delta_p_,
              states_[i].imu_preintegration->delta_v_),
          nullptr,
          gravity_.data(),
          states_[i].v.data(),
          states_[i + 1].v.data());
    }
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    // first optimal
    ceres::Solve(options, &problem, &summary);
    // second optimal after rejecting outliers
    ScanMatcher::RefineByRejectOutliersWithFrac(problem, 6, 0.15);
    ceres::Solve(options, &problem, &summary);

    LOG(WARNING) << "Gravity and velocity init done, G=" << gravity_.transpose() << "\n, report=\n"
                 << summary.FullReport();

    // todo kk init failed condition?
    this->is_initialized_ = true;
  }
}

std::shared_ptr<IntegrationBase> BuildPreintegration(
    const std::vector<ImuData> &imu_buf,
    const Time &start_time,
    const Time &end_time) {
  // first imu data where t >= start_time
  auto it_start = std::lower_bound(imu_buf.begin(), imu_buf.end(), start_time, [](const ImuData &imu, const Time &time) { return imu.time < time; });
  // first imu data where t >= end_time
  auto it_end                  = std::lower_bound(imu_buf.begin(), imu_buf.end(), end_time, [](const ImuData &imu, const Time &time) { return imu.time < time; });
  double lidar_imu_time_offset = ToSeconds(it_start->time - start_time);
  // here start_time <= imu_buf[si].time ... imu_buf[ei].time < end_time
  size_t si = std::distance(imu_buf.begin(), it_start);
  size_t ei = std::distance(imu_buf.begin(), it_end) - 1;  // this apply to 'end_time == Time::max()' also
  LOG_IF(ERROR, lidar_imu_time_offset >= 0.01)
      << fmt::format(
             "imu preintegration failed: lidar_imu_time_offset={} @ imu={} lidar={}",
             lidar_imu_time_offset,
             imu_buf[si].time,
             start_time);

  // add first phony imu measurement for lidar-imu sync
  auto imu_preintegration = std::make_shared<IntegrationBase>(imu_buf[si].linear_acceleration, imu_buf[si].angular_velocity, Vector3d::Zero(), Vector3d::Zero());
  imu_preintegration->push_back(ToSeconds(imu_buf[si].time - start_time), imu_buf[si].linear_acceleration, imu_buf[si].angular_velocity);
  for (size_t i = si; i < ei - 1; ++i) {
    imu_preintegration->push_back(ToSeconds(imu_buf[i + 1].time - imu_buf[i].time), imu_buf[i + 1].linear_acceleration, imu_buf[i + 1].angular_velocity);
  }
  if (end_time != Time::max()) {
    // add last phony imu measurement for lidar-imu sync
    imu_preintegration->push_back(ToSeconds(end_time - imu_buf[ei - 1].time), imu_buf[ei - 1].linear_acceleration, imu_buf[ei - 1].angular_velocity);
  }

  return imu_preintegration;
}
