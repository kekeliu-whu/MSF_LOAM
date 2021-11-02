#pragma once

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <ceres/solver.h>
#include <fmt/format.h>
#include <fmt/ostream.h>

#include "common/common.h"
#include "common/rigid_transform.h"
#include "common/time.h"
#include "common/timestamped_pointcloud.h"
#include "slam/imu_fusion/integration_base.h"
#include "slam/imu_fusion/types.h"

using LaserOdometryResultType = TimestampedPointCloud<PointTypeOriginal>;

struct ObservationRigid {
  Time time;

  Rigid3d pose;
  Vector3d v;
  std::shared_ptr<IntegrationBase> imu_preintegration;
};

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

class Estimator {
 public:
  Estimator() = default;

  void AddData(const LaserOdometryResultType &prev_odom, const LaserOdometryResultType &curr_odom, const std::vector<ImuData> &imu_buf) {
    // first imu which timestamp GE than cur_odom.time
    // first imu data where t >= prev_odom.time
    auto it_start = std::upper_bound(imu_buf.begin(), imu_buf.end(), prev_odom.time, [](const Time &t, const ImuData &imu) { return t < imu.time; });
    // first imu data where t >= cur_odom.time
    auto it_end                  = std::upper_bound(imu_buf.begin(), imu_buf.end(), curr_odom.time, [](const Time &t, const ImuData &imu) { return t < imu.time; });
    double lidar_imu_time_offset = ToSeconds(it_start->time - prev_odom.time);
    auto si                      = std::distance(imu_buf.begin(), it_start);
    auto ei                      = std::distance(imu_buf.begin(), it_end);
    LOG_IF(ERROR, lidar_imu_time_offset >= 0.01)
        << fmt::format(
               "imu preintegration failed: lidar_imu_time_offset={} @ imu={} lidar={}",
               lidar_imu_time_offset,
               imu_buf[si].time,
               prev_odom.time);

    auto imu_preintegration = std::make_shared<IntegrationBase>(imu_buf[si].linear_acceleration, imu_buf[si].angular_velocity, Vector3d::Zero(), Vector3d::Zero());
    // add first fake imu measurement for lidar-imu sync
    imu_preintegration->push_back(ToSeconds(imu_buf[si].time - prev_odom.time), imu_buf[si].linear_acceleration, imu_buf[si].angular_velocity);
    for (size_t i = si; i < ei - 1; ++i) {
      imu_preintegration->push_back(ToSeconds(imu_buf[i + 1].time - imu_buf[i].time), imu_buf[i + 1].linear_acceleration, imu_buf[i + 1].angular_velocity);
    }
    // add last fake imu measurement for lidar-imu sync
    CHECK_GT(ToSeconds(curr_odom.time - imu_buf[ei - 1].time), 0);
    imu_preintegration->push_back(ToSeconds(curr_odom.time - imu_buf[ei - 1].time), imu_buf[ei - 1].linear_acceleration, imu_buf[ei - 1].angular_velocity);

    if (!obs_.empty()) {
      // CHECK_EQ(obs_.back().time, prev_odom.time);
    }
    auto ob               = ObservationRigid{};
    ob.time               = prev_odom.time;
    ob.pose               = prev_odom.map_pose;
    ob.imu_preintegration = std::move(imu_preintegration);
    ob.v.setZero();
    obs_.push_back(ob);

    // todo do not use magic number here
    if (obs_.size() == 100) {
      Vector3d G{0, 0, 9.8055};
      ceres::Problem problem;
      problem.AddParameterBlock(G.data(), 3, new ceres::HomogeneousVectorParameterization(3));
      for (int i = 0; i < obs_.size() - 1; ++i) {
        problem.AddResidualBlock(
            VelocityGravityInitFactor::Create(
                obs_[i].pose,
                obs_[i + 1].pose,
                ToSeconds(obs_[i + 1].time - obs_[i].time),
                obs_[i].imu_preintegration->delta_p_,
                obs_[i].imu_preintegration->delta_v_),
            nullptr,
            G.data(),
            obs_[i].v.data(),
            obs_[i + 1].v.data());
      }
      ceres::Solver::Options options;
      options.minimizer_progress_to_stdout = true;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      // todo
      LOG(WARNING) << G.transpose();
      for (auto e : obs_) {
        LOG(WARNING) << e.time << " " << e.v.transpose();
      }
      LOG(FATAL) << summary.FullReport();
    }
  }

  virtual ~Estimator() = default;

 private:
  std::vector<ObservationRigid> obs_;
};