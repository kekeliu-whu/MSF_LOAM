#include "slam/gps_fusion/gps_fusion.h"
#include "slam/gps_fusion/gps_factor.h"

namespace {

bool CompareTimeLT(const LocalPose& l, const FixedPoint& r) {
  return l.timestamp < r.timestamp;
}

}  // namespace

GpsFusion::GpsFusion() {}

GpsFusion::~GpsFusion() {}

void GpsFusion::AddFixedPoint(const Time& time,
                              const Eigen::Vector3d& translation) {
  fixed_points_.push_back({time, translation});
}

void GpsFusion::AddLocalPose(const Time& time, const Rigid3d& pose) {
  local_poses_.push_back({time, pose});
}

void GpsFusion::Optimize() {
  CHECK_GT(local_poses_.size(), 2);
  CHECK_GT(fixed_points_.size(), 2);
  CHECK_LT(local_poses_.front().timestamp, fixed_points_.front().timestamp);
  CHECK_LT(fixed_points_.back().timestamp, local_poses_.back().timestamp);

  ceres::Problem problem;
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 10;
  ceres::Solver::Summary summary;
  ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
  ceres::LocalParameterization* local_parameterization =
      new ceres::EigenQuaternionParameterization();

  for (auto& local_pose : local_poses_) {
    problem.AddParameterBlock(local_pose.pose.translation().data(), 3);
    problem.AddParameterBlock(local_pose.pose.rotation().coeffs().data(), 4,
                              local_parameterization);
  }

  for (auto& fixed_point : fixed_points_) {
    auto local_pose_i = std::lower_bound(
        local_poses_.begin(), local_poses_.end(), fixed_point, CompareTimeLT);
    auto cost_function = GpsFactor::Create(fixed_point.translation, 0.01);
    problem.AddResidualBlock(cost_function, loss_function,
                             local_pose_i->pose.translation().data());
  }

  for (size_t i = 0; i < local_poses_.size() - 1; ++i) {
    auto& local_pose_i = local_poses_[i];
    auto& local_pose_j = local_poses_[i + 1];
    auto cost_function = RelativePoseFactor::Create(
        local_pose_i.pose, local_pose_j.pose, 0.01, 0.1);
    problem.AddResidualBlock(cost_function, loss_function,
                             local_pose_i.pose.rotation().coeffs().data(),
                             local_pose_i.pose.translation().data(),
                             local_pose_j.pose.rotation().coeffs().data(),
                             local_pose_j.pose.translation().data());
  }

  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport();
}
