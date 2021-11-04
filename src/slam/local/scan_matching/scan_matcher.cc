//
// Created by kekeliu on 12/7/20.
//

#include "scan_matcher.h"

#include <glog/logging.h>

namespace {
const double kResidualThreshold = 0.2;
}

void ScanMatcher::RefineByRejectOutliersWithThreshold(
    ceres::Problem &problem,
    int residual_block_size,
    double threshold) {
  std::vector<ceres::ResidualBlockId> residual_block_ids;
  problem.GetResidualBlocks(&residual_block_ids);

  std::vector<double> residuals;
  ceres::Problem::EvaluateOptions evaluate_option;
  evaluate_option.apply_loss_function = false;
  problem.Evaluate(evaluate_option, nullptr, &residuals, nullptr, nullptr);

  CHECK_EQ(residual_block_size * residual_block_ids.size(), residuals.size());

  int outlier_count = 0;
  for (int i = 0; i < residual_block_ids.size(); ++i) {
    Eigen::Map<Eigen::VectorXd> v(&residuals[i * residual_block_size], residual_block_size);
    if (v.norm() > kResidualThreshold) {
      problem.RemoveResidualBlock(residual_block_ids[i]);
      ++outlier_count;
    }
  }
  LOG(INFO) << outlier_count << " outliers removed from "
            << residual_block_ids.size() << " PL/PP match pairs with threshold " << kResidualThreshold;
}

void ScanMatcher::RefineByRejectOutliersWithFrac(
    ceres::Problem &problem,
    int residual_block_size,
    double frac) {
  std::vector<ceres::ResidualBlockId> residual_block_ids;
  problem.GetResidualBlocks(&residual_block_ids);

  std::vector<double> residuals;
  ceres::Problem::EvaluateOptions evaluate_option;
  evaluate_option.apply_loss_function = false;
  problem.Evaluate(evaluate_option, nullptr, &residuals, nullptr, nullptr);

  CHECK_EQ(residual_block_size * residual_block_ids.size(), residuals.size());

  // get residual value for each block
  std::vector<std::pair<int, double>> residual_block_id2residual_norm;
  for (int i = 0; i < residual_block_ids.size(); ++i) {
    Eigen::Map<Eigen::VectorXd> v(&residuals[i * residual_block_size], residual_block_size);
    residual_block_id2residual_norm.emplace_back(i, v.norm());
  }
  std::sort(
      residual_block_id2residual_norm.begin(),
      residual_block_id2residual_norm.end(),
      [](std::pair<int, double> &l, std::pair<int, double> &r) {
        return l.second < r.second;
      });
  // reject first ?% outliers
  int outlier_count = 0;
  for (int i = 0; i < static_cast<int>(residual_block_ids.size()) * frac; ++i) {
    auto &e = residual_block_id2residual_norm[residual_block_id2residual_norm.size() - 1 - i];
    problem.RemoveResidualBlock(residual_block_ids[e.first]);
    ++outlier_count;
  }

  LOG(INFO) << outlier_count << " outliers removed from "
            << residual_block_ids.size() << " observations.";
}
