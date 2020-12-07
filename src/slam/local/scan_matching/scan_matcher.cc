//
// Created by kekeliu on 12/7/20.
//

#include "scan_matcher.h"

void ScanMatcher::RefinePoseByRejectOutliers(ceres::Problem &problem) {
  std::vector<ceres::ResidualBlockId> residual_block_ids;
  problem.GetResidualBlocks(&residual_block_ids);

  std::vector<double> residuals;
  ceres::Problem::EvaluateOptions evaluate_option;
  evaluate_option.apply_loss_function = false;
  problem.Evaluate(evaluate_option, nullptr, &residuals, nullptr, nullptr);

  int outlier_count = 0;
  for (int i = 0; i < residual_block_ids.size(); ++i) {
    // remove outliers instead of using loss function only
    if (residuals[i] > 0.3) {
      problem.RemoveResidualBlock(residual_block_ids[i]);
      ++outlier_count;
    }
  }
  LOG(INFO) << outlier_count << " outliers removed from "
            << residual_block_ids.size() << " PL/PP match pairs.";
}
