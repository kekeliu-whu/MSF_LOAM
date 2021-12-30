#pragma once

#include "slam/imu_fusion/integration_base.h"
#include "slam/imu_fusion/parameters.h"
#include "slam/imu_fusion/utility.h"

#include <ceres/ceres.h>

class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9> {
 public:
  IMUFactor() = delete;

  IMUFactor(const std::shared_ptr<IntegrationBase> &pre_integration)
      : pre_integration_(pre_integration) {
  }

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

 private:
  std::shared_ptr<IntegrationBase> pre_integration_;
};
