#pragma once

#include "common/time.h"
#include <Eigen/Eigen>

struct ImuData {
  Time time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};
