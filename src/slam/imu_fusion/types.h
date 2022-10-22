#pragma once

#include "common/common.h"
#include "common/time.h"

struct ImuData {
  Time time;
  Vector3d linear_acceleration;
  Vector3d angular_velocity;
};
