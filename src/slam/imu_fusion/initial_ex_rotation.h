#pragma once

#include <vector>
// #include "../parameters.h"
using namespace std;

#include <Eigen/Eigen>

/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
class InitialEXRotation {
 public:
  InitialEXRotation();
  bool CalibrationExRotation(Eigen::Quaterniond delta_q_lidar, Eigen::Quaterniond delta_q_imu, Eigen::Quaterniond *calib_ric_result);

 private:
  int frame_count;

  vector<Eigen::Quaterniond> Rc;
  vector<Eigen::Quaterniond> Rimu;
  Eigen::Quaterniond ric;
};
