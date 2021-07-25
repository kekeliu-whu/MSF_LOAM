#pragma once

#include <vector>
// #include "../parameters.h"
using namespace std;

#include <ros/console.h>
#include <Eigen/Eigen>

/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
class InitialEXRotation {
 public:
  InitialEXRotation();
  bool CalibrationExRotation(Eigen::Quaterniond delta_q_lidar, Eigen::Quaterniond delta_q_imu, Eigen::Matrix3d &calib_ric_result);

 private:
  int frame_count;

  vector<Eigen::Matrix3d> Rc;
  vector<Eigen::Matrix3d> Rimu;
  vector<Eigen::Matrix3d> Rc_g;
  Eigen::Matrix3d ric;
};
