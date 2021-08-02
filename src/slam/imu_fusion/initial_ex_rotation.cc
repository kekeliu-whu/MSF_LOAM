#include "initial_ex_rotation.h"
#include <Eigen/src/Geometry/Quaternion.h>
#include <cstdio>
#include "utility.h"

using namespace Eigen;

namespace {

const int kWindowSize = 10;

}

InitialEXRotation::InitialEXRotation() {
  frame_count = 0;
  ric         = Quaterniond::Identity();
}

bool InitialEXRotation::CalibrationExRotation(Quaterniond delta_q_lidar, Quaterniond delta_q_imu, Quaterniond *ric_result) {
  frame_count++;
  Rc.push_back(delta_q_lidar);
  Rimu.push_back(delta_q_imu);

  Eigen::MatrixXd A(frame_count * 4, 4);
  A.setZero();
  for (int i = 0; i < frame_count; i++) {
    Quaterniond r1(Rc[i]);
    Quaterniond r2 = ric * Rimu[i] * ric.inverse();

    double angular_distance = 180 / M_PI * r1.angularDistance(r2);
    double a1               = 180 / M_PI * r1.angularDistance(Quaterniond::Identity());
    double a2               = 180 / M_PI * r2.angularDistance(Quaterniond::Identity());
    printf("%10.5f%10.5f%10.5f\n", a1, a2, angular_distance);
    ROS_DEBUG(
        "%d %f", i, angular_distance);

    double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;
    Matrix4d L, R;

    double w            = Rc[i].w();
    Vector3d q          = Rc[i].vec();
    L.block<3, 3>(0, 0) = w * Matrix3d::Identity() + Utility::skewSymmetric(q);
    L.block<3, 1>(0, 3) = q;
    L.block<1, 3>(3, 0) = -q.transpose();
    L(3, 3)             = w;

    Quaterniond R_ij(Rimu[i]);
    w                   = R_ij.w();
    q                   = R_ij.vec();
    R.block<3, 3>(0, 0) = w * Matrix3d::Identity() - Utility::skewSymmetric(q);
    R.block<3, 1>(0, 3) = q;
    R.block<1, 3>(3, 0) = -q.transpose();
    R(3, 3)             = w;

    A.block<4, 4>(i * 4, 0) = huber * (L - R);
  }

  JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
  ric.coeffs() = svd.matrixV().col(3);
  ric.normalize();
  Vector3d ric_cov;
  ric_cov = svd.singularValues().tail<3>();
  if (frame_count >= kWindowSize && ric_cov(1) > 0.25) {
    *ric_result = ric;
    return true;
  } else
    return false;
}
