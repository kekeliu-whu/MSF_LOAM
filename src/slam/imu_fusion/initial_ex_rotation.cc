#include "initial_ex_rotation.h"
#include "utility.h"

using namespace Eigen;

namespace {

const int kWindowSize = 10;

}

InitialEXRotation::InitialEXRotation() {
  frame_count = 0;
  Rc.push_back(Matrix3d::Identity());
  Rc_g.push_back(Matrix3d::Identity());
  Rimu.push_back(Matrix3d::Identity());
  ric = Matrix3d::Identity();
}

bool InitialEXRotation::CalibrationExRotation(Quaterniond delta_q_lidar, Quaterniond delta_q_imu, Matrix3d &calib_ric_result) {
  frame_count++;
  Rc.push_back(delta_q_lidar.toRotationMatrix());
  Rimu.push_back(delta_q_imu.toRotationMatrix());
  Rc_g.push_back(ric.inverse() * delta_q_imu * ric);

  Eigen::MatrixXd A(frame_count * 4, 4);
  A.setZero();
  int sum_ok = 0;
  for (int i = 1; i <= frame_count; i++) {
    Quaterniond r1(Rc[i]);
    Quaterniond r2(Rc_g[i]);

    double angular_distance = 180 / M_PI * r1.angularDistance(r2);
    ROS_DEBUG(
        "%d %f", i, angular_distance);

    double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;
    ++sum_ok;
    Matrix4d L, R;

    double w            = Quaterniond(Rc[i]).w();
    Vector3d q          = Quaterniond(Rc[i]).vec();
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

    A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
  }

  JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
  Matrix<double, 4, 1> x = svd.matrixV().col(3);
  Quaterniond estimated_R(x);
  ric = estimated_R.toRotationMatrix().inverse();
  //cout << svd.singularValues().transpose() << endl;
  //cout << ric << endl;
  Vector3d ric_cov;
  ric_cov = svd.singularValues().tail<3>();
  if (frame_count >= kWindowSize && ric_cov(1) > 0.25) {
    calib_ric_result = ric;
    return true;
  } else
    return false;
}
