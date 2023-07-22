#pragma once

#include <Eigen/Eigen>
#include <iostream>

template <int N>
Eigen::Matrix<double, N, 1> CubicBSplineApprox(
    const Eigen::Matrix<double, N, 1> &p_1,
    const Eigen::Matrix<double, N, 1> &p0,
    const Eigen::Matrix<double, N, 1> &p1,
    const Eigen::Matrix<double, N, 1> &p2,
    double s) {
  double s2 = s * s;
  double s3 = s * s * s;

  return (p_1 * std::pow(1 - s, 3) + p0 * (3 * s3 - 6 * s2 + 4) + p1 * (-3 * s3 + 3 * s2 + 3 * s + 1) + p2 * s3) / 6;
}
