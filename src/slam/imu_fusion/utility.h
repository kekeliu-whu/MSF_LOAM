#pragma once

#include <Eigen/Eigen>

class Utility {
 public:
  template <typename Derived>
  static Eigen::Quaternion<typename Derived::Scalar> deltaQ(
      const Eigen::MatrixBase<Derived> &v3d) {
    typedef typename Derived::Scalar Scalar_t;

    const double kAngleEpisode = 1e-6;
    double theta               = v3d.norm();
    double half_theta          = 0.5 * theta;

    double imag_factor;
    double real_factor = cos(half_theta);
    if (theta < kAngleEpisode) {
      double theta_sq  = theta * theta;
      double theta_po4 = theta_sq * theta_sq;
      // taylor expansion of sin(t/2)/t, visit https://www.wolframalpha.com/input/?i=sin%28t%2F2%29%2Ft for reference.
      imag_factor = 0.5 - (1 / 48.) * theta_sq + (1 / 3840.) * theta_po4;
    } else {
      double sin_half_theta = sin(half_theta);
      imag_factor           = sin_half_theta / theta;
    }

    // return {cos(|t|/2), sin(|t|/2)/|t|*t}
    return Eigen::Quaterniond(real_factor, imag_factor * v3d.x(), imag_factor * v3d.y(), imag_factor * v3d.z())
        .cast<Scalar_t>();
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(
      const Eigen::MatrixBase<Derived> &q) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
        q(2), typename Derived::Scalar(0), -q(0),
        -q(1), q(0), typename Derived::Scalar(0);
    return ans;
  }

  template <typename Derived>
  static Eigen::Quaternion<typename Derived::Scalar> positify(
      const Eigen::QuaternionBase<Derived> &q) {
    // printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
    // Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(),
    // -q.z()); printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z()); return
    // q.template w() >= (typename Derived::Scalar)(0.0) ? q :
    // Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(),
    // -q.z());
    return q;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(
      const Eigen::QuaternionBase<Derived> &q) {
    Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans.template block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * qq.w() + skewSymmetric(qq.vec());
    ans.template block<3, 1>(0, 3) = qq.vec();
    ans.template block<1, 3>(3, 0) = -qq.vec();
    ans(3, 3)                      = qq.w();
    return ans;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(
      const Eigen::QuaternionBase<Derived> &q) {
    Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans.template block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * qq.w() - skewSymmetric(qq.vec());
    ans.template block<3, 1>(0, 3) = qq.vec();
    ans.template block<1, 3>(3, 0) = -qq.vec();
    ans(3, 3)                      = qq.w();
    return ans;
  }
};