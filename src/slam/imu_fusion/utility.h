#pragma once

#include <Eigen/Core>

class Utility {
 public:
  template <typename Derived>
  static Eigen::Quaternion<typename Derived::Scalar> deltaQ(
      const Eigen::MatrixBase<Derived> &theta) {
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
  }
  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(
      const Eigen::MatrixBase<Derived> &q) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1), q(2),
        typename Derived::Scalar(0), -q(0), -q(1), q(0),
        typename Derived::Scalar(0);
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
    ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
    ans.template block<3, 1>(1, 0) = qq.vec();
    ans.template block<3, 3>(1, 1) =
        qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() +
        skewSymmetric(qq.vec());
    return ans;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(
      const Eigen::QuaternionBase<Derived> &p) {
    Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
    ans.template block<3, 1>(1, 0) = pp.vec();
    ans.template block<3, 3>(1, 1) =
        pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() -
        skewSymmetric(pp.vec());
    return ans;
  }
};