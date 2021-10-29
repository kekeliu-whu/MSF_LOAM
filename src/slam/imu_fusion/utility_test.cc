#include "utility.h"
#include <gtest/gtest.h>
#include "Eigen/src/Core/Matrix.h"
#include "common/rigid_transform.h"

const double kEpsilon = 1e-8;

TEST(UtilsTest, Ql) {
  auto q1 = Quaterniond::UnitRandom();
  auto q2 = Quaterniond::UnitRandom();

  Eigen::Vector4d ans1 = Utility::Qleft(q1) * q2.coeffs();
  Eigen::Vector4d ans2 = (q1 * q2).coeffs();
  EXPECT_LT((ans1 - ans2).norm(), kEpsilon);
}

TEST(UtilsTest, Qr) {
  auto q1 = Quaterniond::UnitRandom();
  auto q2 = Quaterniond::UnitRandom();

  Eigen::Vector4d ans1 = Utility::Qright(q2) * q1.coeffs();
  Eigen::Vector4d ans2 = (q1 * q2).coeffs();
  EXPECT_LT((ans1 - ans2).norm(), kEpsilon);
}

TEST(UtilsTest, QrQl) {
  auto q1 = Quaterniond::UnitRandom();
  auto q2 = Quaterniond::UnitRandom();

  Eigen::Matrix4d ans1 = Utility::Qleft(q1) * Utility::Qright(q2);
  Eigen::Matrix4d ans2 = Utility::Qright(q2) * Utility::Qleft(q1);

  EXPECT_LT((ans1 - ans2).norm(), kEpsilon);
}
