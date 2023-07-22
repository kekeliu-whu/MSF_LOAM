#include <gtest/gtest.h>

#include "common/bspline.h"

using Vector1d = Eigen::Matrix<double, 1, 1>;

TEST(BSplineTest, ArithmeticProgression) {
  EXPECT_DOUBLE_EQ(CubicBSplineApprox(Vector1d{1}, Vector1d{2}, Vector1d{3}, Vector1d{4}, 0)[0], 2);
  EXPECT_DOUBLE_EQ(CubicBSplineApprox(Vector1d{1}, Vector1d{2}, Vector1d{3}, Vector1d{4}, 1)[0], 3);
  EXPECT_DOUBLE_EQ(CubicBSplineApprox(Vector1d{1}, Vector1d{2}, Vector1d{3}, Vector1d{4}, 0.4)[0], 2.4);
  EXPECT_DOUBLE_EQ(CubicBSplineApprox(Vector1d{1}, Vector1d{2}, Vector1d{3}, Vector1d{4}, 0.5)[0], 2.5);
}

TEST(BSplineTest, Const) {
  EXPECT_DOUBLE_EQ(CubicBSplineApprox(Vector1d{2}, Vector1d{2}, Vector1d{2}, Vector1d{2}, 0)[0], 2);
  EXPECT_DOUBLE_EQ(CubicBSplineApprox(Vector1d{2}, Vector1d{2}, Vector1d{2}, Vector1d{2}, 1)[0], 2);
  EXPECT_DOUBLE_EQ(CubicBSplineApprox(Vector1d{2}, Vector1d{2}, Vector1d{2}, Vector1d{2}, 0.5)[0], 2);
  EXPECT_DOUBLE_EQ(CubicBSplineApprox(Vector1d{2}, Vector1d{2}, Vector1d{2}, Vector1d{2}, 0.4)[0], 2);
}

TEST(BSplineTest, 1) {
  // for (double d = 0; d <= 1.01; d += 0.02) {
  //   auto ret = CubicBSpline(Vector1d{4}, Vector1d{3}, Vector1d{2}, Vector1d{1}, d);
  //   std::cout << d << " " << ret << std::endl;
  // }
}
