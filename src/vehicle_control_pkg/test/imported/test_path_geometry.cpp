#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "hybrid_path_tracking/path_geometry.hpp"

namespace hybrid_path_tracking {
namespace {

TEST(PathGeometry, StraightPathHasZeroDiscreteCurvature) {
  const std::vector<Point2D> points{
      {0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {15.0, 0.0}};

  const CurvatureResult result = maximumAbsoluteCurvature(points);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.max_abs_curvature, 0.0);
}

TEST(PathGeometry, CircularArcReportsInverseRadius) {
  constexpr double kRadiusM = 10.0;
  const std::vector<Point2D> points{
      {kRadiusM, 0.0},
      {kRadiusM * std::cos(0.2), kRadiusM * std::sin(0.2)},
      {kRadiusM * std::cos(0.4), kRadiusM * std::sin(0.4)}};

  const CurvatureResult result = maximumAbsoluteCurvature(points);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.max_abs_curvature, 0.1, 1e-9);
}

TEST(PathGeometry, NonFinitePointIsMalformed) {
  const std::vector<Point2D> points{
      {0.0, 0.0},
      {5.0, std::numeric_limits<double>::quiet_NaN()},
      {10.0, 0.0}};

  EXPECT_FALSE(maximumAbsoluteCurvature(points).valid);
}

TEST(PathGeometry, ReversingOntoSamePointIsMalformed) {
  const std::vector<Point2D> points{
      {0.0, 0.0}, {5.0, 0.0}, {0.0, 0.0}};

  EXPECT_FALSE(maximumAbsoluteCurvature(points).valid);
}

}  // namespace
}  // namespace hybrid_path_tracking
