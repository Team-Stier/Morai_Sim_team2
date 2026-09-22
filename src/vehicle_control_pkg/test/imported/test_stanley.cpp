#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "hybrid_path_tracking/stanley.hpp"

namespace hybrid_path_tracking {
namespace {

constexpr double kPi = 3.14159265358979323846;

VehicleState2D stateAtOrigin(double speed_kph) {
  return {0.0, 0.0, 0.0, speed_kph};
}

Path2D straightPath(double length_m) {
  return Path2D{{{0.0, 0.0}, {length_m, 0.0}}, 1.0, 0};
}

Path2D leftArcPath() {
  Path2D path{{}, 1.0, 0};
  constexpr double kRadiusM = 15.0;
  for (double theta = 0.0; theta <= 0.8; theta += 0.04) {
    path.points.push_back(
        {kRadiusM * std::sin(theta), kRadiusM * (1.0 - std::cos(theta))});
  }
  return path;
}

void expectFiniteCandidate(const ControllerCandidate& candidate) {
  EXPECT_TRUE(std::isfinite(candidate.steering_angle_rad));
  EXPECT_TRUE(std::isfinite(candidate.cross_track_error_m));
  EXPECT_TRUE(std::isfinite(candidate.heading_error_rad));
  EXPECT_TRUE(std::isfinite(candidate.path_curvature));
}

TEST(Stanley, CenteredStraightPathProducesZeroSteer) {
  const auto out = Stanley(defaultStanleyConfig())
                       .calculate(stateAtOrigin(30.0), straightPath(30.0));

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.steering_angle_rad, 0.0, 1e-6);
  EXPECT_NEAR(out.cross_track_error_m, 0.0, 1e-6);
  EXPECT_NEAR(out.heading_error_rad, 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(out.path_curvature, 0.0);
  expectFiniteCandidate(out);
}

TEST(Stanley, CurvedPathReportsCurvatureFromPathGeometry) {
  const auto out = Stanley(defaultStanleyConfig())
                       .calculate(stateAtOrigin(30.0), leftArcPath());

  ASSERT_TRUE(out.valid);
  EXPECT_NEAR(out.path_curvature, 1.0 / 15.0, 1e-6);
}

TEST(Stanley, VehicleRightOfPathProducesPositiveCorrection) {
  VehicleState2D state{0.0, -1.0, 0.0, 20.0};
  const auto out = Stanley(defaultStanleyConfig()).calculate(state, straightPath(30.0));

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.cross_track_error_m, 1.0, 1e-6);
  EXPECT_GT(out.steering_angle_rad, 0.0);
  expectFiniteCandidate(out);
}

TEST(Stanley, VehicleLeftOfPathProducesNegativeCorrection) {
  VehicleState2D state{0.0, 1.0, 0.0, 20.0};
  const auto out = Stanley(defaultStanleyConfig()).calculate(state, straightPath(30.0));

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.cross_track_error_m, -1.0, 1e-6);
  EXPECT_LT(out.steering_angle_rad, 0.0);
  expectFiniteCandidate(out);
}

TEST(Stanley, HeadingErrorUsesPathMinusVehicleYawWithCenteredFrontAxle) {
  constexpr double kYaw = 0.2;
  const VehicleState2D state{-3.0 * std::cos(kYaw), -3.0 * std::sin(kYaw),
                             kYaw, 20.0};
  const auto out = Stanley(defaultStanleyConfig()).calculate(state, straightPath(30.0));

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.cross_track_error_m, 0.0, 1e-6);
  EXPECT_NEAR(out.heading_error_rad, -kYaw, 1e-6);
  EXPECT_NEAR(out.steering_angle_rad, -kYaw, 1e-6);
}

TEST(Stanley, LowAndZeroSpeedCalculationsAreFinite) {
  Stanley stanley(defaultStanleyConfig());
  const auto stopped = stanley.calculate(stateAtOrigin(0.0), leftArcPath());
  const auto crawling = stanley.calculate(stateAtOrigin(0.036), leftArcPath());

  EXPECT_TRUE(stopped.valid);
  EXPECT_TRUE(crawling.valid);
  expectFiniteCandidate(stopped);
  expectFiniteCandidate(crawling);
}

TEST(Stanley, NegativeFiniteSpeedIsClampedToStoppedSpeed) {
  VehicleState2D negative_speed{0.0, -1.0, 0.0, -18.0};
  VehicleState2D stopped{0.0, -1.0, 0.0, 0.0};
  Stanley stanley(defaultStanleyConfig());

  const auto negative_out = stanley.calculate(negative_speed, straightPath(30.0));
  const auto stopped_out = stanley.calculate(stopped, straightPath(30.0));
  EXPECT_TRUE(negative_out.valid);
  EXPECT_TRUE(stopped_out.valid);
  EXPECT_NEAR(negative_out.cross_track_error_m, 1.0, 1e-6);
  EXPECT_NEAR(negative_out.steering_angle_rad, stopped_out.steering_angle_rad,
              1e-6);
  expectFiniteCandidate(negative_out);
}

TEST(Stanley, ClampsSteeringToConfiguredLimit) {
  StanleyConfig config = defaultStanleyConfig();
  config.heading_gain = 10.0;
  const Path2D path{{{0.0, 0.0}, {30.0 * std::cos(kPi / 3.0),
                                  30.0 * std::sin(kPi / 3.0)}},
                    1.0, 0};
  const auto out = Stanley(config).calculate(stateAtOrigin(20.0), path);

  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(out.saturated);
  EXPECT_NEAR(out.steering_angle_rad, config.max_steering_rad, 1e-6);
}

TEST(Stanley, UsesNearestSegmentInteriorProjection) {
  const Path2D path{{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}}, 1.0, 0};
  const VehicleState2D state{8.0, 1.0, 0.0, 20.0};  // Front axle is (11, 1).
  StanleyConfig config = defaultStanleyConfig();
  config.min_path_length_m = 1.0;
  const auto out = Stanley(config).calculate(state, path);

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.cross_track_error_m, 1.0, 1e-6);
  EXPECT_NEAR(out.heading_error_rad, kPi / 2.0, 1e-6);
}

TEST(Stanley, ChoosesOutgoingTangentAtAnExactCornerProjection) {
  StanleyConfig config = defaultStanleyConfig();
  config.min_path_length_m = 1.0;
  const Path2D path{{{0.0, 0.0}, {10.0, 0.0}, {10.0, 30.0}}, 1.0, 0};
  const VehicleState2D state{7.0, 0.0, 0.0, 20.0};  // Front axle is (10, 0).
  const auto out = Stanley(config).calculate(state, path);

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.cross_track_error_m, 0.0, 1e-6);
  EXPECT_NEAR(out.heading_error_rad, kPi / 2.0, 1e-6);
  EXPECT_TRUE(out.saturated);
  EXPECT_NEAR(out.steering_angle_rad, config.max_steering_rad, 1e-6);
}

TEST(Stanley, RejectsAdjacentReverseTangentAtAnExactCornerProjection) {
  StanleyConfig config = defaultStanleyConfig();
  config.min_path_length_m = 1.0;
  const Path2D path{{{0.0, 0.0}, {10.0, 0.0}, {0.0, 0.0}}, 1.0, 0};
  const VehicleState2D state{7.0, 0.0, 0.0, 20.0};  // Front axle is (10, 0).
  const auto out = Stanley(config).calculate(state, path);

  EXPECT_FALSE(out.valid);
  expectFiniteCandidate(out);
}

TEST(Stanley, AcceptsCollinearSameDirectionAdjacentTie) {
  const Path2D path{{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}}, 1.0, 0};
  const VehicleState2D state{7.0, 0.0, 0.0, 20.0};  // Front axle is (10, 0).
  const auto out = Stanley(defaultStanleyConfig()).calculate(state, path);

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.cross_track_error_m, 0.0, 1e-6);
  EXPECT_NEAR(out.heading_error_rad, 0.0, 1e-6);
  EXPECT_NEAR(out.steering_angle_rad, 0.0, 1e-6);
}

TEST(Stanley, RejectsEqualDistanceNonCollinearSelfIntersectionBranches) {
  const Path2D path{{{0.0, 0.0}, {10.0, 10.0}, {0.0, 10.0},
                     {10.0, 0.0}, {30.0, 0.0}},
                    1.0, 0};
  const VehicleState2D state{2.0, 5.0, 0.0, 20.0};  // Front axle is (5, 5).
  const auto out = Stanley(defaultStanleyConfig()).calculate(state, path);

  EXPECT_FALSE(out.valid);
  expectFiniteCandidate(out);
}

TEST(Stanley, UsesClosestSegmentEndpointProjection) {
  StanleyConfig config = defaultStanleyConfig();
  config.min_path_length_m = 1.0;
  const Path2D path{{{0.0, 0.0}, {10.0, 0.0}}, 1.0, 0};
  const VehicleState2D state{9.0, -2.0, 0.0, 20.0};  // Front axle is past endpoint.
  const auto out = Stanley(config).calculate(state, path);

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.cross_track_error_m, std::sqrt(8.0), 1e-6);
  EXPECT_NEAR(out.heading_error_rad, 0.0, 1e-6);
}

TEST(Stanley, UsesClosestSegmentStartEndpointProjection) {
  const Path2D path{{{10.0, 0.0}, {30.0, 0.0}}, 1.0, 0};
  const VehicleState2D state{0.0, -2.0, 0.0, 20.0};  // Front axle is before start.
  const auto out = Stanley(defaultStanleyConfig()).calculate(state, path);

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.cross_track_error_m, std::sqrt(53.0), 1e-6);
  EXPECT_NEAR(out.heading_error_rad, 0.0, 1e-6);
}

TEST(Stanley, SkipsRepeatedPointsButRejectsFullyDegeneratePath) {
  const Path2D repeated{{{0.0, 0.0}, {0.0, 0.0}, {10.0, 0.0},
                         {10.0, 0.0}, {30.0, 0.0}},
                        1.0, 0};
  const Path2D degenerate{{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}, 1.0, 0};
  Stanley stanley(defaultStanleyConfig());

  const auto repeated_out = stanley.calculate(stateAtOrigin(20.0), repeated);
  const auto degenerate_out = stanley.calculate(stateAtOrigin(20.0), degenerate);
  EXPECT_TRUE(repeated_out.valid);
  EXPECT_FALSE(degenerate_out.valid);
  expectFiniteCandidate(repeated_out);
  expectFiniteCandidate(degenerate_out);
}

TEST(Stanley, RejectsInsufficientForwardPathLength) {
  const auto out = Stanley(defaultStanleyConfig())
                       .calculate(stateAtOrigin(20.0), straightPath(4.9));

  EXPECT_FALSE(out.valid);
  expectFiniteCandidate(out);
}

TEST(Stanley, RejectsNonFiniteStatePathAndConfiguration) {
  VehicleState2D invalid_state = stateAtOrigin(20.0);
  invalid_state.yaw = std::numeric_limits<double>::quiet_NaN();
  Path2D invalid_path = straightPath(30.0);
  invalid_path.points[1].y = std::numeric_limits<double>::infinity();
  StanleyConfig invalid_config = defaultStanleyConfig();
  invalid_config.cross_track_gain = std::numeric_limits<double>::quiet_NaN();

  const auto state_out = Stanley(defaultStanleyConfig())
                             .calculate(invalid_state, straightPath(30.0));
  const auto path_out = Stanley(defaultStanleyConfig())
                            .calculate(stateAtOrigin(20.0), invalid_path);
  const auto config_out = Stanley(invalid_config)
                              .calculate(stateAtOrigin(20.0), straightPath(30.0));
  EXPECT_FALSE(state_out.valid);
  EXPECT_FALSE(path_out.valid);
  EXPECT_FALSE(config_out.valid);
  expectFiniteCandidate(state_out);
  expectFiniteCandidate(path_out);
  expectFiniteCandidate(config_out);
}

TEST(Stanley, NormalizesHeadingErrorAcrossPlusMinusPi) {
  const Path2D path{{{0.0, 0.0}, {-30.0, 0.0}}, 1.0, 0};
  const VehicleState2D state{-3.0 * std::cos(-kPi + 0.1),
                             -3.0 * std::sin(-kPi + 0.1),
                             -kPi + 0.1, 20.0};
  const auto out = Stanley(defaultStanleyConfig()).calculate(state, path);

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.cross_track_error_m, 0.0, 1e-6);
  EXPECT_NEAR(out.heading_error_rad, -0.1, 1e-6);
}

}  // namespace
}  // namespace hybrid_path_tracking
