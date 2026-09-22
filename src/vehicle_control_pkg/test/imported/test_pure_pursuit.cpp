#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "hybrid_path_tracking/pure_pursuit.hpp"

namespace hybrid_path_tracking {
namespace {

constexpr double kPi = 3.14159265358979323846;

VehicleState2D stateAtOrigin(double speed_kph) {
  return {0.0, 0.0, 0.0, speed_kph};
}

Path2D straightPath(double length_m) {
  Path2D path{{}, 1.0, 0};
  for (double x = 0.0; x <= length_m; x += 0.5) {
    path.points.push_back({x, 0.0});
  }
  return path;
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

Path2D rightArcPath() {
  Path2D path = leftArcPath();
  for (auto& point : path.points) {
    point.y = -point.y;
  }
  return path;
}

Path2D shortPath() {
  return Path2D{{{0.0, 0.0}, {1.0, 0.0}}, 1.0, 0};
}

Path2D rayPath(double heading_rad, double length_m) {
  return Path2D{{{0.0, 0.0},
                 {length_m * std::cos(heading_rad),
                  length_m * std::sin(heading_rad)}},
                1.0, 0};
}

PurePursuitConfig defaultPurePursuitConfig() { return PurePursuitConfig{}; }

void expectFiniteCommand(const ControllerCandidate& candidate) {
  EXPECT_TRUE(std::isfinite(candidate.steering_angle_rad));
  EXPECT_TRUE(std::isfinite(candidate.cross_track_error_m));
  EXPECT_TRUE(std::isfinite(candidate.heading_error_rad));
  EXPECT_TRUE(std::isfinite(candidate.path_curvature));
}

TEST(PurePursuit, StraightPathProducesZeroSteer) {
  PurePursuit pp({3.0, 40.0 * kPi / 180.0, 3.0, 15.0, 0.15, 5.0});
  const auto out = pp.calculate(stateAtOrigin(36.0), straightPath(30.0));

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.steering_angle_rad, 0.0, 1e-6);
  EXPECT_NEAR(out.cross_track_error_m, 0.0, 1e-6);
  EXPECT_NEAR(out.heading_error_rad, 0.0, 1e-6);
  expectFiniteCommand(out);
}

TEST(PurePursuit, LeftPathProducesPositiveSteer) {
  PurePursuit pp(defaultPurePursuitConfig());
  const auto out = pp.calculate(stateAtOrigin(20.0), leftArcPath());

  EXPECT_TRUE(out.valid);
  EXPECT_GT(out.steering_angle_rad, 0.0);
  EXPECT_GT(out.cross_track_error_m, -1e-6);
  EXPECT_GT(out.heading_error_rad, 0.0);
  expectFiniteCommand(out);
}

TEST(PurePursuit, RightPathProducesNegativeSteer) {
  PurePursuit pp(defaultPurePursuitConfig());
  const auto out = pp.calculate(stateAtOrigin(20.0), rightArcPath());

  EXPECT_TRUE(out.valid);
  EXPECT_LT(out.steering_angle_rad, 0.0);
  EXPECT_LT(out.heading_error_rad, 0.0);
  expectFiniteCommand(out);
}

TEST(PurePursuit, CrossTrackErrorIsPositiveWhenPathIsLeftOfVehicle) {
  VehicleState2D state = stateAtOrigin(0.0);
  state.y = -1.0;
  const auto out = PurePursuit(defaultPurePursuitConfig())
                       .calculate(state, straightPath(30.0));

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.cross_track_error_m, 1.0, 1e-6);
  EXPECT_NEAR(out.heading_error_rad, 0.0, 1e-6);
}

TEST(PurePursuit, ClampsSteeringToRoadWheelLimit) {
  PurePursuit pp(defaultPurePursuitConfig());
  const auto out = pp.calculate(stateAtOrigin(0.0), rayPath(kPi / 3.0, 10.0));

  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(out.saturated);
  EXPECT_NEAR(out.steering_angle_rad, 40.0 * kPi / 180.0, 1e-6);
  expectFiniteCommand(out);
}

TEST(PurePursuit, UsesMinimumLookaheadAtLowSpeed) {
  PurePursuit pp(defaultPurePursuitConfig());
  const auto out = pp.calculate(stateAtOrigin(0.0), rayPath(kPi / 8.0, 30.0));

  // atan2(2 * 3 * sin(pi/8), 3) = atan2(sqrt(2 - sqrt(2)), 1).
  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.steering_angle_rad,
              std::atan2(std::sqrt(2.0 - std::sqrt(2.0)), 1.0), 1e-6);
  EXPECT_FALSE(out.saturated);
}

TEST(PurePursuit, UsesMaximumLookaheadAtHighSpeed) {
  PurePursuit pp(defaultPurePursuitConfig());
  const auto out = pp.calculate(stateAtOrigin(360.0), rayPath(kPi / 4.0, 30.0));

  // atan2(2 * 3 * sin(pi/4), 15) = atan2(sqrt(2), 5).
  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.steering_angle_rad, std::atan2(std::sqrt(2.0), 5.0),
              1e-6);
  EXPECT_FALSE(out.saturated);
}

TEST(PurePursuit, WorldPathAlignedWithVehicleYawProducesZeroSteer) {
  VehicleState2D state{10.0, -5.0, kPi / 2.0, 0.0};
  Path2D path{{{10.0, -5.0}, {10.0, 25.0}}, 1.0, 0};
  const auto out = PurePursuit(defaultPurePursuitConfig()).calculate(state, path);

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.steering_angle_rad, 0.0, 1e-6);
  EXPECT_NEAR(out.cross_track_error_m, 0.0, 1e-6);
  EXPECT_NEAR(out.heading_error_rad, 0.0, 1e-6);
}

TEST(PurePursuit, SelectsFirstForwardCircleIntersectionInPathOrder) {
  constexpr double kFirstAlphaRad = kPi / 8.0;
  Path2D path{{{0.0, 0.0},
               {10.0 * std::cos(kFirstAlphaRad),
                10.0 * std::sin(kFirstAlphaRad)},
               {1.0, -2.0},
               {10.0, -10.0}},
              1.0, 0};
  const auto out = PurePursuit(defaultPurePursuitConfig())
                       .calculate(stateAtOrigin(0.0), path);

  // The first segment intersects the 3 m circle at alpha = pi/8. Later
  // intersections are right of the vehicle and must not replace that target.
  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.steering_angle_rad,
              std::atan2(std::sqrt(2.0 - std::sqrt(2.0)), 1.0), 1e-6);
  EXPECT_GT(out.steering_angle_rad, 0.0);
  EXPECT_FALSE(out.saturated);
}

TEST(PurePursuit, RejectsShortForwardPath) {
  PurePursuit pp(defaultPurePursuitConfig());
  const auto out = pp.calculate(stateAtOrigin(20.0), shortPath());

  EXPECT_FALSE(out.valid);
  expectFiniteCommand(out);
}

TEST(PurePursuit, RejectsPathWithoutPointAtLookahead) {
  PurePursuitConfig config = defaultPurePursuitConfig();
  config.min_forward_path_m = 1.0;
  PurePursuit pp(config);
  const auto out = pp.calculate(stateAtOrigin(0.0), straightPath(2.9));

  EXPECT_FALSE(out.valid);
  expectFiniteCommand(out);
}

TEST(PurePursuit, RejectsLongLoopWithoutRequiredForwardReach) {
  PurePursuitConfig config = defaultPurePursuitConfig();
  config.min_forward_path_m = 5.0;
  Path2D looping_path{{{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0},
                       {0.0, 4.0}, {0.0, 0.0}},
                      1.0, 0};
  const auto out = PurePursuit(config).calculate(stateAtOrigin(0.0), looping_path);

  // The loop has 16 m of arc length but reaches only x = 4 m forward.
  EXPECT_FALSE(out.valid);
  expectFiniteCommand(out);
}

TEST(PurePursuit, SkipsRepeatedPointsWhenSelectingTarget) {
  Path2D path{{{0.0, 0.0}, {0.0, 0.0}, {2.0, 0.0}, {2.0, 0.0},
               {10.0, 0.0}},
              1.0, 0};
  PurePursuit pp(defaultPurePursuitConfig());
  const auto out = pp.calculate(stateAtOrigin(0.0), path);

  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.steering_angle_rad, 0.0, 1e-6);
  expectFiniteCommand(out);
}

TEST(PurePursuit, RejectsDegenerateRepeatedPointPath) {
  Path2D path{{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}, 1.0, 0};
  PurePursuit pp(defaultPurePursuitConfig());
  const auto out = pp.calculate(stateAtOrigin(0.0), path);

  EXPECT_FALSE(out.valid);
  expectFiniteCommand(out);
}

TEST(PurePursuit, RejectsNonFiniteStateWithFiniteCommand) {
  VehicleState2D invalid_state = stateAtOrigin(20.0);
  invalid_state.yaw = std::numeric_limits<double>::quiet_NaN();
  const auto out = PurePursuit(defaultPurePursuitConfig())
                       .calculate(invalid_state, straightPath(30.0));

  EXPECT_FALSE(out.valid);
  expectFiniteCommand(out);
}

TEST(PurePursuit, RejectsNonFinitePathWithFiniteCommand) {
  Path2D invalid_path = straightPath(30.0);
  invalid_path.points[3].y = std::numeric_limits<double>::infinity();
  const auto out = PurePursuit(defaultPurePursuitConfig())
                       .calculate(stateAtOrigin(20.0), invalid_path);

  EXPECT_FALSE(out.valid);
  expectFiniteCommand(out);
}

TEST(PurePursuit, RejectsNonFinitePathConfidenceWithFiniteCommand) {
  Path2D invalid_path = straightPath(30.0);
  invalid_path.confidence = std::numeric_limits<double>::quiet_NaN();
  const auto out = PurePursuit(defaultPurePursuitConfig())
                       .calculate(stateAtOrigin(20.0), invalid_path);

  EXPECT_FALSE(out.valid);
  expectFiniteCommand(out);
}

TEST(PurePursuit, RejectsNonFiniteConfigWithFiniteCommand) {
  PurePursuitConfig invalid_config = defaultPurePursuitConfig();
  invalid_config.wheelbase_m = std::numeric_limits<double>::quiet_NaN();
  const auto out = PurePursuit(invalid_config)
                       .calculate(stateAtOrigin(20.0), straightPath(30.0));

  EXPECT_FALSE(out.valid);
  expectFiniteCommand(out);
}

}  // namespace
}  // namespace hybrid_path_tracking
