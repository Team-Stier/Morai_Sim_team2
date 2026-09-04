#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <string>

#include "localization_pkg/localization_health.hpp"

namespace localization_pkg {
namespace {

// TRACKING 출력을 허용하는 모든 필수 조건을 채운 기준 HealthInput을 만든다.
// 각 테스트는 이 기준에서 한 조건만 바꿔 health 결정의 우선순위를 확인한다.
HealthInput healthyInput() {
  HealthInput input;
  input.imu_received = true;
  input.imu_valid = true;
  input.imu_fresh = true;
  input.twist_received = true;
  input.twist_valid = true;
  input.twist_fresh = true;
  input.gps_state_received = true;
  input.gps_state_valid = true;
  input.gps_state_fresh = true;
  input.gps_state = "TRACKING";
  input.global_odometry_received = true;
  input.global_odometry_valid = true;
  input.global_odometry_fresh = true;
  input.global_anchor_received = true;
  input.global_anchor_valid = true;
  input.global_anchor_fresh = true;
  input.global_anchor_confirmed = true;
  return input;
}

// evaluateHealth 결과의 상태, relay 허용 여부와 거부 이유를 한 번에 비교한다.
void expectDecision(const HealthInput& input, LocalizationState expected_state,
                    bool expected_publish, const std::string& expected_reason) {
  const HealthDecision decision = evaluateHealth(input);
  EXPECT_EQ(expected_state, decision.state);
  EXPECT_EQ(expected_publish, decision.publish_output);
  EXPECT_EQ(expected_reason, decision.rejection_reason);
}

// 아래 TEST들은 센서·filter·GPS·anchor 상태 조합과 validation helper의
// 성공·실패 결과가 안전 우선순위대로 결정되는지 검증한다.
TEST(LocalizationHealthTest, MissingRequiredSensorsAreUninitialized) {
  HealthInput input;
  expectDecision(input, LocalizationState::UNINITIALIZED, false,
                 "waiting_for_imu");

  input.imu_received = true;
  input.imu_valid = true;
  input.imu_fresh = true;
  expectDecision(input, LocalizationState::UNINITIALIZED, false,
                 "waiting_for_vehicle_twist");
}

TEST(LocalizationHealthTest, HealthyMotionWithoutFilterIsInitializing) {
  HealthInput input = healthyInput();
  input.global_odometry_received = false;
  expectDecision(input, LocalizationState::INITIALIZING, false,
                 "waiting_for_global_odometry");
}

TEST(LocalizationHealthTest, MissingGpsStateIsInitializing) {
  HealthInput input = healthyInput();
  input.gps_state_received = false;
  expectDecision(input, LocalizationState::INITIALIZING, false,
                 "waiting_for_gps_state");
}

TEST(LocalizationHealthTest, WaitingForFirstGpsFixIsInitializing) {
  HealthInput input = healthyInput();
  input.gps_state = "WAITING_FOR_FIX";
  expectDecision(input, LocalizationState::INITIALIZING, false,
                 "waiting_for_gps_fix");
}

TEST(LocalizationHealthTest, StaleGpsStateHeartbeatIsFault) {
  HealthInput input = healthyInput();
  input.gps_state_fresh = false;
  expectDecision(input, LocalizationState::FAULT, false,
                 "stale_gps_state");
}

TEST(LocalizationHealthTest, TrackingBeforeGlobalAnchorIsInitializing) {
  HealthInput input = healthyInput();
  input.global_anchor_received = false;
  input.global_anchor_valid = false;
  input.global_anchor_fresh = false;
  input.global_anchor_confirmed = false;
  expectDecision(input, LocalizationState::INITIALIZING, false,
                 "waiting_for_global_anchor");
}

TEST(LocalizationHealthTest, InvalidGlobalAnchorBeforeConfirmationIsFault) {
  HealthInput input = healthyInput();
  input.global_anchor_valid = false;
  input.global_anchor_confirmed = false;
  input.global_anchor_rejection_reason = "global_anchor_frame_mismatch";
  expectDecision(input, LocalizationState::FAULT, false,
                 "global_anchor_frame_mismatch");
}

TEST(LocalizationHealthTest, StaleGlobalAnchorBeforeConfirmationWaitsForFreshAnchor) {
  HealthInput input = healthyInput();
  input.global_anchor_fresh = false;
  input.global_anchor_confirmed = false;
  expectDecision(input, LocalizationState::INITIALIZING, false,
                 "waiting_for_fresh_global_anchor");
}

TEST(LocalizationHealthTest, PositionMustMatchAnchorWithinConfiguredTolerance) {
  EXPECT_TRUE(globalPositionMatchesAnchor(10.0, -5.0, 11.2, -3.4, 2.0));
  EXPECT_FALSE(globalPositionMatchesAnchor(10.0, -5.0, 0.0, 0.0, 2.0));
  EXPECT_FALSE(globalPositionMatchesAnchor(
      10.0, -5.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 2.0));
  EXPECT_FALSE(globalPositionMatchesAnchor(10.0, -5.0, 10.0, -5.0, -1.0));
}

TEST(LocalizationHealthTest, AnchorConfirmationRequiresPostAnchorOdometry) {
  GlobalAnchorReadiness readiness;
  EXPECT_TRUE(readiness.updateAnchor(100.0, 200.0, 10.0, -5.0));
  EXPECT_FALSE(readiness.tryConfirm(
      true, 99.99, 200.1, 10.0, -5.0, 2.0));
  EXPECT_FALSE(readiness.tryConfirm(
      true, 100.0, 199.99, 10.0, -5.0, 2.0));
  EXPECT_FALSE(readiness.tryConfirm(
      true, 100.0, 200.1, 0.0, 0.0, 2.0));
  EXPECT_TRUE(readiness.tryConfirm(
      true, 100.01, 200.1, 10.5, -5.5, 2.0));
  EXPECT_TRUE(readiness.confirmed());
}

TEST(LocalizationHealthTest, RelocalizationClearRequiresFreshTrackingAnchor) {
  GlobalAnchorReadiness readiness;
  ASSERT_TRUE(readiness.updateAnchor(100.0, 200.0, 10.0, -5.0));
  ASSERT_TRUE(readiness.tryConfirm(
      true, 100.1, 200.1, 10.0, -5.0, 2.0));

  readiness.clear();
  EXPECT_FALSE(readiness.confirmed());
  EXPECT_FALSE(readiness.received());
  ASSERT_TRUE(readiness.updateAnchor(101.0, 201.0, 20.0, 0.0));
  EXPECT_FALSE(readiness.tryConfirm(
      false, 101.1, 201.1, 20.0, 0.0, 2.0));
  EXPECT_FALSE(readiness.confirmed());
  EXPECT_TRUE(readiness.tryConfirm(
      true, 101.1, 201.1, 20.0, 0.0, 2.0));
}

TEST(LocalizationHealthTest, LatestAnchorReplacesOlderAnchorUntilConfirmed) {
  GlobalAnchorReadiness readiness;
  ASSERT_TRUE(readiness.updateAnchor(100.0, 200.0, 10.0, -5.0));
  ASSERT_TRUE(readiness.updateAnchor(100.1, 200.1, 11.0, -5.0));
  EXPECT_FALSE(readiness.tryConfirm(
      true, 100.2, 200.2, 10.0, -5.0, 0.5));
  EXPECT_TRUE(readiness.tryConfirm(
      true, 100.2, 200.2, 11.0, -5.0, 0.5));
}

TEST(LocalizationHealthTest, TrackingPublishesOutput) {
  const HealthInput input = healthyInput();
  expectDecision(input, LocalizationState::TRACKING, true, "none");
  EXPECT_STREQ("TRACKING",
               localizationStateName(LocalizationState::TRACKING));
}

TEST(LocalizationHealthTest, GpsBlackoutIsDegradedAndStillPublishes) {
  HealthInput input = healthyInput();
  input.gps_state = "DEGRADED";
  expectDecision(input, LocalizationState::DEGRADED, true,
                 "gps_degraded");
}

TEST(LocalizationHealthTest, ReacquisitionQuarantinesOutput) {
  HealthInput input = healthyInput();
  input.gps_state = "RELOCALIZING";
  input.global_anchor_confirmed = false;
  expectDecision(input, LocalizationState::RELOCALIZING, false,
                 "gps_relocalizing");
}

TEST(LocalizationHealthTest, DegradedBeforeAnchorDoesNotPublish) {
  HealthInput input = healthyInput();
  input.gps_state = "DEGRADED";
  input.global_anchor_received = false;
  input.global_anchor_valid = false;
  input.global_anchor_fresh = false;
  input.global_anchor_confirmed = false;
  expectDecision(input, LocalizationState::INITIALIZING, false,
                 "waiting_for_global_anchor");
}

TEST(LocalizationHealthTest, InvalidOrStaleImuIsFault) {
  HealthInput input = healthyInput();
  input.imu_valid = false;
  input.imu_rejection_reason = "imu_quaternion_not_unit";
  expectDecision(input, LocalizationState::FAULT, false,
                 "imu_quaternion_not_unit");

  input = healthyInput();
  input.imu_fresh = false;
  expectDecision(input, LocalizationState::FAULT, false, "stale_imu");
}

TEST(LocalizationHealthTest, InvalidOrStaleVehicleTwistIsFault) {
  HealthInput input = healthyInput();
  input.twist_valid = false;
  input.twist_rejection_reason = "vehicle_twist_nonfinite";
  expectDecision(input, LocalizationState::FAULT, false,
                 "vehicle_twist_nonfinite");

  input = healthyInput();
  input.twist_fresh = false;
  expectDecision(input, LocalizationState::FAULT, false,
                 "stale_vehicle_twist");
}

TEST(LocalizationHealthTest, WrongGlobalOdometryFramesAreFault) {
  HealthInput input = healthyInput();
  input.global_odometry_valid = false;
  input.global_odometry_rejection_reason =
      "global_odometry_frame_mismatch";
  expectDecision(input, LocalizationState::FAULT, false,
                 "global_odometry_frame_mismatch");
}

TEST(LocalizationHealthTest, NonfiniteGlobalOdometryPoseIsFault) {
  HealthInput input = healthyInput();
  input.global_odometry_valid = false;
  input.global_odometry_rejection_reason =
      "global_odometry_nonfinite_pose";
  expectDecision(input, LocalizationState::FAULT, false,
                 "global_odometry_nonfinite_pose");
}

TEST(LocalizationHealthTest, ZeroGlobalOdometryQuaternionIsFault) {
  HealthInput input = healthyInput();
  input.global_odometry_valid = false;
  input.global_odometry_rejection_reason =
      "global_odometry_zero_quaternion";
  expectDecision(input, LocalizationState::FAULT, false,
                 "global_odometry_zero_quaternion");
}

TEST(LocalizationHealthTest, StaleGlobalOdometryIsFault) {
  HealthInput input = healthyInput();
  input.global_odometry_fresh = false;
  expectDecision(input, LocalizationState::FAULT, false,
                 "stale_global_odometry");
}

TEST(LocalizationHealthTest, InvalidGpsStateIsFault) {
  HealthInput input = healthyInput();
  input.gps_state_valid = false;
  input.gps_state_rejection_reason = "gps_state_not_whitelisted";
  expectDecision(input, LocalizationState::FAULT, false,
                 "gps_state_not_whitelisted");

  input = healthyInput();
  input.gps_state = "UNKNOWN";
  expectDecision(input, LocalizationState::FAULT, false,
                 "gps_state_not_whitelisted");
}

TEST(LocalizationHealthTest, RequiredSensorFaultPrecedesFilterAndGpsState) {
  HealthInput input = healthyInput();
  input.imu_fresh = false;
  input.global_odometry_valid = false;
  input.global_odometry_rejection_reason =
      "global_odometry_nonfinite_pose";
  input.gps_state = "DEGRADED";
  expectDecision(input, LocalizationState::FAULT, false, "stale_imu");
}

TEST(LocalizationHealthTest, FilterFaultPrecedesGpsDegradation) {
  HealthInput input = healthyInput();
  input.global_odometry_fresh = false;
  input.gps_state = "DEGRADED";
  expectDecision(input, LocalizationState::FAULT, false,
                 "stale_global_odometry");
}

TEST(LocalizationHealthTest, ValidatesEveryEntryAndDiagonalOf3x3Covariance) {
  std::array<double, 9> covariance{{0.1, 0.01, 0.02,
                                    0.01, 0.2, 0.03,
                                    0.02, 0.03, 0.3}};
  EXPECT_EQ(CovarianceValidity::VALID,
            validateCovarianceMatrix(covariance.data(), 3, true));

  covariance[5] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(CovarianceValidity::NONFINITE,
            validateCovarianceMatrix(covariance.data(), 3, true));
  covariance[5] = 0.03;
  covariance[4] = -0.1;
  EXPECT_EQ(CovarianceValidity::NEGATIVE_DIAGONAL,
            validateCovarianceMatrix(covariance.data(), 3, true));
  covariance[4] = 0.2;
  covariance[0] = -1.0;
  EXPECT_EQ(CovarianceValidity::UNAVAILABLE,
            validateCovarianceMatrix(covariance.data(), 3, true));
}

TEST(LocalizationHealthTest, ValidatesCrossTermsAndDiagonalsOf6x6Covariance) {
  std::array<double, 36> covariance{};
  covariance[0] = 0.1;
  covariance[7] = 0.2;
  covariance[14] = 0.3;
  covariance[21] = 0.4;
  covariance[28] = 0.5;
  covariance[35] = 0.6;
  covariance[1] = 0.01;
  covariance[6] = 0.01;
  EXPECT_EQ(CovarianceValidity::VALID,
            validateCovarianceMatrix(covariance.data(), 6, false));

  covariance[13] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(CovarianceValidity::NONFINITE,
            validateCovarianceMatrix(covariance.data(), 6, false));
  covariance[13] = 0.0;
  covariance[35] = -0.1;
  EXPECT_EQ(CovarianceValidity::NEGATIVE_DIAGONAL,
            validateCovarianceMatrix(covariance.data(), 6, false));
}

TEST(LocalizationHealthTest, ValidatesFiniteNonzeroUnitQuaternion) {
  EXPECT_EQ(QuaternionValidity::VALID,
            validateUnitQuaternion(0.0, 0.0, 0.0, 1.0, 1.0e-12,
                                   1.0e-3));
  EXPECT_EQ(QuaternionValidity::NONFINITE,
            validateUnitQuaternion(
                0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 1.0,
                1.0e-12, 1.0e-3));
  EXPECT_EQ(QuaternionValidity::ZERO,
            validateUnitQuaternion(0.0, 0.0, 0.0, 0.0, 1.0e-12,
                                   1.0e-3));
  EXPECT_EQ(QuaternionValidity::NOT_UNIT,
            validateUnitQuaternion(0.0, 0.0, 0.0, 2.0, 1.0e-12,
                                   1.0e-3));
}

}  // namespace
}  // namespace localization_pkg

int main(int argc, char** argv) {
  // 이 파일의 GoogleTest 항목을 등록하고 전체 결과를 반환한다.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

