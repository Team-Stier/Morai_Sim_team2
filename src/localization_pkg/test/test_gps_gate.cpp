#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "localization_pkg/gps_gate.hpp"

namespace localization_pkg {
namespace {

constexpr double kMaxInnovationM = 10.0;
constexpr double kStableReturnRadiusM = 2.0;
constexpr int kRequiredConsecutiveFixes = 3;

// 모든 테스트에서 같은 innovation·복귀 임곗값을 사용하는 새 GPS gate를 만든다.
GpsGate makeGate() {
  return GpsGate(kMaxInnovationM, kStableReturnRadiusM,
                 kRequiredConsecutiveFixes);
}

// 아래 TEST들은 첫 fix, 정상 이상치 거부, GPS timeout, 안정 복귀,
// reset 성공·실패와 잘못된 입력에서 상태가 설계대로 바뀌는지 검증한다.
TEST(GpsGateTest, AcceptsInitialFixAndEntersTracking) {
  GpsGate gate = makeGate();
  const MapPoint first_fix{0.0, 0.0, 0.0};

  EXPECT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(first_fix, false, 0.0, 0.0).action);
  EXPECT_EQ("TRACKING", gate.stateName());
}

TEST(GpsGateTest, RejectsIsolatedTrackingOutlierWithoutLeavingTracking) {
  GpsGate gate = makeGate();
  ASSERT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{0.0, 0.0, 0.0}, false, 0.0, 0.0).action);

  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{11.0, 0.0, 0.0}, true, 0.0, 0.0).action);
  EXPECT_EQ("TRACKING", gate.stateName());
}

TEST(GpsGateTest, RequestsResetAfterConfiguredStableReturningFixes) {
  GpsGate gate = makeGate();
  ASSERT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{0.0, 0.0, 0.0}, false, 0.0, 0.0).action);
  gate.markTimeout();
  EXPECT_EQ("DEGRADED", gate.stateName());

  const MapPoint return_fix_1{20.0, 0.0, 0.0};
  const MapPoint return_fix_2{20.5, 0.0, 0.0};
  const MapPoint return_fix_3{19.5, 0.0, 0.0};
  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(return_fix_1, true, 0.0, 0.0).action);
  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(return_fix_2, true, 0.0, 0.0).action);
  EXPECT_EQ(GpsGateAction::RESET_FILTER,
            gate.evaluate(return_fix_3, true, 0.0, 0.0).action);
  EXPECT_EQ("RELOCALIZING", gate.stateName());
}

TEST(GpsGateTest, UnstableReturningFixResetsConsecutiveCount) {
  GpsGate gate = makeGate();
  ASSERT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{0.0, 0.0, 0.0}, false, 0.0, 0.0).action);
  gate.markTimeout();

  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{20.0, 0.0, 0.0}, true, 0.0, 0.0).action);
  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{24.5, 0.0, 0.0}, true, 0.0, 0.0).action);
  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{24.5, 0.5, 0.0}, true, 0.0, 0.0).action);
  EXPECT_EQ(GpsGateAction::RESET_FILTER,
            gate.evaluate(MapPoint{24.0, 0.0, 0.0}, true, 0.0, 0.0).action);
}

TEST(GpsGateTest, FailedResetStaysRelocalizing) {
  GpsGate gate = makeGate();
  ASSERT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{0.0, 0.0, 0.0}, false, 0.0, 0.0).action);
  gate.markTimeout();
  gate.evaluate(MapPoint{20.0, 0.0, 0.0}, true, 0.0, 0.0);
  gate.evaluate(MapPoint{20.5, 0.0, 0.0}, true, 0.0, 0.0);
  ASSERT_EQ(GpsGateAction::RESET_FILTER,
            gate.evaluate(MapPoint{19.5, 0.0, 0.0}, true, 0.0, 0.0).action);

  gate.confirmReset(false);
  EXPECT_EQ("RELOCALIZING", gate.stateName());
  EXPECT_EQ(GpsGateAction::RESET_FILTER,
            gate.evaluate(MapPoint{19.5, 0.0, 0.0}, true, 0.0, 0.0).action);
}

TEST(GpsGateTest, RequiresStableReturnBeforeAcceptingWithoutPrediction) {
  GpsGate gate = makeGate();
  ASSERT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{0.0, 0.0, 0.0}, false, 0.0, 0.0).action);
  gate.markTimeout();

  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{20.0, 0.0, 0.0}, false, 0.0, 0.0).action);
  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{20.5, 0.0, 0.0}, false, 0.0, 0.0).action);
  EXPECT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{19.5, 0.0, 0.0}, false, 0.0, 0.0).action);
  EXPECT_EQ("TRACKING", gate.stateName());
}

TEST(GpsGateTest, PromotesStableNoPredictionReturnWhenPredictionShowsOutlier) {
  GpsGate gate = makeGate();
  ASSERT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{0.0, 0.0, 0.0}, false, 0.0, 0.0).action);
  gate.markTimeout();

  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{20.0, 0.0, 0.0}, false, 0.0, 0.0).action);
  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{20.5, 0.0, 0.0}, true, 0.0, 0.0).action);
  EXPECT_EQ(GpsGateAction::RESET_FILTER,
            gate.evaluate(MapPoint{19.5, 0.0, 0.0}, true, 0.0, 0.0).action);
}

TEST(GpsGateTest, UsesCurrentInnovationWhenUnstableFixStartsNewSequence) {
  GpsGate gate = makeGate();
  ASSERT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{0.0, 0.0, 0.0}, false, 0.0, 0.0).action);
  gate.markTimeout();

  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{20.0, 0.0, 0.0}, true, 0.0, 0.0).action);
  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{1.0, 0.0, 0.0}, true, 0.0, 0.0).action);
  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{1.5, 0.0, 0.0}, true, 0.0, 0.0).action);
  EXPECT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{0.5, 0.0, 0.0}, true, 0.0, 0.0).action);
  EXPECT_EQ("TRACKING", gate.stateName());
}

TEST(GpsGateTest, RejectsNonFiniteMapPointWithoutChangingTrackingState) {
  GpsGate gate = makeGate();
  ASSERT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{0.0, 0.0, 0.0}, false, 0.0, 0.0).action);

  EXPECT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{std::numeric_limits<double>::quiet_NaN(),
                                   0.0, 0.0},
                          false, 0.0, 0.0)
                .action);
  EXPECT_EQ("TRACKING", gate.stateName());
}

TEST(GpsGateTest, RejectsInvalidConstructorThresholds) {
  EXPECT_THROW(GpsGate(-1.0, kStableReturnRadiusM, kRequiredConsecutiveFixes),
               std::invalid_argument);
  EXPECT_THROW(GpsGate(std::numeric_limits<double>::quiet_NaN(),
                       kStableReturnRadiusM, kRequiredConsecutiveFixes),
               std::invalid_argument);
  EXPECT_THROW(GpsGate(kMaxInnovationM, -1.0, kRequiredConsecutiveFixes),
               std::invalid_argument);
  EXPECT_THROW(GpsGate(kMaxInnovationM,
                       std::numeric_limits<double>::quiet_NaN(),
                       kRequiredConsecutiveFixes),
               std::invalid_argument);
}

TEST(GpsGateTest, RejectsZeroRequiredConsecutiveFixes) {
  EXPECT_THROW(GpsGate(kMaxInnovationM, kStableReturnRadiusM, 0),
               std::invalid_argument);
}

TEST(GpsGateTest, IgnoresSuccessfulResetConfirmationOutsideRelocalizing) {
  GpsGate gate = makeGate();
  ASSERT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{0.0, 0.0, 0.0}, false, 0.0, 0.0).action);

  gate.confirmReset(true);
  EXPECT_EQ("TRACKING", gate.stateName());
  gate.markTimeout();
  gate.confirmReset(true);
  EXPECT_EQ("DEGRADED", gate.stateName());
}

TEST(GpsGateTest, IgnoresResetConfirmationBeforeResetIsPending) {
  GpsGate gate = makeGate();
  ASSERT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{0.0, 0.0, 0.0}, false, 0.0, 0.0).action);
  gate.markTimeout();
  ASSERT_EQ(GpsGateAction::REJECT,
            gate.evaluate(MapPoint{20.0, 0.0, 0.0}, true, 0.0, 0.0).action);

  gate.confirmReset(true);
  EXPECT_EQ("RELOCALIZING", gate.stateName());
}

TEST(GpsGateTest, SuccessfulResetConfirmationReturnsToTracking) {
  GpsGate gate = makeGate();
  ASSERT_EQ(GpsGateAction::ACCEPT,
            gate.evaluate(MapPoint{0.0, 0.0, 0.0}, false, 0.0, 0.0).action);
  gate.markTimeout();
  gate.evaluate(MapPoint{20.0, 0.0, 0.0}, true, 0.0, 0.0);
  gate.evaluate(MapPoint{20.5, 0.0, 0.0}, true, 0.0, 0.0);
  ASSERT_EQ(GpsGateAction::RESET_FILTER,
            gate.evaluate(MapPoint{19.5, 0.0, 0.0}, true, 0.0, 0.0).action);

  gate.confirmReset(true);
  EXPECT_EQ("TRACKING", gate.stateName());
}

}  // namespace
}  // namespace localization_pkg

int main(int argc, char** argv) {
  // 이 파일의 GoogleTest 항목을 등록하고 전체 결과를 반환한다.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

