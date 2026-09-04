#include <gtest/gtest.h>

#include <limits>

#include "localization_pkg/freshness.hpp"

namespace localization_pkg {
namespace {

constexpr double kGpsMaxAgeSec = 0.75;
constexpr double kOdometryTimeoutSec = 0.5;
constexpr double kMaxFutureStampSec = 0.2;

// 아래 TEST들은 ROS timestamp와 monotonic 수신 시간의 과거·미래 경계,
// 비동기 SetPose snapshot 만료 및 잘못된 임곗값 거부를 각각 검증한다.
TEST(FreshnessTest, AcceptsPastAndFutureTimestampBounds) {
  EXPECT_TRUE(timestampIsFresh(99.25, 100.0, kGpsMaxAgeSec,
                               kMaxFutureStampSec));
  EXPECT_TRUE(timestampIsFresh(100.2, 100.0, kGpsMaxAgeSec,
                               kMaxFutureStampSec));
}

TEST(FreshnessTest, RejectsTimestampOutsidePastAndFutureBounds) {
  EXPECT_FALSE(timestampIsFresh(99.249, 100.0, kGpsMaxAgeSec,
                                kMaxFutureStampSec));
  EXPECT_FALSE(timestampIsFresh(100.201, 100.0, kGpsMaxAgeSec,
                                kMaxFutureStampSec));
}

TEST(FreshnessTest, RejectsZeroOrNonfiniteTimeInputs) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(timestampIsFresh(99.5, 0.0, kGpsMaxAgeSec,
                                kMaxFutureStampSec));
  EXPECT_FALSE(timestampIsFresh(0.0, 100.0, kGpsMaxAgeSec,
                                kMaxFutureStampSec));
  EXPECT_FALSE(timestampIsFresh(nan, 100.0, kGpsMaxAgeSec,
                                kMaxFutureStampSec));
  EXPECT_FALSE(timestampIsFresh(99.5, infinity, kGpsMaxAgeSec,
                                kMaxFutureStampSec));
}

TEST(FreshnessTest, ValidatesMonotonicReceiptAge) {
  EXPECT_TRUE(receiptIsFresh(199.5, 200.0, kOdometryTimeoutSec));
  EXPECT_FALSE(receiptIsFresh(199.499, 200.0, kOdometryTimeoutSec));
  EXPECT_FALSE(receiptIsFresh(200.001, 200.0, kOdometryTimeoutSec));
}

TEST(FreshnessTest, SnapshotCanExpireBetweenAdmissionAndDispatch) {
  EXPECT_TRUE(resetSnapshotIsFresh(
      99.9, 99.75, 199.9, 199.75, 100.0, 200.0, kGpsMaxAgeSec,
      kOdometryTimeoutSec, kMaxFutureStampSec));
  EXPECT_FALSE(resetSnapshotIsFresh(
      99.9, 99.75, 199.9, 199.75, 100.51, 200.51, kGpsMaxAgeSec,
      kOdometryTimeoutSec, kMaxFutureStampSec));
}

TEST(FreshnessTest, SnapshotRejectsStaleGpsReceipt) {
  EXPECT_FALSE(resetSnapshotIsFresh(
      99.9, 99.75, 199.0, 199.75, 100.0, 200.0, kGpsMaxAgeSec,
      kOdometryTimeoutSec, kMaxFutureStampSec));
}

TEST(FreshnessTest, SnapshotRejectsGpsAndOdometryPastOrFuture) {
  EXPECT_FALSE(resetSnapshotIsFresh(
      99.249, 99.75, 199.9, 199.75, 100.0, 200.0, kGpsMaxAgeSec,
      kOdometryTimeoutSec, kMaxFutureStampSec));
  EXPECT_FALSE(resetSnapshotIsFresh(
      100.201, 99.75, 199.9, 199.75, 100.0, 200.0, kGpsMaxAgeSec,
      kOdometryTimeoutSec, kMaxFutureStampSec));
  EXPECT_FALSE(resetSnapshotIsFresh(
      99.9, 99.499, 199.9, 199.75, 100.0, 200.0, kGpsMaxAgeSec,
      kOdometryTimeoutSec, kMaxFutureStampSec));
  EXPECT_FALSE(resetSnapshotIsFresh(
      99.9, 100.201, 199.9, 199.75, 100.0, 200.0, kGpsMaxAgeSec,
      kOdometryTimeoutSec, kMaxFutureStampSec));
}

TEST(FreshnessTest, RejectsInvalidNonfiniteOrNegativeThresholds) {
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(timestampIsFresh(99.5, 100.0, 0.0,
                                kMaxFutureStampSec));
  EXPECT_FALSE(timestampIsFresh(99.5, 100.0, infinity,
                                kMaxFutureStampSec));
  EXPECT_FALSE(timestampIsFresh(99.5, 100.0, kGpsMaxAgeSec, -0.1));
}

}  // namespace
}  // namespace localization_pkg

int main(int argc, char** argv) {
  // 이 파일의 GoogleTest 항목을 등록하고 전체 결과를 반환한다.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

