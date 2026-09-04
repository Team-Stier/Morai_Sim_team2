/*
freshness.cpp
- 역할: Localization에서 사용하는 ROS timestamp와 monotonic 수신 시간
        검사를 구현한다.
- 주요 함수: timestampIsFresh, receiptIsFresh, resetSnapshotIsFresh
- ROS 인터페이스: 없음
*/
#include "localization_pkg/freshness.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace localization_pkg {
namespace {

// 함수이름: comparisonTolerance
// 기능: 부동소수점 경계 비교에 사용할 반올림 허용폭을 계산한다.
// 인자: first, second, threshold
// 반환값: 입력 크기에 비례한 비교 허용폭
double comparisonTolerance(double first, double second, double threshold) {
  const double scale = std::max(
      {1.0, std::abs(first), std::abs(second), std::abs(threshold)});
  return 4.0 * std::numeric_limits<double>::epsilon() * scale;
}

}  // namespace

// 함수이름: timestampIsFresh
// 기능: header timestamp의 과거 age와 미래 오차를 설정 한계와 비교한다.
// 인자: stamp_sec, now_sec, max_past_age_sec, max_future_age_sec
// 반환값: timestamp가 허용 범위 안이면 true, 아니면 false
bool timestampIsFresh(double stamp_sec, double now_sec,
                      double max_past_age_sec,
                      double max_future_age_sec) {
  if (!std::isfinite(stamp_sec) || !std::isfinite(now_sec) ||
      !std::isfinite(max_past_age_sec) ||
      !std::isfinite(max_future_age_sec) || stamp_sec <= 0.0 ||
      now_sec <= 0.0 || max_past_age_sec <= 0.0 ||
      max_future_age_sec < 0.0) {
    return false;
  }
  const double age_sec = now_sec - stamp_sec;
  const double tolerance = comparisonTolerance(
      stamp_sec, now_sec, std::max(max_past_age_sec, max_future_age_sec));
  return std::isfinite(age_sec) &&
         age_sec >= -max_future_age_sec - tolerance &&
         age_sec <= max_past_age_sec + tolerance;
}

// 함수이름: receiptIsFresh
// 기능: monotonic 수신 age를 timeout과 비교한다.
// 인자: receipt_sec, now_sec, max_age_sec
// 반환값: 수신 시간이 허용 범위 안이면 true, 아니면 false
bool receiptIsFresh(double receipt_sec, double now_sec, double max_age_sec) {
  if (!std::isfinite(receipt_sec) || !std::isfinite(now_sec) ||
      !std::isfinite(max_age_sec) || receipt_sec <= 0.0 || now_sec <= 0.0 ||
      max_age_sec <= 0.0) {
    return false;
  }
  const double age_sec = now_sec - receipt_sec;
  const double tolerance = comparisonTolerance(receipt_sec, now_sec,
                                                max_age_sec);
  return std::isfinite(age_sec) && age_sec >= -tolerance &&
         age_sec <= max_age_sec + tolerance;
}

// 함수이름: resetSnapshotIsFresh
// 기능: 비동기 SetPose 전후에 GPS와 odometry snapshot이 여전히
//       유효한지 확인한다.
// 인자: GPS/odometry timestamp, 수신 시간, 현재 시간과 허용 timeout
// 반환값: snapshot의 모든 시간이 유효하면 true, 아니면 false
bool resetSnapshotIsFresh(
    double gps_stamp_sec, double odometry_stamp_sec,
    double gps_receipt_steady_sec, double prediction_receipt_steady_sec,
    double ros_now_sec, double steady_now_sec, double gps_message_max_age_sec,
    double global_odometry_timeout_sec, double max_future_stamp_sec) {
  return timestampIsFresh(gps_stamp_sec, ros_now_sec,
                          gps_message_max_age_sec, max_future_stamp_sec) &&
         timestampIsFresh(odometry_stamp_sec, ros_now_sec,
                          global_odometry_timeout_sec,
                          max_future_stamp_sec) &&
         receiptIsFresh(gps_receipt_steady_sec, steady_now_sec,
                        gps_message_max_age_sec) &&
         receiptIsFresh(prediction_receipt_steady_sec, steady_now_sec,
                        global_odometry_timeout_sec);
}

}  // namespace localization_pkg

