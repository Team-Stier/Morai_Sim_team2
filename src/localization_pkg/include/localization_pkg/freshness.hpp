/*
freshness.hpp
- 역할: ROS timestamp와 monotonic 수신 시간의 freshness를 판정한다.
- 주요 함수: timestampIsFresh, receiptIsFresh, resetSnapshotIsFresh
- ROS 인터페이스: 없음
*/
#pragma once

namespace localization_pkg {

// 함수이름: timestampIsFresh
// 기능: 메시지 timestamp가 ROS 시간 기준 허용된 과거·미래 범위 안인지
//       판정한다.
// 인자: stamp_sec, now_sec, max_past_age_sec, max_future_age_sec
// 반환값: timestamp가 허용 범위 안이면 true, 아니면 false
bool timestampIsFresh(double stamp_sec, double now_sec,
                      double max_past_age_sec,
                      double max_future_age_sec);

// 함수이름: receiptIsFresh
// 기능: monotonic 수신 시간이 현재 시각보다 앞서지 않고 timeout 안인지
//       판정한다.
// 인자: receipt_sec, now_sec, max_age_sec
// 반환값: 수신 시간이 허용 범위 안이면 true, 아니면 false
bool receiptIsFresh(double receipt_sec, double now_sec, double max_age_sec);

// 함수이름: resetSnapshotIsFresh
// 기능: SetPose 요청에 묶인 GPS와 odometry의 timestamp 및 수신 시간을
//       함께 검사한다.
// 인자: GPS/odometry timestamp, 수신 시간, 현재 시간과 허용 timeout
// 반환값: reset snapshot의 모든 시간이 유효하면 true, 아니면 false
bool resetSnapshotIsFresh(
    double gps_stamp_sec, double odometry_stamp_sec,
    double gps_receipt_steady_sec, double prediction_receipt_steady_sec,
    double ros_now_sec, double steady_now_sec, double gps_message_max_age_sec,
    double global_odometry_timeout_sec, double max_future_stamp_sec);

}  // namespace localization_pkg

