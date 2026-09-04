/*
localization_health.hpp
- 역할: Localization 입력과 Global EKF 출력의 건강 상태 및 relay 허용
        여부를 판정한다.
- 주요 클래스: GlobalAnchorReadiness
- ROS 인터페이스: 없음
*/
#pragma once

#include <cstddef>
#include <string>

namespace localization_pkg {

enum class LocalizationState {
  // Supervisor가 매 평가 주기마다 계산하는 localization의 외부 상태다.
  // 상태 이름과 최종 odometry 발행 가능 여부는 HealthDecision으로 함께 결정된다.
  UNINITIALIZED,
  INITIALIZING,
  TRACKING,
  DEGRADED,
  RELOCALIZING,
  FAULT,
};

enum class CovarianceValidity {
  // covariance 행렬 검증 결과를 성공과 구체적인 실패 원인으로 구분한다.
  VALID,
  UNAVAILABLE,
  NONFINITE,
  NEGATIVE_DIAGONAL,
  INVALID_ARGUMENT,
};

enum class QuaternionValidity {
  // quaternion 검증 결과를 유한성, 영벡터 및 단위 norm 위반으로 구분한다.
  VALID,
  NONFINITE,
  ZERO,
  NOT_UNIT,
  INVALID_ARGUMENT,
};

struct HealthInput {
  // Supervisor가 수집한 각 입력 stream의 수신·형식·freshness 상태를 한데 모은다.
  // 이 구조체에는 센서 원본 값이 아니라 출력 허용 여부를 판단하는 상태만 담긴다.
  bool imu_received = false;
  bool imu_valid = false;
  bool imu_fresh = false;
  std::string imu_rejection_reason;

  bool twist_received = false;
  bool twist_valid = false;
  bool twist_fresh = false;
  std::string twist_rejection_reason;

  bool gps_state_received = false;
  bool gps_state_valid = false;
  bool gps_state_fresh = false;
  std::string gps_state;
  std::string gps_state_rejection_reason;

  bool global_odometry_received = false;
  bool global_odometry_valid = false;
  bool global_odometry_fresh = false;
  std::string global_odometry_rejection_reason;

  bool global_anchor_received = false;
  bool global_anchor_valid = false;
  bool global_anchor_fresh = false;
  bool global_anchor_confirmed = false;
  std::string global_anchor_rejection_reason;
};

struct HealthDecision {
  // evaluateHealth의 결과다. state는 외부 상태, publish_output은 relay 허용 여부,
  // rejection_reason은 diagnostics에 기록할 결정 원인을 나타낸다.
  LocalizationState state;
  bool publish_output;
  std::string rejection_reason;
};

// 함수이름: evaluateHealth
// 기능: 필수 센서, Global EKF, GPS heartbeat와 anchor 상태를 안전
//       우선순위로 평가한다.
// 인자: input - 각 stream의 수신, 유효성 및 freshness 상태
// 반환값: Localization 상태와 최종 odometry 발행 허용 여부
HealthDecision evaluateHealth(const HealthInput& input);
// 함수이름: localizationStateName
// 기능: LocalizationState를 ROS diagnostics에 사용할 고정 문자열로 변환한다.
// 인자: state - 변환할 LocalizationState
// 반환값: 상태를 나타내는 고정 문자열
const char* localizationStateName(LocalizationState state);
// 함수이름: globalPositionMatchesAnchor
// 기능: Global EKF XY 위치가 GPS anchor의 허용 오차 이내인지 확인한다.
// 인자: anchor_x, anchor_y, global_x, global_y, max_error_m
// 반환값: 두 위치의 평면 거리가 허용 오차 이내이면 true, 아니면 false
bool globalPositionMatchesAnchor(double anchor_x, double anchor_y,
                                 double global_x, double global_y,
                                 double max_error_m);

class GlobalAnchorReadiness {
  // 입력: 검증된 GPS map pose와 그 이후 생성된 Global EKF odometry.
  // 처리: timestamp, 수신 순서와 XY 오차를 비교해 전역 기준점 반영 여부를 확인한다.
 // 출력: 최초 시작 또는 재위치 추정 뒤 최종 odometry relay에 쓰이는
 //       confirmed 상태.
 // 상태: 확인 전에는 더 최신 GPS anchor로 기준을 갱신하고, 확인 후에는
 //       latch한다.
 public:
  // 함수이름: clear
  // 기능: 저장된 GPS anchor와 확인 상태를 초기화한다.
  // 인자: 없음
  // 반환값: 없음
  void clear();
  // 함수이름: updateAnchor
  // 기능: 확인 전의 최신 GPS anchor와 ROS·monotonic 시간을 저장한다.
  // 인자: stamp_sec, receipt_steady_sec, x, y
  // 반환값: anchor를 갱신했으면 true, 무시했으면 false
  bool updateAnchor(double stamp_sec, double receipt_steady_sec,
                    double x, double y);
  // 함수이름: tryConfirm
  // 기능: GPS 상태, 수신 순서와 위치 오차를 확인해 Global EKF anchor
  //       반영을 확정한다.
  // 인자: gps_tracking, odometry 시간, XY 위치와 max_error_m
  // 반환값: anchor 반영이 확인됐으면 true, 아니면 false
  bool tryConfirm(bool gps_tracking, double odometry_stamp_sec,
                  double odometry_receipt_steady_sec, double x, double y,
                  double max_error_m);
  // 함수이름: received
  // 기능: GPS anchor 메시지를 하나 이상 저장했는지 확인한다.
  // 인자: 없음
  // 반환값: anchor를 저장했으면 true, 아니면 false
  bool received() const { return received_; }
  // 함수이름: confirmed
  // 기능: Global EKF가 현재 GPS anchor에 맞춰졌는지 확인한다.
  // 인자: 없음
  // 반환값: anchor 반영이 확인됐으면 true, 아니면 false
  bool confirmed() const { return confirmed_; }
  // 함수이름: stampSec
  // 기능: 현재 anchor 메시지의 ROS timestamp를 조회한다.
  // 인자: 없음
  // 반환값: anchor ROS timestamp 초
  double stampSec() const { return stamp_sec_; }
  // 함수이름: receiptSteadySec
  // 기능: 현재 anchor가 callback에 도착한 monotonic 시간을 조회한다.
  // 인자: 없음
  // 반환값: anchor monotonic 수신 시간 초
  double receiptSteadySec() const { return receipt_steady_sec_; }

 private:
  bool received_ = false;
  bool confirmed_ = false;
  double stamp_sec_ = 0.0;
  double receipt_steady_sec_ = 0.0;
  double x_ = 0.0;
  double y_ = 0.0;
};

// 함수이름: validateCovarianceMatrix
// 기능: 정사각 covariance의 모든 값과 대각 성분을 검사한다.
// 인자: covariance, dimension, reject_unavailable_marker
// 반환값: 구체적인 CovarianceValidity 결과
CovarianceValidity validateCovarianceMatrix(const double* covariance,
                                            std::size_t dimension,
                                            bool reject_unavailable_marker);
// 함수이름: validateUnitQuaternion
// 기능: quaternion의 유한성, 최소 norm과 단위 norm 허용 오차를 검사한다.
// 인자: x, y, z, w, min_norm, unit_norm_tolerance
// 반환값: 구체적인 QuaternionValidity 결과
QuaternionValidity validateUnitQuaternion(double x, double y, double z,
                                          double w, double min_norm,
                                          double unit_norm_tolerance);

}  // namespace localization_pkg

