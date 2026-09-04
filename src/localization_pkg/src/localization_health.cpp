/*
localization_health.cpp
- 역할: Localization 상태, Global EKF anchor 반영과 입력 수치 검증을 구현한다.
- 주요 클래스: GlobalAnchorReadiness
- ROS 인터페이스: 없음
*/
#include "localization_pkg/localization_health.hpp"

#include <cmath>

namespace localization_pkg {
namespace {

// 함수이름: decision
// 기능: 상태, 출력 허용 여부와 원인을 하나의 HealthDecision으로 묶는다.
// 인자: state, publish_output, rejection_reason
// 반환값: 입력값으로 구성한 HealthDecision
HealthDecision decision(LocalizationState state, bool publish_output,
                        const std::string& rejection_reason) {
  return HealthDecision{state, publish_output, rejection_reason};
}

// 함수이름: rejectionOr
// 기능: callback의 구체적 거부 이유가 없으면 일관된 기본 이유를 선택한다.
// 인자: rejection_reason, fallback
// 반환값: 사용할 거부 이유 문자열
std::string rejectionOr(const std::string& rejection_reason,
                        const char* fallback) {
  return rejection_reason.empty() ? fallback : rejection_reason;
}

}  // namespace

// 함수이름: evaluateHealth
// 기능: invalid·stale 입력부터 검사하고 Localization 상태와 relay 허용
//       여부를 결정한다.
// 인자: input - 각 stream의 수신, 유효성 및 freshness 상태
// 반환값: Localization 상태와 최종 odometry 발행 허용 여부
HealthDecision evaluateHealth(const HealthInput& input) {
  if (input.imu_received && !input.imu_valid) {
    return decision(LocalizationState::FAULT, false,
                    rejectionOr(input.imu_rejection_reason, "invalid_imu"));
  }
  if (input.twist_received && !input.twist_valid) {
    return decision(
        LocalizationState::FAULT, false,
        rejectionOr(input.twist_rejection_reason, "invalid_vehicle_twist"));
  }
  if (input.imu_received && !input.imu_fresh) {
    return decision(LocalizationState::FAULT, false, "stale_imu");
  }
  if (input.twist_received && !input.twist_fresh) {
    return decision(LocalizationState::FAULT, false,
                    "stale_vehicle_twist");
  }
  if (!input.imu_received) {
    return decision(LocalizationState::UNINITIALIZED, false,
                    "waiting_for_imu");
  }
  if (!input.twist_received) {
    return decision(LocalizationState::UNINITIALIZED, false,
                    "waiting_for_vehicle_twist");
  }

  if (input.global_odometry_received && !input.global_odometry_valid) {
    return decision(
        LocalizationState::FAULT, false,
        rejectionOr(input.global_odometry_rejection_reason,
                    "invalid_global_odometry"));
  }
  if (input.global_odometry_received && !input.global_odometry_fresh) {
    return decision(LocalizationState::FAULT, false,
                    "stale_global_odometry");
  }
  if (!input.global_odometry_received) {
    return decision(LocalizationState::INITIALIZING, false,
                    "waiting_for_global_odometry");
  }

  if (!input.gps_state_received) {
    return decision(LocalizationState::INITIALIZING, false,
                    "waiting_for_gps_state");
  }
  if (!input.gps_state_valid) {
    return decision(
        LocalizationState::FAULT, false,
        rejectionOr(input.gps_state_rejection_reason,
                    "gps_state_not_whitelisted"));
  }
  if (!input.gps_state_fresh) {
    return decision(LocalizationState::FAULT, false, "stale_gps_state");
  }

  if (input.gps_state == "WAITING_FOR_FIX") {
    return decision(LocalizationState::INITIALIZING, false,
                    "waiting_for_gps_fix");
  }
  if (input.gps_state == "RELOCALIZING") {
    return decision(LocalizationState::RELOCALIZING, false,
                    "gps_relocalizing");
  }
  if (!input.global_anchor_confirmed) {
    if (input.global_anchor_received && !input.global_anchor_valid) {
      return decision(
          LocalizationState::FAULT, false,
          rejectionOr(input.global_anchor_rejection_reason,
                      "invalid_global_anchor"));
    }
    if (!input.global_anchor_received) {
      return decision(LocalizationState::INITIALIZING, false,
                      "waiting_for_global_anchor");
    }
    if (!input.global_anchor_fresh) {
      return decision(LocalizationState::INITIALIZING, false,
                      "waiting_for_fresh_global_anchor");
    }
    return decision(LocalizationState::INITIALIZING, false,
                    "waiting_for_global_anchor");
  }
  if (input.gps_state == "TRACKING") {
    return decision(LocalizationState::TRACKING, true, "none");
  }
  if (input.gps_state == "DEGRADED") {
    return decision(LocalizationState::DEGRADED, true, "gps_degraded");
  }
  return decision(LocalizationState::FAULT, false,
                  "gps_state_not_whitelisted");
}

// 함수이름: globalPositionMatchesAnchor
// 기능: Global EKF 위치가 GPS anchor의 허용 오차 이내인지 검사한다.
// 인자: anchor_x, anchor_y, global_x, global_y, max_error_m
// 반환값: 평면 거리가 허용 오차 이내이면 true, 아니면 false
bool globalPositionMatchesAnchor(double anchor_x, double anchor_y,
                                 double global_x, double global_y,
                                 double max_error_m) {
  return std::isfinite(anchor_x) && std::isfinite(anchor_y) &&
         std::isfinite(global_x) && std::isfinite(global_y) &&
         std::isfinite(max_error_m) && max_error_m >= 0.0 &&
         std::hypot(global_x - anchor_x, global_y - anchor_y) <= max_error_m;
}

// 함수이름: clear
// 기능: 기존 GPS anchor와 Global EKF 반영 확인 상태를 초기화한다.
// 인자: 없음
// 반환값: 없음
void GlobalAnchorReadiness::clear() {
  received_ = false;
  confirmed_ = false;
  stamp_sec_ = 0.0;
  receipt_steady_sec_ = 0.0;
  x_ = 0.0;
  y_ = 0.0;
}

// 함수이름: updateAnchor
// 기능: 확인 전의 최신 GPS anchor 위치와 수신 시간을 저장한다.
// 인자: stamp_sec, receipt_steady_sec, x, y
// 반환값: anchor를 갱신했으면 true, 무시했으면 false
bool GlobalAnchorReadiness::updateAnchor(double stamp_sec,
                                         double receipt_steady_sec,
                                         double x, double y) {
  if (confirmed_ || !std::isfinite(stamp_sec) || stamp_sec <= 0.0 ||
      !std::isfinite(receipt_steady_sec) || receipt_steady_sec <= 0.0 ||
      !std::isfinite(x) || !std::isfinite(y)) {
    return false;
  }
  received_ = true;
  stamp_sec_ = stamp_sec;
  receipt_steady_sec_ = receipt_steady_sec;
  x_ = x;
  y_ = y;
  return true;
}

// 함수이름: tryConfirm
// 기능: 수신 순서와 위치 오차를 검사해 Global EKF의 GPS anchor 반영을
//       확인한다.
// 인자: gps_tracking, odometry 시간, XY 위치와 max_error_m
// 반환값: anchor 반영이 확인됐으면 true, 아니면 false
bool GlobalAnchorReadiness::tryConfirm(
    bool gps_tracking, double odometry_stamp_sec,
    double odometry_receipt_steady_sec, double x, double y,
    double max_error_m) {
  if (confirmed_) {
    return true;
  }
  if (!received_ || !gps_tracking || !std::isfinite(odometry_stamp_sec) ||
      !std::isfinite(odometry_receipt_steady_sec) ||
      odometry_stamp_sec < stamp_sec_ ||
      odometry_receipt_steady_sec < receipt_steady_sec_ ||
      !globalPositionMatchesAnchor(x_, y_, x, y, max_error_m)) {
    return false;
  }
  confirmed_ = true;
  return true;
}

// 함수이름: localizationStateName
// 기능: LocalizationState를 diagnostics에서 사용하는 문자열로 변환한다.
// 인자: state - 변환할 LocalizationState
// 반환값: 상태를 나타내는 고정 문자열
const char* localizationStateName(LocalizationState state) {
  switch (state) {
    case LocalizationState::UNINITIALIZED:
      return "UNINITIALIZED";
    case LocalizationState::INITIALIZING:
      return "INITIALIZING";
    case LocalizationState::TRACKING:
      return "TRACKING";
    case LocalizationState::DEGRADED:
      return "DEGRADED";
    case LocalizationState::RELOCALIZING:
      return "RELOCALIZING";
    case LocalizationState::FAULT:
      return "FAULT";
  }
  return "FAULT";
}

// 함수이름: validateCovarianceMatrix
// 기능: covariance의 유한성, 미제공 marker와 대각 분산을 검사한다.
// 인자: covariance, dimension, reject_unavailable_marker
// 반환값: 구체적인 CovarianceValidity 결과
CovarianceValidity validateCovarianceMatrix(
    const double* covariance, std::size_t dimension,
    bool reject_unavailable_marker) {
  if (covariance == nullptr || dimension == 0 || dimension > 6) {
    return CovarianceValidity::INVALID_ARGUMENT;
  }
  const std::size_t value_count = dimension * dimension;
  for (std::size_t index = 0; index < value_count; ++index) {
    if (!std::isfinite(covariance[index])) {
      return CovarianceValidity::NONFINITE;
    }
  }
  if (reject_unavailable_marker && covariance[0] == -1.0) {
    return CovarianceValidity::UNAVAILABLE;
  }
  for (std::size_t row = 0; row < dimension; ++row) {
    if (covariance[row * dimension + row] < 0.0) {
      return CovarianceValidity::NEGATIVE_DIAGONAL;
    }
  }
  return CovarianceValidity::VALID;
}

// 함수이름: validateUnitQuaternion
// 기능: quaternion의 유한성, 최소 norm과 단위 norm 오차를 검사한다.
// 인자: x, y, z, w, min_norm, unit_norm_tolerance
// 반환값: 구체적인 QuaternionValidity 결과
QuaternionValidity validateUnitQuaternion(double x, double y, double z,
                                          double w, double min_norm,
                                          double unit_norm_tolerance) {
  if (!std::isfinite(min_norm) || min_norm <= 0.0 ||
      !std::isfinite(unit_norm_tolerance) || unit_norm_tolerance <= 0.0 ||
      unit_norm_tolerance >= 1.0) {
    return QuaternionValidity::INVALID_ARGUMENT;
  }
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(w)) {
    return QuaternionValidity::NONFINITE;
  }
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm)) {
    return QuaternionValidity::NONFINITE;
  }
  if (norm < min_norm) {
    return QuaternionValidity::ZERO;
  }
  if (std::abs(norm - 1.0) > unit_norm_tolerance) {
    return QuaternionValidity::NOT_UNIT;
  }
  return QuaternionValidity::VALID;
}

}  // namespace localization_pkg

