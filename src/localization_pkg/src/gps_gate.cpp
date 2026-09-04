/*
gps_gate.cpp
- 역할: GPS 수신, 단절과 재수신 과정의 gate 상태 전이 및 EKF 동작
        결정을 구현한다.
- 주요 클래스: GpsGate
- ROS 인터페이스: 없음
*/
#include "localization_pkg/gps_gate.hpp"

#include <cmath>
#include <stdexcept>

namespace localization_pkg {

// 함수이름: GpsGate
// 기능: gate 임곗값을 검증하고 첫 GPS fix를 기다리는 초기 상태를 만든다.
// 인자: max_innovation_m, stable_return_radius_m, required_consecutive_fixes
// 반환값: 없음
GpsGate::GpsGate(double max_innovation_m, double stable_return_radius_m,
                 std::size_t required_consecutive_fixes)
    : max_innovation_m_(max_innovation_m),
      stable_return_radius_m_(stable_return_radius_m),
      required_consecutive_fixes_(required_consecutive_fixes),
      consecutive_fixes_(0),
      stable_return_point_{0.0, 0.0, 0.0},
      requires_filter_reset_(false),
      reset_pending_(false),
      state_(GpsGateState::WAITING_FOR_FIX) {
  if (!std::isfinite(max_innovation_m_) || max_innovation_m_ < 0.0) {
    throw std::invalid_argument(
        "max_innovation_m must be finite and nonnegative");
  }
  if (!std::isfinite(stable_return_radius_m_) ||
      stable_return_radius_m_ < 0.0) {
    throw std::invalid_argument(
        "stable_return_radius_m must be finite and nonnegative");
  }
  if (required_consecutive_fixes_ == 0) {
    throw std::invalid_argument("required_consecutive_fixes must be positive");
  }
}

// 함수이름: evaluate
// 기능: 상태별 규칙으로 GPS fix를 평가하고 수용, 거부 또는 EKF reset을
//       결정한다.
// 인자: point, have_prediction, predicted_x, predicted_y
// 반환값: 이번 fix에 적용할 GateDecision
GateDecision GpsGate::evaluate(const MapPoint& point, bool have_prediction,
                               double predicted_x, double predicted_y) {
  if (!isFinite(point)) {
    return GateDecision{GpsGateAction::REJECT};
  }

  switch (state_) {
    case GpsGateState::WAITING_FOR_FIX:
      state_ = GpsGateState::TRACKING;
      return GateDecision{GpsGateAction::ACCEPT};

    case GpsGateState::TRACKING:
      if (have_prediction && !isWithinInnovation(point, predicted_x, predicted_y)) {
        return GateDecision{GpsGateAction::REJECT};
      }
      return GateDecision{GpsGateAction::ACCEPT};

    case GpsGateState::DEGRADED:
      if (have_prediction && isWithinInnovation(point, predicted_x, predicted_y)) {
        state_ = GpsGateState::TRACKING;
        return GateDecision{GpsGateAction::ACCEPT};
      }
      beginRelocalization(point,
                          have_prediction &&
                              !isWithinInnovation(point, predicted_x, predicted_y));
      break;

    case GpsGateState::RELOCALIZING:
      if (isStableReturn(point)) {
        if (have_prediction && !isWithinInnovation(point, predicted_x,
                                                    predicted_y)) {
          requires_filter_reset_ = true;
        }
        ++consecutive_fixes_;
      } else {
        beginRelocalization(point,
                            have_prediction &&
                                !isWithinInnovation(point, predicted_x,
                                                    predicted_y));
      }
      break;
  }

  if (consecutive_fixes_ < required_consecutive_fixes_) {
    return GateDecision{GpsGateAction::REJECT};
  }
  if (requires_filter_reset_) {
    reset_pending_ = true;
    return GateDecision{GpsGateAction::RESET_FILTER};
  }
  state_ = GpsGateState::TRACKING;
  consecutive_fixes_ = 0;
  reset_pending_ = false;
  return GateDecision{GpsGateAction::ACCEPT};
}

// 함수이름: markTimeout
// 기능: GPS timeout을 반영하고 진행 중인 복귀 후보를 폐기한다.
// 인자: 없음
// 반환값: 없음
void GpsGate::markTimeout() {
  state_ = GpsGateState::DEGRADED;
  consecutive_fixes_ = 0;
  requires_filter_reset_ = false;
  reset_pending_ = false;
}

// 함수이름: confirmReset
// 기능: 진행 중인 reset의 성공 결과를 반영해 정상 추적으로 복귀한다.
// 인자: succeeded - SetPose 성공 여부
// 반환값: 없음
void GpsGate::confirmReset(bool succeeded) {
  if (succeeded && state_ == GpsGateState::RELOCALIZING && reset_pending_) {
    state_ = GpsGateState::TRACKING;
    consecutive_fixes_ = 0;
    requires_filter_reset_ = false;
    reset_pending_ = false;
  }
}

// 함수이름: stateName
// 기능: 현재 gate 상태를 외부 ROS 메시지에서 사용할 문자열로 변환한다.
// 인자: 없음
// 반환값: 현재 GPS gate 상태 문자열
std::string GpsGate::stateName() const {
  switch (state_) {
    case GpsGateState::WAITING_FOR_FIX:
      return "WAITING_FOR_FIX";
    case GpsGateState::TRACKING:
      return "TRACKING";
    case GpsGateState::DEGRADED:
      return "DEGRADED";
    case GpsGateState::RELOCALIZING:
      return "RELOCALIZING";
  }
  return "UNKNOWN";
}

// 함수이름: isFinite
// 기능: MapPoint의 모든 좌표가 유한한지 검사한다.
// 인자: point - 검사할 map 좌표
// 반환값: 모든 좌표가 유한하면 true, 아니면 false
bool GpsGate::isFinite(const MapPoint& point) const {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

// 함수이름: isWithinInnovation
// 기능: GPS와 예측 위치 사이의 평면 거리를 innovation 임곗값과 비교한다.
// 인자: point, predicted_x, predicted_y
// 반환값: innovation 임곗값 이내이면 true, 아니면 false
bool GpsGate::isWithinInnovation(const MapPoint& point, double predicted_x,
                                 double predicted_y) const {
  if (!std::isfinite(max_innovation_m_) || max_innovation_m_ < 0.0 ||
      !std::isfinite(predicted_x) || !std::isfinite(predicted_y)) {
    return false;
  }
  return std::hypot(point.x - predicted_x, point.y - predicted_y) <=
         max_innovation_m_;
}

// 함수이름: isStableReturn
// 기능: 복귀 GPS fix가 현재 기준점 주변에 안정적으로 모였는지 판정한다.
// 인자: point - 검사할 GPS map 좌표
// 반환값: 안정화 반경 이내이면 true, 아니면 false
bool GpsGate::isStableReturn(const MapPoint& point) const {
  return std::isfinite(stable_return_radius_m_) &&
         stable_return_radius_m_ >= 0.0 &&
         std::hypot(point.x - stable_return_point_.x,
                    point.y - stable_return_point_.y) <= stable_return_radius_m_;
}

// 함수이름: beginRelocalization
// 기능: 현재 fix를 복귀 기준점으로 저장하고 안정화 횟수와 reset 판단을
//       초기화한다.
// 인자: point, requires_filter_reset
// 반환값: 없음
void GpsGate::beginRelocalization(const MapPoint& point,
                                  bool requires_filter_reset) {
  state_ = GpsGateState::RELOCALIZING;
  stable_return_point_ = point;
  consecutive_fixes_ = 1;
  requires_filter_reset_ = requires_filter_reset;
  reset_pending_ = false;
}

}  // namespace localization_pkg

