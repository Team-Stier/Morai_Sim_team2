/*
gps_gate.hpp
- 역할: GPS 수신 상태와 재수신 안정성을 관리하고 EKF 처리 동작을
        결정한다.
- 주요 클래스: GpsGate
- ROS 인터페이스: 없음
*/
#pragma once

#include <cstddef>
#include <string>

#include "localization_pkg/gps_projection.hpp"

namespace localization_pkg {

enum class GpsGateState {
  // GPS gate의 내부 상태다. 첫 fix 대기, 정상 추적, GPS 단절, 복귀 검증을 구분한다.
  WAITING_FOR_FIX,
  TRACKING,
  DEGRADED,
  RELOCALIZING,
};

enum class GpsGateAction {
  // 현재 GPS fix를 EKF에 전달할지, 버릴지, filter reset에 사용할지 나타낸다.
  ACCEPT,
  REJECT,
  RESET_FILTER,
};

struct GateDecision {
  // GpsGate::evaluate가 이번 GPS fix에 대해 내린 단일 처리 결과다.
  GpsGateAction action;
};

class GpsGate {
  // 입력: 투영된 GPS 위치와 선택적인 Global EKF 예측 위치.
  // 처리: GPS innovation 거리와 연속 복귀 fix를 기준으로 상태 전이와 reset 필요성을 판정한다.
  // 출력: ACCEPT, REJECT 또는 RESET_FILTER 동작과 현재 GPS gate 상태를 제공한다.
 // 상태: 복귀 기준점, 연속 fix 수와 reset 대기 여부를 내부에 기억한다.
 public:
  // 함수이름: GpsGate
  // 기능: GPS innovation과 복귀 안정성 판정 기준을 검증해 저장한다.
  // 인자: max_innovation_m, stable_return_radius_m, required_consecutive_fixes
  // 반환값: 없음
  GpsGate(double max_innovation_m, double stable_return_radius_m,
          std::size_t required_consecutive_fixes);

  // 함수이름: evaluate
  // 기능: 현재 상태와 새 GPS 위치를 평가하고 gate 상태를 갱신한다.
  // 인자: point, have_prediction, predicted_x, predicted_y
  // 반환값: 이번 GPS fix에 적용할 GateDecision
  GateDecision evaluate(const MapPoint& point, bool have_prediction,
                        double predicted_x, double predicted_y);
  // 함수이름: markTimeout
  // 기능: GPS 수신 timeout을 기록하고 상태를 DEGRADED로 전환한다.
  // 인자: 없음
  // 반환값: 없음
  void markTimeout();
  // 함수이름: confirmReset
  // 기능: SetPose 결과를 반영하고 성공한 경우 TRACKING으로 복귀한다.
  // 인자: succeeded - SetPose 성공 여부
  // 반환값: 없음
  void confirmReset(bool succeeded);
  // 함수이름: stateName
  // 기능: 현재 GPS gate 상태를 ROS 출력용 문자열로 변환한다.
  // 인자: 없음
  // 반환값: 현재 상태 문자열
  std::string stateName() const;

 private:
  // 함수이름: isFinite
  // 기능: MapPoint의 모든 좌표가 유한한지 검사한다.
  // 인자: point - 검사할 map 좌표
  // 반환값: 모든 좌표가 유한하면 true, 아니면 false
  bool isFinite(const MapPoint& point) const;
  // 함수이름: isWithinInnovation
  // 기능: GPS와 Global EKF 예측의 XY 거리가 innovation 한계 안인지 확인한다.
  // 인자: point, predicted_x, predicted_y
  // 반환값: innovation 한계 안이면 true, 아니면 false
  bool isWithinInnovation(const MapPoint& point, double predicted_x,
                          double predicted_y) const;
  // 함수이름: isStableReturn
  // 기능: 새 복귀 fix가 현재 복귀 기준점의 허용 반경 안인지 확인한다.
  // 인자: point - 검사할 GPS map 좌표
  // 반환값: 연속 복귀 fix로 인정할 수 있으면 true, 아니면 false
  bool isStableReturn(const MapPoint& point) const;
  // 함수이름: beginRelocalization
  // 기능: 새 복귀 묶음을 시작하고 첫 fix와 filter reset 필요 여부를
  //       저장한다.
  // 인자: point, requires_filter_reset
  // 반환값: 없음
  void beginRelocalization(const MapPoint& point, bool requires_filter_reset);

  double max_innovation_m_;
  double stable_return_radius_m_;
  std::size_t required_consecutive_fixes_;
  std::size_t consecutive_fixes_;
  MapPoint stable_return_point_;
  bool requires_filter_reset_;
  bool reset_pending_;
  GpsGateState state_;
};

}  // namespace localization_pkg

