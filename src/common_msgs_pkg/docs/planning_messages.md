# Planning baseline message semantics

이 문서는 필드 해석을 설명하며 공개 topic 계약을 복제하지 않는다. 공개 이름과 주기·timeout은 `ros_architecture_pkg/config/interface_contract.yaml`만 기준으로 한다.

## `LeadVehicleState`

- `pose`는 `header.frame_id`에서 추적 객체 bounding box 중심이다.
- `longitudinal_distance_m`는 ego 앞 bumper부터 객체 뒤 bumper까지의 선택 route 차로 방향 거리다.
- `lane_link_id`는 time-aligned World Model이 대응한 canonical MGeo Link ID다.
- World Model은 객체가 없을 때도 fresh, finite한 `valid=false` heartbeat를 계속 발행한다. topic 미수신, stale/future timestamp, wrong frame, 현재 route Link 불일치, NaN/Inf는 모두 정지 요구이며 `valid=false`의 대체 표현이 아니다.
- 유효하지 않거나 stale인 메시지는 새 차선 변경 권한을 만들 수 없다.

## `PlannedTrajectory`

- 각 점 pose 원점은 rear-wheel midpoint다.
- `points`는 Hybrid A* primitive 끝점만이 아니라 실제 검증된 원호의 조밀 표본이다.
- 각 point의 `lane_link_id`는 해당 표본에 가장 가까운 선택 MGeo Link다. 여러 Link를 지나는 local path와 차선 변경 중에는 point별로 달라질 수 있다.
- `minimum_boundary_clearance_m`는 보수적 outer wheel-contact proxy의 최소 hard-wall clearance다.
- `STATUS_VALID` header stamp는 입력 snapshot을 수락하고 탐색을 시작한 시각이며, `valid_until`은 그 시각에서 계산한다. 경로를 노출하지 않는 실패 status의 header stamp는 fail-closed 메시지 발행 시각이다.
- `STATUS_VALID`이어도 `valid_until`을 지났으면 제어 입력으로 사용할 수 없다.
- 다른 상태 또는 빈 점 배열은 명시적인 정지 요구다. 이전 valid trajectory를 계속 쓰지 않는다.
