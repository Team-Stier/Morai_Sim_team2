# common_msgs_pkg

> **INTERFACE LOCK:** 이 패키지는 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 계약을 의무적으로 따른다. 중앙 계약에 없는 타입이나 공개 이름을 먼저 만들지 않는다.

## 담당 범위

- 중앙 승인을 받은 공유 ROS message, service, action 타입 구현
- 필드 의미, 단위, frame, timestamp와 invalid 표현을 계약과 일치하도록 유지
- producer와 consumer가 함께 실행하는 serialization·contract test 제공

## 담당하지 않는 범위

- 인터페이스 의미와 이름의 독자적 결정
- 기능 알고리즘, UDP 송수신, launch 조합
- 패키지 한 곳에서만 쓰는 내부 자료구조의 무조건적인 공용화

## 승인된 planning baseline 타입

- `RouteContext`: 유효성·사유, 단조 route progress, 현재/전방 MGeo Link와 고주로 규정 context
- `LeadVehicleState`: World Model이 산출한 동일 차로 선행차 중심 pose, 크기, 속도, confidence와 bumper gap
- `TrajectoryPoint`: rear-axle 기준 경로 pose, 곡률, 누적거리, 목표속도와 선택 Link
- `PlannedTrajectory`: 유효기간, fail-closed 상태, 벽 개방 여부와 최소 경계 clearance를 포함한 조밀 경로

정확한 topic, frame, timestamp, timeout과 invalid 정책은 이 README가 아니라 중앙 계약이 원본이다. 숫자 필드는 NaN을 invalid sentinel로 사용하지 않으며, `valid=false` 또는 trajectory status/expiry를 확인해야 한다.

## 디렉터리

- `config/`: 타입 검증용 로컬 설정
- `docs/`: schema 결정과 migration 근거
- `launch/`: 타입/contract 검사 실행용 placeholder
- `src/`: 생성 타입 보조 검증 코드
- `msg/`, `srv/`, `action/`: 중앙 승인 후에만 사용
