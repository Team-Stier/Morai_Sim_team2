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

현재 중앙 계약이 draft이므로 승인된 공유 타입은 없다. 빈 `msg/`, `srv/`, `action/` 디렉터리는 임의 정의 권한이 아니라 향후 승인 타입의 위치다.

## 디렉터리

- `config/`: 타입 검증용 로컬 설정
- `docs/`: schema 결정과 migration 근거
- `launch/`: 타입/contract 검사 실행용 placeholder
- `src/`: 생성 타입 보조 검증 코드
- `msg/`, `srv/`, `action/`: 중앙 승인 후에만 사용
