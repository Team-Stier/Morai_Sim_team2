# morai_interface_pkg

> **INTERFACE LOCK:** 이 패키지는 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 ROS 계약을 따른다. 구체 node/topic/message/frame 이름은 여기서 정의하지 않는다.

## 담당 범위

- MORAI와 통신하는 유일한 외부 네트워크 경계
- 허용 UDP 패킷의 길이·필드·byte order·sequence·timestamp 검증
- Camera, LiDAR, GPS, IMU, 차량 상태와 충돌 정보의 내부 형식 정규화
- 최종 승인된 차량 명령을 대회 패킷으로 직렬화하여 송신
- 연결 여부, 수신 age, packet drop과 decode 오류 상태 제공

## 대회 규정상 유의사항

- 허용 항목은 Ego 제어, CollisionData, Competition Vehicle Status, GPS, IMU, Camera와 3D LiDAR뿐이다.
- 제어는 `cmd type = 1`, `ctrl mode = 2` 계약을 지켜야 한다.
- Ground Truth, Bounding Box, V2I/V2V, ROS Bridge 또는 숨은 시뮬레이터 상태를 주행 입력으로 사용하지 않는다.
- 참고 카메라 JSON의 loopback IP와 port는 현재 파일 값일 뿐 본선 네트워크 계약이 아니다.

## 논리 입출력

- 입력: Safety Supervisor가 승인한 최종 제어 명령
- 출력: 검증된 센서 관측, 차량 상태, 충돌 이벤트와 통신 health

패킷 명세가 확보되기 전에는 포트나 필드 구조를 추측해 구현하지 않는다. stale 패킷을 새 데이터처럼 재발행하지 않으며 연결 상실 시 명시적인 invalid 상태를 제공한다.

## 디렉터리

- `config/`: 검증된 네트워크와 packet 설정
- `docs/`: 실제 packet capture, 버전과 연동 근거
- `launch/`: 이 interface만 단독 실행
- `src/`: transport, parser, serializer와 health 구현
