# global_route_manager_pkg

> **INTERFACE LOCK:** 이 패키지는 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 ROS 계약을 따른다. 구체 node/topic/message/frame 이름은 여기서 정의하지 않는다.

## 담당 범위

- 공식 전역경로와 체크포인트 순서 관리
- ego pose를 이용한 단조롭고 연속적인 route progress 추정
- 다음 경로 구간, local route corridor와 high-level route context 제공
- 시작 1분/5%, 체크포인트 반경 3 m와 순차 통과 상태의 관측 지원
- HD Map topology와 규정 속도 구간의 대응 관리

## 담당하지 않는 범위

- 정적 HD Map 원본 소유, 동적 장애물 회피와 local trajectory
- pose 추정, actuator 또는 UDP 명령
- 실제 판정 프로그램을 대체하는 공식 채점 판정

## 현재 경로 유의사항

- 원본은 4,430점 폐곡선이며 연속 중복점 38개를 포함한다.
- 0 길이 구간의 yaw와 progress 계산을 안전하게 처리하고 원본/필터 후 인덱스 대응을 보존한다.
- 60 km/h 예외 Link 구간은 실제 HD Map Link 대응이 검증되기 전까지 좌표로 추측하지 않는다.
- checkpoint 좌표는 progress 검증 기준이지 Localization Ground Truth가 아니다.

## 논리 입출력

- 입력: 검증된 전역경로, HD Map topology, ego pose와 localization quality
- 출력: route validity, progress, checkpoint state, local route context와 규정 context

## 디렉터리

- `config/`: progress, projection, checkpoint와 route-local 설정
- `docs/`: 경로 provenance, topology 대응과 검증 보고서
- `launch/`: Global Route Manager 단독 실행
- `src/`: route loader, matcher와 progress 구현
