# path_planning_pkg

> **INTERFACE LOCK:** 이 패키지는 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 ROS 계약을 따른다. 구체 node/topic/message/frame 이름은 여기서 정의하지 않는다.

## 담당 범위

- route context와 world model을 이용한 behavior decision
- 차로·신호·정지선·장애물·NPC·합류 상황을 고려한 local trajectory 생성
- 차량 곡률·최소 회전 반경·가감속 한계를 만족하는 시간 파라미터 trajectory
- 새 객체, 신호 변화, route/localization quality 저하에 대한 재계획
- 실행 가능한 경로가 없을 때 명시적 invalid/stop-required 상태 제공

## 담당하지 않는 범위

- sensor fusion, Localization, 전역 route progress 계산
- actuator 값 생성, Safety 최종 판단과 MORAI UDP 송신
- sample scene의 고정 객체 위치를 본선 계획 정답으로 사용

## 대회 규정상 유의사항

- 출발 후 1분 이내 경로 5%를 통과하되 다른 안전 규칙을 희생하지 않는다.
- 체크포인트를 순서대로 반경 3 m 이내 통과하도록 route context를 따른다.
- 기본 제한속도는 60 km/h이며 공식 Link 예외는 과속 의무가 아니라 제한 예외다.
- 신호, 실선·중앙선, 충돌 회피, 랜덤 장애물·끼어들기와 15분 완주를 함께 고려한다.
- GPS blackout 구간에서도 Localization/World Model quality에 맞춰 보수적으로 계획한다.

## 논리 입출력

- 입력: planner-ready world model, route progress/context, localization/route quality와 차량 제약
- 출력: 유효기간과 quality가 명시된 time-parameterized candidate trajectory

직접 accel/brake/steer 또는 UDP 명령을 만들지 않는다.

## 디렉터리

- `config/`: behavior, cost, horizon, kinematic/dynamic constraint 파라미터
- `docs/`: planner 설계, 시나리오와 성능·안전 평가
- `launch/`: Path Planning 단독 실행
- `src/`: behavior and motion planning 구현
