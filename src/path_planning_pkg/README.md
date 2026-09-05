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
- 출력: 유효기간과 quality, 곡률·목표속도가 명시된 dense local path 후보

직접 accel/brake/steer 또는 UDP 명령을 만들지 않는다.

## 현재 구현

- KATRI MGeo의 선택 route Link만 rolling corridor로 만드는 adapter
- 대회 제공 4,430점 `/planning/global_path`를 불변 polyline으로 검증·저장하고,
  `RouteContext` 진행도와 원본 인덱스를 기준으로 일반 주행용 전방 slice/목표를 생성
- 작은 후퇴 noise는 단조 진행도로 clamp하고, 지역 인덱스 탐색·횡방/종방/
  heading tolerance를 넘는 관측은 전역 경로 분기 jump로 채택하지 않는 보수적 matching
- 일반 주행은 대회 전역경로에 대한 heuristic과 누적 횡방 cost를 동시에 쓰되,
  차선 변경/인접 차로 follow 중은 승인된 MGeo branch guide를 유지
- 발행 trajectory의 `reason`에 공식 경로·검증된 maneuver·안전 MGeo fallback
  reference mode를 남겨 rosbag/track test에서 실제 사용 경로를 구분
- 전진 bicycle primitive와 `(x, y, yaw, steering)` 상태를 쓰는 Hybrid A*
- 선택 reference를 search ordering과 누적 횡방 cost에 사용하는 route guide
  (안전 판정 권한은 없으며 최종 허용은 지도 hard wall이 결정)
- 보수적 네 wheel-contact proxy의 strict-positive hard-wall clearance와 차체 기반 객체 충돌 검사
- primitive 사이 구간까지 연속 안전성을 보수적으로 판정하고, 조밀한 검증 표본을 그대로 출력
- 모든 측면 실선·점선·미상·합성 경계를 기본 hard wall로 유지
- 선택 route의 topology가 검증된 종방향 connector mouth만 개방
- 고주로, 동일 Link 선행차 gap 10 m 이하, fresh/confident 입력, 검증된 인접 Link, 실제 pure-dashed 공유 경계가 모두 맞을 때만 횡방향 벽 개방
- 입력 stale, 지도 모순, 시작/목표 invalid 또는 no-path 때 기존 경로를 재사용하지 않는 stop-required 출력과 planning tick 사이 50 Hz input watchdog
- link seam의 보수적 footprint에 목표가 걸리면 전방의 첫 valid centerline 목표를 다시 선택
- 절대 wall-clock 탐색·연속 충돌검사 예산, 탐색 후 최신 odometry/route/lead 재검사, 발행 직전 변경 입력에 대한 20 ms locked 재검증과 최신 ego의 dense-path 추종오차 gate
- 탐색 시작 시각 기준으로 만료되는 결과와, valid 결과를 발행한 후에만 commit하는 차선 변경 상태

현재 구현은 dense geometric path에 곡률별 목표속도를 붙인 후보 생성기다. 점별
도착시각과 종방향 가감속 profile은 아직 없으므로 완전한 time-parameterized
trajectory라고 부르지 않는다. Localization·tracking·controller 오차를 합친 안전
보증과 controller의 nearest-point/latency 추종, target-lane 전후방 객체 gap 확인은 World Model 및 Safety Supervisor 통합
전에는 완료로 간주하지 않는다.

## 디렉터리

- `config/`: behavior, cost, horizon, kinematic/dynamic constraint 파라미터
- `docs/`: planner 설계, 시나리오와 성능·안전 평가
- `launch/`: Path Planning 단독 실행
- `src/`: behavior and motion planning 구현

상세 설계와 지도별 제한은 [`docs/hybrid_astar_design.md`](docs/hybrid_astar_design.md)를 참고한다.
