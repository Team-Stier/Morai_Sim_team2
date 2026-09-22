# path_planning_pkg

2026-09-22 bag 기반 시간 단축 설정과 검증 한계는
[분석 기록](docs/bag_tuning_20260922.md)을 참고한다.

## Frenet RDDF 개발 구현

[Frenet Planner](docs/frenet_rddf.md)는 지정한 RDDF 14개와 전역경로를 사용해
후보 생성 → 규정·클러스터 충돌 검사 → ETA 비용 비교 → 변경 상태 유지를 수행한다.
`frenet_planner.launch`가 단독 실행, `system_bringup_pkg/frenet_rddf.launch`가 통합 실행이다.
출력은 기존 odom Trajectory이며 첫 시험 상한은 10 km/h다.

현재 일반 경로는 최소 0.5초 유지한 뒤 최신 계산 결과로 교체한다.
유지 중에도 활성 경로의 충돌을 재검사하며 정지·정지 접근, 입력 오류,
Localization reset과 시간 역행은 즉시 반영하고 대기 경로를 폐기한다.
`minimum_active_path_hold_sec`는 경로 교체 간격이며 궤적 발행은 10 Hz를 유지한다.

개발 RDDF 형상 전용 모드에서는 WorldModel 입력 지연만 0.7초까지 허용한다.
위치·경로 입력은 기존 0.5초 기준을 유지하고, 무효 scene·epoch 불일치는 거부한다.
[현재 주행 기반 조정과 검증](docs/live_parameter_tuning_20260923.md)을 참고한다.

발행할 때 선택 경로의 좌표·접선은 보존하고 차량에 가까운 앞부분만 잘라낸다.
첫 점을 차량 위치로 강제 이동하여 추종 오차를 숨기거나 꺾임을 만들지 않는다.
Localization reset 시 활성·대기 경로를 폐기하고 새 상태로 계산한 경로를 기다린다.

정적 LiDAR 클러스터로 RDDF 유지 경로가 막히면 근거리 우회·복귀 후보도 비교한다.
`local_detour_offsets_m`의 양수는 RDDF 왼쪽, 음수는 오른쪽이며 양쪽 모두
1.5/2.5/3.5 m 우회 후보를 생성해 같은 비용식으로 비교한다. 회피 구간은 최대 10 km/h이며 차량 footprint 충돌·조향 한계를 검사한다.

> **PUBLIC INTERFACE LOCK v1.0.0:** 아래 node/topic/type은
> [`interface_contract.yaml`](../ros_architecture_pkg/config/interface_contract.yaml)의
> 읽기용 투영이다. 통합 시 정확히 일치해야 하며 이 README에서 독립 변경하지 않는다.

## 담당 범위

- Localization의 local odometry·ego state·quality를 사용한 현재 차량 상태 확인
- route context와 진행도를 사용한 behavior planning
- time-aligned World Model의 차로·신호·정지선·장애물·NPC·합류 상황을
  고려한 local motion planning
- 차량 곡률·최소 회전 반경·가감속 한계를 만족하는 시간 파라미터 trajectory
- 입력 freshness, 재계획 상태, 계산 지연, 준비 여부와 trajectory 유효기간 제공
- 실행 가능한 경로가 없을 때 명시적 invalid/stop-required 상태 제공

## 담당하지 않는 범위

- sensor fusion, Localization, 전역 route progress 계산
- actuator 값 생성, Safety 최종 판단과 MORAI UDP 송신
- sample scene의 고정 객체 위치를 본선 계획 정답으로 사용

raw Camera/LiDAR와 개별 Perception 관측은 직접 구독하지 않는다. Camera/LiDAR
Perception은 센서 관측을 소유하고, World Model은 좌표 변환·시간 정렬·융합과
tracking을 소유하며, Planner는 통합된 scene만 사용한다.

## 대회 규정상 유의사항

- 출발 후 1분 이내 경로 5%를 통과하되 다른 안전 규칙을 희생하지 않는다.
- 체크포인트를 순서대로 반경 3 m 이내 통과하도록 route context를 따른다.
- 기본 제한속도는 60 km/h이며 공식 Link 예외는 과속 의무가 아니라 제한 예외다.
- 신호, 실선·중앙선, 충돌 회피, 랜덤 장애물·끼어들기와 15분 완주를 함께 고려한다.
- GPS blackout 구간에서도 Localization/World Model quality에 맞춰 보수적으로 계획한다.

## 공개 ROS 입출력

현재 상태는 **개발용 Frenet RDDF 후보 생성·ETA 선택 구현**이며 공개 경계 노드는
`path_planner_node`다. 실행 범위는 [Frenet 설계](docs/frenet_rddf.md)를 따르며, `Trajectory` schema는
[중앙 제어 계약](../ros_architecture_pkg/docs/controller_integration.md)에 구현됐다.

![Path Planning 공개 입출력](docs/interface_io.svg)

- [Mermaid 원본](docs/interface_io.mmd)
- [PNG 이미지](docs/interface_io.png)

**공개 node (exact):** `path_planner_node`

| 구분 | Topic | Type |
|---|---|---|
| 입력 | `/molit/map/hd_map` | `common_msgs_pkg/HdMap` |
| 입력 | `/molit/localization/local/odometry` | `nav_msgs/Odometry` |
| 입력 | `/molit/localization/ego_state` | `common_msgs_pkg/EgoState` |
| 입력 | `/molit/localization/status` | `common_msgs_pkg/LocalizationStatus` |
| 입력 | `/molit/route/context` | `common_msgs_pkg/RouteContext` |
| 입력 | `/molit/route/status` | `common_msgs_pkg/ComponentStatus` |
| 입력 | `/molit/world_model/scene` | `common_msgs_pkg/WorldModel` |
| 입력 | `/molit/world_model/status` | `common_msgs_pkg/ComponentStatus` |
| 출력 | `/molit/planning/trajectory` | `common_msgs_pkg/Trajectory` |
| 출력 | `/molit/planning/status` | `common_msgs_pkg/ComponentStatus` |

각 입력은 원본 측정 `header.stamp`와 중앙 timestamp 계약을 따른다. 입력이
누락·stale하거나 clock domain이 다르면 이를 숨기거나 직전 scene을 현재
관측처럼 재사용하지 않고 planning status에 명시한다.

Trajectory의 공개 frame은 제어 연속성을 위해 `odom`으로 고정한다.
`ComponentStatus`, `EgoState`, `LocalizationStatus`, `WorldModel` 스키마는 구현됐으며
[기반 메시지 계약](../ros_architecture_pkg/docs/core_messages.md)을 따른다.
`RouteContext`와 Frenet Planner 노드가 구현됐다. `Trajectory`는 poses/speed_mps/time_from_start의
동일 길이 배열과 valid/stop_required/valid_for/reset_id를 제공한다. v1에서 이 패키지가 생성하는 주행
출력은 `/molit/planning/trajectory`뿐이며 직접 accel/brake/steer 또는
UDP 출력은 금지한다.

생성된 trajectory는 반드시 `vehicle_controller_node`의 nominal command와
`safety_supervisor_node`의 최종 gate를 거친다.

## 통합 전 자체 확인

- 노드의 통합 실행 이름이 정확히 `path_planner_node`인지 확인한다.
- 정적 HdMap의 map_id와 Localization, Route 및 World Model 입력의 freshness·frame·timestamp를
  검사하고, 승인되지 않은 raw sensor 또는 Perception topic을 구독하지 않는다.
- 재계획 이유와 trajectory 유효성을 status에 남기고 stale trajectory를
  계속 출력하지 않게 검증한다.
- trajectory의 `header.frame_id=odom`, stamp와 유효기간을 보존한다.
- 위 topic/type만 공개하고 내부 topic은 `/molit/internal/path_planning/...`만 사용한다.
- 공개 이름을 remap하지 않고 중앙 계약 생성 검사를 통과시킨다.

## 디렉터리

- `config/`: behavior, replanning, horizon과 차량 제약 파라미터
- `docs/`: behavior/motion planner 설계와 성능·안전 평가
- `launch/`: Path Planning 단독 실행
- `src/`: behavior and motion planning 구현

## 전역경로 추종 시험 (2026-09-21)

사용자가 요청한 현재 시뮬레이터 전용 실행은
[global_path_demo 중앙 프로필](../ros_architecture_pkg/config/messages/global_path_demo.yaml)을 따른다.
`roslaunch system_bringup_pkg global_path_demo.launch`로 기존 Localization에 연결해
전역경로만 10 km/h로 추종한다. 일반 실행과 구분된 개발용 직접 전달 경로이며
장애물·신호 판단을 수행하지 않는다. 별도 방어 계층은 추가하지 않았다.
실제 상태와 실행·중지 방법은 [실행 기록](../ros_architecture_pkg/docs/global_path_demo.md)에 기록한다.

전역경로 실행의 고정 속도 규칙은 중앙 `config/map/course_speed_policy.yaml`이다. 일반 상한 58 km/h(순항 56), 고주로 제한 없음(순항 150)을 곡률·출구 전 감속 프로파일에 적용한다.
