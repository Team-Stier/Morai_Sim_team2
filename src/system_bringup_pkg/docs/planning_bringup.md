# Planning-stack bringup

## 구성

`planning_stack.launch`는 다음 component launch를 조합한다.

1. `global_route_manager_pkg`: 공식 global path와 route context
2. `path_planning_pkg`: local trajectory/path와 active corridor marker
3. `runtime_evaluation_pkg`: accepted odometry trace와 ego body marker
4. GUI 모드의 `hd_map_pkg`: 정적 KATRI HD-map marker
5. optional ROS1 RViz: 위 시각화 topic을 `map` fixed frame에서 표시

각 component가 topic, frame과 알고리즘 parameter의 소유권을 유지한다. 시스템
launch는 동일 parameter를 복제하지 않는다. RViz 프로세스는 command path에 없고
종료되어도 차량 명령에 영향을 주지 않는다.

## 의도적으로 시작하지 않는 것

이 개발 profile은 simulator fake publisher, rosbag replay, MORAI UDP bridge,
localization 또는 world-model node를 시작하지 않는다. 특히 reference global path를
ego 위치의 Ground Truth로 변환하지 않는다. localization input이 없으면 route
progress와 local planning은 유효 상태가 될 수 없다. World Model은 선행 객체가
없을 때도 fresh, finite `valid=false` heartbeat를 발행해야 한다. lead-vehicle topic
미수신·stale·malformed는 단순 차선 변경 금지가 아니라 planner 정지 요구다.
Planner의 50 Hz input watchdog은 2 Hz planning tick 사이에도 이 상태를 감시해
기존 trajectory를 즉시 non-VALID로 덮어쓴다.

## 실행

ROS1 Noetic workspace를 build하고 source한 terminal에서:

```bash
roslaunch system_bringup_pkg planning_stack.launch
```

`use_rviz`의 기본값은 `true`다. 따라서 `planning_stack.launch`, 이를 포함하는
`path_control_test.launch`와 `system_bringup_pkg.launch`를 기본 인자로 시작하면
동일한 planning RViz profile이 함께 시작된다. 이 기본 동작과 필수 display topic은
launch contract test로 고정한다.

headless 확인:

```bash
roslaunch system_bringup_pkg planning_stack.launch use_rviz:=false
```

`use_rviz:=false`는 RViz와 시각화 전용 HD-map marker publisher를 둘 다
생성하지 않는다. planner는 여전히 MGeo source를 직접 읽으므로 map
경로는 headless에서도 유효해야 한다.

## 데이터와 config 경로

시스템 launch는 다음 arg를 각 component launch로 그대로 전달한다.

| launch arg | 사용 component | 기본값 |
|---|---|---|
| `map_source_directory` | HD-map RViz, planner | `hd_map_pkg/vendor/verdict_sdk/map-data/KATRI` |
| `map_config_file` | HD-map RViz, planner | 설치된 `hd_map_pkg/config/map_conversion.yaml` |
| `route_file` | global route manager | HD-map config에 선언된 reference file |
| `route_config_file` | global route manager | 설치된 `global_route_manager_pkg/config/competition_route.yaml` |
| `planner_config_file` | planner | 설치된 `path_planning_pkg/config/hybrid_astar.yaml` |

MORAI 원본 map과 참조 global-path 파일은 재배포 권한이 확정되지 않아
catkin install에 포함하지 않는다. 따라서 install/deploy workspace에서는
외부 데이터 절대경로를 명시해야 한다.

```bash
roslaunch system_bringup_pkg planning_stack.launch \
  map_source_directory:=/absolute/path/to/KATRI \
  route_file:=/absolute/path/to/2026_molit_comp_global_path.txt
```

패키지 config를 외부에서 관리할 때는 `map_config_file`,
`route_config_file`, `planner_config_file`도 같은 방식으로 절대경로를 넘긴다.

기존 기본 이름을 사용하는 `system_bringup_pkg.launch`도 같은 profile을 include한다.

## RViz display

`rviz/planning_stack.rviz`에는 다음 중앙 계약 topic만 등록되어 있다.

- `/hd_map/markers`
- `/planning/global_path`
- `/planning/local_path`
- `/planning/ego_trace`
- `/planning/vehicle_marker`
- `/planning/corridor_markers`

사용자 입력을 발행하는 2D Pose Estimate, Navigation Goal과 Publish Point tool은
제외했다. map 전체는 넓으므로 처음 실행 후 `F` 또는 RViz의 focus 기능으로 원하는
경로 구간에 camera를 맞출 수 있다.

## 검증 경계

XML/YAML 문법 검사와 launch dependency 검사는 offline으로 가능하다. 실제 성공
판정에는 ROS master에서 각 producer/consumer 연결, latched map/global path 수신,
odometry timestamp age, vehicle marker stale 삭제와 RViz rendering을 확인해야 한다.
