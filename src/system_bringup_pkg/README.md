# system_bringup_pkg

> **INTERFACE LOCK:** 전체 실행도 [`ros_architecture_pkg`](../ros_architecture_pkg/README.md)의 중앙 계약과 readiness 순서를 따른다. 이 README에는 구체 node/topic 이름을 정의하지 않는다.

## 담당 범위

- 전체 시스템 launch 조합의 단일 소유권
- 파라미터 파일 연결, 시작 순서와 readiness gate
- 중복 publisher, 잘못된 실행 모드와 필수 패키지 누락 방지
- 실시간 MORAI 모드와 offline replay 모드 분리

## 반드시 지킬 것

- 개별 패키지 launch는 해당 기능만 시작한다.
- MORAI와 rosbag/replay 공급자를 동시에 시작하지 않는다.
- Safety Supervisor가 준비되기 전에 주행 명령을 활성화하지 않는다.
- 실행 이후 Operator 조작 없이 상태 확인과 fail-closed 종료가 가능해야 한다.

## 현재 planning 개발 profile

중앙 계약의 planning baseline을 한 번에 확인하는 개발 profile을 제공한다.

```bash
roslaunch system_bringup_pkg planning_stack.launch
```

이 profile은 `global_route_manager_pkg`, `path_planning_pkg`,
`runtime_evaluation_pkg`를 실행하고, GUI 모드에서는 `hd_map_pkg`의 시각화
node와 RViz도 조합한다. RViz에는 중앙 계약에서 승인한 HD map
marker, global/local path, active corridor, ego trace와 vehicle envelope가 등록되어
있다. GUI가 필요 없으면 다음과 같이 실행한다.

```bash
roslaunch system_bringup_pkg planning_stack.launch use_rviz:=false
```

headless 모드에서는 RViz와 HD-map marker publisher를 둘 다 시작하지
않지만, planner는 MGeo를 직접 읽으므로 `map_source_directory`는 그대로
필요하다.

source workspace에서 map 기본값은 `hd_map_pkg` 안의 고정 submodule을
가리킨다. 데이터 재배포 권한이 확정되지 않아 submodule map과 참조
global-path 파일은 catkin install 대상이 아니다. install/deploy workspace에서는
외부 데이터의 절대경로를 launch arg로 전달한다.

```bash
roslaunch system_bringup_pkg planning_stack.launch \
  map_source_directory:=/absolute/path/to/KATRI \
  route_file:=/absolute/path/to/2026_molit_comp_global_path.txt
```

`map_config_file`, `route_config_file`, `planner_config_file`은 기본적으로
설치된 각 패키지 config를 사용하며 필요하면 같은 이름의 launch arg로
명시적으로 바꿀 수 있다.

이 launch는 MORAI UDP, localization, world model 또는 가짜 입력 publisher를
시작하지 않는다. 따라서 실제 `/localization/odometry` producer와, 조건부 차선 변경을
시험하려면 실제 `/world_model/lead_vehicle` producer를 별도로 중앙 계약에 맞게
연결해야 한다. 입력이 없거나 stale이면 planner와 시각화 노드는 각자의 fail-closed
정책을 따른다. World Model은 탐지 객체가 없을 때도 fresh, finite `valid=false`
heartbeat를 발행해야 하며 topic 미수신은 빈 도로가 아니라 정지 요구다. 실행 구성과 제한은
[`docs/planning_bringup.md`](docs/planning_bringup.md)에 기록한다.

## 디렉터리

- `config/`: 시스템 조합과 모드별 파라미터
- `docs/`: startup sequence, readiness와 운영 절차
- `launch/`: 승인된 전체 시스템 조합
- `src/`: 향후 readiness 보조 도구
