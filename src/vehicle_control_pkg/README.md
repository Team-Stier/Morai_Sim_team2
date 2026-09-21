# vehicle_control_pkg

> **PUBLIC INTERFACE LOCK v1.0.0:** 아래 node/topic/type은
> [`interface_contract.yaml`](../ros_architecture_pkg/config/interface_contract.yaml)의
> 읽기용 투영이다. 통합 시 정확히 일치해야 하며 이 README에서 독립 변경하지 않는다.

## 담당 범위

- 승인된 trajectory의 lateral/longitudinal tracking
- nominal steering, accel과 brake 계산
- 물리 saturation, rate limit, anti-windup과 command watchdog
- tracking error, controller state와 command freshness 제공
- 정지 trajectory와 controlled stop 추종

## 담당하지 않는 범위

- 경로 탐색, 객체 융합, route progress와 신호 판단
- 최종 Safety 승인과 MORAI UDP 직렬화·송신
- 여러 독립 제어 명령 경로 생성

## 대회 규정상 유의사항

- 차량의 공지 최대 휠 조향각은 40°이고 최소 회전 반경은 5.87 m다.
- 40°가 UDP steering 필드와 같은 단위·부호·정의라고 추측하지 않는다. 실제 차량 응답으로 변환 계약을 검증한다.
- longitudinal 제어는 최종적으로 `cmd type = 1` accel/brake 계약에 맞아야 한다.
- 과도한 제어 진동과 overspeed가 패널티·경로 이탈·충돌로 이어지지 않도록 제한한다.

## 공개 ROS 입출력

현재 상태는 **Closedteam2 제어 코어 이식·ROS 연결 구현**이며 공개 경계 노드는
`vehicle_controller_node`다. Safety에서 Controller로 돌아오는 제어 feedback
topic은 두지 않는다.

![Vehicle Control 공개 입출력](docs/interface_io.svg)

- [Mermaid 원본](docs/interface_io.mmd)
- [PNG 이미지](docs/interface_io.png)

**공개 node (exact):** `vehicle_controller_node`

| 구분 | Topic | Type |
|---|---|---|
| 입력 | `/molit/localization/local/odometry` | `nav_msgs/Odometry` |
| 입력 | `/molit/localization/status` | `common_msgs_pkg/LocalizationStatus` |
| 입력 | `/molit/planning/trajectory` | `common_msgs_pkg/Trajectory` |
| 입력 | `/molit/planning/status` | `common_msgs_pkg/ComponentStatus` |
| 출력 | `/molit/control/nominal_command` | `common_msgs_pkg/ActuatorCommand` |
| 출력 | `/molit/control/status` | `common_msgs_pkg/ControllerStatus` |

`Trajectory`, `ControllerStatus`와 기존 `ActuatorCommand`를 사용한다.
필드와 통합 범위는 [중앙 제어 계약](../ros_architecture_pkg/docs/controller_integration.md),
Localization/ComponentStatus는 [기반 메시지 계약](../ros_architecture_pkg/docs/core_messages.md)을 따른다.

Safety Supervisor가 Controller 뒤에서 최종 gate를 수행하므로 이 출력은 아직 MORAI 송신 승인을 의미하지 않는다.

## 통합 전 자체 확인

- 노드의 통합 실행 이름이 정확히 `vehicle_controller_node`인지 확인한다.
- nominal command를 MORAI 또는 UDP로 직접 송신하지 않는다.
- 위 topic/type/unit/stamp를 유지하고 내부 topic은 `/molit/internal/vehicle_control/...`만 사용한다.
- 공개 이름을 remap하지 않고 중앙 계약 생성 검사를 통과시킨다.

## 디렉터리

- `config/`: gain, saturation, rate, watchdog과 차량 모델 파라미터
- `docs/`: 제어기 설계, 식별, 단위·부호와 응답 검증
- `launch/`: Vehicle Control 단독 실행
- `src/`: tracking controller와 nominal command 구현

## 실행과 이식 범위

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch vehicle_control_pkg vehicle_control_pkg.launch
```

전체 조합에서는 기존 `system_bringup_pkg.launch start_vehicle_control:=true`가
같은 launch를 포함한다. `use_sim_time`은 전체 실행 환경에서 결정한다.

- Closedteam2의 Pure Pursuit, Stanley, 전환 supervisor와 bounded PI를 사용한다.
- ROS `odom` pose/quaternion과 `base_link` 속도를 원본 코어 입력으로 변환한다.
  ROS 속도는 m/s이며 원본 PI와 조향 코어에 들어갈 때만 km/h로 바꾼다.
- Planner trajectory의 현재 reference time 속도를 보간한다. 자체 고정 속도,
  global-path 입력이나 원본의 별도 ROS 어댑터를 실행하지 않는다.
- 출력 조향은 rad이며 MORAI 정규화/부호 변환을 하지 않는다.
- 원본의 검사·제한은 유지하고 별도 입력 검증·watchdog·추가 제한은 넣지 않았다.
  Producer가 중앙 스키마 조건을 만족해야 한다.

Planner, Safety와 System Readiness 노드는 현재 main에서 미구현이다.
따라서 지금 연결된 범위는 네 ROS 입력부터 nominal command/status까지다.
개발 Localization의 `stop_required=true`는 Controller에도 전달된다.
MORAI 주행과 조향 부호·스케일은 별도 확인이 필요하다.

[원본 출처와 변경 범위](docs/closedteam2_import.md),
[원본 파일 해시](docs/upstream_manifest.yaml)를 함께 보관한다.

## 검증

```bash
PYTHONNOUSERSITE=1 catkin_make run_tests_vehicle_control_pkg run_tests_common_msgs_pkg
catkin_test_results --all build/test_results
```

원본 조향 단위시험, 원본 PI, m/s↔km/h·yaw·timestamp·목표 속도 보간,
ROS publisher/subscriber 연결 및 상류 정지 상태 전달을 확인한다.

[이번 이식의 검증 결과](docs/validation.md)를 참고한다.

## 전역경로 추종 시험 (2026-09-21)

사용자가 요청한 현재 시뮬레이터 전용 실행은
[global_path_demo 중앙 프로필](../ros_architecture_pkg/config/messages/global_path_demo.yaml)을 따른다.
`roslaunch system_bringup_pkg global_path_demo.launch`로 기존 Localization에 연결해
전역경로만 10 km/h로 추종한다. 일반 실행과 구분된 개발용 직접 전달 경로이며
장애물·신호 판단을 수행하지 않는다. 별도 방어 계층은 추가하지 않았다.
실제 상태와 실행·중지 방법은 [실행 기록](../ros_architecture_pkg/docs/global_path_demo.md)에 기록한다.

전역경로 실행은 `config/global_path_demo.yaml`의 lookahead 튜닝을 사용한다. 원본 알고리즘은 동일하며 Planner의 현재 구간 속도를 추종한다. 일반 상한 50·고주로 목표 100 km/h 규칙은 중앙 코스 정책에 있다.
