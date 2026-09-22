# world_model_pkg

Frenet 통합에서는 LiDAR의 실제 클러스터 점을 측정시각 map TF로 변환해
TrackedObject.points에 보존한다. 추적 중심점과 속도는 별도로 계산하며
점군을 박스·볼록껍질로 대체하거나 예측 위치로 덮어쓰지 않는다.
속도 예측은 Planner가 source_stamp 이후 시간에 적용한다.

> **PUBLIC INTERFACE LOCK v1.0.0:** 아래 node/topic/type은
> [`interface_contract.yaml`](../ros_architecture_pkg/config/interface_contract.yaml)의
> 읽기용 투영이다. 통합 시 정확히 일치해야 하며 이 README에서 독립 변경하지 않는다.

## 이 패키지가 필요한 이유

Camera/LiDAR 결과를 각 패키지가 임의로 HD Map 위에 투영하면 서로 다른 timestamp, calibration과 pose를 사용해 Planner에서 충돌한다. 이 패키지가 지도·ego·동적 객체 융합의 단일 소유자가 된다.

## 담당 범위

- 관측 timestamp에 해당하는 ego pose history 조회와 보간
- 승인된 sensor extrinsic을 사용한 좌표 변환
- Camera/LiDAR cross-sensor association, fusion과 dynamic tracking
- 정적 HD Map, ego footprint, lane/signal/free-space와 객체의 일관된 scene 구성
- 각 객체와 layer의 source, age, confidence와 uncertainty 유지
- planner-ready local world model과 freshness/quality 상태 제공

## 담당하지 않는 범위

- 개별 센서 raw inference, ego Localization 자체
- 행동 결정, route progress, trajectory와 actuator 제어
- sample scene의 객체·신호 상태를 live world state로 주입

## 안전 원칙

- 최신 메시지끼리 단순 결합하지 않고 측정시각을 기준으로 정렬한다.
- Localization uncertainty가 증가하면 투영된 객체 uncertainty도 함께 증가시킨다.
- stale observation, frame 불일치와 calibration 누락은 명시적으로 reject/degraded 처리한다.
- 같은 객체의 Camera/LiDAR 관측을 이중 장애물로 세지 않도록 association 근거를 유지한다.

## 공개 ROS 입출력

현재 상태는 **LiDAR 객체의 scan-time map 변환과 개발용 tracking 구현**이며
공개 경계 노드는 `world_model_node`다. HD Map·Route·Camera·free-space 융합과
주행 readiness 승인은 아직 구현하지 않았다.

![World Model 공개 입출력](docs/interface_io.svg)

- [Mermaid 원본](docs/interface_io.mmd)
- [PNG 이미지](docs/interface_io.png)

**공개 node (exact):** `world_model_node`

| 구분 | Topic | Type |
|---|---|---|
| 입력 | `/molit/map/hd_map` | `common_msgs_pkg/HdMap` |
| 입력 | `/molit/map/status` | `common_msgs_pkg/ComponentStatus` |
| 입력 | `/molit/perception/camera/front/observations` | `common_msgs_pkg/CameraObservationArray` |
| 입력 | `/molit/perception/camera/left/observations` | `common_msgs_pkg/CameraObservationArray` |
| 입력 | `/molit/perception/camera/right/observations` | `common_msgs_pkg/CameraObservationArray` |
| 입력 | `/molit/perception/camera/status` | `common_msgs_pkg/ComponentStatus` |
| 입력 | `/molit/perception/lidar/observations` | `common_msgs_pkg/LidarObservationArray` |
| 입력 | `/molit/perception/lidar/status` | `common_msgs_pkg/ComponentStatus` |
| 입력 | `/molit/localization/local/odometry` | `nav_msgs/Odometry` |
| 입력 | `/molit/localization/ego_state` | `common_msgs_pkg/EgoState` |
| 입력 | `/molit/localization/status` | `common_msgs_pkg/LocalizationStatus` |
| 입력 | `/molit/route/context` | `common_msgs_pkg/RouteContext` |
| 입력 | `/molit/route/status` | `common_msgs_pkg/ComponentStatus` |
| 출력 | `/molit/world_model/scene` | `common_msgs_pkg/WorldModel` |
| 출력 | `/molit/world_model/status` | `common_msgs_pkg/ComponentStatus` |

공유 타입 중 `ComponentStatus`, `EgoState`, `LocalizationStatus`,
`LidarObservationArray`, `TrackedObject`, `WorldModel` 스키마가 구현됐다.
해당 타입을 사용하는 공개 I/O는 [기반 메시지 계약](../ros_architecture_pkg/docs/core_messages.md)을 따른다.
`HdMap`과 `RouteContext` 스키마도 구현됐으며, 교차 센서 융합 계층은 미구현이다.

좌표 변환에는 중앙 [`TF 계약`](../ros_architecture_pkg/config/tf/frame_contract.yaml)에서 승인된 frame과 extrinsic만 사용한다. 시간 정렬에는 중앙 [`Timestamp 계약`](../ros_architecture_pkg/config/timestamp/timestamp_contract.yaml)을 적용하고, 각 관측의 source stamp를 fusion publication time으로 교체하지 않는다.

## 통합 전 자체 확인

- 노드의 통합 실행 이름이 정확히 `world_model_node`인지 확인한다.
- 모든 관측은 source stamp의 pose로 변환하고 age/uncertainty를 보존한다.
- 위 topic/type/frame/stamp를 유지하고 내부 topic은 `/molit/internal/world_model/...`만 사용한다.
- 공개 이름을 remap하지 않고 중앙 계약 생성 검사를 통과시킨다.

## 디렉터리

- `config/`: sync, association, tracking, uncertainty와 stale 파라미터
- `docs/`: calibration, frame, fusion schema와 평가 근거
- `launch/`: World Model 단독 실행
- `src/`: temporal buffer, transform, fusion과 tracking 구현

## 현재 LiDAR-only 개발 구현

`world_model_node`는 `/molit/perception/lidar/observations`의 원본 scan stamp로
`map <- lidar_link` TF를 조회한다. latest-time TF fallback은 사용하지 않는다.
LiDAR 클러스터의 원본 점을 map으로 변환한 뒤, 점들의 중심으로 map XY
최근접 association을 수행하여 process-local `track_id`를 부여한다.
형상은 실제 점으로 유지하며 속도에 따른 연결 거리는 관측 간격을 반영한다.

속도는 관측 중심점 차분 대신 연속 map 점군의 XY 평행이동 정합으로
추정한다. 0 이동과 중심점 차분에서 각각 시작하여 부분 관측에 대한
양방향 trimmed 최근접 잔차를 비교한다. 정합용 표본 수·반복 수·사용
비율은 `tracking.motion_*` 설정이다. 출력 points와 source stamp는 원본을 유지한다.
긴 무특징 표면에서 잔차가 같은 해는 최소 이동을 선택하는 개발용 가정이다.
이는 정적 물체임을 입증하지 않으며 회전·가림·대칭 형상에 대한 검증은 남아 있다.

- 같은 정적 물체는 차량이 이동해 상대좌표가 달라져도 map 위치와 track ID를 유지한다.
- 새 track은 tentative이며 두 번 관측되면 confirmed가 된다.
- 검출 누락은 설정된 `maximum_coast_sec` 동안만 예측하며 이후 삭제한다.
- 세 번 이상 연속 관측된 track만 map 선속도를 개발 추정값으로 표시한다.
- Localization `reset_id`, ROS clock reset 또는 관측 stamp 역행 시 전체 track을 비운다.
- Localization status가 ROS·wall 시간 모두 freshness 제한을 벗어나면 scene 처리를 보류한다.
- 빈 검출은 free-space 증명이 아니며 `free_space_valid=false`를 유지한다.

현재 LiDAR producer가 `calibration_verified=false`, `freshness_verified=false`를
명시하고 position uncertainty도 아직 bounded하지 못한다. 따라서 map geometry는
개발 진단용으로 발행하지만 `objects_verified=false`, `planner_ready=false`,
`/molit/world_model/status.stop_required=true`를 유지한다. 일반 주행 readiness로
사용하지 않는다. 관측 클러스터만 사용하는 개발 시험의 제한적 사용은
[Frenet 중앙 프로필](../ros_architecture_pkg/config/messages/frenet_runtime.yaml)에 명시한다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch world_model_pkg world_model_pkg.launch
rostopic echo /molit/world_model/scene
```

실행 전 Localization, 중앙 `base_link -> lidar_link` 정적 TF와 LiDAR Perception이
동작해야 한다. HD Map의 RViz 마커는 표시 전용이며 이 첫 구현에는 융합되지 않는다.
