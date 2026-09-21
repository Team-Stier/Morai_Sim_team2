# lidar_perception_pkg

> **PUBLIC INTERFACE LOCK v1.0.0:** 아래 node/topic/type은
> [`interface_contract.yaml`](../ros_architecture_pkg/config/interface_contract.yaml)의
> 읽기용 투영이다. 통합 시 정확히 일치해야 하며 이 README에서 독립 변경하지 않는다.

## 담당 범위

- point cloud 유효성 검사, ROI와 지면 분리
- 3D 장애물·객체 군집화, 크기·상대 위치·속도 관측
- free-space와 occupancy 관측
- 측정 timestamp, calibration ID, confidence와 입력 freshness 제공

## 담당하지 않는 범위

- Camera와의 최종 융합, 전역 객체 추적과 planner-ready world model
- 행동 결정, trajectory와 차량 명령
- 시뮬레이터 UDP 직접 수신

## 대회 규정상 유의사항

- 3D LiDAR는 최대 1대이며 `VLP16`, Intensity 방식만 허용된다.
- 회전율은 최대 15 Hz이고 공지 권장은 10 Hz 이하이다.
- 저장소의 제공 Camera 설정에는 LiDAR가 없다. 로컬 MORAI 저장 프로필에서 확인된 LiDAR 위치는 활성 loadout 검증 전까지 후보값으로만 사용한다.
- sample scene의 객체 목록이나 Ground Truth를 검출 결과로 사용하지 않는다.

## 공개 ROS 입출력

현재 상태는 **기존 객체 검출부 개발 구현, ROS/MORAI 실행 검증 대기**다. 공개 경계 노드는
`lidar_perception_node`다.

![LiDAR Perception 공개 입출력](docs/interface_io.svg)

- [Mermaid 원본](docs/interface_io.mmd)
- [PNG 이미지](docs/interface_io.png)

**공개 node (exact):** `lidar_perception_node`

| 구분 | Topic | Type |
|---|---|---|
| 입력 | `/molit/sensors/lidar/points` | `sensor_msgs/PointCloud2` |
| 입력 | `/molit/sensors/lidar/status` | `std_msgs/Bool` |
| 입력 | `/molit/localization/ego_state` | `common_msgs_pkg/EgoState` |
| 출력 | `/molit/perception/lidar/observations` | `common_msgs_pkg/LidarObservationArray` |
| 출력 | `/molit/perception/lidar/cluster_points` | `sensor_msgs/PointCloud2` |
| 출력 | `/molit/perception/lidar/status` | `common_msgs_pkg/ComponentStatus` |

공유 타입 중 `ComponentStatus`, `EgoState`, `LocalizationStatus`와
`LidarObservationArray`, `LidarObjectObservation` 스키마가 구현됐다.
해당 타입을 사용하는 공개 I/O는 [기반 메시지 계약](../ros_architecture_pkg/docs/core_messages.md)을 따른다.
이 패키지의 ROI·VoxelGrid·DBSCAN 노드는 구현됐으며 지면·빈 공간·속도 추정은
이번 객체 검출 범위에 포함하지 않는다. downstream 런타임은 아직 미구현이다.

기본 DBSCAN 경로는 스캔 시각의 EgoState 자세로 roll/pitch를 수평화한 뒤
ROI·VoxelGrid·군집화를 수행한다. 현재 ROI는 X `[-20,50]`, Y `[-15,15]`,
Z `[-1.5,1]` m이며 센서 원점의 임시 수평 좌표 기준이다. `z_min` 경계는 포함되며
수평화는 지면 제거가 아니다. EgoState는 자세 전처리에만 사용하며 전역
객체 융합·추적을 수행하지 않는다. 출력 박스와 private `filtered_points`는
원래 `lidar_link`로 역변환한다. 자세 입력이 없으면 보정 없이 진행하지 않고
invalid 관측을 발행한다. 실행·좌표·시간 정책은 [수평화](docs/horizontalization.md)를 따른다.
보정 전후 기울기·높이 편차·형상 보존·처리율은
[수평화 정량 평가](docs/horizontalization_metrics.md)의 읽기 전용 도구로 측정한다.

DBSCAN이 채택한 voxel에 속한 원본 점들은 `cluster_points`에 별도로 발행한다.
XYZ·intensity 등 원본 record의 바이트와 scan stamp/frame을 보존하고,
`cluster_id`, `source_index`(원본 row-major 인덱스)를 UINT32로 덧붙인다.
Voxel 평균점이나 박스 모서리를 원본 점처럼 내보내지 않는다. 상세 경로와
교차검증은 [원본 군집 점 표시](docs/cluster_points.md)를 따른다.

오래된 장애물을 현재 관측처럼 유지하지 않고, sparse VLP16 환경에서의 miss와 uncertainty를 명시한다.

LiDAR frame과 후보 장착 위치는 중앙 [`TF 계약`](../ros_architecture_pkg/config/tf/frame_contract.yaml)을 따른다. 장착 위치 `(2.0, 0.0, 1.5) m`와 축에 대한 사용자 승인으로 개발용 TF가 활성화됐으며, 발행은 `system_bringup_pkg`가 소유한다. 물리 정합 실측 검증은 별도다. 출력 관측은 [`Timestamp 계약`](../ros_architecture_pkg/config/timestamp/timestamp_contract.yaml)에 따라 원본 scan의 측정시각을 유지한다.

## 통합 전 자체 확인

- 노드의 통합 실행 이름이 정확히 `lidar_perception_node`인지 확인한다.
- 위 입력과 출력의 topic/type/frame/stamp가 중앙 계약과 일치해야 한다.
- 내부 topic은 `/molit/internal/lidar_perception/...` 또는 private name만 사용한다.
- 공개 이름을 remap하지 않고 중앙 계약 생성 검사를 통과시킨다.

## 디렉터리

- `config/`: ROI, filter, clustering과 모델 로컬 파라미터
- `docs/`: calibration, 데이터 특성, 알고리즘과 평가 근거
- `launch/`: LiDAR Perception 단독 실행
- `src/`: point cloud 처리와 observation 생성 구현

## LiDAR 검출부 개발 구현 (2026-09-10)

`LidarObservationArray`, `LidarObjectObservation` 필드와 검출 노드는 개발 구현 상태다.
중앙 [LiDAR 계약](../ros_architecture_pkg/docs/lidar_detection_contract.md)과
[검출부 실행·검증](../lidar_perception_pkg/docs/legacy_port.md)을 따른다.

현재 MORAI 수신 가능 조건과 수정 내역은
[시뮬레이터 입력 점검](docs/sim_input_review.md)에 기록한다.
단독 검출 launch 외에 LiDAR UDP bridge와 watchdog을 별도로 실행해야 한다.

## 사전학습 보행자·차량 검출

`roslaunch lidar_perception_pkg learned_lidar.launch`로 공식 nuScenes
PointPillars-MultiHead 가중치를 사용하는 대체 backend를 선택한다. 기존
DBSCAN launch와 동시에 실행하지 않는다. 설치·클래스·ROI·오류 처리·검증
범위는 [사전학습 모델 연결](docs/pretrained.md)을 따른다.

보행자와 차량 계열을 분류하며 성별·성인 여부는 판별하지 않는다. raw XYZI만
사용하고 scenario JSON의 정답을 읽지 않는다. 단일 VLP16 scan의 분포 차이로
정확도는 미검증이며 주행 readiness는 계속 false다.
