# localization_pkg

> **PUBLIC INTERFACE LOCK v1.0.0:** 아래 node/topic/type은
> [`interface_contract.yaml`](../ros_architecture_pkg/config/interface_contract.yaml)의
> 읽기용 투영이다. 통합 시 정확히 일치해야 하며 이 README에서 독립 변경하지 않는다.

## 담당 범위

- GPS, IMU와 Competition Vehicle Status 기반 ego motion/pose 추정
- 승인된 HD Map landmark 및 필요 시 LiDAR map matching 제약 융합
- GPS 정상·blackout·recovery 상태 전이 및 추정 불확실성 관리
- smooth local motion과 globally referenced pose 관계 유지
- pose history, velocity, covariance, freshness와 localization quality 제공

## 담당하지 않는 범위

- 동적 객체 tracking, 경로 진행도, 행동 결정과 제어
- 체크포인트나 전역경로 좌표를 위치 센서로 주입
- sample scene ego pose를 본선 Ground Truth로 사용

## 대회 규정상 유의사항

- GPS는 최대 1대·30 Hz, IMU는 최대 1대·50 Hz다. 올해는 한시적으로 Noise를 인가하지 않는다(루트 README의 규정 발췌 근거 참조).
- GPS blackout은 예외가 아니라 반드시 지원해야 하는 운용 상태다.
- Vehicle Status에는 절대 위치와 일부 운동 상태가 제공되지 않는다.
- blackout 중 마지막 GPS 값을 새 절대 위치처럼 계속 내보내지 않는다.

## 공개 ROS 입출력

현재 상태는 **GPS/IMU 개발 추정기 및 진단 모드 구현**이며 공개 경계 노드는
`localization_node`다.

![Localization 공개 입출력](docs/interface_io.svg)

- [Mermaid 원본](docs/interface_io.mmd)
- [PNG 이미지](docs/interface_io.png)

**공개 node (exact):** `localization_node`

| 구분 | Topic | Type |
|---|---|---|
| 입력 | `/molit/sensors/gps/fix` | `sensor_msgs/NavSatFix` |
| 입력 | `/molit/sensors/imu/data` | `sensor_msgs/Imu` |
| 입력 | `/molit/sensors/lidar/points` | `sensor_msgs/PointCloud2` |
| 입력 | `/molit/vehicle/twist` | `geometry_msgs/TwistWithCovarianceStamped` |
| 입력 | `/molit/map/hd_map` | `common_msgs_pkg/HdMap` |
| 입력 | `/molit/map/status` | `common_msgs_pkg/ComponentStatus` |
| 출력 | `/molit/localization/local/odometry` | `nav_msgs/Odometry` |
| 출력 | `/molit/localization/ego_state` | `common_msgs_pkg/EgoState` |
| 출력 | `/molit/localization/status` | `common_msgs_pkg/LocalizationStatus` |

`/molit/sensors/lidar/points`는 HD Map 정합을 구현할 때만 사용하며 현재 LiDAR
transport 검증 전에는 필수 입력으로 활성화하지 않는다. `/molit/vehicle/twist`는
Competition packet 검증 전 사용 금지다. `ComponentStatus`, `EgoState`,
`LocalizationStatus` 스키마와 GPS/IMU 추정 출력이 구현됐다. 나머지
custom type은 미구현이다. 표는 승인된 전체 경계이고 현재 활성 I/O는 아래와 같다.

세 타입의 필드·단위·invalid 계약과 이식 지침은
[core messages](../ros_architecture_pkg/docs/core_messages.md)를 따른다.
`EgoState`는 허용 센서를 이용한 추정값이지 Competition Vehicle Status의
절대 위치가 아니다. 패킷에 없는 vel_y/vel_z 및 가속도를 0 관측으로 융합하지 않는다.
미추정 성분은 validity mask를 false로 두고, 모델 가정과 실제 관측을 구분한다.

Local Odometry는 연속 motion 추정이지 절대 Ground Truth가 아니다. World Model이 과거 관측을 정확한 시각의 pose로 변환할 수 있도록 bounded pose history를 제공해야 한다.

동적 `map -> odom -> base_link` 관계는 중앙 [`TF 계약`](../ros_architecture_pkg/config/tf/frame_contract.yaml)에 따라 이 패키지가 단일 소유한다. 추정값과 pose history의 시각 의미는 중앙 [`Timestamp 계약`](../ros_architecture_pkg/config/timestamp/timestamp_contract.yaml)을 따르며, GPS 수신시각과 상태 추정 유효시각을 혼동하지 않는다.

## 통합 전 자체 확인

- 노드의 통합 실행 이름이 정확히 `localization_node`인지 확인한다.
- Local Odometry를 절대 `map` 위치나 Ground Truth로 취급하지 않는다.
- `nav_msgs/Odometry`는 `header.frame_id=odom`, `child_frame_id=base_link`를 사용한다.
- `EgoState`는 map pose와 base_link 기준 motion을 분리해 보존한다.
- 위 topic/type/frame/stamp와 GPS blackout quality 전이를 중앙 계약에 맞춘다.
- 내부 topic은 `/molit/internal/localization/...` 또는 private name만 사용한다.
- 공개 이름을 remap하지 않고 중앙 계약 생성 검사를 통과시킨다.

## 디렉터리

- `config/`: filter, gate, timeout과 상태 전이 파라미터
- `docs/`: 좌표계, sensor model, blackout/recovery와 검증 근거
- `launch/`: Localization 단독 실행
- `src/`: projection, estimation, gating과 quality 구현

## 현재 런타임 상태

기본 `mode:=estimator`는 GPS/IMU만 입력으로 사용하며 EgoState(map),
Odometry(odom), LocalizationStatus와 `map → odom → base_link`를 발행한다.
IMU quaternion을 자세 관측으로 사용하고, 회전한 GPS 안테나 오프셋을 빼서
base_link 위치를 추정한다. 위치·속도·body 가속도 편향의 9-state Kalman filter이며
자세·gyro bias를 동시에 추정하는 15-state EKF가 아니다. GPS 위치 잔차로
가속도 편향도 학습하고, 여러 GPS와 IMU가 정지를 뒷받침할 때만 약한 속도 0
보정을 적용한다. 경로·체크포인트는 관측에 쓰지 않는다.

- map은 EPSG:32652에서 중앙 원점 `[302595,4124145,0]`을 뺀 좌표다.
- odom 축은 map ENU와 평행하며 위치는 초기화 후 예측 이동량만 적분한다.
  GPS 보정과 센서 기반 재배치는 odom 위치를 점프시키지 않는다. 재배치 시
  map 기준 위치와 reset_id가 바뀌고, clock/IMU 단절 reset은 odom 원점도 초기화한다.
- GPS는 60 ms reorder buffer 안에서 IMU 자세를 보간하여 측정시각에 보정한다.
  이미 처리한 상태보다 오래된 GPS나 보간 양 끝 IMU가 없는 GPS는 거부한다.
- EgoState/Odometry/동적 TF는 동일한 원본 IMU stamp를 정수 ns로 보존한다.
  상태는 wall loop에서 10 Hz 평가하며 ROS clock 정지·역행과 입력 단절을 검사한다.
- unknown GPS covariance에는 모델 오차 floor를 부여한다. 수평 위치 불확실성은
  `sqrt(var_x + var_y)`다. GPS blackout 중 covariance가 증가한다.
- 모든 개발 상태는 `stop_required=true`다. 센서 축·rear axle pivot·고도 datum의
  물리 정합과 전체 경로 정확도는 별도 검증 대상이다. IMU covariance는 브리지의
  모델 placeholder이며 실측 정확도로 해석하지 않는다.

```bash
source devel/setup.bash
roslaunch localization_pkg localization_pkg.launch mode:=estimator
# 상태만 발행하는 기존 진단 모드
roslaunch localization_pkg localization_pkg.launch mode:=diagnostic
# GPS/IMU 브리지가 실행 중일 때 정적 TF와 RViz까지 함께 시작
roslaunch system_bringup_pkg localization_visualization.launch
```

진단 모드는 중앙 TF 활성화와 무관하게 pose/TF를 발행하지 않는다. 두 모드는
같은 공개 node 이름을 쓰므로 동시에 실행하지 않는다. `src/ego_state_estimator_node.cpp`
및 15-state EKF는 이식 참고용으로 보존한다. legacy adapter는 빌드/설치하지 않는다.

검증 및 실제 실행 기록: [TF 개발 검증](docs/tf_localization_validation.md).

## GPS 음영 구간 정지 드리프트 개선

GPS 수신 중 학습한 가속도 편향을 음영 구간에서도 빼고 적분한다. 편향의
불확실성과 시간 변화도 covariance에 반영한다. 정지 보정은 새 GPS가 들어온
경우에만 적용하며, GPS가 사라지면 기존 정지 판단으로 위치를 고정하지 않는다.
등속 주행을 정지로 오판하거나 출발 가속도를 편향으로 학습하는 일을 피하기 위해서다.

처음부터 GPS가 없으면 위치를 초기화하지 않는다. GPS가 한 번만 들어왔거나
편향 학습이 부족한 상태에서는 드리프트를 제거할 수 없다. 터널 안에서 새로운
정지를 확정하려면 검증된 차속 또는 퇴화 검사를 통과한 LiDAR 등 독립 근거가 필요하다.
개발용 `stop_required=true`와 중앙 dead-reckoning 유효시간 제한은 유지한다.

[설계·대안 비교·재현 방법과 검증 결과](docs/blackout_drift.md)를 참고한다.

## 터널 주행 중 속도 추정 개선

터널 주행에서는 보정 후 가속도가 커질 때 연속시간 운동 모델 불확실성을 추가해
GPS 수신 중 속도 보정 지연을 줄인다. quiet/등속 bias 학습은 유지하고,
음영 구간에서 횡속도를 0으로 강제하거나 위치를 고정하지 않는다.
[터널 주행 개선·A/B 결과·검증 한계](docs/tunnel_motion.md)를 참고한다.
이 변경은 플래너 사용 승인이나 LiDAR 활성화를 의미하지 않는다.

## 센서 기반 위치 재설정

GPS innovation χ²가 `gps_innovation_gate_chi2: 25.0`을 넘으면 리스폰으로
간주하여 해당 GPS로 map 위치·속도·공분산을 즉시 재초기화한다. 기준 이하이면
일반 Kalman 보정을 적용한다. 여러 GPS 확인, 정지 조건, 최소 점프 거리와
대기시간은 없다. GPS blackout 복귀 시에도 같은 규칙을 적용한다.

재설정 시 odom 위치를 유지하고 `reset_id`를 증가시킨다. 다음 IMU 측정시각에
새 pose/status/TF를 발행하며 소비자는 새 epoch의 쌍으로 표시를 갱신한다.
[판정과 검증](docs/sensor_relocation.md)을 참고한다. `stop_required=true`는 유지한다.

## 위치 갱신에 맞춘 RViz 표시

추정값마다 원본 측정시각을 유지한 pose와 대응 status를 발행한다. 입력이 없을 때는 status heartbeat가 10 Hz로 동작한다.
차량 마커는 exact pose/status 쌍 수신 즉시 갱신하며 별도의 10 Hz 표시 제한을 두지 않는다.
표시 watchdog은 입력 중단·clock 이상을 계속 검사한다. RViz 렌더링 상한은 60 FPS다.
