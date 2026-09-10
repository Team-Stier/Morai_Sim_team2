# Localization 차량 표시 계약

사용자가 요청한 “로컬리제이션값을 받아 실제 차량과 같은 네모를 RViz에 표시”를
위해 `visualization_pkg`의 읽기 전용 입력 경계를 등록한다. 승인 범위는
이 문서의 최초 범위는 Localization 표시와 RViz 실행이다. 이후 사용자 요청으로
GPS/IMU 개발 추정·TF를 별도 중앙 계약에 추가했으며, 제어 기능은 포함하지 않는다. 이름과 의미의 원본은
[`interface_contract.yaml`](../config/interface_contract.yaml)이다.

## Producer와 consumer 영향

| Producer | Topic | Type | 추가 consumer |
|---|---|---|---|
| `localization_node` | `/molit/localization/ego_state` | `common_msgs_pkg/EgoState` | `vehicle_visualizer_node` |
| `localization_node` | `/molit/localization/local/odometry` | `nav_msgs/Odometry` | `vehicle_visualizer_node` |
| `localization_node` | `/molit/localization/status` | `common_msgs_pkg/LocalizationStatus` | `vehicle_visualizer_node` |
| `vehicle_visualizer_node` | `/molit/internal/visualization/vehicle_markers` | `visualization_msgs/MarkerArray` | `vehicle_rviz` |

기존 Localization 공개 topic·메시지·주기·producer는 유지하고 consumer만 추가한다.
`vehicle_visualizer_node`는 root namespace의 공개 경계 node이며 공개 출력은 없다.
`vehicle_rviz`와 MarkerArray는 `visualization_pkg` 내부 경계다. 다른 패키지는
이 표시 출력을 위치 추정이나 제어 입력으로 구독하지 않는다. 표준
`visualization_msgs/MarkerArray`를 재사용하므로 공유 custom message 변경은 없다.

## Frame과 시간

- `EgoState`는 `map` 기준의 `base_link` 자세, Odometry는 `odom` 기준의
  `base_link` 자세로 해석한다. 선택된 입력과 같은 frame으로 차량을 표시하며
  `map`과 `odom`을 같은 좌표계로 간주하거나 이름만 바꾸지 않는다.
- 차체 표시 중심의 로컬 offset은 입력 자세의 회전으로 변환한 뒤 위치에 더한다.
  입력의 quaternion을 유지하며, 차량 중심을 GPS 안테나 위치로 대체하지 않는다.
- 차량 geometry의 stamp는 원본 추정값 유효시각을 유지한다. 대기 안내와 삭제의
  stamp만 ROS 표시 평가시각을 사용한다. 이는
  [`visualization_marker_time`](../config/timestamp/timestamp_contract.yaml)의
  표시 전용 규칙이며 운행 입력 freshness의 별도 원본이 아니다.
- 유효하지 않거나 오래된 입력, Localization reset 또는 입력 부재 시 차량
  geometry를 지우고 대기 안내를 표시한다. 이전 위치를 현재 위치처럼 표시하지 않는다.
- Visualizer는 TF를 발행하지 않는다. 중앙
  [`frame_contract.yaml`](../config/tf/frame_contract.yaml)의 미검증 TF 발행
  gate를 따르며 개발용 odom/base_link/gps_link/imu_link만 발행 가능하다.
  Localization·Odometry·TF의 구현 소유권은 유지한다.

## 차량 치수와 표시 원점의 근거

[MORAI SIM Drive 24.R2 공식 Vehicle Specification](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/vehicle-specification)의
`2023_Hyundai_Ioniq5` 항목은 다음 값을 명시한다. 저장소 루트 README의 차량
제원과 일치하며, 이 값은 현재 `25.S4.MolitComp03` 차량 mesh를 직접 측정한
결과가 아니다.

| 항목 | 값 (m) |
|---|---:|
| 길이 | 4.635 |
| 너비 | 1.892 |
| 높이 | 2.434 |
| 축거 | 3.000 |
| 전방 오버행 | 0.845 |
| 후방 오버행 | 0.790 |

같은 공식 문서는 차량 기준을 뒷바퀴 사이 중심으로 설명하며 그림에서 원점을
뒷바퀴 hub 높이에 표시한다. 이 기준과 `base_link`가 일치한다고 가정하면
차체의 x 범위는 `[-0.790, 3.845]`, y 범위는 `[-0.946, 0.946]`이고
기하 중심의 x offset은 `(3.845 - 0.790) / 2 = 1.5275 m`다.

그러나 현재 중앙 계약의 `base_link` 원점은
`provisional_morai_sensor_target_pivot`이다. MORAI 센서 target pivot과 위 차량
기준의 일치는 아직 검증하지 않았다. 따라서 표시 offset `[1.5275, 0, 0]`은
시각화용 후보이며 TF나 Localization 원점의 검증 완료를 의미하지 않는다.

기본 표시는 길이·너비를 유지한 **두께 0.06 m의 평면 사각형 footprint**다.
차체 좌표계의 중심 z offset은 0이며 입력 자세로 회전·이동한다. 지면으로
투영하거나 도로면 높이를 추정하지 않는다. 선택 가능한
box 표시는 문서상 전체 높이 2.434 m를 사용하지만, 중심 z offset 역시 명시적인
시각화 후보 파라미터다. 축 중심이 지면보다 높으므로 `높이 / 2`만으로 정확한
수직 정합이 확보되었다고 볼 수 없다.

[현대자동차의 양산 IONIQ 5 제원](https://www.hyundai.com/kr/ko/brand/brandstory/model/ioniq-history/2021-ioniq5)은
너비 1.890 m·높이 1.605 m로 MORAI 표와 다르다. 시뮬레이터 표시에는 MORAI
문서의 값을 사용하고, 양산차 치수로 임의 교체하지 않는다.

## 현재 연결 범위와 검증

현재 Localization 진단 실행 파일은 GPS/IMU 입력 검사와 `LocalizationStatus`
발행만 수행한다. `EgoState`와 Odometry의 유효한 추정값은 아직 발행하지 않으므로
실제 시뮬레이터 연결에서 차량 대신 입력 대기 안내가 나오는 것이 현재 동작이다.
시각화를 위해 Ground Truth, sample scene 또는 고정 경로를 차량 위치로 사용하지 않는다.

검증은 중앙 producer-consumer 계약·README·다이어그램 정합, pose 회전에 따른
offset 변환, frame/stamp 유지, invalid·stale·reset 시 삭제와 RViz 실행을
포함한다. 합성 입력 테스트는 별도 ROS master에서 수행해야 하며 실제 센서
master에 가짜 Localization 또는 clock을 주입하지 않는다. RViz 실행 성공과
합성 입력 표시는 실제 시뮬레이터 Localization 정확도 검증과 구분한다.
