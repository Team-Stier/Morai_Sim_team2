# HD맵 위 원본 LiDAR 군집 점 표시

## 데이터 경로

`lidar_perception_node`는 측정시각 EgoState의 roll/pitch로 임시 수평화한 점에
구역별 지면 제거 → ROI → voxel → DBSCAN을 적용한다. 검출용 voxel 중심점마다 원본 입력 인덱스를
유지하고, 채택된 군집의 원본 PointCloud2 레코드만 복사한다. 기본 ROI는
X -20~50 m, Y -15~15 m, Z -1.5~1 m이며 임시 수평 좌표 기준이다.

공개 출력 `/molit/perception/lidar/cluster_points`의 이름·소유자·소비자·시각 정책은
`ros_architecture_pkg/config/interface_contract.yaml`과 timestamp 계약이 원본이다.
프레임은 `lidar_link`, 시각은 원본 측정 stamp이며 XYZ/intensity 등 기존 필드와
각 point 레코드 바이트를 그대로 보존한다. 새 UINT32 필드 `cluster_id`와
`source_index`만 추가한다. `source_index`는 원본 organized cloud의 행 우선 인덱스다.
출력은 height=1로 모으므로 원본 행 사이 padding은 복사하지 않는다.
ROS publisher별로 달라질 수 있는 header.seq를 측정 ID로 사용하지 않는다.

- 내부 voxel 집계는 PCL과 같은 전역 격자 및 x/y/z 순서를 사용한다.
  누적 평균은 double로 계산하므로 기존 PCL float 누적과 미세한 차이가 날 수 있다.
  대표 입력에서 PCL 중심점과 1e-6 m 이내 일치하는 단위 테스트를 포함한다.
- 복원할 때 역회전한 voxel 평균점이나 최근접 중심점 배정을 사용하지 않는다.
  자차·지면·ROI 제외점, DBSCAN noise 및 크기 제한으로 탈락한 군집은 출력하지 않는다.
- 군집 ID는 스캔 내 번호다. 색상은 객체 종류나 시간에 걸친 추적 ID가 아니다.
- 처리 실패/빈 결과에는 빈 cloud를 보내 기존 화면을 지운다. 기존 observations
  출력과 private filtered_points 진단은 유지된다. filtered_points는 voxel 중심점이다.

## RViz와 TF

`vehicle_visualizer_node`의 기본 `lidar_display_mode: points`가 새 출력을 구독한다.
private `/molit/internal/visualization/lidar_markers`에 POINTS marker를 발행하며
원본 XYZ, 원본 stamp/frame, identity pose, `frame_locked=false`를 사용한다.
RViz **LiDAR raw cluster points**, Fixed Frame **map**에서 다음 변환을 적용한다.
점 표시 크기는 `vehicle_display.yaml`의 `lidar_point_size_m: 0.18`로 설정한다.
이 크기는 가시성 조절용이며 원본 점 좌표나 실제 장애물 크기를 바꾸지 않는다.

```text
p_map(t) = T_map_odom(t) × T_odom_base_link(t) × T_base_link_lidar_link × p_raw
```

TF는 Localization과 승인된 sensor TF publisher가 소유한다. 시각 0의 최신 TF로
대체하지 않으며 센서 장착 offset을 두 번 적용하지 않는다. 정확한 스캔 시각 TF가
없으면 기다리고 stale, ROS clock 정지/역행, Localization reset에서 표시를 삭제한다.
지도는 기존 표시 정책에 따라 z=-0.10 m로 평면화하지만 LiDAR 점 높이는 변경하지 않는다.
따라서 TopDownOrtho 화면의 XY 중첩은 확인할 수 있고 지도 원본과의 3차원 높이 일치는
이 표시로 증명하지 않는다. 물리적 센서 장착과 localization 절대 오차도 별도 검증 대상이다.

이미 센서 bridge, Localization, sensor TF가 실행 중이면:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch lidar_perception_pkg lidar_perception_pkg.launch
# 별도 터미널, 동일 setup 적용
roslaunch visualization_pkg visualization_pkg.launch
```

같은 이름의 노드를 중복 실행하지 않는다. 전체 bringup 소유자는 system_bringup_pkg다.
비교용 legacy `lidar_display_mode: boxes`만 기존 observations 박스를 사용한다.

## 검증 기록 — 2026-09-14

catkin build 성공. catkin_test_results 기준 LiDAR 36, visualization 30건 모두
오류/실패/skip 0건. 중앙 아키텍처 unittest 46건 및 interface diagram check 통과.

- 단위: voxel 원본 소속 관계, noise/ROI 제외, organized row stride와 point bytes,
  intensity/추가 필드 보존, 빈 입력/잘못된 입력 처리.
- 표시: POINTS 원본 좌표/identity pose, scan stamp TF 요청, TF 대기,
  stale/clock 정지·역행/Localization reset/잘못된 메시지 삭제.
- producer-consumer 통합: 실제 C++ 검출기 → raw cluster cloud → Python visualizer → TF.
  scan 전후 차량 위치를 다르게 넣어 최신 TF와 측정시각 TF가 1 m 차이나게 만들고,
  별도 회전·이동 식과 지도 좌표가 1e-6 m 이내임을 확인했다.

실제 MORAI에서 13초 동안 읽기 전용 수집 후 원본/군집/marker/EgoState를 시각으로 대조했다.

| 항목 | 결과 |
|---|---:|
| 원본 / 군집 수신 스캔 | 100 / 100 |
| 교차검증 스캔 / 점 | 99 / 9,292 |
| 원본 point bytes / 필드 보존 | 전부 일치 |
| 원본 stamp / frame 보존 | 전부 일치 |
| 군집 cloud XYZ와 RViz marker XYZ | 전부 일치 |
| LiDAR CUBE marker | 0 |
| TF 대 별도 EgoState 보간 + 중앙 extrinsic 계산 최대 오차 | 8.380175e-9 m |
| 캡처 경계 때문에 제외한 스캔 | 1 |

별도 계산은 EgoState 위치 선형보간 + quaternion SLERP와 중앙 장착값
[2.0, 0.0, 1.5] m, RPY [0, 0, 0]을 사용했다. TF 계산과 같은 추정 입력을 사용하므로
위 수치는 **소프트웨어 좌표/시각 일관성 오차**이지 물리적 장애물 위치 정확도가 아니다.
RViz 실화면에서 Global Status OK, HD맵·차량·LiDAR 점 동시 표시를 확인했다.

호스트 검증 산출물: `/home/paik/morai-artifacts/lidar-cluster-map-20260914/`
(`crosscheck.py`, `results.json`, `rviz.png`). 수집 스크립트는 발행/제어 없이 읽기만 수행한다.
원본 Simulator Ground Truth나 Bounding Box 입력은 사용하지 않았다.

## 자차 반사점 제외 (2026-09-21)

수평화 전에 센서 좌표의 자차 영역을 제외한다. 살아남은 점의 원본 인덱스와
XYZ/intensity는 보존하며 원본 센서 토픽은 변경하지 않는다. 제외 범위와
후보 높이의 한계는 [패키지 README](../README.md#자차-반사점-제외-2026-09-21),
설정은 `config/detector.yaml`의 `self_filter`를 따른다.
