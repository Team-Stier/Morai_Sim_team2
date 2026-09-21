# LiDAR roll/pitch 수평화

## 출처와 적용 범위

- 원본: [Team-Stier/E2E-OpenLoop-ROS-Validator](https://github.com/Team-Stier/E2E-OpenLoop-ROS-Validator/tree/f067fde6deb0dd7256be632149344b4f25159fd3/src/Validator_pkg/horizontal_pkg)
- branch: `horizontal_pkg/Paik`, commit: `f067fde6deb0dd7256be632149344b4f25159fd3`.
- 원 함수: `lidar_horizontal.py:24`의 `rotation_matrix_xyz`, `:50`의
  `level_rotation`, `:82`의 스캔 시각 보간 정책.
- 이식 구현: `src/horizontalization.{h,cpp}`, 실시간 연결:
  `src/lidar_perception_node.cpp`. `paik`의 기존 DBSCAN 경로에 적용한다.

원본은 전체 rosbag의 IMU roll/pitch를 4 Hz 2차 Butterworth zero-phase
필터로 처리한다. 미래 샘플을 필요로 하는 오프라인 필터는 실시간에 이식하지
않는다. 현재 Localization의 유효 quaternion을 스캔 시각에 SLERP하여 원본의
회전식을 적용한다. 원본의 Euler 각 선형 보간 대신 quaternion 보간을 사용해
yaw ±π 경계와 q/-q 표현을 처리한다. 별도 저역통과 필터는 적용하지 않는다.

## 좌표와 처리 순서

`R_level_from_lidar = Rz(-yaw) × R_map_from_body × R_body_from_lidar`.
이는 `Ry(pitch) × Rx(roll) × R_body_from_lidar`이며 원본의 회전 방향과 같다.
`EgoState`는 이미 base_link 자세이므로 raw IMU 장착각을 다시 적용하지 않는다.
중앙 static TF에서 LiDAR 장착 회전만 읽는다. LiDAR 원점을 유지하는 회전이므로
장착 위치 `(2,0,1.5) m`를 점군에 더하지 않는다.

1. 원본 `lidar_link` scan + 같은 시각의 localization attitude.
2. roll/pitch 수평화. yaw를 0 방향으로 정렬하는 처리는 하지 않는다.
3. 수평 좌표에서 ROI X `[-20,50]`, Y `[-15,15]`, Z `[-1.5,1]` m 적용.
4. VoxelGrid → DBSCAN → 수평 좌표의 AABB.
5. AABB 모서리와 필터 점군을 원래 lidar_link로 역회전하여 기존 관측/디버그 출력.

역회전 AABB는 8개 모서리를 모두 포함하므로 실제 객체 치수보다 커질 수 있다.
RViz Fixed Frame이 `lidar_link`이면 지면은 여전히 센서 기준으로 기울어져 보일
수 있다. 출력 좌표를 수평으로 잘못 재명명하지 않기 때문이다. 수평화는
ROI·군집화에 적용됐으며 Fixed Frame `odom`에서는 측정시각 TF로 world 수직축에
맞춰 보인다. 실제 도로 경사 자체를 평면으로 만드는 기능은 아니다.

## 제동 시 바닥 유입과 z_min

제동으로 차체 앞쪽이 숙여지면 센서 좌표의 바닥 z가 ROI 안으로 들어올 수 있다.
수평화는 **ROI 전에** 수행하므로 기울기로 인한 유입을 줄인다. `z_min=-1.5`는
LiDAR 원점에서 수평 방향으로 1.5 m 아래를 의미하며 지면으로부터의 높이가 아니다.
경계값은 포함되므로 보정 후 바닥이 정확히 -1.5 m이거나 그보다 높으면 여전히
군집 후보가 된다. 차체 높이 변화·실제 경사로·지면 분리는 별도 문제다.
설정값을 바꿀 때 낮은 장애물까지 제거되지 않는지 확인해야 한다.

RViz의 원본 군집 점은 sensor frame 좌표를 보존한다. 수평으로 돌아간 좌표를
발행하는 기능이 아니라, 수평 좌표에서 어느 점을 검출에 사용할지 판정하는 기능이다.
`leveling_enabled`의 실행 기본값은 launch의 true이며 사전학습 backend에는 적용되지 않는다.

## 시간 및 실패 처리

중앙 `messages/lidar_runtime.yaml#horizontalization`:
보간 양 끝 간격 최대 0.10 s, 스캔 대기 최대 0.25 s, 자세 이력 2 s,
대기 스캔 최대 3개, 자세 wall timeout 0.30 s. 개발용 기본값이며 실제 주행
타이밍의 검증값은 아니다. sensor ingress stamp를 처리 완료 시각으로 바꾸지 않는다.

- 정확히 같은 시각의 자세 또는 양쪽 유효 샘플 사이의 보간만 허용한다.
- 외삽, 최신 자세의 무기한 재사용, 미보정 fallback을 하지 않는다.
- reset_id 변경, 잘못된 frame/validity/quaternion, 비단조 자세 시각,
  ROS clock 역행은 이력 및 대기 스캔을 비운다.
- 시간 정렬 실패·큐 초과·mount 누락은 invalid empty observation과 오류 상태로 알린다.
- 단일 회전이므로 스캔 내 개별 포인트 시각에 따른 motion deskew는 포함하지 않는다.
- calibration_verified/freshness_verified=false, ready=false, stop_required=true.

## 실행

GPS·IMU bridge와 `system_bringup_pkg localization_visualization.launch`의
Localization 및 중앙 static TF publisher가 먼저 실행돼 있어야 한다.
LiDAR bridge/watchdog도 필요하며 기존에 실행 중이면 중복 실행하지 않는다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
# 기존 lidar_perception_node 실행을 중단한 뒤 하나만 실행한다.
roslaunch lidar_perception_pkg lidar_perception_pkg.launch
# 미보정 비교 실행:
# roslaunch lidar_perception_pkg lidar_perception_pkg.launch leveling_enabled:=false
```

공개 출력과 private `/lidar_perception_node/filtered_points`는 원본 `lidar_link`
및 scan stamp를 유지하므로 기존 RViz 설정을 계속 사용할 수 있다.

## 검증

`test_horizontalization.cpp`는 비영 장착각, roll/pitch 동시 회전, yaw 유지,
수평 평면 복원, 역회전 박스 모서리 포함, quaternion 경계·외삽 거부를 검사한다.
`horizontalization.test`는 EgoState와 LiDAR 합성 producer로 스캔보다 늦게
도착하는 자세, ROI 복원, 기존 frame/stamp 보존, 단절과 reset을 검증한다.
## 실행 검증 기록 (2026-09-14)

- `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j4 -l4` 전체 빌드 성공.
- LiDAR, common_msgs, Localization, Visualization 패키지 테스트 통과.
  기존 시각화 연동 테스트도 EgoState·sensor mount를 공급하도록 갱신했다.
- 중앙 계약 unittest 46개 및 `generate_interface_diagrams.py --check` 통과.
- 처음 Localization 테스트는 `PYTHONNOUSERSITE=1` 때문에 사용자 영역에
  설치된 pyproj를 읽지 못했다. 실제 실행과 같은 사용자 Python 환경으로
  다시 실행하여 통과했다. 시스템 패키지를 추가 설치하지 않았다.
- MORAI 10초 수신: raw 73, filtered 73, observations 73, EgoState 429.
  관측 73개 모두 valid, 매 프레임 군집 3개. 상태 누적 439개 처리 시점에
  invalid/dropped count는 모두 0이었다. 이 결과는 객체 인식 정확도 점수가 아니다.
- 자세 마지막 표본: roll −0.08365°, pitch −0.53392°, yaw 152.11060°.
  sample의 보간 양쪽 간격은 41.47 ms, 필터 점군 110개가 수평 좌표 ROI 안에
  있음을 역변환/재변환으로 확인했다. 동일 raw 스캔의 ROI 내 점은 미보정
  125개, 수평화 후 126개였다. 현재 기울기가 작아 시각적 차이도 작다.
- 원 scan stamp부터 관측 수신까지 p50 133.7 ms / p95 145.1 ms.
  이는 드라이버 스캔 취합·정렬·검출·ROS 전달을 포함한다. 마지막 검출 콜백
  계산 시간은 32.9 ms다. 원본 수신과 대응된 72개 출력은 stamp가 일치했고,
  수집 시작/종료 경계의 1개 관측은 raw 대응 표본이 없었다.
- RViz의 raw/ROI/박스 갱신을 확인했다. 개발 안전 상태는 그대로다.

로컬 증거: `/home/paik/morai-artifacts/lidar-horizontalization-20260914/live-audit.json`,
같은 디렉터리의 `rviz.png`. 큰 경사에서의 부호·물리 장착 정합과 실제 주행
정확도는 이번 정지/소각도 수신 검사로 검증된 것이 아니다.

## paik 브랜치 재검증 (2026-09-21)

기존 검출 전 수평화 구현을 유지하고 중앙 장착 TF 조회도 원 scan stamp로
통일했다. 중앙 계약과 AGENTS의 소유권 예외, 현재 ROI 설명을 맞췄다.

- 전체 `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j4 -l4` 성공.
- LiDAR·공유 메시지·Visualization 패키지 테스트 통과. catkin 결과 집계는
  각각 38/34/32 tests, errors/failures/skipped 모두 0이다.
- 중앙 TF·timestamp·공개 계약 unittest 46개와 다이어그램 `--check` 통과.
  graph와 그림은 동일하며 manifest의 중앙 계약 해시만 갱신했다.
- 제동 합성 단위 테스트: roll 0.08 rad, pitch 0.12 rad에서 미보정 검출은
  바닥과 장애물 2개 군집, 보정 검출은 장애물 1개 군집이다. 바닥은 수평 기준
  z=-1.65 m로 하한 -1.5 m보다 낮게 설정했다. 경계 안의 지면 제거를 입증하지 않는다.
- ROS producer-consumer 테스트에서도 roll/pitch 보정 후 바닥 인덱스 제외,
  장애물 원본 XYZ 바이트·source_index·lidar_link·scan stamp 보존을 확인했다.
- YAML·launch XML 파싱 및 git diff 공백 검사 통과.

이번 검증은 합성 입력이며 실제 MORAI 제동·경사로·낮은 장애물 closed-loop
주행은 수행하지 않았다. 실행 중인 노드를 재시작하거나 UDP/제어를 변경하지 않았다.
빌드·테스트 로그는 `/home/paik/morai-artifacts/lidar-leveling-20260921/`의
`paik-build.log`, `paik-tests.log`에 보관했다.
