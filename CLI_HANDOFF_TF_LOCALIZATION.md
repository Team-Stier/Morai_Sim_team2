# CLI 인계 프롬프트 — Localization·TF 활성화 및 전역경로 주행 측정

아래 작업을 현재 로컬 변경에서 이어서 완료해 주세요. 이 파일은 작업 인계 기록이며 중앙 ROS 계약을 대체하지 않습니다.

## 사용자 목표와 제한

- 작업 디렉터리: `/home/paik/Morai_Sim_team2`, ROS1 Noetic / catkin.
- **현재 브랜치 `PAIK`에서만 작업합니다. 브랜치를 바꾸지 말고, 사용자 허락 없이 push 또는 PR 생성하지 마세요.** 기존 dirty/untracked 파일을 보존하세요. 이번 작업에서는 commit도 하지 않았습니다.
- 사용자는 MORAI 센서 원점과 `base_link`의 관계, 위치축·roll/pitch/yaw 방향을 확인하고, 실제 GPS·IMU 추정기로 `map → odom → base_link`를 계산하여 활성화하길 원합니다.
- Localization의 실제 EgoState/Odometry를 받아 기존 visualization_pkg의 차량 크기 사각형을 RViz에 표시해야 합니다.
- 사용자는 직접 차량을 조작해 측정하는 것을 허용했고, **전역경로를 따라 이동하면서 측정**하라고 했습니다. 이는 아직 완료되지 않았습니다.
- 사용자는 ultra를 요청했습니다. CLI가 지원하면 시작할 때 해당 reasoning 설정을 선택하세요. 기존에 요청한 GPT-5.3-Codex-Spark는 한도 소진 후 사용자가 현재 모델로 마무리를 승인했습니다.
- AGENTS.md, 루트 README, architecture README/interface_contract, TF/timestamp/MORAI 계약과 대상 패키지 README를 먼저 읽으세요. 편집 전 fetch와 branch/worktree 확인을 수행하세요.
- 참고파일들 원본을 수정하지 마세요. 경로·체크포인트·sample scene 차량 위치·숨은 시뮬레이터 상태를 Localization 관측값/정답으로 사용하지 마세요.

## 중요한 인계 상태

2026-09-10 KST 작업 중 사용자 요청으로 병렬 작업자들을 중단했습니다. **현재 변경은 구현 도중이며 빌드·테스트·live TF 활성화 완료 상태가 아닙니다.** YAML의 enabled 값만 보고 검증 완료로 해석하면 안 됩니다.

- 기존 GPS/IMU/Camera/LiDAR 브리지와 `/vehicle_visualizer_node`, `/vehicle_rviz`가 live ROS master에서 실행 중입니다.
- 마지막 rosnode 조회에는 **localization_node와 sensor_tf_publisher가 없었습니다.** 실제 EgoState/Odometry/TF 추정 출력을 켜지 않았습니다.
- 현재 localization launch는 여전히 C++ 진단 노드만 실행합니다. 새 bringup launch는 `mode:=estimator`를 전달하지만 localization launch에는 아직 이 arg가 없습니다. 그대로 실행하면 실패할 수 있습니다.
- 새 ROS adapter `src/localization_pkg/src/localization_estimator_node.py`는 인계 시점에 **아직 생성되지 않았습니다.**
- 이전 시각화/진단 구현은 과거에 테스트했으나 이번 TF/추정기 변경 전체는 재검증하지 않았습니다.

## 센서와 좌표 기준

- 사용자 확정 장착: **IMU translation `[0,0,0]`, GPS `[0,0,1.3]` m**, 두 rotation `[0,0,0]` deg.
- 실제 저장 프로필:
  `/home/paik/MoraiLauncher_Stage/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Sensor/25.S4.MolitComp03/SensorInfo_2023_Hyundai_Ioniq5.json`
- 확인 SHA256: `1f7432b56041d5e6c47ff44155c0d96e47893aaf978ab92879125eae31a3193f`.
- GPS ID4/GPS-4, IMU ID5/IMU-5. 재개 시 hash와 활성 설정을 재확인하세요.
- LiDAR 중앙 장착 위치는 사용자 지정 `[1.43,0,1.22] m`로 갱신했습니다. LiDAR TF는 이후 사용자 좌표계 확인 및 명시적 요청으로 개발용 활성화했습니다. Camera TF는 비활성입니다.
- 기존 지도 좌표: WGS84 → EPSG:32652 → `[302595,4124145,0]` m를 뺀 simulator local map. 중앙 `config/tf/map_projection.yaml`을 새로 작성했습니다. HD map 기존 config와 정합하세요.
- GPS altitude의 map z datum 정합은 아직 개발 가정입니다. route z에 맞춰 보정값을 끼워 넣지 마세요.
- 원점 공식 근거: https://help-morai-sim.scrollhelp.site/en/morai-sim-drive/23.R1.0/-7 — 센서 위치 원점은 뒷바퀴 사이 중심.
- 축 공식 근거: https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/sensor-coordinate-system — IMU x forward, y left, z up, 양의 각속도 CCW.
- 문서의 정의와 해당 simulator build에서 직접 측정한 결과를 구분하세요. 직진만으로 roll/pitch와 전체 extrinsic을 검증 완료 처리하면 안 됩니다.

## 작성 중인 코드

1. `src/localization_pkg/src/localization_pkg/live_estimator.py`
   - 새 pure Python 6-state map position/velocity Kalman core가 작성되어 있습니다. 우선 내용을 검토하고 완성도를 확인하세요.
   - API 예정: EstimatorConfig, ImuObservation, GpsObservation, GpsImuEstimator, process_imu/process_gps/snapshot.
   - IMU quaternion을 body→world 자세 관측으로 사용하고 `R*a - gravity`로 예측합니다.
   - GPS antenna lever arm은 `p_base = p_gps - R_map_base*[0,0,1.3]`로 보정해야 합니다.
   - map 위치 GPS 보정이 continuous odom 위치를 점프시키지 않도록 분리합니다.
   - `T_map_odom = T_map_base * inverse(T_odom_base)`를 동일 측정시각에 계산해야 합니다.
   - pyproj 3.5.0을 현재 사용자 Python에 설치했습니다. numpy는 기존 설치되어 있습니다. package dependency 선언도 확인하세요.
2. `src/localization_pkg/setup.py` 및 test의 `localization_estimator.test`, `test_localization_estimator.py`가 작성 중입니다. 테스트가 요구하는 API와 실제 core를 맞추세요.
3. 기존 `localization_node.cpp`는 상태만 발행하는 진단 모드입니다. 중앙 TF가 모두 false인지 검사하던 코드라 새 계약과 충돌할 수 있습니다. 진단 모드를 보존하되 새 estimator 모드와 올바르게 분리하세요.
4. 기존 15-state `localization_ekf.*`는 첫 GPS를 자체 원점으로 삼고 GPS 이동 후 초기화하며 IMU quaternion을 쓰지 않습니다. 이것을 그대로 map/continuous odom으로 연결하지 마세요. legacy `ego_state_estimator_node.cpp`는 빌드 제외 참고 소스입니다.
5. `system_bringup_pkg/src/sensor_tf_publisher.py`, `config/localization_visualization.yaml`, `launch/localization_visualization.launch`가 새로 작성됐습니다. 단일 정적 publisher가 중앙 계약을 읽어 GPS/IMU TF만 발행하도록 검토하세요.
6. ros_architecture_pkg의 frame/extrinsic/map projection/interface/timestamp/core metadata와 validator/tests가 변경 중입니다. **개발 scope에서 odom, base_link, gps_link, imu_link 네 child만 허용**하고 `physical_alignment_verified:false`, autonomous driving 미준비를 유지하는 방향입니다. 부분 변경과 문서 모순을 찾아 완료하세요.
7. 기존 visualization_pkg는 EgoState 또는 Odometry와 LocalizationStatus를 받아 사각형을 표시합니다. 메시지 stamp/reset 일치, covariance/mask/freshness 검사와 invalid 시 삭제가 있습니다. TF를 직접 발행하지 않습니다.

## 구현해야 할 런타임 계약

- 공개 root node basename `localization_node`, 승인된 GPS/IMU 입력과 EgoState/Odometry/LocalizationStatus 출력 이름을 그대로 사용하세요.
- EgoState: frame map, child base_link, pose는 base_link의 map pose, twist는 base_link 좌표.
- Odometry: frame odom, child base_link, 연속 local pose.
- IMU 측정시각을 estimate와 두 동적 TF에 동일하게 보존하세요. 처리 완료 시각으로 덮어쓰지 마세요.
- 개발 후보: status 10 Hz, IMU freshness 0.30 s, GPS freshness 0.65 s, reorder buffer 0.06 s. 최종 중앙 계약과 구현을 일치시키고 실제 지연으로 타당성을 확인하세요.
- GPS/IMU 측정 순서, 지연 GPS의 보정 시각/자세 보간, 0·미래·중복·역행 stamp, NaN, 잘못된 quaternion/covariance, GPS blackout/recovery를 처리하세요.
- ROS clock reset 시 temporal buffers/reset_id 처리, 정지된 ROS clock과 입력 단절에 대한 monotonic wall watchdog이 필요합니다.
- covariance는 유한·대칭·PSD여야 합니다. unknown GPS covariance 0을 완벽한 관측으로 쓰지 마세요. position stddev 상태값은 중앙 정의대로 sqrt(var_x+var_y), yaw stddev는 local yaw 기준입니다.
- 개발 추정이 유효해도 `stop_required:true`를 유지하세요. TF/시각화 성공을 자율주행 준비 완료로 선언하지 마세요.
- raw UDP 제어 송신이나 임시 ROS 제어 topic으로 Controller/Safety 경계를 우회하지 마세요. 현재 대회용 제어 송신 채널은 잠겨 있습니다. 직접 측정 조작은 simulator GUI 수동 조작 범위입니다.

## 시뮬레이터 조작 중 발생한 일과 현재 상태

- **앞선 작업자가 Q로 모드를 전환하다 MORAI 내장 자동주행을 켜서 계획보다 멀리 이동했습니다.** 사용자에게 이미 보고했습니다. 이 구간을 의도한 직진/전역경로 추종 검증으로 취급하지 마세요.
- 이후 Manual-Keyboard, P 기어, 0 km/h 정지를 화면으로 확인했습니다.
- 마지막 GPS로 계산한 위치는 약 `(-90.05,-13.11)` m, IMU yaw 약126.65°, 전역경로 최근접점에서 약29.77 m 떨어져 있었습니다. 이는 과거 관측이며 재개 후 새 센서로 확인해야 합니다.
- 전역경로 시작부에 다시 배치하기 위해 GUI `Edit → Scenario → Load Scenario`를 열었습니다. 마지막 화면은 이 창에서 **Ego Vehicle Data와 Pause Mode만 체크**, 나머지 surrounding/pedestrian/object/network는 해제한 상태였습니다.
- 원본 sample scene을 수정 없이 아래에 복사했습니다:
  `/home/paik/MoraiLauncher_Stage/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/tf_route_start_sample.json`
- **시나리오 Load는 아직 누르지 않았으며 재배치도 완료하지 않았습니다.** 마지막 화면에 GNOME Activities가 끼어들었고 목록 갱신도 확인 전입니다. 화면/포커스부터 다시 확인하세요.
- sample scene은 Ego 초기 배치에만 사용하고 Localization 관측/정답으로 읽지 마세요. 센서와 네트워크 설정을 덮어쓰지 않도록 load option을 확인하세요.
- 수동 키: W 가속, S 브레이크, A/D 조향, 1 Drive, P Parking, Q 제어 모드 순환, F1 키 안내. Pause key 일시정지와 edit mode의 Escape 해제를 구분하세요.
- **Q를 무심코 순환하지 마세요.** 가능하면 명시적 GUI dropdown으로 Manual-Keyboard를 선택하고 P/0 km/h를 확인한 후 움직이세요.
- `/tmp/morai_manual_input.py`, `/tmp/morai_click.py`는 X11/XTest 조작 helper입니다. simulator window ID `0x280000b`와 화면 좌표가 하드코딩되어 있으므로 현재 window geometry/포커스를 확인하고 사용하세요. 사용자 데스크톱 조작과 충돌할 수 있습니다.
- `/tmp/tf_motion_capture.py`는 GPS/IMU ROS 구독 전용 기록기, `/tmp/tf_motion_capture.json`은 원시 기록입니다. 인계 시 해당 기록기 process는 pgrep에서 발견되지 않았습니다. 이동 측정 전에 새 기록을 시작하세요.
- `/tmp/tf_activation_sensor_audit.json`도 이전 정지 기록입니다. 성공적인 주행 측정 기록이 이미 있다고 가정하지 마세요.
- 공식 전역경로: `참고파일들/2026_molit_comp_global_path (3).txt`.

## 완료 순서와 결과 보고

1. 작업 상태와 시뮬 정지를 확인하고 부분 구현을 점검하세요.
2. ROS estimator adapter, launch mode, dependency/test 등록, 중앙 계약/producer/consumer/문서를 완성하세요.
3. synthetic 입력 검증은 **격리된 ROS master에서만** 수행하세요. live simulator master에 가짜 GPS/IMU/EgoState를 발행하지 마세요.
4. 의미 있는 core·ROS·consumer 테스트, catkin build, launch XML/YAML, TF 단일 parent/acyclic/단일 publisher, timestamp/reset/watchdog를 검증하세요.
5. `python3 src/ros_architecture_pkg/scripts/generate_interface_diagrams.py --check`를 통과시키고 필요한 Mermaid/SVG/PNG/manifest를 재생성하세요.
6. 기존 sensor bridge와 RViz가 이미 실행 중임을 고려해 중복 노드 없이 실제 estimator와 static TF를 활성화하세요.
7. 전역경로 시작부 배치 후 실제 GPS 위치를 확인하고, 경로를 따라 저속 이동·정지하며 GPS 변위 heading과 IMU yaw/각속도 부호를 비교하세요. 짧고 관찰 가능한 구간부터 진행하세요. 완료하지 않은 경로 추종/회전/roll-pitch 검증을 완료로 보고하지 마세요.
8. 실제 TF 합성식, 원본 stamp 보존, EgoState/Odometry/상태 callback, RViz 차량 위치·방향을 검증하고 화면을 확인하세요. RViz가 map 원점만 보고 있으면 실제 차량 위치로 시점을 맞추세요.
9. 사용자에게 실측 수치, 활성화된 TF, 남은 물리 검증/주행 기능 제한과 테스트 결과를 구분해 한국어로 보고하세요. 끝날 때 차량은 정지·P 상태를 확인하세요. push하지 마세요.
