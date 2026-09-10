# GPS/IMU 개발 Localization·TF 검증 (2026-09-10)

PAIK의 기존 dirty/untracked 변경을 이어 구현했다. push·commit은 수행하지 않았다.

## 구현과 계약

`localization_estimator_node.py`가 GPS/IMU만 구독하여 6-state map position/velocity
Kalman filter를 실행한다. quaternion 자세는 IMU 관측이며 bias를 추정하지 않는다.
`p_base = p_gps - R_map_base * [0,0,1.3]`를 적용한다. map 투영은
EPSG:32652 및 `[302595,4124145,0]` m로 기존 HD Map 설정과 일치한다.
GPS 높이 datum은 개발 가정이며 경로 고도로 맞추지 않았다.

동적 TF는 localization_node가 map→odom, odom→base_link만 발행한다.
정적 TF는 sensor_tf_publisher가 base_link→gps_link, base_link→imu_link만
발행한다. Camera/LiDAR, Vehicle Status와 제어 채널은 활성화하지 않았다.

GPS는 60 ms reorder buffer와 IMU 양 끝 자세 보간을 사용한다. 오른쪽 IMU가
늦으면 최대 중앙 integration bound까지 기다리고, 너무 오래된 관측은 거부한다.
IMU freshness 0.30 s, GPS 0.65 s, estimate 0.30 s, clock stall 0.50 s는
중앙 개발 profile에서 직접 읽는다. 메시지는 원본 IMU 정수 ns stamp를 보존한다.
상태는 wall loop에서 10 Hz로 발행하며 clock 정지에도 invalid를 전달한다.
입력 큐·IMU 이력은 bounded이고 clock 역행에 clear/reset_id 증가를 수행한다.

C++ 진단 모드와 legacy EKF는 보존했다. 진단 모드는 TF 활성 여부와 관계없이
status만 발행한다. 기본 launch는 estimator이며 `mode:=diagnostic`으로 선택한다.

## 실행 근거

- 저장 GPS/IMU 프로필 SHA256:
  `1f7432b56041d5e6c47ff44155c0d96e47893aaf978ab92879125eae31a3193f`.
- 재개 시 ROS master/브리지가 없어 GPS/IMU 수신을 다시 시작했다.
- 정지 12초 입력 기록: IMU 512개, GPS 60개. 간격 p99 약 94.5/266.1 ms.
  [원시 기록 해시와 통계](evidence/sensor_timing_20260910.json).
- 정지 IMU의 `R*a - g` 중앙값은 대략 `[-0.000015,-0.000021,0.00337] m/s²`.
  norm p99 0.00473 m/s². 중력 제거 정합의 정지 관측이며 전체 축 검증이 아니다.
- 실제 출력 15초: EgoState/Odometry 각 625개, 상태 152개가 유효하고 모두
  stop_required=true. 원본 IMU·Odometry·두 TF가 일치하는 619개 묶음의
  합성 위치 오차 최대 `3.55e-15 m`. 구독 시작/끝 경계의 미매칭은 제외했다.
- 이 기록의 출력 지연 p50/p95/p99는 136/159/175 ms이다. 이후 처리 loop를
  상태 10 Hz와 분리해 100 Hz로 변경했고 입력 기반 발행·원본 stamp는 유지한다.
  [출력·그래프 기록](evidence/tf_live_stationary_20260910.json).

## 자동 검증

- 전체 catkin build 성공 (`/usr/bin/python3`, `-j4`).
- catkin 결과: Localization 24, visualization 15, static TF 2 항목, 실패 0.
  catkin 합계에는 rostest wrapper 항목도 포함된다.
- ROS 추정기 5 시나리오: TF 합성/실제 표시 consumer, 잘못된 입력,
  clock 정지·reset, GPS blackout·recovery, 원본 ns·지연 GPS 거부.
- C++ EKF 4, Python core 3, 진단 ROS 4, 표시 ROS 7 시나리오 포함.
- 중앙 계약 47개 중 46 통과, 과거 host profile 경로 부재 1 skip.
  현재 사용자가 승인한 profile 해시/값 검사는 통과한다.
- launch XML 분기, YAML, TF 단일 parent/cycle/gate 검사와 diagram `--check` 통과.
  Mermaid 11.16.0으로 SVG/PNG/manifest를 재생성했다. 이 호스트의 Node 10 대신
  설치된 VS Code의 Node 24 런타임과 cached Chrome을 사용했다.
- 모든 synthetic 센서 발행 테스트는 rostest의 별도 ROS master에서 실행했다.

## 문서 정의와 물리 검증의 구분

[MORAI 원점 문서](https://help-morai-sim.scrollhelp.site/en/morai-sim-drive/23.R1.0/-7)는
센서 장착 원점을 후륜 사이 중심으로 정의한다.
[MORAI 센서 축 문서](https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/sensor-coordinate-system)는
IMU x 전방, y 좌측, z 위쪽 및 양의 각속도 CCW를 정의한다.
이 정의를 개발 base_link에 사용하되 현재 build의 완전한 extrinsic·roll/pitch·고도
실측 검증으로 해석하지 않는다. physical_alignment_verified와 autonomous_driving_ready는 false다.

## 시뮬레이터 재시작

P·0 km/h를 확인한 뒤 UI workspace 복구 중 native Vulkan SIGSEGV로 종료됐다.
로그에는 `vk::ImagePool::ProcessFrontImage`와 `GfxDeviceVK`가 있다.
종료 전 가속/주행 입력은 주지 않았다. 입력 단절 시 status LOST, validity false,
stop_required true와 표시 삭제를 확인했다. 원시 crash log는 `/tmp/tf_simulator_crash.log`.
