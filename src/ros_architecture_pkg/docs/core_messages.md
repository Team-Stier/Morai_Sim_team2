# 기반 메시지 계약 및 이식 지침

## 범위

[설계] 공개 node/topic/type 이름 v1.0.0은 유지하며 `core_messages` 하위 계약
v0.1.0에서 세 타입의 필드를 구현한다. 필드와 의미의 유일한 원본은
[`config/messages/core_messages.yaml`](../config/messages/core_messages.yaml)이다.
`common_msgs_pkg/msg`는 해당 `wire_definition`과 정확히 같은 구현이며 테스트가
이 관계를 검사한다. 타입 구현은 노드 구현·런타임 활성화·대회 검증과 다르다.
다른 feature branch의 Localization 구현을 이번 변경에서 병합하지 않는다.

## 규정 반영 근거

[규정] 2026-09-10 사용자가 제공한 규정집 2-6-2, 2-8 발췌를 적용했다.
[공식 링크](https://morai.atlassian.net/wiki/external/YTRiNWMwZTNjODc3NDBkYzgxMTMzOWMwMWY3YWRkZDc)의
전체 원문·개정번호를 이번 변경에서 새로 검증했다고 주장하지 않는다.

- 올해 한시적으로 GPS·IMU에 Noise를 인가하지 않는다. GPS blackout 요구는 유지된다.
- Competition Vehicle Status에는 pos_x/y/z, vel_y/z, accel_x/y/z 및 지정된
  타이어 lateral force/side-slip/cornering stiffness 정보가 없다.
- 허용된 GPS·IMU·차량 상태를 이용한 Localization 추정은 원시 UDP 관측과 다르다.
  특히 없는 vel_y/vel_z를 0 관측으로 EKF에 융합하거나 acceleration=0을
  실제 측정값으로 처리하지 않는다. 모델의 비홀로노믹/2D 가정은 별도 모델 근거이며
  센서가 그 값을 측정했다는 뜻이 아니다.
- IMU의 허용 센서 출력과 Competition Vehicle Status의 미제공 가속도는 별개다.
  세 기반 메시지에는 가속도·타이어 상태 필드를 추가하지 않는다.
- Noise 미인가는 추정 covariance=0의 근거가 아니다. 좌표계, 시간 정합과
  적분·모델 오차는 별도로 검증한다.

## 필드 요약

아래는 중앙 계약의 읽기용 요약이다.

| 타입 | 주요 필드 | 의미 |
|---|---|---|
| ComponentStatus | header, component, state, ready, stop_required | 평가시각·소유 패키지·상태. ready는 주행 허가가 아님 |
| ComponentStatus | data_stamp, data_age_sec, processing_latency_sec | 평가한 데이터 시각·age·처리시간. 부재 age/latency는 -1 |
| ComponentStatus | processed_count, dropped_count, invalid_count, reason | 프로세스 시작 이후 카운터와 설명. network packet 수라고 가정하지 않음 |
| EgoState | header, child_frame_id, pose, twist | map 기준 base_link pose 및 base_link 기준 twist, 같은 추정 유효시각 |
| EgoState | pose_valid[6], twist_valid[6], reset_id | 성분별 추정 유효성과 불연속/reset 구분 |
| LocalizationStatus | header, mode, gps_fix_valid, map_pose_valid, local_odometry_valid, stop_required | GPS 가용성과 map/local 추정 품질을 구분 |
| LocalizationStatus | ego_state_stamp, local_odometry_stamp, reset_id | 평가 대상 데이터와 reset epoch |
| LocalizationStatus | gps_age_sec, map_position_stddev_m, local_position_stddev_m, yaw_stddev_rad, reason | 마지막 accepted GPS age와 추정 uncertainty |

### 유효성·좌표·불확실성

`EgoState.header.frame_id=map`, `child_frame_id=base_link`다. pose의 validity 순서는
`[x,y,z,roll,pitch,yaw]`, twist는 `[vx,vy,vz,wx,wy,wz]`다. 단위는 m, rad, m/s,
rad/s 및 해당 covariance다. 미추정 성분은 false이며 해당 수치와 covariance
행/열을 소비하지 않는다. true는 유효한 **추정**을 뜻하며 원시 관측 가용성을 뜻하지 않는다.

2D 모델은 z/roll/pitch placeholder를 가질 수 있으나 그 성분을 실제로 추정하지
않으면 validity는 false다. yaw를 사용하는 경우에도 quaternion은 finite/unit이어야 한다.
유효 성분의 covariance 부분행렬은 finite·대칭·PSD여야 한다. map pose 사용에는
x/y/yaw가 모두 필요하다. 양방향 속도가 필요한 consumer는 speed magnitude가 아닌
signed twist를 사용한다. 단순 km/h→m/s 변환으로 잃은 부호가 복구되지 않는다.

map/local position stddev는 각각 sqrt(var_x+var_y), yaw stddev는 **local** yaw의
표준편차다. 상태 메시지는 별도 평가시각을 가지므로 ego_state_stamp/reset_id로
원본 데이터와 대응시킨다. map 불확실성과 local 불확실성을 혼용하지 않는다.
threshold는 실측 profile에서 정하며 이번 변경은 허용 오차 숫자를 정하지 않는다.

### 상태와 시간

ComponentStatus는 UNKNOWN/INITIALIZING/READY/DEGRADED/FAULT/DISABLED를 사용한다.
ready=true는 READY/DEGRADED에서만 가능하며 stop_required=false 및 data_stamp가
필요하다. DEAD_RECKONING은 Localization의 별도 모드이며 GPS invalid여도
local estimate가 유효할 수 있다. LOST/UNINITIALIZED/INITIALIZING은 map/local
유효성을 선언하지 않고 stop_required=true다. RELOCALIZING은 품질 플래그와
실측 profile 조건을 함께 검사하며 모드명만으로 주행 가능 여부를 결정하지 않는다.

상태 Header는 frame_id가 빈 문자열이고 평가시각이다. 데이터 Header를 now()로
덮어써서는 안 된다. latched 상태를 받았다고 현재 healthy로 인정하지 않는다.
consumer는 상태 heartbeat와 데이터 age를 각각 검사하고 canonical clock이
멈춘 경우 별도 monotonic watchdog으로 처리한다. map 정적 데이터의 load time을
센서 freshness timeout에 넣지 않는다. 모든 데이터 결합은 같은 clock domain에서 한다.
process restart, reset_id 변경, clock reset에는 캐시/pose history를 초기화한다.

## 생산자·소비자 변경 영향

| 경계 | 이번 변경 | 이식 시 필수 조건 |
|---|---|---|
| Localization → Route/WorldModel/Planning/Safety/Evaluation | EgoState/LocalizationStatus 필드 구현 | 원본 Odometry covariance·stamp 보존. 기존 DiagnosticArray 단순 type 변경 금지 |
| Localization → Controller | LocalizationStatus 구현, local Odometry 유지 | Controller에 새 EgoState 구독 추가하지 않음 |
| Map/Perception/Route/WorldModel/Planning → 각 consumer | ComponentStatus 공통 스키마 | component는 중앙 topic owner 패키지와 같아야 함 |
| common_msgs_pkg → 모든 consumer | message_generation/runtime 의존성 추가 | C++ executable에는 catkin_EXPORTED_TARGETS 빌드 순서 의존성 필요 |

현재 main의 해당 producer/consumer 런타임은 placeholder다. 이번 변경은 manifest
의존성과 모든 관련 README를 갱신하되 fake publisher나 launch를 추가하지 않는다.
단순 fixture 테스트를 live callback 검증으로 보고하지 않는다.

## 검증 함수 사용 범위

`common_msgs_pkg.validation`은 generated message에 대한 순수 검사 함수다.
`validate_ego`, `validate_localization`, `validate_component`, `validate_pair`는
위반 시 ValueError를 발생시킨다. `validate_freshness`에는 consumer가 승인된 timeout과
동일 clock의 now를 전달해야 한다. 함수 import만으로 ROS callback 검사가 적용되지 않는다.
consumer는 메시지 stamp 순서, `component`와 중앙 topic owner의 일치, reset epoch,
입력 조합, local Odometry와의
정합, 측정 품질 threshold 및 monotonic watchdog을 별도로 구현해야 한다.
이 helper는 모델 정확성, UDP 가용성, TF 검증, Safety gate를 대신하지 않는다.

## 실행할 검사

```bash
python3 -m unittest discover -s src/common_msgs_pkg/test -p 'test_*.py'
python3 src/ros_architecture_pkg/scripts/generate_interface_diagrams.py --check
```

ROS Noetic 환경에서 실제 빌드·네이티브 직렬화 검사:

```bash
source /opt/ros/noetic/setup.bash
PYTHONNOUSERSITE=1 catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
PYTHONNOUSERSITE=1 catkin_make run_tests_common_msgs_pkg
catkin_test_results --all build/test_results
```

직렬화 테스트는 catkin이 생성한 genpy 타입을 우선 사용한다. 비ROS 환경에서는
선택적 rosbags의 ROS1_NOETIC typestore를 사용한다. 둘 다 없으면 명시적으로 skip한다.
rosbags 성공은 catkin 빌드, C++ 헤더 생성, 실제 ROS topic 연결의 증거가 아니다.

## 변경하지 않은 것

공개 graph·topic 이름·TF/UDP 활성화·센서 주기·timeout 수치는 바꾸지 않았다.
CollisionEvent, Competition packet layout, ActuatorCommand와 제어 송신은 이번 범위가 아니다.
