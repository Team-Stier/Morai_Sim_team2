# Timestamp 계약

기계 판독 가능한 원본은
[`timestamp_contract.yaml`](../../config/timestamp/timestamp_contract.yaml)이다.
이 문서는 live MORAI, rosbag replay와 offline 처리에서 같은 데이터가 서로 다른 시각으로
해석되지 않도록 적용 방법을 설명한다.

모든 공개 topic의 `timestamp_source`는 위 파일의
`timestamp_source_registry`에 등록되어야 한다. 등록되지 않았거나 허용 topic/owner
범위를 벗어난 이름은 중앙 계약 검사에서 거부한다.

## 핵심 규칙

```text
header.stamp = 데이터가 실제 관측되거나 상태 추정이 유효한 시각
```

Camera inference가 영상 수신 후 끝나더라도 detection에는 inference 종료 시각이 아니라
원본 영상의 `header.stamp`를 유지한다. LiDAR 처리, Localization과 World Model도 같은
원칙을 따른다.

```text
sensor measurement time
        ↓ preserve
morai_interface_pkg
        ↓ preserve
perception/localization
        ↓ align at observation time
world_model_pkg
        ↓
planner/controller/safety age checks
```

개발 환경에서 packet frame time과 host ROS time의 정합을 확인한 Camera 3개만
`morai_camera_packet_frame_time`을 사용한다. 이 확인은 본선 최종 packet 검증을
대체하지 않는다. 아직 packet time을 확인하지 못한 LiDAR·Collision을 포함해 원본
데이터에 사용할 수 있는 측정시각이 없으면 `morai_interface_pkg`가 데이터를 받은
즉시 기록한 ROS ingress time을 fallback으로 사용한다. 이 경우 timestamp provenance는
`ingress_fallback`, 품질은 `degraded`로 해석한다. GPS처럼 한 topic의 timestamp source가
항상 ingress fallback으로 고정된 표준 ROS 메시지는 중앙 topic 계약 자체가 provenance다.
표준 메시지에 임의 field를 덧붙이지 않는다. 한 topic에서 source measurement와 ingress
fallback을 섞어야 한다면 per-sample provenance를 담는 custom field 또는 companion status
계약을 먼저 승인해야 하며, 전체 채널 품질은 향후 `/molit/interface/status` schema에서
노출한다. 현재 그 custom schema는 아직 구현되지 않았다.

MORAI adapter의 로컬 YAML 값 `packet`·`receive`는 코드가 읽는 실행 enum이고
새 중앙 timestamp 이름이 아니다. 두 값과 중앙 registry 이름의 대응은
`config/morai_interface/udp_ros_bridge.yaml`의
`runtime_timestamp_parameter_mapping`에서 단일 관리한다.

## 실행 모드별 clock

| 모드 | `use_sim_time` | 기준 | 현재 상태 |
|---|---:|---|---|
| Live MORAI | `false` | ROS time, 가능하면 변환된 센서 측정시각 | MORAI `/clock` 미검증 |
| rosbag replay | `true` | `rosbag play --clock`의 bag clock | replay 구성 시 필수 |
| Offline file | `false` | 파일에 저장된 원본 측정시각 | 처리 wall time 사용 금지 |

Live MORAI에서 `/clock` 토픽이 보인다는 이유만으로 `use_sim_time=true`를 활성화하지 않는다.
clock의 단조 증가, pause/reset 동작과 모든 producer의 동일 clock-domain 사용을 함께
확인해야 한다.

## 두 종류의 age 검사

데이터 freshness와 프로세스 watchdog은 같은 clock을 무조건 공유하지 않는다.

- 데이터 age: 메시지와 동일한 canonical message clock으로 계산한다.
- transport/process watchdog: ROS clock 정지도 검출할 수 있도록 monotonic wall time을 쓴다.

ROS clock이 멈췄을 때 `now - header.stamp`도 함께 멈추므로 데이터 age만 검사하면 통신
단절을 놓칠 수 있다.

## 현재 센서 주기의 의미

| 센서 | 설정 주기 | 설정상 주파수 |
|---|---:|---:|
| Camera | 0.05 s | 20 Hz |
| LiDAR | 0.10 s | 10 Hz |
| GPS | 0.20 s | 5 Hz |
| IMU | 0.02 s | 50 Hz |

이 값은 MORAI 저장 설정의 목표값이지 실제 측정 주파수가 아니다. 시스템 timeout은 이
주기에 임의 배수를 곱해 정하지 않는다. 실제 source stamp, ingress time과 publication
time을 함께 기록하여 period, jitter, transport delay와 processing latency의 p95/p99를
구한 뒤 별도 계약 변경으로 승인한다.

## World Model 적용

`world_model_pkg`는 가장 최근 메시지끼리 단순히 묶지 않는다.

1. Camera/LiDAR 관측의 원본 `header.stamp`를 읽는다.
2. 해당 시각의 ego pose를 `localization_pkg` pose history에서 조회하거나 보간한다.
3. 승인된 sensor extrinsic으로 `map` frame에 변환한다.
4. fusion reference time을 World Model stamp로 사용한다.
5. 각 구성요소의 source stamp, age와 provenance를 내부에 보존한다.

## 오류 처리

- zero stamp: invalid
- source별 stamp 역행: 메시지 reject 후 상태 보고
- clock reset 또는 rosbag seek: 모든 temporal buffer 초기화
- 서로 다른 clock domain 직접 비교: 금지
- 미래 timestamp: 측정으로 허용 오차를 승인하기 전 reject 또는 degraded
- 중복 timestamp: source sequence나 동일 payload 정책이 있을 때만 허용

공개 topic과 message type **이름**은 `interface_contract.yaml` v1.0.0으로
승인됐다. 다만 custom message의 실제 field schema와 timeout 수치는 아직
승인되지 않았다. 다른 패키지가 임시 field나 임계값을 만들지 말고 중앙 계약
변경을 먼저 제안해야 한다.
