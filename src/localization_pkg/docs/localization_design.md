> 이 문서의 이식·진단 기록은 과거 단계다. 현재 GPS/IMU 개발 추정 구현과
> 검증 범위는 [TF Localization 검증](tf_localization_validation.md)을 따른다.

> 이 문서는 원격 브랜치의 과거 구현 설계다. ROS adapter는 현재 빌드·실행 대상이 아니며, 공개 인터페이스는 중앙 계약과 패키지 README를 따른다.

## 현재 상태: 진단 런타임

이 저장소는 중앙 TF/timestamp 계약 게이트가 잠긴 상태를 존중한 진단 런타임을
우선 제공합니다.

- 구독: `/molit/sensors/gps/fix`, `/molit/sensors/imu/data`
- 발행: `/molit/localization/status`
- 진단 이유: `/molit/localization/local/odometry`, `/molit/localization/ego_state`는
  게이트가 해제될 때까지 publish 하지 않음
- `map -> odom`, `odom -> base_link` TF publish_enabled가 false인 상태를 실패-차단 근거로
  명시적으로 표시

`status`는 센서 유효성(duplicate/regression/reject), ROS 시간 스탬프 상태(정지/회귀),
그리고 중앙 계약 블로킹 사유를 노드 reason에 기록한다.

2026-09-10 사용자 확정 장착 위치는 IMU `(0, 0, 0)` m, GPS `(0, 0, 1.3)` m다.
두 센서의 저장된 회전은 `(0, 0, 0)`도다. 이전 IMU 오프셋이나 GPS 원점 장착값을
현재 값으로 사용하지 않는다. [중앙 extrinsic 계약](../../ros_architecture_pkg/config/tf/sensor_extrinsics.yaml)이
값과 근거를 소유하며, MORAI pivot과 `base_link` 정합·축 검증은 여전히 남아 있다.
아래 EKF의 첫 GPS 원점 초기화는 과거 알고리즘 설명이며 센서 장착 위치를 뜻하지 않는다.

# GPS + IMU Localization Design

## Coordinates and initialization

`local_enu` is a local East-North-Up frame. The first valid GPS fix is `(0, 0, 0)`.
The node uses a local tangent-plane approximation to convert subsequent WGS84 fixes to metres.
The estimator remains `INITIALIZING` until the horizontal distance from the first fix exceeds
`minimum_yaw_initialization_distance_m`. Its heading is then the GPS travel direction.

This local frame is intentionally not yet the HD Map frame. The route-start ENU alignment and
the verified GPS datum/projection must be approved jointly by Localization, HD Map and the
central architecture before map-relative output is used in driving decisions.

## Estimator

The 15-state error-state EKF has nominal position, velocity, quaternion attitude,
accelerometer bias and gyroscope bias. IMU specific force and angular velocity drive prediction.
GPS supplies a position-only measurement update. GPS correction is gated by Mahalanobis distance
and capped in magnitude to avoid a discontinuous pose jump after recovery.
GPS may arrive up to `max_gps_measurement_delay_sec` after the latest IMU state; older or
out-of-order fixes are rejected.

The measurement update is deliberately isolated from ROS callbacks, so an approved static-map
LiDAR pose constraint can later use the same update boundary. LiDAR scan ingestion, perception,
ICP/NDT and map matching are not part of this package version.

## Quality and failures

| State | Meaning |
|---|---|
| `INITIALIZING` | GPS origin or GPS-travel yaw is unavailable. |
| `NOMINAL` | Recent IMU and GPS data are accepted. |
| `DEGRADED` | GPS is stale/blackout or recent fixes were rejected; IMU prediction continues. |
| `INVALID` | IMU is stale, dead-reckoning time limit is exceeded, or data cannot safely be used. |

Rejected GPS samples never update the state. Out-of-order or non-finite IMU samples never produce
a newly timestamped odometry output. Consumers must use `LocalizationQuality` together with
odometry; an `INVALID` state is not a current trusted pose.

## Simulator integration hold point

Before MORAI connection, capture and verify the actual UDP packet's GPS coordinate representation,
IMU axes, specific-force/gravity convention, timestamp source, frame extrinsics and covariance
meaning. Those facts belong to `morai_interface_pkg` normalization and the central contract;
they must not be guessed from generic MORAI examples.
