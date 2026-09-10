> 이 문서는 원격 브랜치의 과거 구현 설계다. ROS adapter는 현재 빌드·실행 대상이 아니며, 공개 인터페이스는 중앙 계약과 패키지 README를 따른다.

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
