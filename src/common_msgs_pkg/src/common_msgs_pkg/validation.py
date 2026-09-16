"""Pure contract checks, not a localization estimator or driving safety gate.

Authority: ros_architecture_pkg/config/messages/core_messages.yaml.
Accept generated genpy messages or equivalent typed fixtures. Raise ValueError
on malformed data. Freshness/reset/sequence checks remain mandatory at consumers.
"""

import math


def seconds(stamp):
    if hasattr(stamp, 'to_sec'):
        return stamp.to_sec()
    if hasattr(stamp, 'secs'):
        return stamp.secs + stamp.nsecs * 1e-9
    return stamp.sec + stamp.nanosec * 1e-9


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def nonnegative(value):
    return math.isfinite(value) and value >= 0


def available_or_unknown(value):
    return value == -1 or nonnegative(value)


def validate_header(header, frame):
    require(header.frame_id == frame, 'frame mismatch')
    require(nonnegative(seconds(header.stamp)) and seconds(header.stamp) > 0,
            'header stamp must be positive')


def validate_freshness(stamp, now_sec, max_age_sec, previous_sec=None):
    """Caller supplies an approved timeout in the SAME canonical clock.

    Strict future rejection; no unmeasured clock tolerance is invented here.
    Callers separately detect frozen ROS clocks with a monotonic watchdog.
    """
    require(nonnegative(now_sec) and nonnegative(max_age_sec), 'invalid clock/timeout')
    value = seconds(stamp)
    require(math.isfinite(value) and 0 < value <= now_sec, 'zero/future stamp')
    require(now_sec - value <= max_age_sec, 'stale data')
    if previous_sec is not None:
        require(value >= previous_sec, 'regressing stamp')


def validate_component(message):
    validate_header(message.header, '')
    require(bool(message.component), 'missing component owner')
    require(message.state in range(6), 'unknown component state')
    require(available_or_unknown(message.data_age_sec), 'invalid data age')
    require(available_or_unknown(message.processing_latency_sec), 'invalid latency')
    data_time = seconds(message.data_stamp)
    require(nonnegative(data_time) and data_time <= seconds(message.header.stamp),
            'invalid/future data stamp')
    if data_time == 0:
        require(message.data_age_sec == -1, 'missing data must have unknown age')
    else:
        require(math.isclose(message.data_age_sec,
                             seconds(message.header.stamp) - data_time,
                             abs_tol=1e-6), 'age must describe monitored data')
    if message.ready:
        require(message.state in (2, 3) and not message.stop_required and data_time > 0,
                'unsafe readiness')


def validate_covariance(covariance, mask):
    """Check the valid subspace using PSD Cholesky; ignore unavailable axes."""
    require(len(mask) == 6 and len(covariance) == 36, 'invalid covariance shape')
    indices = [i for i, valid in enumerate(mask) if valid]
    matrix = [[covariance[6*i+j] for j in indices] for i in indices]
    n = len(matrix)
    for i in range(n):
        for j in range(n):
            require(math.isfinite(matrix[i][j]), 'nonfinite covariance')
            require(math.isclose(matrix[i][j], matrix[j][i], abs_tol=1e-9),
                    'asymmetric covariance')
    # Relative numerical tolerance only; this is not a sensor uncertainty limit.
    tolerance = 1e-10 * max([1.0] + [abs(v) for row in matrix for v in row])
    lower = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            value = matrix[i][j] - sum(lower[i][k] * lower[j][k] for k in range(j))
            if i == j:
                require(value >= -tolerance, 'non-PSD covariance')
                lower[i][j] = math.sqrt(max(0.0, value))
            elif lower[j][j] > 0:
                lower[i][j] = value / lower[j][j]
            else:
                require(abs(value) <= tolerance, 'non-PSD covariance')


def validate_ego(message):
    validate_header(message.header, 'map')
    require(message.child_frame_id == 'base_link', 'motion frame mismatch')
    require(len(message.pose_valid) == 6 and len(message.twist_valid) == 6,
            'invalid component mask')
    pose = message.pose.pose
    for valid, value in zip(message.pose_valid[:3],
                            (pose.position.x, pose.position.y, pose.position.z)):
        require(not valid or math.isfinite(value), 'invalid estimated position')
    if any(message.pose_valid[3:]):
        q = pose.orientation
        require(all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)), 'invalid quaternion')
        require(math.isclose(sum(v*v for v in (q.x, q.y, q.z, q.w)), 1.0,
                             abs_tol=1e-6), 'non-unit quaternion')
    twist = message.twist.twist
    values = (twist.linear.x, twist.linear.y, twist.linear.z,
              twist.angular.x, twist.angular.y, twist.angular.z)
    for valid, value in zip(message.twist_valid, values):
        require(not valid or math.isfinite(value), 'invalid estimated motion')
    validate_covariance(message.pose.covariance, message.pose_valid)
    validate_covariance(message.twist.covariance, message.twist_valid)


def validate_localization(message):
    validate_header(message.header, '')
    require(message.mode in range(6), 'unknown localization mode')
    for name in ('gps_age_sec', 'map_position_stddev_m',
                 'local_position_stddev_m', 'yaw_stddev_rad'):
        require(available_or_unknown(getattr(message, name)), 'invalid ' + name)
    for name in ('ego_state_stamp', 'local_odometry_stamp'):
        value = seconds(getattr(message, name))
        require(nonnegative(value) and value <= seconds(message.header.stamp),
                'invalid/future estimate stamp')
    if message.mode in (0, 1, 5):
        require(message.stop_required and not message.map_pose_valid
                and not message.local_odometry_valid, 'unavailable localization must stop')
    if message.mode == 4:
        require(message.stop_required and not message.map_pose_valid
                and not message.local_odometry_valid, 'relocalizing must invalidate pending poses')
    if message.mode == 2:
        require(message.gps_fix_valid, 'tracking requires GPS')
    if message.mode == 3:
        require(not message.gps_fix_valid and message.local_odometry_valid,
                'dead reckoning requires local estimate without GPS')
    if message.gps_fix_valid:
        require(nonnegative(message.gps_age_sec), 'GPS age unavailable')
    if message.map_pose_valid:
        require(seconds(message.ego_state_stamp) > 0
                and nonnegative(message.map_position_stddev_m), 'invalid map quality')
    if message.local_odometry_valid:
        require(seconds(message.local_odometry_stamp) > 0
                and nonnegative(message.local_position_stddev_m)
                and nonnegative(message.yaw_stddev_rad), 'invalid local quality')
    else:
        require(message.stop_required, 'missing local motion must require stop')


def validate_pair(ego, status):
    """Validate same-epoch map data/status; no inference of local odometry quality."""
    validate_ego(ego)
    validate_localization(status)
    require(ego.reset_id == status.reset_id, 'reset epoch mismatch')
    require(seconds(ego.header.stamp) == seconds(status.ego_state_stamp),
            'estimate stamp mismatch')
    if status.map_pose_valid:
        require(all(ego.pose_valid[i] for i in (0, 1, 5)), 'map pose axes unavailable')
        uncertainty = math.sqrt(max(0.0, ego.pose.covariance[0] + ego.pose.covariance[7]))
        require(math.isclose(status.map_position_stddev_m, uncertainty,
                             abs_tol=1e-6), 'map uncertainty mismatch')
