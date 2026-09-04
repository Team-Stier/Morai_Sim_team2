"""ROS-independent validation used by the temporary Team1 controller adapter."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Tuple


@dataclass(frozen=True)
class CheckResult:
    valid: bool
    reason: str


@dataclass(frozen=True)
class AdapterConfig:
    trajectory_timeout_sec: float
    odometry_timeout_sec: float
    raw_command_timeout_sec: float
    minimum_path_points: int
    queue_size: int
    maximum_accel_command: float
    maximum_brake_command: float
    maximum_steering_angle_rad: float
    controller_target_speed_mps: float

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "AdapterConfig":
        if not isinstance(values, Mapping):
            raise ValueError("adapter config must be a mapping")
        expected = {
            "trajectory_timeout_sec",
            "odometry_timeout_sec",
            "raw_command_timeout_sec",
            "minimum_path_points",
            "queue_size",
            "maximum_accel_command",
            "maximum_brake_command",
            "maximum_steering_angle_rad",
            "controller_target_speed_mps",
        }
        missing = expected.difference(values)
        extra = set(values).difference(expected)
        if missing or extra:
            raise ValueError(
                "adapter config keys mismatch: missing={}, extra={}".format(
                    sorted(missing), sorted(extra)
                )
            )
        config = cls(
            trajectory_timeout_sec=float(values["trajectory_timeout_sec"]),
            odometry_timeout_sec=float(values["odometry_timeout_sec"]),
            raw_command_timeout_sec=float(values["raw_command_timeout_sec"]),
            minimum_path_points=_strict_int(
                "minimum_path_points", values["minimum_path_points"]
            ),
            queue_size=_strict_int("queue_size", values["queue_size"]),
            maximum_accel_command=float(values["maximum_accel_command"]),
            maximum_brake_command=float(values["maximum_brake_command"]),
            maximum_steering_angle_rad=float(
                values["maximum_steering_angle_rad"]
            ),
            controller_target_speed_mps=float(
                values["controller_target_speed_mps"]
            ),
        )
        for name in (
            "trajectory_timeout_sec",
            "odometry_timeout_sec",
            "raw_command_timeout_sec",
            "maximum_accel_command",
            "maximum_brake_command",
            "maximum_steering_angle_rad",
            "controller_target_speed_mps",
        ):
            value = getattr(config, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        if config.maximum_accel_command > 1.0:
            raise ValueError("maximum_accel_command must not exceed 1")
        if config.maximum_brake_command > 1.0:
            raise ValueError("maximum_brake_command must not exceed 1")
        if config.maximum_steering_angle_rad > math.radians(40.0) + 1.0e-12:
            raise ValueError("maximum_steering_angle_rad must not exceed 40 deg")
        if config.minimum_path_points < 2:
            raise ValueError("minimum_path_points must be at least 2")
        if config.queue_size < 1:
            raise ValueError("queue_size must be positive")
        return config


def _strict_int(name: str, value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("{} must be an integer".format(name))
    return value


def stamp_seconds(stamp: Any) -> float:
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    if hasattr(stamp, "secs") and hasattr(stamp, "nsecs"):
        return float(stamp.secs) + float(stamp.nsecs) * 1.0e-9
    return float(stamp)


def _source_time_check(stamp: Any, now_sec: float, timeout_sec: float) -> CheckResult:
    try:
        source_sec = stamp_seconds(stamp)
    except (TypeError, ValueError, OverflowError):
        return CheckResult(False, "malformed_stamp")
    if not math.isfinite(now_sec) or not math.isfinite(source_sec):
        return CheckResult(False, "non_finite_time")
    if source_sec <= 0.0:
        return CheckResult(False, "zero_stamp")
    age_sec = now_sec - source_sec
    if age_sec < 0.0:
        return CheckResult(False, "future_stamp")
    if age_sec > timeout_sec:
        return CheckResult(False, "stale_stamp")
    return CheckResult(True, "valid")


def _all_finite(values: Any) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError, OverflowError):
        return False


def validate_trajectory(
    message: Any,
    now_sec: float,
    config: AdapterConfig,
    expected_frame: str = "map",
) -> CheckResult:
    try:
        if message.header.frame_id != expected_frame:
            return CheckResult(False, "wrong_trajectory_frame")
        time_check = _source_time_check(
            message.header.stamp, now_sec, config.trajectory_timeout_sec
        )
        if not time_check.valid:
            return CheckResult(False, "trajectory_{}".format(time_check.reason))
        valid_status = int(getattr(message, "STATUS_VALID", 0))
        if int(message.status) != valid_status:
            return CheckResult(False, "trajectory_status_not_valid")
        source_sec = stamp_seconds(message.header.stamp)
        valid_until_sec = stamp_seconds(message.valid_until)
        if not math.isfinite(valid_until_sec):
            return CheckResult(False, "non_finite_valid_until")
        if valid_until_sec < source_sec:
            return CheckResult(False, "valid_until_precedes_stamp")
        if now_sec >= valid_until_sec:
            return CheckResult(False, "trajectory_expired")
        clearance = float(message.minimum_boundary_clearance_m)
        if not math.isfinite(clearance) or clearance <= 0.0:
            return CheckResult(False, "non_positive_boundary_clearance")
        points = message.points
        if len(points) < config.minimum_path_points:
            return CheckResult(False, "too_few_trajectory_points")

        previous_s = None
        previous_xy = None
        for point in points:
            numeric = (
                point.x_m,
                point.y_m,
                point.z_m,
                point.yaw_rad,
                point.curvature_1pm,
                point.s_m,
                point.target_speed_mps,
            )
            if not _all_finite(numeric):
                return CheckResult(False, "non_finite_trajectory_point")
            s_m = float(point.s_m)
            if s_m < 0.0:
                return CheckResult(False, "negative_arc_length")
            if previous_s is not None and s_m <= previous_s:
                return CheckResult(False, "non_increasing_arc_length")
            target_speed_mps = float(point.target_speed_mps)
            if target_speed_mps < 0.0:
                return CheckResult(False, "negative_target_speed")
            if target_speed_mps + 1.0e-9 < config.controller_target_speed_mps:
                return CheckResult(False, "planner_speed_below_controller_target")
            xy = (float(point.x_m), float(point.y_m))
            if previous_xy is not None and xy == previous_xy:
                return CheckResult(False, "duplicate_path_point")
            previous_s = s_m
            previous_xy = xy
    except (AttributeError, TypeError, ValueError, OverflowError):
        return CheckResult(False, "malformed_trajectory")
    return CheckResult(True, "valid")


def validate_odometry(
    message: Any,
    now_sec: float,
    config: AdapterConfig,
    expected_frame: str = "map",
    expected_child_frame: str = "base_link",
) -> CheckResult:
    try:
        if message.header.frame_id != expected_frame:
            return CheckResult(False, "wrong_odometry_frame")
        if message.child_frame_id != expected_child_frame:
            return CheckResult(False, "wrong_odometry_child_frame")
        time_check = _source_time_check(
            message.header.stamp, now_sec, config.odometry_timeout_sec
        )
        if not time_check.valid:
            return CheckResult(False, "odometry_{}".format(time_check.reason))

        pose = message.pose.pose
        twist = message.twist.twist
        orientation = pose.orientation
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
        if not _all_finite(values):
            return CheckResult(False, "non_finite_odometry")
        quaternion_norm_squared = sum(
            float(value) * float(value)
            for value in (
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        )
        if quaternion_norm_squared <= 1.0e-12:
            return CheckResult(False, "invalid_odometry_quaternion")
        if len(message.pose.covariance) != 36 or not _all_finite(
            message.pose.covariance
        ):
            return CheckResult(False, "invalid_pose_covariance")
        if len(message.twist.covariance) != 36 or not _all_finite(
            message.twist.covariance
        ):
            return CheckResult(False, "invalid_twist_covariance")
    except (AttributeError, TypeError, ValueError, OverflowError):
        return CheckResult(False, "malformed_odometry")
    return CheckResult(True, "valid")


def validate_raw_command(
    message: Any,
    now_sec: float,
    config: AdapterConfig,
    expected_frame: str = "base_link",
) -> CheckResult:
    try:
        if message.header.frame_id != expected_frame:
            return CheckResult(False, "wrong_command_frame")
        time_check = _source_time_check(
            message.header.stamp, now_sec, config.raw_command_timeout_sec
        )
        if not time_check.valid:
            return CheckResult(False, "command_{}".format(time_check.reason))
        accel = float(message.accel)
        brake = float(message.brake)
        steering = float(message.steering_angle_rad)
        if not _all_finite((accel, brake, steering)):
            return CheckResult(False, "non_finite_command")
        if accel < 0.0 or accel > config.maximum_accel_command:
            return CheckResult(False, "accel_out_of_range")
        if brake < 0.0 or brake > config.maximum_brake_command:
            return CheckResult(False, "brake_out_of_range")
        if abs(steering) > config.maximum_steering_angle_rad:
            return CheckResult(False, "steering_out_of_range")
        if accel > 0.0 and brake > 0.0:
            return CheckResult(False, "simultaneous_accel_and_brake")
    except (AttributeError, TypeError, ValueError, OverflowError):
        return CheckResult(False, "malformed_command")
    return CheckResult(True, "valid")


def quaternion_from_yaw(yaw_rad: float) -> Tuple[float, float, float, float]:
    if not math.isfinite(yaw_rad):
        raise ValueError("yaw_rad must be finite")
    return (0.0, 0.0, math.sin(0.5 * yaw_rad), math.cos(0.5 * yaw_rad))
