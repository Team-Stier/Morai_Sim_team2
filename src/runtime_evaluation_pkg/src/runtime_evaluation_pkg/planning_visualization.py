"""ROS-independent validation and geometry for the planning RViz observer.

The public ROS names and timeout semantics live in the central interface
contract. Keeping these functions free of ROS types makes the reject path
unit-testable on machines that do not have ROS installed.
"""

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Iterable, Optional, Sequence, Tuple


EXPECTED_FRAME = "map"
EXPECTED_CHILD_FRAME = "base_link"
ODOMETRY_MAX_AGE_SEC = 0.20
MARKER_TIMEOUT_SEC = 0.25
QUATERNION_NORM_TOLERANCE = 1.0e-3

IONIQ5_LENGTH_M = 4.635
IONIQ5_WIDTH_M = 1.892
IONIQ5_HEIGHT_M = 2.434
IONIQ5_REAR_OVERHANG_M = 0.790
IONIQ5_BODY_CENTER_X_M = IONIQ5_LENGTH_M * 0.5 - IONIQ5_REAR_OVERHANG_M


@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True)
class PoseSample:
    stamp_sec: float
    frame_id: str
    child_frame_id: str
    position: Tuple[float, float, float]
    orientation: Quaternion
    linear_velocity: Tuple[float, float, float]
    angular_velocity: Tuple[float, float, float]
    pose_covariance: Sequence[float]
    twist_covariance: Sequence[float]


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    reason: str
    age_sec: Optional[float] = None


@dataclass(frozen=True)
class TracePose:
    stamp_sec: float
    position: Tuple[float, float, float]
    orientation: Quaternion


def _all_finite(values: Iterable[float]) -> bool:
    try:
        return all(math.isfinite(value) for value in values)
    except (TypeError, ValueError):
        return False


def _quaternion_norm(quaternion: Quaternion) -> float:
    return math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )


def validate_pose_sample(
    sample: PoseSample,
    now_sec: float,
    expected_frame: str = EXPECTED_FRAME,
    expected_child_frame: str = EXPECTED_CHILD_FRAME,
    max_age_sec: float = ODOMETRY_MAX_AGE_SEC,
) -> ValidationResult:
    """Validate one odometry snapshot without repairing malformed data."""

    if not math.isfinite(now_sec):
        return ValidationResult(False, "non_finite_clock")
    if sample.frame_id != expected_frame:
        return ValidationResult(False, "wrong_parent_frame")
    if sample.child_frame_id != expected_child_frame:
        return ValidationResult(False, "wrong_child_frame")
    if not math.isfinite(sample.stamp_sec) or sample.stamp_sec <= 0.0:
        return ValidationResult(False, "invalid_timestamp")

    age_sec = now_sec - sample.stamp_sec
    if age_sec < 0.0:
        return ValidationResult(False, "future_timestamp", age_sec)
    if age_sec > max_age_sec:
        return ValidationResult(False, "stale_timestamp", age_sec)

    if len(sample.position) != 3 or not _all_finite(sample.position):
        return ValidationResult(False, "non_finite_position", age_sec)
    quaternion_values = (
        sample.orientation.x,
        sample.orientation.y,
        sample.orientation.z,
        sample.orientation.w,
    )
    if not _all_finite(quaternion_values):
        return ValidationResult(False, "non_finite_orientation", age_sec)
    try:
        quaternion_norm = _quaternion_norm(sample.orientation)
    except (TypeError, ValueError):
        return ValidationResult(False, "non_finite_orientation", age_sec)
    if abs(quaternion_norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        return ValidationResult(False, "non_unit_orientation", age_sec)
    if len(sample.linear_velocity) != 3 or not _all_finite(sample.linear_velocity):
        return ValidationResult(False, "non_finite_linear_velocity", age_sec)
    if len(sample.angular_velocity) != 3 or not _all_finite(sample.angular_velocity):
        return ValidationResult(False, "non_finite_angular_velocity", age_sec)
    if len(sample.pose_covariance) != 36 or not _all_finite(sample.pose_covariance):
        return ValidationResult(False, "invalid_pose_covariance", age_sec)
    if len(sample.twist_covariance) != 36 or not _all_finite(sample.twist_covariance):
        return ValidationResult(False, "invalid_twist_covariance", age_sec)
    return ValidationResult(True, "ok", age_sec)


def _normalized(quaternion: Quaternion) -> Quaternion:
    norm = _quaternion_norm(quaternion)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("quaternion must be finite and non-zero")
    return Quaternion(
        quaternion.x / norm,
        quaternion.y / norm,
        quaternion.z / norm,
        quaternion.w / norm,
    )


def _rotate_vector(
    vector: Tuple[float, float, float], quaternion: Quaternion
) -> Tuple[float, float, float]:
    """Rotate a vector by a unit quaternion using its rotation matrix."""

    q = _normalized(quaternion)
    x, y, z = vector
    xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
    xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
    wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
    return (
        (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z,
        2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z,
        2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z,
    )


def marker_center(sample: PoseSample) -> Tuple[float, float, float]:
    """Return the IONIQ 5 body-envelope center in the map frame."""

    offset = _rotate_vector(
        (IONIQ5_BODY_CENTER_X_M, 0.0, IONIQ5_HEIGHT_M * 0.5),
        sample.orientation,
    )
    return tuple(sample.position[index] + offset[index] for index in range(3))


def remaining_marker_lifetime_sec(
    sample_stamp_sec: float,
    now_sec: float,
    timeout_sec: float = MARKER_TIMEOUT_SEC,
) -> float:
    """Return lifetime remaining until the sample's absolute expiry.

    Invalid, future, and already-expired timestamps return zero so callers can
    fail closed with a DELETE marker.  Capping at ``timeout_sec`` prevents a
    malformed timestamp from extending the visualization lifetime.
    """

    values = (sample_stamp_sec, now_sec, timeout_sec)
    if not _all_finite(values) or sample_stamp_sec <= 0.0 or timeout_sec <= 0.0:
        return 0.0
    if sample_stamp_sec > now_sec:
        return 0.0
    remaining_sec = sample_stamp_sec + timeout_sec - now_sec
    return max(0.0, min(timeout_sec, remaining_sec))


class TraceHistory:
    """A bounded, strictly time-ordered history of accepted poses."""

    def __init__(self, max_samples: int) -> None:
        if max_samples <= 0:
            raise ValueError("max_samples must be positive")
        self._poses: Deque[TracePose] = deque(maxlen=max_samples)

    def append(self, sample: PoseSample) -> bool:
        if self._poses and sample.stamp_sec <= self._poses[-1].stamp_sec:
            return False
        self._poses.append(
            TracePose(sample.stamp_sec, sample.position, sample.orientation)
        )
        return True

    def clear(self) -> None:
        self._poses.clear()

    def snapshot(self) -> Tuple[TracePose, ...]:
        return tuple(self._poses)

    def __len__(self) -> int:
        return len(self._poses)
