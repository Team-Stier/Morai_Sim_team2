"""Pose composition and display validity, independent of ROS execution.

Numeric timeouts here control a drawing only. They do not approve sensor timing,
localization validity, TF publication, or driving readiness.
"""

import math
from collections import OrderedDict
from dataclasses import dataclass

from common_msgs_pkg.validation import (
    validate_covariance, validate_ego, validate_header, validate_localization,
    validate_pair,
)


def stamp_ns(stamp):
    """Keep ROS nanoseconds exact when matching status and estimates."""
    return int(stamp.secs) * 1000000000 + int(stamp.nsecs)


@dataclass(frozen=True)
class DisplayConfig:
    reference_frame: str = 'map'
    display_mode: str = 'footprint'
    vehicle_length_m: float = 4.635
    vehicle_width_m: float = 1.892
    vehicle_height_m: float = 2.434
    footprint_thickness_m: float = 0.06
    body_center_offset_m: tuple = (1.5275, 0.0, 0.0)
    body_center_offset_verified: bool = False
    display_timeout_sec: float = 0.5
    clock_stall_sec: float = 0.5
    buffer_size: int = 100
    marker_publish_rate_hz: float = 10.0

    def __post_init__(self):
        if self.reference_frame not in ('map', 'odom'):
            raise ValueError('reference_frame must be map or odom')
        if self.display_mode not in ('footprint', 'box'):
            raise ValueError('display_mode must be footprint or box')
        for name in ('vehicle_length_m', 'vehicle_width_m', 'vehicle_height_m',
                     'footprint_thickness_m', 'display_timeout_sec',
                     'clock_stall_sec', 'marker_publish_rate_hz'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        if (len(self.body_center_offset_m) != 3 or
                not all(math.isfinite(v) for v in self.body_center_offset_m)):
            raise ValueError('body_center_offset_m must contain three finite values')
        if (isinstance(self.buffer_size, bool) or
                not isinstance(self.buffer_size, int) or self.buffer_size < 1):
            raise ValueError('buffer_size must be a positive integer')
        if not isinstance(self.body_center_offset_verified, bool):
            raise ValueError('body_center_offset_verified must be a boolean')


@dataclass(frozen=True)
class PoseValue:
    position: tuple
    quaternion: tuple


@dataclass(frozen=True)
class DisplayState:
    valid: bool
    reason: str
    frame_id: str
    pose: object = None
    stamp_ns: int = 0
    stop_required: bool = True
    projected: bool = False


def pose_value(pose):
    return PoseValue(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def compose_body_pose(pose, offset):
    """Compose T_reference_base_link with a translation expressed in base_link.

    This is drawing geometry, not a TF broadcast. EgoState already describes
    base_link; GPS antenna extrinsics must not be applied here again.
    """
    if not isinstance(pose, PoseValue):
        pose = pose_value(pose)
    x, y, z, w = pose.quaternion
    ox, oy, oz = offset
    # q * offset * conjugate(q), for a validated unit quaternion.
    tx, ty, tz = 2 * (y * oz - z * oy), 2 * (z * ox - x * oz), 2 * (x * oy - y * ox)
    rotated = (ox + w * tx + y * tz - z * ty,
               oy + w * ty + z * tx - x * tz,
               oz + w * tz + x * ty - y * tx)
    return PoseValue(tuple(a + b for a, b in zip(pose.position, rotated)), pose.quaternion)


def _display_ego_pose(message):
    """Use a full pose when available, explicitly project missing axes otherwise."""
    if not all(message.pose_valid[i] for i in (0, 1, 5)):
        raise ValueError('map x/y/yaw estimates unavailable')
    pose = pose_value(message.pose.pose)
    position = list(pose.position)
    if not message.pose_valid[2]:
        position[2] = 0.0
    quaternion = pose.quaternion
    if not all(message.pose_valid[i] for i in (3, 4)):
        x, y, z, w = quaternion
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        quaternion = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return PoseValue(tuple(position), quaternion), not all(message.pose_valid)


def validate_odometry(message):
    validate_header(message.header, 'odom')
    if message.child_frame_id != 'base_link':
        raise ValueError('local odometry child frame must be base_link')
    pose = pose_value(message.pose.pose)
    if not all(math.isfinite(v) for v in pose.position + pose.quaternion):
        raise ValueError('nonfinite local pose')
    if not math.isclose(sum(v * v for v in pose.quaternion), 1.0, abs_tol=1e-6):
        raise ValueError('non-unit local quaternion')
    validate_covariance(message.pose.covariance, [True] * 6)


class VehicleDisplay:
    """Bounded sample buffers; only display an exact valid status/sample pair.

    Callers serialize access and supply ROS nanoseconds plus monotonic seconds.
    A rejected active input clears its buffer. Display never extrapolates a pose
    or refreshes an estimate's original timestamp.
    """

    def __init__(self, config):
        self.config = config
        self._ego = OrderedDict()
        self._odom = OrderedDict()
        self._status = None
        self._epoch = None
        self._last_stamps = {}
        self._last_ros_ns = None
        self._last_advance_sec = None
        self._clock_stalled = False
        self._reason = 'no localization status'

    def _clear(self, reason, reset_order=False):
        self._ego.clear()
        self._odom.clear()
        self._status = None
        self._reason = reason
        if reset_order:
            self._last_stamps.clear()
            self._epoch = None

    def _clock(self, now_ns, steady_sec):
        if self._last_ros_ns is None:
            self._last_ros_ns = now_ns
            self._last_advance_sec = steady_sec
        elif now_ns < self._last_ros_ns:
            self._clear('ROS clock reset; awaiting new localization', reset_order=True)
            self._last_advance_sec = steady_sec
            self._clock_stalled = False
        elif now_ns > self._last_ros_ns:
            self._last_advance_sec = steady_sec
            self._clock_stalled = False
        self._last_ros_ns = now_ns
        if now_ns <= 0:
            self._clear('ROS clock is zero')
            return False
        if steady_sec - self._last_advance_sec >= self.config.clock_stall_sec:
            self._clear('ROS clock stalled')
            self._clock_stalled = True
            return False
        return True

    def _fresh(self, sample_ns, receipt_sec, now_ns, steady_sec):
        return (0 < sample_ns <= now_ns and
                (now_ns - sample_ns) * 1e-9 <= self.config.display_timeout_sec and
                0 <= steady_sec - receipt_sec <= self.config.display_timeout_sec)

    def _check_stamp(self, source, message, now_ns):
        value = stamp_ns(message.header.stamp)
        if value <= 0 or value > now_ns:
            raise ValueError('zero or future ' + source + ' stamp')
        if (now_ns - value) * 1e-9 > self.config.display_timeout_sec:
            raise ValueError('stale ' + source + ' stamp')
        if value <= self._last_stamps.get(source, -1):
            raise ValueError('duplicate or regressing ' + source + ' stamp')
        return value

    def _adopt_epoch(self, epoch):
        if self._epoch is not None and epoch != self._epoch:
            self._clear('localization reset; awaiting matching estimate')
        self._epoch = epoch

    def _append(self, buffer, key, value):
        buffer[key] = value
        while len(buffer) > self.config.buffer_size:
            buffer.popitem(last=False)

    def ingest_ego(self, message, now_ns, steady_sec):
        if self.config.reference_frame != 'map':
            return False
        if not self._clock(now_ns, steady_sec):
            return False
        try:
            validate_ego(message)
            _display_ego_pose(message)
            value = self._check_stamp('ego', message, now_ns)
        except (ValueError, TypeError, AttributeError) as error:
            self._ego.clear()
            self._reason = str(error)
            return False
        self._adopt_epoch(message.reset_id)
        self._last_stamps['ego'] = value
        self._append(self._ego, (message.reset_id, value), (message, steady_sec))
        return True

    def ingest_odometry(self, message, now_ns, steady_sec):
        if self.config.reference_frame != 'odom':
            return False
        if not self._clock(now_ns, steady_sec):
            return False
        try:
            validate_odometry(message)
            value = self._check_stamp('odometry', message, now_ns)
        except (ValueError, TypeError, AttributeError) as error:
            self._odom.clear()
            self._reason = str(error)
            return False
        self._last_stamps['odometry'] = value
        self._append(self._odom, value, (message, steady_sec))
        return True

    def ingest_status(self, message, now_ns, steady_sec):
        if not self._clock(now_ns, steady_sec):
            return False
        try:
            validate_localization(message)
            value = self._check_stamp('status', message, now_ns)
        except (ValueError, TypeError, AttributeError) as error:
            self._status = None
            self._reason = str(error)
            return False
        self._adopt_epoch(message.reset_id)
        self._last_stamps['status'] = value
        self._status = (message, steady_sec)
        return True

    def _invalid(self, reason):
        return DisplayState(False, reason, self.config.reference_frame)

    def evaluate(self, now_ns, steady_sec):
        if not self._clock(now_ns, steady_sec) or self._status is None:
            return self._invalid(self._reason)
        status, status_receipt = self._status
        if not self._fresh(stamp_ns(status.header.stamp), status_receipt, now_ns, steady_sec):
            return self._invalid('localization status stale')
        if self.config.reference_frame == 'map':
            if not status.map_pose_valid:
                return self._invalid('map pose unavailable; ' + status.reason[:160])
            sample_ns = stamp_ns(status.ego_state_stamp)
            sample = self._ego.get((status.reset_id, sample_ns))
        else:
            if not status.local_odometry_valid:
                return self._invalid('local odometry unavailable; ' + status.reason[:160])
            sample_ns = stamp_ns(status.local_odometry_stamp)
            sample = self._odom.get(sample_ns)
        if sample is None:
            return self._invalid('awaiting exact estimate/status stamp and reset match')
        message, receipt_sec = sample
        if not self._fresh(sample_ns, receipt_sec, now_ns, steady_sec):
            return self._invalid('localization estimate stale')
        try:
            if self.config.reference_frame == 'map':
                validate_pair(message, status)
                pose, projected = _display_ego_pose(message)
            else:
                covariance = message.pose.covariance
                if not math.isclose(status.local_position_stddev_m,
                                    math.sqrt(max(0.0, covariance[0] + covariance[7])),
                                    abs_tol=1e-6):
                    raise ValueError('local position uncertainty mismatch')
                if not math.isclose(status.yaw_stddev_rad,
                                    math.sqrt(max(0.0, covariance[35])), abs_tol=1e-6):
                    raise ValueError('local yaw uncertainty mismatch')
                pose, projected = pose_value(message.pose.pose), False
        except ValueError as error:
            return self._invalid(str(error))
        return DisplayState(True, status.reason, self.config.reference_frame,
                            compose_body_pose(pose, self.config.body_center_offset_m),
                            sample_ns, status.stop_required, projected)
