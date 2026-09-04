"""ROS-independent value types for path planning."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


def _require_finite(value: float, name: str) -> None:
    if not math.isfinite(value):
        raise ValueError("{} must be finite".format(name))


def _require_positive(value: float, name: str) -> None:
    _require_finite(value, name)
    if value <= 0.0:
        raise ValueError("{} must be positive".format(name))


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class Pose2D:
    """Planar vehicle pose located at the rear axle center."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CircleObstacle:
    center: Point2D
    radius_m: float
    obstacle_id: str = ""

    def __post_init__(self) -> None:
        _require_finite(self.center.x, "obstacle center x")
        _require_finite(self.center.y, "obstacle center y")
        _require_positive(self.radius_m, "obstacle radius")


@dataclass(frozen=True)
class BoxObstacle:
    """Tracked oriented box whose pose is at the geometric body centre."""

    pose: Pose2D
    length_m: float
    width_m: float
    obstacle_id: str = ""

    def __post_init__(self) -> None:
        _require_finite(self.pose.x, "obstacle pose x")
        _require_finite(self.pose.y, "obstacle pose y")
        _require_finite(self.pose.yaw, "obstacle pose yaw")
        _require_positive(self.length_m, "obstacle length")
        _require_positive(self.width_m, "obstacle width")

    def corners(self) -> Tuple[Point2D, ...]:
        half_length = 0.5 * self.length_m
        half_width = 0.5 * self.width_m
        return (
            VehicleGeometry._transform(self.pose, -half_length, -half_width),
            VehicleGeometry._transform(self.pose, half_length, -half_width),
            VehicleGeometry._transform(self.pose, half_length, half_width),
            VehicleGeometry._transform(self.pose, -half_length, half_width),
        )


@dataclass(frozen=True)
class VehicleGeometry:
    """Vehicle envelope relative to a rear-axle planning pose.

    The rules give no wheel-track value.  Wheel contact proxies therefore use
    the full body width by default.  This is intentionally conservative: if the
    proxies clear a marking, the physical tyres (which are inboard of the body
    sides) clear it as well.
    """

    wheelbase_m: float = 3.000
    length_m: float = 4.635
    width_m: float = 1.892
    front_overhang_m: float = 0.845
    rear_overhang_m: float = 0.790
    wheel_track_m: float = 1.892
    minimum_turning_radius_m: float = 5.87

    def __post_init__(self) -> None:
        for value, name in (
            (self.wheelbase_m, "wheelbase"),
            (self.length_m, "vehicle length"),
            (self.width_m, "vehicle width"),
            (self.front_overhang_m, "front overhang"),
            (self.rear_overhang_m, "rear overhang"),
            (self.wheel_track_m, "wheel track"),
            (self.minimum_turning_radius_m, "minimum turning radius"),
        ):
            _require_positive(value, name)
        expected_length = (
            self.rear_overhang_m + self.wheelbase_m + self.front_overhang_m
        )
        if abs(self.length_m - expected_length) > 1.0e-6:
            raise ValueError(
                "vehicle length must equal rear overhang + wheelbase + front overhang"
            )
        if self.wheel_track_m > self.width_m:
            raise ValueError("wheel track cannot exceed vehicle width")
        if self.minimum_turning_radius_m <= self.wheelbase_m:
            raise ValueError("minimum turning radius must exceed wheelbase")

    @classmethod
    def ioniq5(cls) -> "VehicleGeometry":
        return cls()

    @property
    def longitudinal_min_m(self) -> float:
        return -self.rear_overhang_m

    @property
    def longitudinal_max_m(self) -> float:
        return self.wheelbase_m + self.front_overhang_m

    @property
    def half_width_m(self) -> float:
        return 0.5 * self.width_m

    @property
    def maximum_kinematic_steering_rad(self) -> float:
        return math.atan(self.wheelbase_m / self.minimum_turning_radius_m)

    @staticmethod
    def _transform(pose: Pose2D, local_x: float, local_y: float) -> Point2D:
        cosine = math.cos(pose.yaw)
        sine = math.sin(pose.yaw)
        return Point2D(
            pose.x + cosine * local_x - sine * local_y,
            pose.y + sine * local_x + cosine * local_y,
        )

    def body_corners(self, pose: Pose2D) -> Tuple[Point2D, ...]:
        """Return a counter-clockwise rectangle around the complete body."""

        return (
            self._transform(pose, self.longitudinal_min_m, -self.half_width_m),
            self._transform(pose, self.longitudinal_max_m, -self.half_width_m),
            self._transform(pose, self.longitudinal_max_m, self.half_width_m),
            self._transform(pose, self.longitudinal_min_m, self.half_width_m),
        )

    def wheel_contact_proxies(self, pose: Pose2D) -> Tuple[Point2D, ...]:
        half_track = 0.5 * self.wheel_track_m
        return (
            self._transform(pose, 0.0, half_track),
            self._transform(pose, 0.0, -half_track),
            self._transform(pose, self.wheelbase_m, half_track),
            self._transform(pose, self.wheelbase_m, -half_track),
        )
