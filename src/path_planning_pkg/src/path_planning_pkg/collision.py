"""Strict vehicle-footprint, lane-wall, and circular-obstacle checks."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple

from .corridor import EffectiveCorridor
from .geometry import (
    GEOMETRY_EPSILON,
    is_finite_pose,
    locate_point_in_polygon,
    point_segment_distance,
    polygon_edges,
    propagate_bicycle,
    segment_distance,
    segments_intersect,
    PointLocation,
)
from .models import BoxObstacle, CircleObstacle, Point2D, Pose2D, VehicleGeometry


@dataclass(frozen=True)
class PoseValidity:
    valid: bool
    reason: str
    minimum_body_boundary_clearance_m: float
    minimum_wheel_boundary_clearance_m: float
    minimum_obstacle_clearance_m: float


def _rectangle_circle_clearance(
    pose: Pose2D, vehicle: VehicleGeometry, obstacle: CircleObstacle
) -> float:
    dx = obstacle.center.x - pose.x
    dy = obstacle.center.y - pose.y
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    nearest_x = max(
        vehicle.longitudinal_min_m,
        min(vehicle.longitudinal_max_m, local_x),
    )
    nearest_y = max(-vehicle.half_width_m, min(vehicle.half_width_m, local_y))
    return math.hypot(local_x - nearest_x, local_y - nearest_y) - obstacle.radius_m


def _polygon_clearance(first, second):
    first_edges = polygon_edges(first)
    second_edges = polygon_edges(second)
    if any(
        locate_point_in_polygon(point, second) is not PointLocation.OUTSIDE
        for point in first
    ) or any(
        locate_point_in_polygon(point, first) is not PointLocation.OUTSIDE
        for point in second
    ):
        return 0.0
    if any(
        segments_intersect(a, b, c, d)
        for a, b in first_edges
        for c, d in second_edges
    ):
        return 0.0
    return min(
        segment_distance(a, b, c, d)
        for a, b in first_edges
        for c, d in second_edges
    )


def _point_in_polygon_fast(point, vertices):
    """Ray test used after hard-wall clearance handles boundary contact."""
    inside = False
    previous = vertices[-1]
    for current in vertices:
        if (current.y > point.y) != (previous.y > point.y):
            crossing_x = current.x + (
                (point.y - current.y)
                * (previous.x - current.x)
                / (previous.y - current.y)
            )
            if point.x < crossing_x:
                inside = not inside
        previous = current
    return inside


def _point_in_union_fast(point, polygons, polygon_bounds):
    for polygon, bounds in zip(polygons, polygon_bounds):
        minimum_x, minimum_y, maximum_x, maximum_y = bounds
        if not (
            minimum_x <= point.x <= maximum_x
            and minimum_y <= point.y <= maximum_y
        ):
            continue
        if _point_in_polygon_fast(point, polygon):
            return True
    return False


def _local_point(pose, point):
    dx = point.x - pose.x
    dy = point.y - pose.y
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return Point2D(cosine * dx + sine * dy, -sine * dx + cosine * dy)


def _point_rectangle_distance(point, minimum_x, maximum_x, half_width):
    dx = max(minimum_x - point.x, 0.0, point.x - maximum_x)
    dy = max(-half_width - point.y, 0.0, point.y - half_width)
    return math.hypot(dx, dy)


def _segment_intersects_rectangle(start, finish, minimum_x, maximum_x, half_width):
    """Liang-Barsky intersection against a closed axis-aligned rectangle."""
    dx = finish.x - start.x
    dy = finish.y - start.y
    lower = 0.0
    upper = 1.0
    for p_value, q_value in (
        (-dx, start.x - minimum_x),
        (dx, maximum_x - start.x),
        (-dy, start.y + half_width),
        (dy, half_width - start.y),
    ):
        if abs(p_value) <= GEOMETRY_EPSILON:
            if q_value < 0.0:
                return False
            continue
        ratio = q_value / p_value
        if p_value < 0.0:
            lower = max(lower, ratio)
        else:
            upper = min(upper, ratio)
        if lower > upper:
            return False
    return True


def _bounded_rectangle_segment_clearance(
    pose, minimum_x, maximum_x, half_width, start, finish
):
    local_start = _local_point(pose, start)
    local_finish = _local_point(pose, finish)
    if _segment_intersects_rectangle(
        local_start, local_finish, minimum_x, maximum_x, half_width
    ):
        return 0.0
    corners = (
        Point2D(minimum_x, -half_width),
        Point2D(maximum_x, -half_width),
        Point2D(maximum_x, half_width),
        Point2D(minimum_x, half_width),
    )
    return min(
        _point_rectangle_distance(
            local_start, minimum_x, maximum_x, half_width
        ),
        _point_rectangle_distance(
            local_finish, minimum_x, maximum_x, half_width
        ),
        *(point_segment_distance(corner, local_start, local_finish) for corner in corners)
    )


def _rectangle_segment_clearance(pose, vehicle, start, finish):
    return _bounded_rectangle_segment_clearance(
        pose,
        vehicle.longitudinal_min_m,
        vehicle.longitudinal_max_m,
        vehicle.half_width_m,
        start,
        finish,
    )


class FootprintCollisionChecker:
    """Validate wheel contact against map walls and body against obstacles.

    ``boundary_clearance_m`` must be strictly positive.  A pose is accepted at
    or above that clearance (with a small numerical tolerance); actual line
    contact is consequently always rejected.
    """

    def __init__(
        self,
        vehicle: VehicleGeometry,
        boundary_clearance_m: float,
        obstacle_clearance_m: float = 0.0,
    ) -> None:
        if not math.isfinite(boundary_clearance_m) or boundary_clearance_m <= 0.0:
            raise ValueError("boundary clearance must be finite and strictly positive")
        if not math.isfinite(obstacle_clearance_m) or obstacle_clearance_m < 0.0:
            raise ValueError("obstacle clearance must be finite and non-negative")
        self.vehicle = vehicle
        self.boundary_clearance_m = boundary_clearance_m
        self.obstacle_clearance_m = obstacle_clearance_m

    def check_pose(
        self,
        pose: Pose2D,
        corridor: EffectiveCorridor,
        obstacles: Sequence[object] = (),
        include_body_boundary_clearance: bool = True,
    ) -> PoseValidity:
        if not is_finite_pose(pose):
            return PoseValidity(False, "non_finite_pose", -math.inf, -math.inf, -math.inf)

        wheels = self.vehicle.wheel_contact_proxies(pose)
        body = (
            self.vehicle.body_corners(pose)
            if obstacles or include_body_boundary_clearance
            else ()
        )
        polygons = corridor.polygons
        # The competition lane rule is explicitly wheel-contact based.  Body
        # overhang can legitimately sweep over paint in a tight turn while all
        # tyres stay inside their lane, so lane walls constrain conservative
        # outer tyre-contact proxies rather than the rectangular body.  The
        # complete body is still used below for dynamic-obstacle collision.
        if any(
            not _point_in_union_fast(wheel, polygons, corridor.polygon_bounds)
            for wheel in wheels
        ):
            return PoseValidity(
                False, "wheel_outside_corridor", -math.inf, -math.inf, math.inf
            )
        # Exact segment checks are only needed near the vehicle.  Every edge
        # rejected by this AABB lower bound is at least this distance from the
        # complete body (and therefore from the in-body wheel proxies).  The
        # capped value remains a conservative lower bound for both the hard
        # clearance test and the continuous-motion Lipschitz certificate.
        query_distance_m = max(0.75, self.boundary_clearance_m + 0.50)
        query_footprint = body if include_body_boundary_clearance else wheels
        footprint_minimum_x = min(point.x for point in query_footprint)
        footprint_minimum_y = min(point.y for point in query_footprint)
        footprint_maximum_x = max(point.x for point in query_footprint)
        footprint_maximum_y = max(point.y for point in query_footprint)
        minimum_body = query_distance_m if include_body_boundary_clearance else math.inf
        minimum_wheel = query_distance_m
        nearby_indices = corridor.nearby_hard_edge_indices(
            footprint_minimum_x,
            footprint_minimum_y,
            footprint_maximum_x,
            footprint_maximum_y,
            query_distance_m,
        )
        for edge_index in nearby_indices:
            hard_edge = corridor.hard_edges[edge_index]
            edge_bounds = corridor.hard_edge_bounds[edge_index]
            edge_minimum_x, edge_minimum_y, edge_maximum_x, edge_maximum_y = (
                edge_bounds
            )
            lower_dx = max(
                edge_minimum_x - footprint_maximum_x,
                footprint_minimum_x - edge_maximum_x,
                0.0,
            )
            lower_dy = max(
                edge_minimum_y - footprint_maximum_y,
                footprint_minimum_y - edge_maximum_y,
                0.0,
            )
            if math.hypot(lower_dx, lower_dy) > query_distance_m:
                continue
            if include_body_boundary_clearance:
                minimum_body = min(
                    minimum_body,
                    _rectangle_segment_clearance(
                        pose, self.vehicle, hard_edge.start, hard_edge.finish
                    ),
                )
            # Treat the convex hull of the four conservative tyre-contact
            # points as the wall barrier footprint.  This rejects a malformed
            # or teleported pose with wheels already on opposite sides of a
            # closed marking, while still excluding body overhangs from the
            # lane-contact rule.
            minimum_wheel = min(
                minimum_wheel,
                _bounded_rectangle_segment_clearance(
                    pose,
                    0.0,
                    self.vehicle.wheelbase_m,
                    0.5 * self.vehicle.wheel_track_m,
                    hard_edge.start,
                    hard_edge.finish,
                ),
            )
        if minimum_wheel + GEOMETRY_EPSILON < self.boundary_clearance_m:
            return PoseValidity(
                False, "wheel_boundary_clearance", minimum_body, minimum_wheel, math.inf
            )

        minimum_obstacle = math.inf
        for obstacle in obstacles:
            if isinstance(obstacle, CircleObstacle):
                clearance = _rectangle_circle_clearance(
                    pose, self.vehicle, obstacle
                )
            elif isinstance(obstacle, BoxObstacle):
                clearance = _polygon_clearance(body, obstacle.corners())
            else:
                return PoseValidity(
                    False,
                    "unknown_obstacle_type",
                    minimum_body,
                    minimum_wheel,
                    -math.inf,
                )
            minimum_obstacle = min(minimum_obstacle, clearance)
            if clearance <= self.obstacle_clearance_m + GEOMETRY_EPSILON:
                return PoseValidity(
                    False,
                    "obstacle_clearance",
                    minimum_body,
                    minimum_wheel,
                    minimum_obstacle,
                )
        return PoseValidity(True, "valid", minimum_body, minimum_wheel, minimum_obstacle)

    def check_poses(
        self,
        poses: Iterable[Pose2D],
        corridor: EffectiveCorridor,
        obstacles: Sequence[object] = (),
    ) -> Tuple[bool, Tuple[PoseValidity, ...]]:
        results = tuple(self.check_pose(pose, corridor, obstacles) for pose in poses)
        return bool(results) and all(result.valid for result in results), results

    def check_bicycle_primitive(
        self,
        start: Pose2D,
        steering_rad: float,
        distance_m: float,
        wheelbase_m: float,
        maximum_sample_step_m: float,
        corridor: EffectiveCorridor,
        obstacles: Sequence[object] = (),
        deadline_monotonic: float = None,
        maximum_evaluated_poses: int = 4096,
    ) -> Tuple[bool, Tuple[Pose2D, ...], Tuple[PoseValidity, ...]]:
        """Generate and continuously certify a constant-steering primitive.

        Fixed sampling alone can miss a tangent contact between samples.  Each
        interval is therefore certified with a Lipschitz displacement bound for
        every point of the rigid vehicle.  Ambiguous intervals are bisected
        until proven safe; failure to prove safety is rejected.
        """

        for value, name in (
            (steering_rad, "steering"),
            (distance_m, "primitive distance"),
            (wheelbase_m, "wheelbase"),
            (maximum_sample_step_m, "maximum sample step"),
        ):
            if not math.isfinite(value):
                raise ValueError("{} must be finite".format(name))
        if deadline_monotonic is not None and not math.isfinite(
            deadline_monotonic
        ):
            raise ValueError("collision-check deadline must be finite")
        if (
            isinstance(maximum_evaluated_poses, bool)
            or not isinstance(maximum_evaluated_poses, int)
            or maximum_evaluated_poses <= 0
        ):
            raise ValueError("maximum evaluated poses must be a positive integer")
        if distance_m <= 0.0 or wheelbase_m <= 0.0 or maximum_sample_step_m <= 0.0:
            raise ValueError("primitive distance, wheelbase, and sample step must be positive")
        count = int(math.ceil(distance_m / maximum_sample_step_m))
        interval_distance = distance_m / float(count)
        samples = []
        evaluated = []

        def deadline_reached():
            return (
                deadline_monotonic is not None
                and time.monotonic() >= deadline_monotonic
            )

        def append_time_limit():
            evaluated.append(
                PoseValidity(
                    False,
                    "collision_check_time_limit",
                    -math.inf,
                    -math.inf,
                    -math.inf,
                )
            )

        def evaluation_limit_reached():
            if len(evaluated) < maximum_evaluated_poses:
                return False
            evaluated.append(
                PoseValidity(
                    False,
                    "collision_check_evaluation_limit",
                    -math.inf,
                    -math.inf,
                    -math.inf,
                )
            )
            return True

        if deadline_reached():
            append_time_limit()
            return False, tuple(samples), tuple(evaluated)
        start_validity = self.check_pose(
            start,
            corridor,
            obstacles,
            include_body_boundary_clearance=False,
        )
        evaluated.append(start_validity)
        if not start_validity.valid:
            return False, tuple(samples), tuple(evaluated)

        curvature = math.tan(steering_rad) / wheelbase_m
        wheel_radius = max(
            math.hypot(longitudinal, 0.5 * self.vehicle.wheel_track_m)
            for longitudinal in (0.0, self.vehicle.wheelbase_m)
        )
        body_radius = max(
            math.hypot(longitudinal, self.vehicle.half_width_m)
            for longitudinal in (
                self.vehicle.longitudinal_min_m,
                self.vehicle.longitudinal_max_m,
            )
        )
        wheel_point_speed_bound = 1.0 + wheel_radius * abs(curvature)
        body_point_speed_bound = 1.0 + body_radius * abs(curvature)

        def interval_is_safe(
            interval_start: Pose2D,
            first: PoseValidity,
            interval_finish: Pose2D,
            second: PoseValidity,
            travel: float,
            depth: int,
        ) -> bool:
            if deadline_reached():
                append_time_limit()
                return False
            if not first.valid or not second.valid:
                return False
            # Every instant is at most half an interval from its nearer endpoint.
            maximum_wheel_displacement = (
                0.5 * travel * wheel_point_speed_bound
            )
            maximum_body_displacement = 0.5 * travel * body_point_speed_bound
            first_boundary = first.minimum_wheel_boundary_clearance_m
            second_boundary = second.minimum_wheel_boundary_clearance_m
            boundary_proven = min(first_boundary, second_boundary) > (
                self.boundary_clearance_m
                + maximum_wheel_displacement
                + GEOMETRY_EPSILON
            )
            obstacle_proven = min(
                first.minimum_obstacle_clearance_m,
                second.minimum_obstacle_clearance_m,
            ) > (
                self.obstacle_clearance_m
                + maximum_body_displacement
                + GEOMETRY_EPSILON
            )
            if boundary_proven and obstacle_proven:
                return True
            if depth >= 32:
                evaluated.append(
                    PoseValidity(
                        False,
                        "continuous_safety_not_proven",
                        min(
                            first.minimum_body_boundary_clearance_m,
                            second.minimum_body_boundary_clearance_m,
                        ),
                        min(
                            first.minimum_wheel_boundary_clearance_m,
                            second.minimum_wheel_boundary_clearance_m,
                        ),
                        min(
                            first.minimum_obstacle_clearance_m,
                            second.minimum_obstacle_clearance_m,
                        ),
                    )
                )
                return False
            # The interval may already be certified by its evaluated endpoints.
            # Consume the pose budget only when another midpoint evaluation is
            # actually required, so a cap of N permits exactly N evaluations.
            if evaluation_limit_reached():
                return False
            midpoint = propagate_bicycle(
                interval_start, steering_rad, 0.5 * travel, wheelbase_m
            )
            midpoint_validity = self.check_pose(
                midpoint,
                corridor,
                obstacles,
                include_body_boundary_clearance=False,
            )
            evaluated.append(midpoint_validity)
            if not midpoint_validity.valid:
                return False
            return interval_is_safe(
                interval_start,
                first,
                midpoint,
                midpoint_validity,
                0.5 * travel,
                depth + 1,
            ) and interval_is_safe(
                midpoint,
                midpoint_validity,
                interval_finish,
                second,
                0.5 * travel,
                depth + 1,
            )

        previous_pose = start
        previous_validity = start_validity
        for index in range(1, count + 1):
            if deadline_reached():
                append_time_limit()
                return False, tuple(samples), tuple(evaluated)
            if evaluation_limit_reached():
                return False, tuple(samples), tuple(evaluated)
            sample = propagate_bicycle(
                start,
                steering_rad,
                interval_distance * float(index),
                wheelbase_m,
            )
            samples.append(sample)
            sample_validity = self.check_pose(
                sample,
                corridor,
                obstacles,
                include_body_boundary_clearance=False,
            )
            evaluated.append(sample_validity)
            if not interval_is_safe(
                previous_pose,
                previous_validity,
                sample,
                sample_validity,
                interval_distance,
                0,
            ):
                return False, tuple(samples), tuple(evaluated)
            previous_pose = sample
            previous_validity = sample_validity
        return True, tuple(samples), tuple(evaluated)
