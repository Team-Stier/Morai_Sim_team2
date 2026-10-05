"""Route-guided forward Hybrid A* used by the integrated planner node.

The implementation keeps the useful structure of the morai/test2 planner:
continuous bicycle-model poses, discretized search keys, steering-history cost,
reference-route guidance and full rectangular vehicle checks against measured
World Model points.  It intentionally has no ROS publishers, Safety Gate,
occupancy grid or planner-selection policy.
"""

from dataclasses import dataclass, fields
from enum import Enum
import heapq
import itertools
import math
import time
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


def normalize_yaw(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def angular_distance(first: float, second: float) -> float:
    return abs(normalize_yaw(first - second))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class HybridAStarConfig:
    xy_resolution_m: float = 1.0
    yaw_resolution_rad: float = math.radians(15.0)
    primitive_length_m: float = 3.0
    collision_check_step_m: float = 0.30
    steering_fractions: Tuple[float, ...] = (-1.0, -0.66, -0.33, 0.0, 0.33, 0.66, 1.0)
    maximum_steering_expansions: int = 5
    goal_position_tolerance_m: float = 1.25
    goal_yaw_tolerance_rad: float = math.radians(20.0)
    steering_cost_weight: float = 0.20
    steering_change_cost_weight: float = 0.35
    maximum_steering_change_rad: float = math.radians(18.0)
    reference_lateral_cost_weight: float = 1.50
    yaw_heuristic_weight_m_per_rad: float = 2.0
    heuristic_weight: float = 1.5
    max_search_nodes: int = 12000
    maximum_planning_time_sec: float = 0.30
    wheelbase_m: float = 3.0
    front_overhang_m: float = 3.845
    rear_overhang_m: float = 0.790
    vehicle_width_m: float = 1.892
    minimum_turning_radius_m: float = 5.87
    corridor_half_width_m: float = 6.0
    obstacle_margin_m: float = 0.0
    maximum_progress_regression_m: float = 0.50
    reference_join_min_distance_m: float = 12.0
    reference_join_distance_per_offset: float = 8.0
    reference_join_distance_per_heading: float = 30.0
    reference_shortcut_max_heading_change_rad: float = math.radians(45.0)
    reference_shortcut_max_start_heading_error_rad: float = math.radians(20.0)
    forbidden_boundary_height_tolerance_m: float = 1.5

    @classmethod
    def from_mapping(cls, value):
        allowed = {field.name for field in fields(cls)}
        values = {key: item for key, item in dict(value).items() if key in allowed or key in (
            "yaw_resolution_deg", "goal_yaw_tolerance_deg", "maximum_steering_change_deg",
            "reference_shortcut_max_heading_change_deg",
            "reference_shortcut_max_start_heading_error_deg")}
        if "yaw_resolution_deg" in values:
            values["yaw_resolution_rad"] = math.radians(values.pop("yaw_resolution_deg"))
        if "goal_yaw_tolerance_deg" in values:
            values["goal_yaw_tolerance_rad"] = math.radians(values.pop("goal_yaw_tolerance_deg"))
        if "maximum_steering_change_deg" in values:
            values["maximum_steering_change_rad"] = math.radians(values.pop("maximum_steering_change_deg"))
        if "reference_shortcut_max_heading_change_deg" in values:
            values["reference_shortcut_max_heading_change_rad"] = math.radians(
                values.pop("reference_shortcut_max_heading_change_deg"))
        if "reference_shortcut_max_start_heading_error_deg" in values:
            values["reference_shortcut_max_start_heading_error_rad"] = math.radians(
                values.pop("reference_shortcut_max_start_heading_error_deg"))
        if "steering_fractions" in values:
            values["steering_fractions"] = tuple(float(v) for v in values["steering_fractions"])
        return cls(**values)

    def __post_init__(self):
        positive = (
            self.xy_resolution_m,
            self.yaw_resolution_rad,
            self.primitive_length_m,
            self.collision_check_step_m,
            self.goal_position_tolerance_m,
            self.goal_yaw_tolerance_rad,
            self.wheelbase_m,
            self.front_overhang_m,
            self.rear_overhang_m,
            self.vehicle_width_m,
            self.minimum_turning_radius_m,
            self.corridor_half_width_m,
            self.maximum_planning_time_sec,
            self.maximum_steering_change_rad,
            self.heuristic_weight,
            self.reference_join_min_distance_m,
            self.reference_join_distance_per_offset,
            self.reference_join_distance_per_heading,
            self.reference_shortcut_max_heading_change_rad,
            self.reference_shortcut_max_start_heading_error_rad,
            self.forbidden_boundary_height_tolerance_m,
        )
        if any(not math.isfinite(v) or v <= 0.0 for v in positive):
            raise ValueError("Hybrid A* positive parameters must be finite")
        if self.collision_check_step_m > self.primitive_length_m:
            raise ValueError("collision_check_step_m cannot exceed primitive_length_m")
        if self.corridor_half_width_m <= self.vehicle_width_m / 2.0:
            raise ValueError("corridor_half_width_m must contain the vehicle body")
        if self.max_search_nodes <= 0 or self.maximum_steering_expansions <= 0:
            raise ValueError("Hybrid A* search bounds must be positive")
        if self.heuristic_weight < 1.0:
            raise ValueError("heuristic_weight must be at least one")
        if not self.steering_fractions or any(abs(v) > 1.0 for v in self.steering_fractions):
            raise ValueError("steering_fractions must be non-empty values in [-1, 1]")


class HybridPlanStatus(Enum):
    SUCCESS = "success"
    INVALID_REQUEST = "invalid_request"
    NO_PATH = "no_path"
    SEARCH_LIMIT = "search_limit"
    TIME_LIMIT = "time_limit"


@dataclass(frozen=True)
class HybridPathPoint:
    pose: Pose2D
    steering_rad: float
    distance_from_start_m: float


@dataclass(frozen=True)
class HybridPlanResult:
    status: HybridPlanStatus
    path: Tuple[HybridPathPoint, ...]
    expanded_nodes: int
    generated_nodes: int
    rejected_by_corridor: int
    rejected_by_obstacle: int
    elapsed_sec: float

    @property
    def success(self) -> bool:
        return self.status == HybridPlanStatus.SUCCESS


class _ReferenceGuide:
    def __init__(self, points: Sequence[Sequence[float]]) -> None:
        raw = np.asarray(points, dtype=float)
        if raw.ndim != 2 or raw.shape[1] < 2 or len(raw) < 2 or not np.isfinite(raw).all():
            raise ValueError("reference path must contain finite XY points")
        keep = np.r_[True, np.linalg.norm(np.diff(raw[:, :2], axis=0), axis=1) > 1.0e-9]
        self.xy = raw[keep, :2]
        self.z = raw[keep, 2] if raw.shape[1] >= 3 else np.zeros(len(self.xy))
        if len(self.xy) < 2:
            raise ValueError("reference path needs two distinct points")
        self.segment = np.diff(self.xy, axis=0)
        self.segment_length = np.linalg.norm(self.segment, axis=1)
        self.s = np.r_[0.0, np.cumsum(self.segment_length)]
        # RDDF points can contain sub-metre heading kinks.  The shortcut
        # follows a local smoothed centreline while corridor checks retain
        # the original measured route geometry.
        offsets = np.array([-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0])
        weights = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 4.0, 3.0, 2.0, 1.0])
        sampled = np.clip(self.s[:, None] + offsets[None, :], 0.0, self.s[-1])
        self.smooth_xy = np.column_stack([
            np.sum(np.interp(sampled, self.s, self.xy[:, j]) * weights[None, :], axis=1) /
            float(np.sum(weights)) for j in range(2)])
        span = 1.5
        before = np.clip(self.s - span, 0.0, self.s[-1])
        after = np.clip(self.s + span, 0.0, self.s[-1])
        before_xy = np.column_stack([np.interp(before, self.s, self.smooth_xy[:, j]) for j in range(2)])
        after_xy = np.column_stack([np.interp(after, self.s, self.smooth_xy[:, j]) for j in range(2)])
        tangent = (after_xy - before_xy) / np.maximum((after - before)[:, None], 1.0e-6)
        tangent_norm = np.linalg.norm(tangent, axis=1)
        self.tangent = tangent / np.maximum(tangent_norm[:, None], 1.0e-9)

    def sample(self, progress_m: float) -> Tuple[np.ndarray, float, float]:
        index = int(np.clip(np.searchsorted(self.s, progress_m, side="right") - 1,
                            0, len(self.segment) - 1))
        fraction = np.clip((progress_m - self.s[index]) / self.segment_length[index], 0.0, 1.0)
        length = self.segment_length[index]
        u = fraction
        point = ((2 * u ** 3 - 3 * u ** 2 + 1) * self.smooth_xy[index] +
                 (u ** 3 - 2 * u ** 2 + u) * length * self.tangent[index] +
                 (-2 * u ** 3 + 3 * u ** 2) * self.smooth_xy[index + 1] +
                 (u ** 3 - u ** 2) * length * self.tangent[index + 1])
        derivative = ((6 * u ** 2 - 6 * u) * self.smooth_xy[index] / length +
                      (3 * u ** 2 - 4 * u + 1) * self.tangent[index] +
                      (-6 * u ** 2 + 6 * u) * self.smooth_xy[index + 1] / length +
                      (3 * u ** 2 - 2 * u) * self.tangent[index + 1])
        heading = math.atan2(derivative[1], derivative[0])
        height = float(self.z[index] + fraction * (self.z[index + 1] - self.z[index]))
        return point, heading, height

    def project_signed(self, point: Sequence[float]) -> Tuple[float, float, float]:
        progress, _raw_lateral, _raw_heading = self.project(point)
        foot, heading, _height = self.sample(progress)
        delta = np.asarray(point, dtype=float)[:2] - foot
        signed = math.cos(heading) * delta[1] - math.sin(heading) * delta[0]
        return progress, float(signed), heading

    def project(self, point: Sequence[float]) -> Tuple[float, float, float]:
        p = np.asarray(point, dtype=float)[:2]
        u = np.clip(np.sum((p - self.xy[:-1]) * self.segment, axis=1) /
                    (self.segment_length * self.segment_length), 0.0, 1.0)
        projected = self.xy[:-1] + u[:, None] * self.segment
        residual = p - projected
        index = int(np.argmin(np.sum(residual * residual, axis=1)))
        tangent = self.segment[index] / self.segment_length[index]
        return (
            float(self.s[index] + u[index] * self.segment_length[index]),
            float(np.linalg.norm(residual[index])),
            math.atan2(tangent[1], tangent[0]),
        )


class _ForbiddenSegments:
    """Only map markings near this local route enter footprint intersection checks."""

    def __init__(self, lines: Iterable[Sequence[Sequence[float]]], guide: _ReferenceGuide,
                 corridor_half_width_m: float) -> None:
        low = np.min(guide.xy, axis=0) - corridor_half_width_m - 5.0
        high = np.max(guide.xy, axis=0) + corridor_half_width_m + 5.0
        near = []
        for line in lines:
            array = np.asarray(line, dtype=float)
            if array.ndim != 2 or array.shape[1] < 2 or not np.isfinite(array).all():
                raise ValueError("forbidden boundary must contain finite XY points")
            if len(array) < 2:
                continue
            xyz = np.column_stack((array[:, :2], array[:, 2] if array.shape[1] >= 3
                                   else np.zeros(len(array))))
            segments = np.stack((xyz[:-1], xyz[1:]), axis=1)
            keep = np.all(np.min(segments[:, :, :2], axis=1) <= high, axis=1) & np.all(
                np.max(segments[:, :, :2], axis=1) >= low, axis=1)
            near.extend(segments[keep])
        self.segments = np.asarray(near, dtype=float).reshape((-1, 2, 3))

    def hit(self, pose: Pose2D, height: float, config: HybridAStarConfig) -> bool:
        if len(self.segments) == 0:
            return False
        half_length = max(config.front_overhang_m, config.rear_overhang_m)
        radius = math.hypot(half_length, config.vehicle_width_m / 2.0)
        delta = self.segments[:, :, :2] - np.array([pose.x, pose.y])
        height_low = np.min(self.segments[:, :, 2], axis=1)
        height_high = np.max(self.segments[:, :, 2], axis=1)
        nearby = (np.all(delta.min(axis=1) <= radius, axis=1) &
                  np.all(delta.max(axis=1) >= -radius, axis=1) &
                  (height_low <= height + config.forbidden_boundary_height_tolerance_m) &
                  (height_high >= height - config.forbidden_boundary_height_tolerance_m))
        delta = delta[nearby]
        if len(delta) == 0:
            return False
        co, si = math.cos(pose.yaw), math.sin(pose.yaw)
        local = np.stack((co * delta[:, :, 0] + si * delta[:, :, 1],
                          -si * delta[:, :, 0] + co * delta[:, :, 1]), axis=2)
        start, vector = local[:, 0], local[:, 1] - local[:, 0]
        enter, leave = np.zeros(len(start)), np.ones(len(start))
        for axis, lower, upper in ((0, -config.rear_overhang_m, config.front_overhang_m),
                                   (1, -config.vehicle_width_m / 2.0, config.vehicle_width_m / 2.0)):
            parallel = np.abs(vector[:, axis]) < 1.0e-12
            leave[parallel & ((start[:, axis] < lower) | (start[:, axis] > upper))] = -1.0
            first = np.divide(lower - start[:, axis], vector[:, axis],
                              out=np.full(len(start), -np.inf), where=~parallel)
            second = np.divide(upper - start[:, axis], vector[:, axis],
                               out=np.full(len(start), np.inf), where=~parallel)
            enter = np.maximum(enter, np.minimum(first, second))
            leave = np.minimum(leave, np.maximum(first, second))
        return bool(np.any(enter <= leave))


@dataclass
class _Record:
    pose: Pose2D
    cost: float
    parent: int
    steering_index: int
    steering_rad: float
    edge: Tuple[Pose2D, ...]
    progress_m: float


class HybridAStarPlanner:
    def __init__(self, config: Optional[HybridAStarConfig] = None) -> None:
        self.config = config or HybridAStarConfig()
        maximum = math.atan(self.config.wheelbase_m / self.config.minimum_turning_radius_m)
        self.steering = tuple(maximum * fraction for fraction in self.config.steering_fractions)
        self.neutral_steering_index = min(range(len(self.steering)), key=lambda i: abs(self.steering[i]))
        margin = self.config.obstacle_margin_m
        self.obstacle_search_radius_m = math.hypot(
            max(abs(-self.config.rear_overhang_m - margin),
                abs(self.config.front_overhang_m + margin)),
            abs(self.config.vehicle_width_m / 2.0 + margin)) + 1.0e-9

    def _key(self, pose: Pose2D, steering_index: int) -> Tuple[int, int, int, int]:
        yaw_bins = max(1, int(round(2.0 * math.pi / self.config.yaw_resolution_rad)))
        return (
            int(round(pose.x / self.config.xy_resolution_m)),
            int(round(pose.y / self.config.xy_resolution_m)),
            int(round((normalize_yaw(pose.yaw) + math.pi) / self.config.yaw_resolution_rad)) % yaw_bins,
            steering_index,
        )

    def _propagate(self, pose: Pose2D, steering: float, distance: float) -> Pose2D:
        curvature = math.tan(steering) / self.config.wheelbase_m
        if abs(curvature) < 1.0e-10:
            return Pose2D(
                pose.x + distance * math.cos(pose.yaw),
                pose.y + distance * math.sin(pose.yaw),
                pose.yaw,
            )
        finish_yaw = pose.yaw + distance * curvature
        return Pose2D(
            pose.x + (math.sin(finish_yaw) - math.sin(pose.yaw)) / curvature,
            pose.y + (-math.cos(finish_yaw) + math.cos(pose.yaw)) / curvature,
            normalize_yaw(finish_yaw),
        )

    def _edge(self, pose: Pose2D, steering: float) -> Tuple[Pose2D, ...]:
        count = max(1, int(math.ceil(self.config.primitive_length_m /
                                     self.config.collision_check_step_m)))
        return tuple(self._propagate(pose, steering, self.config.primitive_length_m * i / count)
                     for i in range(1, count + 1))

    def _ramped_edge(self, pose: Pose2D, previous_steering: float,
                     target_steering: float) -> Tuple[Pose2D, ...]:
        count = max(1, int(math.ceil(self.config.primitive_length_m /
                                     self.config.collision_check_step_m)))
        distance = self.config.primitive_length_m / count
        points = []
        current = pose
        for index in range(count):
            fraction = (index + 0.5) / count
            steering = previous_steering + (target_steering - previous_steering) * fraction
            current = self._propagate(current, steering, distance)
            points.append(current)
        return tuple(points)

    def _corners(self, pose: Pose2D) -> np.ndarray:
        longitudinal = np.array([
            self.config.front_overhang_m,
            self.config.front_overhang_m,
            -self.config.rear_overhang_m,
            -self.config.rear_overhang_m,
        ])
        lateral = np.array([
            self.config.vehicle_width_m / 2.0,
            -self.config.vehicle_width_m / 2.0,
            self.config.vehicle_width_m / 2.0,
            -self.config.vehicle_width_m / 2.0,
        ])
        co, si = math.cos(pose.yaw), math.sin(pose.yaw)
        return np.column_stack((
            pose.x + co * longitudinal - si * lateral,
            pose.y + si * longitudinal + co * lateral,
        ))

    def _inside_corridor(self, pose: Pose2D, guide: _ReferenceGuide) -> bool:
        # Project all four vehicle corners onto the same route segments at once.
        # This is the same nearest-segment distance used by guide.project(),
        # without repeating its NumPy setup for every corner.
        corners = self._corners(pose)
        delta_x = corners[:, 0, None] - guide.xy[:-1, 0]
        delta_y = corners[:, 1, None] - guide.xy[:-1, 1]
        fraction = np.clip(
            (delta_x * guide.segment[:, 0] + delta_y * guide.segment[:, 1]) /
            (guide.segment_length * guide.segment_length), 0.0, 1.0)
        projected_x = guide.xy[:-1, 0] + fraction * guide.segment[:, 0]
        projected_y = guide.xy[:-1, 1] + fraction * guide.segment[:, 1]
        residual_x = corners[:, 0, None] - projected_x
        residual_y = corners[:, 1, None] - projected_y
        nearest_distance = np.sqrt(np.min(residual_x * residual_x + residual_y * residual_y,
                                          axis=1))
        return bool(np.all(nearest_distance <= self.config.corridor_half_width_m))

    def _hits_obstacle(self, pose: Pose2D, obstacle_points: np.ndarray) -> bool:
        if not len(obstacle_points):
            return False
        # Request setup sorts points by x. A point inside the inflated vehicle
        # rectangle must lie inside this conservative circumscribed circle.
        x = obstacle_points[:, 0]
        begin = int(np.searchsorted(x, pose.x - self.obstacle_search_radius_m, side="left"))
        end = int(np.searchsorted(x, pose.x + self.obstacle_search_radius_m, side="right"))
        if begin == end:
            return False
        delta = obstacle_points[begin:end, :2] - np.array([pose.x, pose.y])
        co, si = math.cos(pose.yaw), math.sin(pose.yaw)
        local_x = co * delta[:, 0] + si * delta[:, 1]
        local_y = -si * delta[:, 0] + co * delta[:, 1]
        margin = self.config.obstacle_margin_m
        return bool(np.any(
            (local_x >= -self.config.rear_overhang_m - margin) &
            (local_x <= self.config.front_overhang_m + margin) &
            (np.abs(local_y) <= self.config.vehicle_width_m / 2.0 + margin)
        ))

    def _pose_clear(self, pose: Pose2D, guide: _ReferenceGuide,
                    obstacles: np.ndarray, boundaries: _ForbiddenSegments) -> bool:
        if not self._inside_corridor(pose, guide) or self._hits_obstacle(pose, obstacles):
            return False
        progress, _lateral, _heading = guide.project((pose.x, pose.y))
        _point, _yaw, height = guide.sample(progress)
        return not boundaries.hit(pose, height, self.config)

    def path_clear(
        self,
        poses: Sequence[Sequence[float]],
        reference_path: Sequence[Sequence[float]],
        obstacle_points: Iterable[Sequence[float]],
        forbidden_boundaries: Iterable[Sequence[Sequence[float]]] = (),
    ) -> bool:
        """Recheck a retained path with the same footprint and map geometry."""
        try:
            guide = _ReferenceGuide(reference_path)
            boundaries = _ForbiddenSegments(forbidden_boundaries, guide,
                                            self.config.corridor_half_width_m)
            raw = np.asarray(poses, dtype=float)
            obstacles = np.asarray(tuple(obstacle_points), dtype=float).reshape((-1, 2))
            if raw.ndim != 2 or raw.shape[1] < 2 or len(raw) < 2 or not np.isfinite(raw).all():
                return False
            if not np.isfinite(obstacles).all():
                return False
        except (TypeError, ValueError):
            return False
        obstacles = obstacles[np.argsort(obstacles[:, 0])]
        cumulative = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(raw[:, :2], axis=0), axis=1))]
        keep = np.r_[True, np.diff(cumulative) > 1.0e-9]
        cumulative, raw = cumulative[keep], raw[keep]
        if len(raw) < 2 or cumulative[-1] <= 0.0:
            return False
        count = max(1, int(math.ceil(cumulative[-1] / self.config.collision_check_step_m)))
        sampled_s = np.linspace(0.0, cumulative[-1], count + 1)
        xy = np.column_stack([np.interp(sampled_s, cumulative, raw[:, column])
                              for column in range(2)])
        headings = np.arctan2(np.gradient(xy[:, 1], sampled_s),
                              np.gradient(xy[:, 0], sampled_s))
        return all(self._pose_clear(Pose2D(float(point[0]), float(point[1]), float(yaw)),
                                    guide, obstacles, boundaries)
                   for point, yaw in zip(xy, headings))

    def _reference_shortcut(
        self,
        start: Pose2D,
        goal: Pose2D,
        guide: _ReferenceGuide,
        obstacles: np.ndarray,
        boundaries: _ForbiddenSegments,
        initial_steering_rad: float,
    ) -> Optional[Tuple[HybridPathPoint, ...]]:
        """Track a nearly straight RDDF with a smooth, distance-scaled join."""
        start_s, offset, route_yaw = guide.project_signed((start.x, start.y))
        goal_s, _goal_lateral, _goal_yaw = guide.project((goal.x, goal.y))
        length = goal_s - start_s
        heading_error = normalize_yaw(start.yaw - route_yaw)
        max_heading = self.config.reference_shortcut_max_heading_change_rad
        if (length <= self.config.reference_join_min_distance_m + 2.0 or
                abs(heading_error) > self.config.reference_shortcut_max_start_heading_error_rad):
            return None
        overlapping = ((guide.s[:-1] <= goal_s) & (guide.s[1:] >= start_s))
        route_headings = np.arctan2(guide.segment[overlapping, 1], guide.segment[overlapping, 0])
        if any(angular_distance(value, route_yaw) > max_heading for value in route_headings):
            return None
        join_distance = max(self.config.reference_join_min_distance_m,
                            self.config.reference_join_distance_per_offset * abs(offset),
                            self.config.reference_join_distance_per_heading * abs(heading_error))
        if join_distance > length - 1.0:
            return None
        # Blend both XY coordinates from the exact vehicle pose, heading and
        # prior-path curvature.  This avoids a jump when smoothed RDDF points
        # differ slightly from the original centreline at the start station.
        base, _base_yaw, _height = guide.sample(start_s)
        epsilon = min(0.3, start_s, float(guide.s[-1]) - start_s)
        if epsilon <= 1.0e-3:
            return None
        before = guide.sample(start_s - epsilon)[0]
        after = guide.sample(start_s + epsilon)[0]
        base_first = (after - before) / (2.0 * epsilon)
        base_second = (after - 2.0 * base + before) / (epsilon * epsilon)
        start_tangent = np.array([math.cos(start.yaw), math.sin(start.yaw)])
        initial_curvature = math.tan(initial_steering_rad) / self.config.wheelbase_m
        start_second = initial_curvature * np.array([-math.sin(start.yaw), math.cos(start.yaw)])
        first = np.stack((np.array([start.x, start.y]) - base,
                          start_tangent - base_first,
                          (start_second - base_second) / 2.0))
        polynomial = np.array([[join_distance ** 3, join_distance ** 4, join_distance ** 5],
                               [3 * join_distance ** 2, 4 * join_distance ** 3,
                                5 * join_distance ** 4],
                               [6 * join_distance, 12 * join_distance ** 2,
                                20 * join_distance ** 3]])
        remainder = np.stack((-first[0] - first[1] * join_distance - first[2] * join_distance ** 2,
                              -first[1] - 2.0 * first[2] * join_distance,
                              -2.0 * first[2]))
        high_order = np.linalg.solve(polynomial, remainder)
        step = self.config.collision_check_step_m
        distances = np.linspace(0.0, length, max(1, int(math.ceil(length / step))) + 1)
        xy = np.empty((len(distances), 2))
        for index, distance in enumerate(distances):
            point, _heading, _height = guide.sample(start_s + distance)
            u = min(distance, join_distance)
            correction = (first[0] + first[1] * u + first[2] * u ** 2 +
                          high_order[0] * u ** 3 + high_order[1] * u ** 4 + high_order[2] * u ** 5)
            xy[index] = point + correction
        xy[0] = (start.x, start.y)
        travelled = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        if np.any(np.diff(travelled) <= 1.0e-6):
            return None
        headings = np.arctan2(np.gradient(xy[:, 1], travelled),
                              np.gradient(xy[:, 0], travelled))
        headings[0] = start.yaw
        steering = np.empty(len(xy))
        steering[0] = initial_steering_rad
        maximum_steering = max(abs(value) for value in self.steering)
        for index in range(1, len(xy)):
            curvature = normalize_yaw(float(headings[index] - headings[index - 1])) / (travelled[index] - travelled[index - 1])
            steering[index] = math.atan(self.config.wheelbase_m * curvature)
            allowed_change = (self.config.maximum_steering_change_rad *
                              (travelled[index] - travelled[index - 1]) /
                              self.config.primitive_length_m)
            if (abs(steering[index]) > maximum_steering + 1.0e-6 or
                    abs(steering[index] - steering[index - 1]) > allowed_change + 0.015):
                return None
        output = []
        for index, point in enumerate(xy):
            pose = Pose2D(float(point[0]), float(point[1]), float(headings[index]))
            if not self._pose_clear(pose, guide, obstacles, boundaries):
                return None
            output.append(HybridPathPoint(pose, float(steering[index]), float(travelled[index])))
        if (math.hypot(xy[-1, 0] - goal.x, xy[-1, 1] - goal.y) >
                self.config.goal_position_tolerance_m or
                angular_distance(float(headings[-1]), goal.yaw) > self.config.goal_yaw_tolerance_rad):
            return None
        return tuple(output)

    def _smooth_solution(
        self,
        start: Pose2D,
        goal: Pose2D,
        chain: Sequence[_Record],
        guide: _ReferenceGuide,
        obstacles: np.ndarray,
        boundaries: _ForbiddenSegments,
        initial_steering_rad: float,
    ) -> Optional[List[HybridPathPoint]]:
        """Use a continuous steering ramp when it preserves the solved route."""
        previous_steering = initial_steering_rad
        previous = start
        travelled = 0.0
        output = [HybridPathPoint(start, previous_steering, travelled)]
        for record in chain:
            if abs(record.steering_rad - previous_steering) > self.config.maximum_steering_change_rad:
                return None
            edge = self._ramped_edge(previous, previous_steering, record.steering_rad)
            for sample_index, pose in enumerate(edge, 1):
                if not self._pose_clear(pose, guide, obstacles, boundaries):
                    return None
                travelled += math.hypot(pose.x - previous.x, pose.y - previous.y)
                fraction = sample_index / len(edge)
                steering = previous_steering + (record.steering_rad - previous_steering) * fraction
                output.append(HybridPathPoint(pose, steering, travelled))
                previous = pose
            previous_steering = record.steering_rad
        if (math.hypot(previous.x - goal.x, previous.y - goal.y) >
                self.config.goal_position_tolerance_m or
                angular_distance(previous.yaw, goal.yaw) > self.config.goal_yaw_tolerance_rad):
            return None
        return output

    def _steering_indices(self, pose: Pose2D, goal: Pose2D,
                          current_index: int) -> Tuple[int, ...]:
        desired = normalize_yaw(math.atan2(goal.y - pose.y, goal.x - pose.x) - pose.yaw)
        desired_steering = math.atan2(self.config.wheelbase_m * desired,
                                      max(self.config.primitive_length_m, 1.0e-6))
        ordered = sorted(range(len(self.steering)), key=lambda i: abs(self.steering[i] - desired_steering))
        priority = [current_index, self.neutral_steering_index, 0, len(self.steering) - 1]
        priority.extend(ordered)
        unique = []
        for index in priority:
            if index not in unique:
                unique.append(index)
            if len(unique) >= self.config.maximum_steering_expansions:
                break
        return tuple(unique)

    def _heuristic(self, pose: Pose2D, goal: Pose2D, lateral_m: float) -> float:
        return (math.hypot(goal.x - pose.x, goal.y - pose.y) +
                self.config.yaw_heuristic_weight_m_per_rad * angular_distance(pose.yaw, goal.yaw) +
                self.config.reference_lateral_cost_weight * lateral_m)

    def _result(self, status, path, expanded, generated, corridor, obstacle, started):
        return HybridPlanResult(status, tuple(path), expanded, generated, corridor, obstacle,
                                time.monotonic() - started)

    def plan(
        self,
        start: Pose2D,
        goal: Pose2D,
        reference_path: Sequence[Sequence[float]],
        obstacle_points: Iterable[Sequence[float]] = (),
        forbidden_boundaries: Iterable[Sequence[Sequence[float]]] = (),
        initial_steering_rad: Optional[float] = None,
    ) -> HybridPlanResult:
        started = time.monotonic()
        try:
            guide = _ReferenceGuide(reference_path)
            boundaries = _ForbiddenSegments(forbidden_boundaries, guide,
                                            self.config.corridor_half_width_m)
            obstacles = np.asarray(tuple(obstacle_points), dtype=float)
            if obstacles.size == 0:
                obstacles = np.empty((0, 2))
            elif obstacles.ndim != 2 or obstacles.shape[1] < 2 or not np.isfinite(obstacles[:, :2]).all():
                raise ValueError("obstacle points must be finite XY values")
            if not all(math.isfinite(v) for v in (start.x, start.y, start.yaw, goal.x, goal.y,
                                                  goal.yaw)):
                raise ValueError("start and goal must be finite")
            if initial_steering_rad is None:
                progress, _lateral, _heading = guide.project((start.x, start.y))
                before = guide.sample(max(0.0, progress - 1.0))[1]
                after = guide.sample(min(float(guide.s[-1]), progress + 1.0))[1]
                span = min(float(guide.s[-1]), progress + 1.0) - max(0.0, progress - 1.0)
                curvature = normalize_yaw(after - before) / max(span, 1.0e-6)
                maximum_steering = max(abs(value) for value in self.steering)
                initial_steering_rad = float(np.clip(math.atan(self.config.wheelbase_m * curvature),
                                                     -maximum_steering, maximum_steering))
            if not math.isfinite(initial_steering_rad):
                raise ValueError("initial steering must be finite")
            if abs(initial_steering_rad) > max(abs(value) for value in self.steering):
                raise ValueError("initial steering exceeds the turning radius")
        except (TypeError, ValueError):
            return self._result(HybridPlanStatus.INVALID_REQUEST, (), 0, 0, 0, 0, started)
        obstacles = obstacles[np.argsort(obstacles[:, 0])]

        start_progress, start_lateral, _ = guide.project((start.x, start.y))
        if (start_lateral > self.config.corridor_half_width_m or
                not self._pose_clear(start, guide, obstacles, boundaries)):
            return self._result(HybridPlanStatus.NO_PATH, (), 0, 0, 1, 0, started)

        shortcut = self._reference_shortcut(start, goal, guide, obstacles,
                                            boundaries, initial_steering_rad)
        if shortcut is not None:
            return self._result(HybridPlanStatus.SUCCESS, shortcut, 0, 0, 0, 0, started)

        start_index = min(range(len(self.steering)),
                          key=lambda i: abs(self.steering[i] - initial_steering_rad))
        records: List[_Record] = [_Record(start, 0.0, -1, start_index,
                                         initial_steering_rad, (), start_progress)]
        best = {self._key(start, start_index): 0.0}
        queue = []
        counter = itertools.count()
        heapq.heappush(queue, (self.config.heuristic_weight *
                               self._heuristic(start, goal, start_lateral), next(counter), 0))
        expanded = generated = rejected_corridor = rejected_obstacle = 0
        goal_index = None
        status = HybridPlanStatus.NO_PATH

        while queue:
            if time.monotonic() - started > self.config.maximum_planning_time_sec:
                status = HybridPlanStatus.TIME_LIMIT
                break
            _priority, _order, index = heapq.heappop(queue)
            record = records[index]
            key = self._key(record.pose, record.steering_index)
            if record.cost > best.get(key, math.inf) + 1.0e-9:
                continue
            expanded += 1
            if (math.hypot(record.pose.x - goal.x, record.pose.y - goal.y) <=
                    self.config.goal_position_tolerance_m and
                    angular_distance(record.pose.yaw, goal.yaw) <= self.config.goal_yaw_tolerance_rad):
                goal_index = index
                status = HybridPlanStatus.SUCCESS
                break
            if expanded >= self.config.max_search_nodes:
                status = HybridPlanStatus.SEARCH_LIMIT
                break

            for steering_index in self._steering_indices(record.pose, goal, record.steering_index):
                steering = self.steering[steering_index]
                edge = self._edge(record.pose, steering)
                progress, lateral, _heading = guide.project((edge[-1].x, edge[-1].y))
                if progress + self.config.maximum_progress_regression_m < record.progress_m:
                    rejected_corridor += 1
                    continue
                if any(not self._inside_corridor(pose, guide) for pose in edge):
                    rejected_corridor += 1
                    continue
                if any(self._hits_obstacle(pose, obstacles) for pose in edge):
                    rejected_obstacle += 1
                    continue
                if len(boundaries.segments) and any(
                        boundaries.hit(pose, guide.sample(guide.project((pose.x, pose.y))[0])[2],
                                       self.config) for pose in edge):
                    rejected_corridor += 1
                    continue
                transition = (self.config.primitive_length_m +
                              self.config.steering_cost_weight * abs(steering) +
                              self.config.steering_change_cost_weight *
                              abs(steering - record.steering_rad) +
                              self.config.reference_lateral_cost_weight * lateral)
                cost = record.cost + transition
                successor = edge[-1]
                successor_key = self._key(successor, steering_index)
                if cost >= best.get(successor_key, math.inf) - 1.0e-9:
                    continue
                best[successor_key] = cost
                records.append(_Record(successor, cost, index, steering_index, steering, edge, progress))
                successor_index = len(records) - 1
                heapq.heappush(queue, (cost + self.config.heuristic_weight *
                                       self._heuristic(successor, goal, lateral),
                                       next(counter), successor_index))
                generated += 1

        if goal_index is None:
            return self._result(status, (), expanded, generated, rejected_corridor,
                                rejected_obstacle, started)

        chain = []
        cursor = goal_index
        while cursor > 0:
            chain.append(records[cursor])
            cursor = records[cursor].parent
        chain.reverse()
        output = [HybridPathPoint(start, initial_steering_rad, 0.0)]
        travelled = 0.0
        previous = start
        for record in chain:
            for pose in record.edge:
                travelled += math.hypot(pose.x - previous.x, pose.y - previous.y)
                output.append(HybridPathPoint(pose, record.steering_rad, travelled))
                previous = pose
        smoothed = self._smooth_solution(start, goal, chain, guide, obstacles,
                                         boundaries, initial_steering_rad)
        if smoothed is not None:
            output = smoothed
        return self._result(status, output, expanded, generated, rejected_corridor,
                            rejected_obstacle, started)
