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
    reference_lateral_cost_weight: float = 1.50
    yaw_heuristic_weight_m_per_rad: float = 2.0
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

    @classmethod
    def from_mapping(cls, value):
        allowed = {field.name for field in fields(cls)}
        values = {key: item for key, item in dict(value).items() if key in allowed or key in (
            "yaw_resolution_deg", "goal_yaw_tolerance_deg")}
        if "yaw_resolution_deg" in values:
            values["yaw_resolution_rad"] = math.radians(values.pop("yaw_resolution_deg"))
        if "goal_yaw_tolerance_deg" in values:
            values["goal_yaw_tolerance_rad"] = math.radians(values.pop("goal_yaw_tolerance_deg"))
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
        )
        if any(not math.isfinite(v) or v <= 0.0 for v in positive):
            raise ValueError("Hybrid A* positive parameters must be finite")
        if self.collision_check_step_m > self.primitive_length_m:
            raise ValueError("collision_check_step_m cannot exceed primitive_length_m")
        if self.corridor_half_width_m <= self.vehicle_width_m / 2.0:
            raise ValueError("corridor_half_width_m must contain the vehicle body")
        if self.max_search_nodes <= 0 or self.maximum_steering_expansions <= 0:
            raise ValueError("Hybrid A* search bounds must be positive")
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
        if raw.ndim != 2 or raw.shape[1] < 2 or len(raw) < 2 or not np.isfinite(raw[:, :2]).all():
            raise ValueError("reference path must contain finite XY points")
        keep = np.r_[True, np.linalg.norm(np.diff(raw[:, :2], axis=0), axis=1) > 1.0e-9]
        self.xy = raw[keep, :2]
        if len(self.xy) < 2:
            raise ValueError("reference path needs two distinct points")
        self.segment = np.diff(self.xy, axis=0)
        self.segment_length = np.linalg.norm(self.segment, axis=1)
        self.s = np.r_[0.0, np.cumsum(self.segment_length)]

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
        return all(guide.project(corner)[1] <= self.config.corridor_half_width_m
                   for corner in self._corners(pose))

    def _hits_obstacle(self, pose: Pose2D, obstacle_points: np.ndarray) -> bool:
        if not len(obstacle_points):
            return False
        delta = obstacle_points[:, :2] - np.array([pose.x, pose.y])
        co, si = math.cos(pose.yaw), math.sin(pose.yaw)
        local_x = co * delta[:, 0] + si * delta[:, 1]
        local_y = -si * delta[:, 0] + co * delta[:, 1]
        margin = self.config.obstacle_margin_m
        return bool(np.any(
            (local_x >= -self.config.rear_overhang_m - margin) &
            (local_x <= self.config.front_overhang_m + margin) &
            (np.abs(local_y) <= self.config.vehicle_width_m / 2.0 + margin)
        ))

    def _steering_indices(self, pose: Pose2D, goal: Pose2D, current_index: int) -> Tuple[int, ...]:
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
    ) -> HybridPlanResult:
        started = time.monotonic()
        try:
            guide = _ReferenceGuide(reference_path)
            obstacles = np.asarray(tuple(obstacle_points), dtype=float)
            if obstacles.size == 0:
                obstacles = np.empty((0, 2))
            elif obstacles.ndim != 2 or obstacles.shape[1] < 2 or not np.isfinite(obstacles[:, :2]).all():
                raise ValueError("obstacle points must be finite XY values")
            if not all(math.isfinite(v) for v in (start.x, start.y, start.yaw, goal.x, goal.y, goal.yaw)):
                raise ValueError("start and goal must be finite")
        except (TypeError, ValueError):
            return self._result(HybridPlanStatus.INVALID_REQUEST, (), 0, 0, 0, 0, started)

        start_progress, start_lateral, _ = guide.project((start.x, start.y))
        if (start_lateral > self.config.corridor_half_width_m or
                not self._inside_corridor(start, guide) or self._hits_obstacle(start, obstacles)):
            return self._result(HybridPlanStatus.NO_PATH, (), 0, 0, 1, 0, started)

        records: List[_Record] = [_Record(start, 0.0, -1, self.neutral_steering_index,
                                         0.0, (), start_progress)]
        best = {self._key(start, self.neutral_steering_index): 0.0}
        queue = []
        counter = itertools.count()
        heapq.heappush(queue, (self._heuristic(start, goal, start_lateral), next(counter), 0))
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
                heapq.heappush(queue, (cost + self._heuristic(successor, goal, lateral),
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
        output = [HybridPathPoint(start, 0.0, 0.0)]
        travelled = 0.0
        previous = start
        for record in chain:
            for pose in record.edge:
                travelled += math.hypot(pose.x - previous.x, pose.y - previous.y)
                output.append(HybridPathPoint(pose, record.steering_rad, travelled))
                previous = pose
        return self._result(status, output, expanded, generated, rejected_corridor,
                            rejected_obstacle, started)
