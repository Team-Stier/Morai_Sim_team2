"""Forward-only Hybrid A* local path planner.

This is a ROS-independent adaptation of Team Stier's AMET planner structure:
search records retain continuous ``(x, y, yaw)`` poses, while the discovered
set uses a discretized key.  Successors are exact constant-steering bicycle
arcs and every interpolated pose is checked against the effective corridor.
"""

from __future__ import annotations

import heapq
import itertools
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Sequence, Tuple

from .collision import FootprintCollisionChecker, PoseValidity
from .corridor import (
    CorridorPolicy,
    CorridorPolicyDecision,
    CorridorPolicyInput,
    DrivingCorridor,
)
from .geometry import angular_distance, distance, is_finite_point, normalize_yaw
from .models import BoxObstacle, CircleObstacle, Point2D, Pose2D, VehicleGeometry


_IONIQ5_STEERING_LIMIT_RAD = math.atan(3.000 / 5.87)


@dataclass(frozen=True)
class HybridAStarConfig:
    xy_resolution_m: float = 0.50
    yaw_resolution_rad: float = math.radians(10.0)
    primitive_length_m: float = 1.00
    collision_check_step_m: float = 0.10
    steering_candidates_rad: Tuple[float, ...] = (
        -_IONIQ5_STEERING_LIMIT_RAD,
        -0.5 * _IONIQ5_STEERING_LIMIT_RAD,
        0.0,
        0.5 * _IONIQ5_STEERING_LIMIT_RAD,
        _IONIQ5_STEERING_LIMIT_RAD,
    )
    goal_position_tolerance_m: float = 0.75
    goal_yaw_tolerance_rad: float = math.radians(15.0)
    steering_cost_weight: float = 0.20
    steering_change_cost_weight: float = 0.35
    boundary_cost_weight: float = 1.50
    preferred_boundary_clearance_m: float = 0.75
    bearing_heuristic_weight_m_per_rad: float = 3.0
    yaw_heuristic_weight_m_per_rad: float = 4.0
    reference_lateral_heuristic_weight: float = 1.5
    reference_lookahead_m: float = 7.0
    maximum_steering_expansions: int = 5
    max_search_nodes: int = 50000
    maximum_planning_time_sec: float = 1.0
    maximum_collision_evaluations_per_primitive: int = 4096

    def __post_init__(self) -> None:
        positive_values = (
            (self.xy_resolution_m, "xy resolution"),
            (self.yaw_resolution_rad, "yaw resolution"),
            (self.primitive_length_m, "primitive length"),
            (self.collision_check_step_m, "collision-check step"),
            (self.goal_position_tolerance_m, "goal position tolerance"),
            (self.goal_yaw_tolerance_rad, "goal yaw tolerance"),
            (self.preferred_boundary_clearance_m, "preferred boundary clearance"),
        )
        for value, name in positive_values:
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        for value, name in (
            (self.steering_cost_weight, "steering cost weight"),
            (self.steering_change_cost_weight, "steering-change cost weight"),
            (self.boundary_cost_weight, "boundary cost weight"),
            (self.bearing_heuristic_weight_m_per_rad, "bearing heuristic weight"),
            (self.yaw_heuristic_weight_m_per_rad, "yaw heuristic weight"),
            (
                self.reference_lateral_heuristic_weight,
                "reference lateral heuristic weight",
            ),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("{} must be finite and non-negative".format(name))
        if self.collision_check_step_m > self.primitive_length_m:
            raise ValueError("collision-check step cannot exceed primitive length")
        if self.yaw_resolution_rad > 2.0 * math.pi:
            raise ValueError("yaw resolution cannot exceed 2*pi")
        if self.goal_yaw_tolerance_rad > math.pi:
            raise ValueError("goal yaw tolerance cannot exceed pi")
        if not math.isfinite(self.reference_lookahead_m) or self.reference_lookahead_m <= 0.0:
            raise ValueError("reference lookahead must be finite and positive")
        candidates = tuple(self.steering_candidates_rad)
        if not candidates:
            raise ValueError("steering candidate list must not be empty")
        if len(set(candidates)) != len(candidates):
            raise ValueError("steering candidates must be unique")
        for steering in candidates:
            if not math.isfinite(steering) or abs(steering) >= 0.5 * math.pi:
                raise ValueError("steering candidates must be finite and inside (-pi/2, pi/2)")
        if self.max_search_nodes <= 0:
            raise ValueError("maximum search node count must be positive")
        if self.maximum_steering_expansions <= 0:
            raise ValueError("maximum steering expansions must be positive")
        if (
            not math.isfinite(self.maximum_planning_time_sec)
            or self.maximum_planning_time_sec <= 0.0
        ):
            raise ValueError("maximum planning time must be finite and positive")
        if (
            isinstance(self.maximum_collision_evaluations_per_primitive, bool)
            or not isinstance(
                self.maximum_collision_evaluations_per_primitive, int
            )
            or self.maximum_collision_evaluations_per_primitive <= 0
        ):
            raise ValueError(
                "maximum collision evaluations per primitive must be a positive integer"
            )
        object.__setattr__(self, "steering_candidates_rad", candidates)


@dataclass(frozen=True)
class DiscreteStateKey:
    x_index: int
    y_index: int
    yaw_index: int
    steering_index: int


@dataclass(frozen=True)
class PlanningRequest:
    """Immutable planning input for one local corridor.

    Route progress and segment selection remain responsibilities of the global
    route manager.  This core validates the supplied ``source_link_id`` and
    ordered link list structurally, but cannot prove live route progress from
    geometry alone.
    """

    start: Pose2D
    goal: Pose2D
    corridor: DrivingCorridor
    policy: CorridorPolicyInput = CorridorPolicyInput()
    obstacles: Tuple[object, ...] = ()
    reference_path: Tuple[Point2D, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "obstacles", tuple(self.obstacles))
        reference = tuple(self.reference_path)
        if reference and len(reference) < 2:
            raise ValueError("reference path must be empty or contain at least two points")
        if any(not is_finite_point(point) for point in reference):
            raise ValueError("reference path contains a non-finite point")
        object.__setattr__(self, "reference_path", reference)


class PlanStatus(Enum):
    SUCCESS = "success"
    INVALID_REQUEST = "invalid_request"
    INVALID_START = "invalid_start"
    INVALID_GOAL = "invalid_goal"
    NO_PATH = "no_path"
    SEARCH_LIMIT = "search_limit"
    TIME_LIMIT = "time_limit"
    POLICY_BLOCKED = "policy_blocked"
    INTERNAL_VALIDATION_FAILURE = "internal_validation_failure"


@dataclass(frozen=True)
class PathPoint:
    pose: Pose2D
    steering_rad: float
    curvature_per_m: float
    distance_from_start_m: float


@dataclass(frozen=True)
class PlanningDiagnostics:
    message: str
    policy: CorridorPolicyDecision
    expanded_nodes: int
    generated_nodes: int
    rejected_by_boundary: int
    rejected_by_obstacle: int
    rejected_by_discretization: int
    minimum_body_boundary_clearance_m: float
    minimum_wheel_boundary_clearance_m: float


@dataclass(frozen=True)
class PlanResult:
    status: PlanStatus
    is_valid: bool
    path: Tuple[PathPoint, ...]
    diagnostics: PlanningDiagnostics


@dataclass
class _SearchRecord:
    key: DiscreteStateKey
    pose: Pose2D
    cost: float
    parent_index: int
    steering_rad: float
    curvature_per_m: float
    edge_samples: Tuple[Pose2D, ...]
    closed: bool = False


class _ReferenceGuide:
    """Small immutable polyline guide used only to order safe search states."""

    def __init__(self, points: Sequence[Point2D]) -> None:
        compact = []
        for point in points:
            if not compact or distance(compact[-1], point) > 1.0e-9:
                compact.append(point)
        if len(compact) < 2:
            raise ValueError("reference guide needs at least two distinct points")
        self.points = tuple(compact)
        cumulative = [0.0]
        for start, finish in zip(self.points, self.points[1:]):
            cumulative.append(cumulative[-1] + distance(start, finish))
        self.cumulative = tuple(cumulative)

    def project(self, point: Point2D) -> Tuple[float, float]:
        best_progress = 0.0
        best_squared = math.inf
        for index, (start, finish) in enumerate(zip(self.points, self.points[1:])):
            dx = finish.x - start.x
            dy = finish.y - start.y
            length_squared = dx * dx + dy * dy
            if length_squared <= 1.0e-18:
                continue
            ratio = (
                (point.x - start.x) * dx + (point.y - start.y) * dy
            ) / length_squared
            ratio = max(0.0, min(1.0, ratio))
            projected_x = start.x + ratio * dx
            projected_y = start.y + ratio * dy
            squared = (point.x - projected_x) ** 2 + (point.y - projected_y) ** 2
            if squared < best_squared:
                best_squared = squared
                best_progress = self.cumulative[index] + ratio * math.sqrt(
                    length_squared
                )
        return best_progress, math.sqrt(best_squared)

    def point_at(self, progress_m: float) -> Point2D:
        progress = max(0.0, min(self.cumulative[-1], progress_m))
        for index, finish_progress in enumerate(self.cumulative[1:]):
            if progress <= finish_progress:
                start_progress = self.cumulative[index]
                span = finish_progress - start_progress
                ratio = 0.0 if span <= 1.0e-12 else (progress - start_progress) / span
                start = self.points[index]
                finish = self.points[index + 1]
                return Point2D(
                    start.x + ratio * (finish.x - start.x),
                    start.y + ratio * (finish.y - start.y),
                )
        return self.points[-1]

    def lookahead(
        self, pose: Pose2D, goal: Pose2D, lookahead_m: float
    ) -> Tuple[Point2D, float, float]:
        pose_progress, lateral_distance = self.project(Point2D(pose.x, pose.y))
        goal_progress, _ = self.project(Point2D(goal.x, goal.y))
        target_progress = min(
            max(pose_progress, goal_progress), pose_progress + lookahead_m
        )
        return self.point_at(target_progress), lateral_distance, max(
            0.0, goal_progress - pose_progress
        )


class HybridAStarPlanner:
    def __init__(
        self,
        vehicle: Optional[VehicleGeometry] = None,
        config: Optional[HybridAStarConfig] = None,
        boundary_clearance_m: float = 0.20,
        obstacle_clearance_m: float = 0.0,
    ) -> None:
        self.vehicle = vehicle if vehicle is not None else VehicleGeometry.ioniq5()
        self.config = config if config is not None else HybridAStarConfig()
        maximum_steering = self.vehicle.maximum_kinematic_steering_rad
        if any(
            abs(candidate) > maximum_steering + 1.0e-12
            for candidate in self.config.steering_candidates_rad
        ):
            raise ValueError(
                "steering candidate exceeds the vehicle minimum-turning-radius limit"
            )
        if self.config.preferred_boundary_clearance_m < boundary_clearance_m:
            raise ValueError(
                "preferred boundary clearance cannot be below the hard clearance"
            )
        self.checker = FootprintCollisionChecker(
            self.vehicle, boundary_clearance_m, obstacle_clearance_m
        )
        self.policy = CorridorPolicy()
        neutral_candidates = [
            index
            for index, steering in enumerate(self.config.steering_candidates_rad)
            if abs(steering) <= 1.0e-12
        ]
        if len(neutral_candidates) != 1:
            raise ValueError("steering candidates must contain exactly one neutral bin")
        self._neutral_steering_index = neutral_candidates[0]

    def discrete_key(
        self, pose: Pose2D, steering_index: Optional[int] = None
    ) -> DiscreteStateKey:
        if steering_index is None:
            steering_index = self._neutral_steering_index
        if steering_index < 0 or steering_index >= len(self.config.steering_candidates_rad):
            raise ValueError("steering index is outside the configured candidate bins")
        yaw = normalize_yaw(pose.yaw)
        yaw_index = int(math.floor((yaw + math.pi) / self.config.yaw_resolution_rad))
        return DiscreteStateKey(
            int(math.floor(pose.x / self.config.xy_resolution_m)),
            int(math.floor(pose.y / self.config.xy_resolution_m)),
            yaw_index,
            steering_index,
        )

    def _reached_goal(self, pose: Pose2D, goal: Pose2D) -> bool:
        return (
            distance(Point2D(pose.x, pose.y), Point2D(goal.x, goal.y))
            <= self.config.goal_position_tolerance_m
            and angular_distance(pose.yaw, goal.yaw)
            <= self.config.goal_yaw_tolerance_rad
        )

    def _heuristic(
        self,
        pose: Pose2D,
        goal: Pose2D,
        guide: Optional[_ReferenceGuide] = None,
    ) -> float:
        goal_distance = distance(
            Point2D(pose.x, pose.y), Point2D(goal.x, goal.y)
        )
        target = Point2D(goal.x, goal.y)
        lateral_distance = 0.0
        guided_distance = goal_distance
        if guide is not None:
            target, lateral_distance, remaining_progress = guide.lookahead(
                pose, goal, self.config.reference_lookahead_m
            )
            guided_distance = max(goal_distance, remaining_progress)
        target_distance = distance(Point2D(pose.x, pose.y), target)
        if target_distance <= 1.0e-9:
            bearing_error = 0.0
        else:
            bearing = math.atan2(target.y - pose.y, target.x - pose.x)
            bearing_error = angular_distance(pose.yaw, bearing)
        # The angular terms intentionally guide this bounded local search; the
        # planner promises feasibility/safety, not globally optimal cost.
        return (
            guided_distance
            + self.config.bearing_heuristic_weight_m_per_rad * bearing_error
            + self.config.yaw_heuristic_weight_m_per_rad
            * angular_distance(pose.yaw, goal.yaw)
            + self.config.reference_lateral_heuristic_weight * lateral_distance
        )

    def _transition_cost(
        self,
        steering_rad: float,
        previous_steering_rad: float,
        minimum_clearance_m: float,
    ) -> float:
        length = self.config.primitive_length_m
        cost = length
        cost += self.config.steering_cost_weight * abs(steering_rad) * length
        cost += (
            self.config.steering_change_cost_weight
            * abs(steering_rad - previous_steering_rad)
            * length
        )
        preferred = self.config.preferred_boundary_clearance_m
        hard = self.checker.boundary_clearance_m
        if minimum_clearance_m < preferred and preferred > hard:
            normalized_deficit = max(
                0.0, min(1.0, (preferred - minimum_clearance_m) / (preferred - hard))
            )
            cost += (
                self.config.boundary_cost_weight
                * normalized_deficit
                * normalized_deficit
                * length
            )
        return cost

    def _steering_expansions(
        self,
        pose: Pose2D,
        goal: Pose2D,
        previous_steering_rad: float,
        guide: Optional[_ReferenceGuide] = None,
    ) -> Tuple[Tuple[int, float], ...]:
        """Select a bounded, goal-guided subset of the steering lattice.

        The complete configured lattice retains fine enough steering values for
        the narrow KATRI lanes.  Expanding every value at every node is too slow
        for a rolling ROS planner, so each node keeps the previous/neutral/end
        controls and fills the remaining budget around a pure-pursuit control.
        This is a bounded Hybrid A* search and does not claim completeness or
        globally optimal cost; every generated primitive is still certified by
        the exact same hard-wall checks.
        """

        candidates = self.config.steering_candidates_rad
        budget = min(self.config.maximum_steering_expansions, len(candidates))
        if budget == len(candidates):
            return tuple(enumerate(candidates))

        target = Point2D(goal.x, goal.y)
        if guide is not None:
            target, _lateral, _remaining = guide.lookahead(
                pose, goal, self.config.reference_lookahead_m
            )
        dx = target.x - pose.x
        dy = target.y - pose.y
        lookahead = max(math.hypot(dx, dy), self.config.primitive_length_m)
        target_bearing = math.atan2(dy, dx)
        heading_error = normalize_yaw(target_bearing - pose.yaw)
        desired_curvature = 2.0 * math.sin(heading_error) / lookahead
        maximum_curvature = 1.0 / self.vehicle.minimum_turning_radius_m
        desired_curvature = max(
            -maximum_curvature,
            min(maximum_curvature, desired_curvature),
        )
        desired_steering = math.atan(
            self.vehicle.wheelbase_m * desired_curvature
        )

        ranked = sorted(
            range(len(candidates)),
            key=lambda index: (
                abs(candidates[index] - desired_steering),
                abs(candidates[index]),
                index,
            ),
        )
        previous_index = min(
            range(len(candidates)),
            key=lambda index: abs(candidates[index] - previous_steering_rad),
        )
        selected = []
        for index in (
            previous_index,
            self._neutral_steering_index,
            0,
            len(candidates) - 1,
        ):
            if index not in selected and len(selected) < budget:
                selected.append(index)
        for index in ranked:
            if index not in selected and len(selected) < budget:
                selected.append(index)
        return tuple((index, candidates[index]) for index in selected)

    @staticmethod
    def _rejection_kind(results: Sequence[PoseValidity]) -> str:
        for result in results:
            if not result.valid:
                if result.reason in (
                    "collision_check_time_limit",
                ):
                    return "time_limit"
                if result.reason == "collision_check_evaluation_limit":
                    return "evaluation_limit"
                if result.reason == "obstacle_clearance":
                    return "obstacle"
                return "boundary"
        return "boundary"

    def validate_path(
        self,
        path: Sequence[PathPoint],
        corridor: DrivingCorridor,
        policy: CorridorPolicyInput,
        obstacles: Sequence[object] = (),
        deadline_monotonic: float = None,
    ) -> bool:
        """Continuously revalidate an existing path against a newer snapshot.

        Map geometry and controls stay fixed; this is primarily used by the
        ROS adapter when an obstacle observation changes during search.
        """

        points = tuple(path)
        effective = self.policy.apply(corridor, policy)
        if not points or effective.decision.fail_closed:
            return False
        if (
            deadline_monotonic is not None
            and time.monotonic() >= deadline_monotonic
        ):
            return False
        if not self.checker.check_pose(
            points[0].pose, effective, obstacles
        ).valid:
            return False
        if (
            deadline_monotonic is not None
            and time.monotonic() >= deadline_monotonic
        ):
            return False
        index = 1
        while index < len(points):
            first_index = index - 1
            steering = points[index].steering_rad
            while (
                index + 1 < len(points)
                and abs(points[index + 1].steering_rad - steering) <= 1.0e-12
            ):
                index += 1
            travel = (
                points[index].distance_from_start_m
                - points[first_index].distance_from_start_m
            )
            if not math.isfinite(travel) or travel <= 0.0:
                return False
            valid, samples, _results = self.checker.check_bicycle_primitive(
                points[first_index].pose,
                steering,
                travel,
                self.vehicle.wheelbase_m,
                self.config.collision_check_step_m,
                effective,
                obstacles,
                deadline_monotonic=deadline_monotonic,
                maximum_evaluated_poses=(
                    self.config.maximum_collision_evaluations_per_primitive
                ),
            )
            if not valid or not samples:
                return False
            expected = points[index].pose
            actual = samples[-1]
            if (
                distance(
                    Point2D(expected.x, expected.y),
                    Point2D(actual.x, actual.y),
                )
                > 1.0e-5
                or angular_distance(expected.yaw, actual.yaw) > 1.0e-5
            ):
                return False
            index += 1
        return True

    def _diagnostics(
        self,
        message: str,
        decision: CorridorPolicyDecision,
        expanded: int,
        generated: int,
        rejected_boundary: int,
        rejected_obstacle: int,
        rejected_discretization: int,
        minimum_body: float = math.inf,
        minimum_wheel: float = math.inf,
    ) -> PlanningDiagnostics:
        return PlanningDiagnostics(
            message,
            decision,
            expanded,
            generated,
            rejected_boundary,
            rejected_obstacle,
            rejected_discretization,
            minimum_body,
            minimum_wheel,
        )

    def _failure(
        self,
        status: PlanStatus,
        message: str,
        decision: CorridorPolicyDecision,
        expanded: int = 0,
        generated: int = 0,
        rejected_boundary: int = 0,
        rejected_obstacle: int = 0,
        rejected_discretization: int = 0,
    ) -> PlanResult:
        return PlanResult(
            status,
            False,
            (),
            self._diagnostics(
                message,
                decision,
                expanded,
                generated,
                rejected_boundary,
                rejected_obstacle,
                rejected_discretization,
            ),
        )

    def plan(
        self, request: PlanningRequest, deadline_monotonic: float = None
    ) -> PlanResult:
        deadline = time.monotonic() + self.config.maximum_planning_time_sec
        if deadline_monotonic is not None:
            if not math.isfinite(deadline_monotonic):
                raise ValueError("planning deadline must be finite")
            deadline = min(deadline, deadline_monotonic)
        effective = self.policy.apply(request.corridor, request.policy)
        decision = effective.decision
        if decision.fail_closed:
            return self._failure(
                PlanStatus.POLICY_BLOCKED,
                "corridor policy denied the requested wall opening",
                decision,
            )
        if time.monotonic() >= deadline:
            return self._failure(
                PlanStatus.TIME_LIMIT,
                "Hybrid A* deadline elapsed before input validation",
                decision,
            )
        guide = (
            _ReferenceGuide(request.reference_path)
            if request.reference_path
            else None
        )
        if time.monotonic() >= deadline:
            return self._failure(
                PlanStatus.TIME_LIMIT,
                "Hybrid A* deadline elapsed while building the route guide",
                decision,
            )
        start_validity = self.checker.check_pose(
            request.start, effective, request.obstacles
        )
        if not start_validity.valid:
            return self._failure(
                PlanStatus.INVALID_START,
                "start pose rejected: {}".format(start_validity.reason),
                decision,
            )
        if time.monotonic() >= deadline:
            return self._failure(
                PlanStatus.TIME_LIMIT,
                "Hybrid A* deadline elapsed while validating the start",
                decision,
            )
        goal_validity = self.checker.check_pose(request.goal, effective, request.obstacles)
        if not goal_validity.valid:
            return self._failure(
                PlanStatus.INVALID_GOAL,
                "goal pose rejected: {}".format(goal_validity.reason),
                decision,
            )

        start_pose = Pose2D(request.start.x, request.start.y, normalize_yaw(request.start.yaw))
        records = [
            _SearchRecord(
                self.discrete_key(start_pose),
                start_pose,
                0.0,
                -1,
                0.0,
                0.0,
                (),
            )
        ]
        discovered: Dict[DiscreteStateKey, int] = {records[0].key: 0}
        counter = itertools.count()
        open_heap = [
            (
                self._heuristic(start_pose, request.goal, guide),
                0.0,
                next(counter),
                0,
            )
        ]
        expanded = 0
        rejected_boundary = 0
        rejected_obstacle = 0
        rejected_discretization = 0
        hit_search_limit = False
        hit_time_limit = False
        final_index = -1

        while open_heap:
            if time.monotonic() >= deadline:
                hit_time_limit = True
                break
            _priority, queued_cost, _tie, record_index = heapq.heappop(open_heap)
            current = records[record_index]
            latest = discovered.get(current.key)
            if current.closed or latest != record_index or queued_cost > current.cost + 1.0e-12:
                continue
            current.closed = True
            expanded += 1
            if self._reached_goal(current.pose, request.goal):
                final_index = record_index
                break

            if len(records) >= self.config.max_search_nodes:
                hit_search_limit = True
                break

            for steering_index, steering_rad in self._steering_expansions(
                current.pose, request.goal, current.steering_rad, guide
            ):
                if time.monotonic() >= deadline:
                    hit_time_limit = True
                    break
                primitive_valid, primitive, validity_results = (
                    self.checker.check_bicycle_primitive(
                        current.pose,
                        steering_rad,
                        self.config.primitive_length_m,
                        self.vehicle.wheelbase_m,
                        self.config.collision_check_step_m,
                        effective,
                        request.obstacles,
                        deadline_monotonic=deadline,
                        maximum_evaluated_poses=(
                            self.config.maximum_collision_evaluations_per_primitive
                        ),
                    )
                )
                if not primitive_valid:
                    rejection_kind = self._rejection_kind(validity_results)
                    if rejection_kind == "time_limit":
                        hit_time_limit = True
                        break
                    if rejection_kind == "obstacle":
                        rejected_obstacle += 1
                    else:
                        # A per-primitive proof budget is conservative local
                        # rejection, not evidence that the planner's absolute
                        # deadline elapsed. Other steering primitives may have
                        # short, certifiable intervals and must still be tried.
                        rejected_boundary += 1
                    continue
                child_pose = primitive[-1]
                child_key = self.discrete_key(child_pose, steering_index)
                minimum_clearance = min(
                    result.minimum_wheel_boundary_clearance_m
                    for result in validity_results
                )
                child_cost = current.cost + self._transition_cost(
                    steering_rad, current.steering_rad, minimum_clearance
                )
                found_index = discovered.get(child_key)
                if found_index is not None and child_cost >= records[found_index].cost - 1.0e-12:
                    rejected_discretization += 1
                    continue
                if len(records) >= self.config.max_search_nodes:
                    hit_search_limit = True
                    break
                child_index = len(records)
                curvature = math.tan(steering_rad) / self.vehicle.wheelbase_m
                records.append(
                    _SearchRecord(
                        child_key,
                        child_pose,
                        child_cost,
                        record_index,
                        steering_rad,
                        curvature,
                        primitive,
                    )
                )
                discovered[child_key] = child_index
                priority = child_cost + self._heuristic(
                    child_pose, request.goal, guide
                )
                heapq.heappush(
                    open_heap,
                    (priority, child_cost, next(counter), child_index),
                )
            if hit_search_limit or hit_time_limit:
                break

        if final_index < 0:
            if hit_time_limit:
                status = PlanStatus.TIME_LIMIT
                message = "Hybrid A* reached its configured wall-clock limit"
            elif hit_search_limit:
                status = PlanStatus.SEARCH_LIMIT
                message = "Hybrid A* reached its configured search-node limit"
            elif decision.fail_closed:
                status = PlanStatus.POLICY_BLOCKED
                message = "no path under the fail-closed corridor policy"
            else:
                status = PlanStatus.NO_PATH
                message = "Hybrid A* exhausted the safe forward search space"
            return self._failure(
                status,
                message,
                decision,
                expanded,
                len(records),
                rejected_boundary,
                rejected_obstacle,
                rejected_discretization,
            )

        chain = []
        cursor = final_index
        while cursor >= 0:
            chain.append(cursor)
            cursor = records[cursor].parent_index
        chain.reverse()
        sample_data = [(records[chain[0]].pose, 0.0, 0.0)]
        for child_index in chain[1:]:
            child = records[child_index]
            sample_data.extend(
                (sample, child.steering_rad, child.curvature_per_m)
                for sample in child.edge_samples
            )

        path_points = []
        travelled = 0.0
        previous_pose = sample_data[0][0]
        previous_validity = None
        minimum_body = math.inf
        minimum_wheel = math.inf
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
        for index, (pose, steering, curvature) in enumerate(sample_data):
            if time.monotonic() >= deadline:
                return self._failure(
                    PlanStatus.TIME_LIMIT,
                    "Hybrid A* reached its configured wall-clock limit during validation",
                    decision,
                    expanded,
                    len(records),
                    rejected_boundary,
                    rejected_obstacle,
                    rejected_discretization,
                )
            segment_travel = 0.0
            if index:
                segment_travel = (
                    angular_distance(pose.yaw, previous_pose.yaw) / abs(curvature)
                    if abs(curvature) > 1.0e-9
                    else distance(
                        Point2D(previous_pose.x, previous_pose.y),
                        Point2D(pose.x, pose.y),
                    )
                )
                travelled += segment_travel
            validity = self.checker.check_pose(pose, effective, request.obstacles)
            if not validity.valid:
                return self._failure(
                    PlanStatus.INTERNAL_VALIDATION_FAILURE,
                    "reconstructed path failed validation: {}".format(validity.reason),
                    decision,
                    expanded,
                    len(records),
                    rejected_boundary,
                    rejected_obstacle,
                    rejected_discretization,
                )
            minimum_body = min(minimum_body, validity.minimum_body_boundary_clearance_m)
            minimum_wheel = min(minimum_wheel, validity.minimum_wheel_boundary_clearance_m)
            if previous_validity is not None:
                # The primitive checker has already proven the whole interval
                # safe. These Lipschitz bounds make the published diagnostic
                # conservative between the dense output samples as well.
                minimum_body = min(
                    minimum_body,
                    max(
                        0.0,
                        min(
                            previous_validity.minimum_body_boundary_clearance_m,
                            validity.minimum_body_boundary_clearance_m,
                        )
                        - 0.5
                        * segment_travel
                        * (1.0 + body_radius * abs(curvature)),
                    ),
                )
                minimum_wheel = min(
                    minimum_wheel,
                    max(
                        self.checker.boundary_clearance_m,
                        min(
                            previous_validity.minimum_wheel_boundary_clearance_m,
                            validity.minimum_wheel_boundary_clearance_m,
                        )
                        - 0.5
                        * segment_travel
                        * (1.0 + wheel_radius * abs(curvature)),
                    ),
                )
            path_points.append(PathPoint(pose, steering, curvature, travelled))
            previous_pose = pose
            previous_validity = validity

        return PlanResult(
            PlanStatus.SUCCESS,
            True,
            tuple(path_points),
            self._diagnostics(
                "valid collision-checked Hybrid A* path",
                decision,
                expanded,
                len(records),
                rejected_boundary,
                rejected_obstacle,
                rejected_discretization,
                minimum_body,
                minimum_wheel,
            ),
        )
