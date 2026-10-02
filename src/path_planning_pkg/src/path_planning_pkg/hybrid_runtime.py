"""Pure adapter from the existing RDDF/World Model representation to Hybrid A*."""

from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Optional, Sequence

import numpy as np

from .frenet import Candidate, Lane
from .hybrid_astar import HybridAStarPlanner, HybridPlanResult, Pose2D


@dataclass(frozen=True)
class HybridCandidateResult:
    candidate: Optional[Candidate]
    plan: HybridPlanResult


def _sample_lane(lane: Lane, query_s: np.ndarray, loop_route: bool) -> np.ndarray:
    length = float(lane.s[-1])
    if loop_route:
        sampled_s = np.mod(query_s, length)
    else:
        sampled_s = np.clip(query_s, lane.s[0], lane.s[-1])
    return np.column_stack([np.interp(sampled_s, lane.s, lane.xy[:, column])
                            for column in range(lane.xy.shape[1])])


def _project_local(reference_xy: np.ndarray, reference_s: np.ndarray, point: Sequence[float]) -> float:
    vectors = np.diff(reference_xy[:, :2], axis=0)
    lengths_squared = np.sum(vectors * vectors, axis=1)
    valid = lengths_squared > 1.0e-12
    ratios = np.zeros(len(vectors))
    ratios[valid] = np.clip(
        np.sum((np.asarray(point)[:2] - reference_xy[:-1, :2])[valid] * vectors[valid], axis=1) /
        lengths_squared[valid], 0.0, 1.0)
    projected = reference_xy[:-1, :2] + ratios[:, None] * vectors
    residual = projected - np.asarray(point)[:2]
    index = int(np.argmin(np.sum(residual * residual, axis=1)))
    return float(reference_s[index] + ratios[index] *
                 (reference_s[index + 1] - reference_s[index]))


def flatten_obstacle_points(objects: Iterable[object]) -> np.ndarray:
    points = []
    for tracked in objects:
        points.extend((point.x, point.y) for point in tracked.points)
    return np.asarray(points, dtype=float).reshape((-1, 2)) if points else np.empty((0, 2))


def _reference_segment(planner: HybridAStarPlanner, lane: Lane, progress_s: float,
                       runtime_config: Mapping[str, object], loop_route: bool):
    step = float(runtime_config["reference_step_m"])
    goal_distance = float(runtime_config["local_goal_distance_m"])
    if step <= 0.0 or goal_distance <= planner.config.primitive_length_m:
        raise ValueError("Hybrid runtime reference step and goal distance are invalid")
    begin = float(progress_s) - max(2.0, planner.config.rear_overhang_m + 1.0)
    finish = float(progress_s) + goal_distance + planner.config.front_overhang_m + 2.0
    reference_s = np.arange(begin, finish + step * 0.5, step)
    return reference_s, _sample_lane(lane, reference_s, loop_route)


def _previous_steering(planner: HybridAStarPlanner, previous_path: Optional[np.ndarray],
                       start: Pose2D) -> Optional[float]:
    if previous_path is None:
        return None
    raw = np.asarray(previous_path, dtype=float)
    if raw.ndim != 2 or raw.shape[1] < 2 or len(raw) < 3 or not np.isfinite(raw).all():
        return None
    cumulative = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(raw[:, :2], axis=0), axis=1))]
    keep = np.r_[True, np.diff(cumulative) > 1.0e-6]
    cumulative, xy = cumulative[keep], raw[keep, :2]
    if len(xy) < 3 or cumulative[-1] < 2.0:
        return None
    nearest = int(np.argmin(np.sum((xy - np.array([start.x, start.y])) ** 2, axis=1)))
    center = float(np.clip(cumulative[nearest], 1.0, cumulative[-1] - 1.0))
    samples = np.column_stack([np.interp([center - 1.0, center, center + 1.0],
                                         cumulative, xy[:, column]) for column in range(2)])
    first = math.atan2(*(samples[1] - samples[0])[::-1])
    second = math.atan2(*(samples[2] - samples[1])[::-1])
    curvature = ((second - first + math.pi) % (2.0 * math.pi) - math.pi)
    steering = math.atan(planner.config.wheelbase_m * curvature)
    maximum = max(abs(value) for value in planner.steering)
    return float(np.clip(steering, -maximum, maximum))


def hybrid_path_clear(
    planner: HybridAStarPlanner,
    lane: Lane,
    progress_s: float,
    candidate: Candidate,
    obstacle_points: Iterable[Sequence[float]],
    runtime_config: Mapping[str, object],
    loop_route: bool,
    forbidden_boundaries: Iterable[Sequence[Sequence[float]]] = (),
) -> bool:
    """Validate a retained candidate suffix against the current local scene."""
    _reference_s, reference_xyz = _reference_segment(
        planner, lane, progress_s, runtime_config, loop_route)
    return planner.path_clear(candidate.xy, reference_xyz, obstacle_points,
                              forbidden_boundaries)


def build_hybrid_candidate(
    planner: HybridAStarPlanner,
    lane: Lane,
    start: Pose2D,
    progress_s: float,
    obstacle_points: Iterable[Sequence[float]],
    runtime_config: Mapping[str, object],
    loop_route: bool,
    previous_path: Optional[np.ndarray] = None,
    forbidden_boundaries: Iterable[Sequence[Sequence[float]]] = (),
) -> HybridCandidateResult:
    step = float(runtime_config["reference_step_m"])
    goal_distance = float(runtime_config["local_goal_distance_m"])
    route_length = float(lane.s[-1])
    reference_s, reference_xyz = _reference_segment(planner, lane, progress_s,
                                                     runtime_config, loop_route)
    goal_xyz = _sample_lane(lane, np.array([progress_s + goal_distance]), loop_route)[0]
    tangent_points = _sample_lane(
        lane,
        np.array([progress_s + goal_distance - step, progress_s + goal_distance + step]),
        loop_route,
    )
    tangent = tangent_points[1, :2] - tangent_points[0, :2]
    goal = Pose2D(float(goal_xyz[0]), float(goal_xyz[1]), math.atan2(tangent[1], tangent[0]))
    plan = planner.plan(start, goal, reference_xyz, obstacle_points,
                        forbidden_boundaries=forbidden_boundaries,
                        initial_steering_rad=_previous_steering(planner, previous_path, start))
    if not plan.success or len(plan.path) < 2:
        return HybridCandidateResult(None, plan)

    xy = np.zeros((len(plan.path), 3))
    route_s = np.zeros(len(plan.path))
    for index, point in enumerate(plan.path):
        xy[index, :2] = (point.pose.x, point.pose.y)
        route_s[index] = _project_local(reference_xyz, reference_s, xy[index, :2])
    route_s = np.maximum.accumulate(route_s)
    wrapped_s = np.mod(route_s, route_length) if loop_route else np.clip(route_s, 0.0, route_length)
    xy[:, 2] = np.interp(wrapped_s, lane.s, lane.xy[:, 2])
    limits = np.interp(wrapped_s, lane.s, lane.limits)
    candidate = Candidate(
        "hybrid_astar",
        lane.id,
        xy,
        route_s,
        limits,
        changes=0,
        change_end=float(route_s[-1]),
    )
    candidate.reason = "hybrid_astar_path"
    return HybridCandidateResult(candidate, plan)
