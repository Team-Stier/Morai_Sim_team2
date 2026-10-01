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


def build_hybrid_candidate(
    planner: HybridAStarPlanner,
    lane: Lane,
    start: Pose2D,
    progress_s: float,
    obstacle_points: Iterable[Sequence[float]],
    runtime_config: Mapping[str, object],
    loop_route: bool,
) -> HybridCandidateResult:
    step = float(runtime_config["reference_step_m"])
    goal_distance = float(runtime_config["local_goal_distance_m"])
    if step <= 0.0 or goal_distance <= planner.config.primitive_length_m:
        raise ValueError("Hybrid runtime reference step and goal distance are invalid")

    route_length = float(lane.s[-1])
    begin = float(progress_s) - max(2.0, planner.config.rear_overhang_m + 1.0)
    finish = float(progress_s) + goal_distance + planner.config.front_overhang_m + 2.0
    reference_s = np.arange(begin, finish + step * 0.5, step)
    reference_xyz = _sample_lane(lane, reference_s, loop_route)
    goal_xyz = _sample_lane(lane, np.array([progress_s + goal_distance]), loop_route)[0]
    tangent_points = _sample_lane(
        lane,
        np.array([progress_s + goal_distance - step, progress_s + goal_distance + step]),
        loop_route,
    )
    tangent = tangent_points[1, :2] - tangent_points[0, :2]
    goal = Pose2D(float(goal_xyz[0]), float(goal_xyz[1]), math.atan2(tangent[1], tangent[0]))
    plan = planner.plan(start, goal, reference_xyz[:, :2], obstacle_points)
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
