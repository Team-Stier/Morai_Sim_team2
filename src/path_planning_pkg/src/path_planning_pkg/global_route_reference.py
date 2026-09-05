"""Noise-tolerant slicing of the immutable competition reference path.

This module deliberately has no ROS dependency.  The global route manager owns
localization/progress; the local planner only cross-checks that progress against
the latched path and extracts a short, forward-only planning reference.
"""

from __future__ import annotations

import bisect
import hashlib
import math
import struct
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

from .geometry import angular_distance, distance, is_finite_point
from .models import Point2D, Pose2D


@dataclass(frozen=True)
class GlobalRouteTrackingConfig:
    expected_point_count: int
    expected_length_m: float
    length_tolerance_m: float
    maximum_point_spacing_m: float
    closure_tolerance_m: float
    maximum_index_progress_error_m: float
    maximum_progress_regression_m: float
    projection_backward_points: int
    projection_forward_points: int
    matching_tube_radius_m: float
    maximum_projection_progress_error_m: float
    maximum_projection_heading_error_rad: float
    turn_detection_heading_change_rad: float
    turn_goal_distance_m: float
    heading_score_weight: float
    goal_scan_step_m: float
    expected_xy_sha256: str = ""

    def __post_init__(self):
        positive = (
            (self.expected_point_count, "expected point count"),
            (self.expected_length_m, "expected length"),
            (self.length_tolerance_m, "length tolerance"),
            (self.maximum_point_spacing_m, "maximum point spacing"),
            (self.closure_tolerance_m, "closure tolerance"),
            (self.maximum_index_progress_error_m, "index/progress tolerance"),
            (self.projection_forward_points, "forward projection points"),
            (self.matching_tube_radius_m, "matching tube radius"),
            (
                self.maximum_projection_progress_error_m,
                "projection/progress tolerance",
            ),
            (
                self.maximum_projection_heading_error_rad,
                "projection heading tolerance",
            ),
            (self.turn_detection_heading_change_rad, "turn heading-change threshold"),
            (self.turn_goal_distance_m, "turn goal distance"),
            (self.goal_scan_step_m, "goal scan step"),
        )
        for value, label in positive:
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError("{} must be finite and positive".format(label))
        if self.projection_backward_points < 0:
            raise ValueError("backward projection points must be non-negative")
        if (
            not math.isfinite(self.maximum_progress_regression_m)
            or self.maximum_progress_regression_m < 0.0
        ):
            raise ValueError("maximum progress regression must be finite and non-negative")
        if not math.isfinite(self.heading_score_weight) or self.heading_score_weight < 0.0:
            raise ValueError("heading score weight must be finite and non-negative")
        if self.maximum_projection_heading_error_rad > math.pi:
            raise ValueError("projection heading tolerance cannot exceed pi")
        if self.turn_detection_heading_change_rad > math.pi:
            raise ValueError("turn heading-change threshold cannot exceed pi")
        digest = str(self.expected_xy_sha256).lower()
        if digest and (
            len(digest) != 64 or any(value not in "0123456789abcdef" for value in digest)
        ):
            raise ValueError("expected XY SHA-256 must contain 64 hexadecimal digits")


@dataclass(frozen=True)
class RouteProjection:
    progress_m: float
    lateral_distance_m: float
    heading_error_rad: float


@dataclass(frozen=True)
class GlobalRouteSlice:
    reference_path: Tuple[Point2D, ...]
    goal: Pose2D
    accepted_progress_m: float
    ego_projection: RouteProjection


@dataclass(frozen=True)
class _Segment:
    start: Point2D
    finish: Point2D
    start_progress_m: float
    length_m: float
    heading_rad: float
    start_source_index: int
    finish_source_index: int


class GlobalRouteReference:
    """Validated geometry corresponding one-for-one with a received Path."""

    def __init__(
        self,
        raw_points: Sequence[Point2D],
        config: GlobalRouteTrackingConfig,
    ):
        points = tuple(raw_points)
        if len(points) != config.expected_point_count:
            raise ValueError(
                "global path point count mismatch: expected {}, got {}".format(
                    config.expected_point_count, len(points)
                )
            )
        if any(not is_finite_point(point) for point in points):
            raise ValueError("global path contains a non-finite point")
        digest = hashlib.sha256(
            b"".join(struct.pack(">dd", float(point.x), float(point.y)) for point in points)
        ).hexdigest()
        if config.expected_xy_sha256 and digest != config.expected_xy_sha256.lower():
            raise ValueError(
                "global path XY SHA-256 mismatch: expected {}, got {}".format(
                    config.expected_xy_sha256, digest
                )
            )

        source_progress = [0.0]
        compact = [(points[0], 0, 0.0)]
        progress = 0.0
        duplicate_count = 0
        for source_index, (start, finish) in enumerate(
            zip(points, points[1:]), 1
        ):
            spacing = distance(start, finish)
            if spacing > config.maximum_point_spacing_m:
                raise ValueError("global path contains an oversized point gap")
            progress += spacing
            source_progress.append(progress)
            if spacing > 1.0e-9:
                compact.append((finish, source_index, progress))
            else:
                duplicate_count += 1
        if len(compact) < 2:
            raise ValueError("global path has no non-zero XY segment")
        if abs(progress - config.expected_length_m) > config.length_tolerance_m:
            raise ValueError(
                "global path length mismatch: expected {:.3f}, got {:.3f}".format(
                    config.expected_length_m, progress
                )
            )
        if distance(points[0], points[-1]) > config.closure_tolerance_m:
            raise ValueError("competition global path is not closed")

        segments = []
        for first, second in zip(compact, compact[1:]):
            start, start_index, start_progress = first
            finish, finish_index, finish_progress = second
            length_m = finish_progress - start_progress
            segments.append(
                _Segment(
                    start=start,
                    finish=finish,
                    start_progress_m=start_progress,
                    length_m=length_m,
                    heading_rad=math.atan2(
                        finish.y - start.y, finish.x - start.x
                    ),
                    start_source_index=start_index,
                    finish_source_index=finish_index,
                )
            )

        self.raw_points = points
        self.source_progress_m = tuple(source_progress)
        self._compact = tuple(compact)
        self.segments = tuple(segments)
        self._segment_ends = tuple(
            segment.start_progress_m + segment.length_m
            for segment in self.segments
        )
        self.length_m = progress
        self.duplicate_count = duplicate_count

    def pose_at(self, progress_m: float) -> Pose2D:
        progress = min(max(float(progress_m), 0.0), self.length_m)
        index = min(
            bisect.bisect_left(self._segment_ends, progress),
            len(self.segments) - 1,
        )
        segment = self.segments[index]
        ratio = (progress - segment.start_progress_m) / segment.length_m
        ratio = min(max(ratio, 0.0), 1.0)
        return Pose2D(
            segment.start.x + ratio * (segment.finish.x - segment.start.x),
            segment.start.y + ratio * (segment.finish.y - segment.start.y),
            segment.heading_rad,
        )

    def _project_local(
        self,
        ego: Pose2D,
        source_index: int,
        context_progress_m: float,
        config: GlobalRouteTrackingConfig,
    ) -> RouteProjection:
        point_count = len(self.raw_points)
        lower = max(0, source_index - config.projection_backward_points)
        forward_end = source_index + config.projection_forward_points
        ranges = [(lower, min(point_count - 1, forward_end))]
        if forward_end >= point_count:
            ranges.append((0, forward_end - point_count + 1))
        best = None
        for segment in self.segments:
            if not any(
                segment.finish_source_index >= range_start
                and segment.start_source_index <= range_finish
                for range_start, range_finish in ranges
            ):
                continue
            dx = segment.finish.x - segment.start.x
            dy = segment.finish.y - segment.start.y
            ratio = (
                (ego.x - segment.start.x) * dx
                + (ego.y - segment.start.y) * dy
            ) / (segment.length_m * segment.length_m)
            ratio = min(max(ratio, 0.0), 1.0)
            projected = Point2D(
                segment.start.x + ratio * dx,
                segment.start.y + ratio * dy,
            )
            lateral = distance(Point2D(ego.x, ego.y), projected)
            heading_error = angular_distance(ego.yaw, segment.heading_rad)
            if heading_error > config.maximum_projection_heading_error_rad:
                continue
            progress = segment.start_progress_m + ratio * segment.length_m
            comparison_progress = progress
            if (
                context_progress_m > self.length_m - config.maximum_projection_progress_error_m
                and progress < config.maximum_projection_progress_error_m
            ):
                comparison_progress += self.length_m
            score = (
                lateral * lateral
                + config.heading_score_weight * heading_error * heading_error
            )
            candidate = (
                score,
                abs(comparison_progress - context_progress_m),
                comparison_progress,
                lateral,
                heading_error,
            )
            if best is None or candidate[:3] < best[:3]:
                best = candidate
        if best is None:
            raise ValueError(
                "no heading-compatible global-path segment in bounded projection window"
            )
        return RouteProjection(
            progress_m=best[2],
            lateral_distance_m=best[3],
            heading_error_rad=best[4],
        )

    def _reference_forward(self, start_progress_m, forward_distance_m):
        values = []

        def append(point):
            if not values or distance(values[-1], point) > 1.0e-9:
                values.append(point)

        start_pose = self.pose_at(start_progress_m)
        append(Point2D(start_pose.x, start_pose.y))
        unwrapped_finish = start_progress_m + forward_distance_m
        if unwrapped_finish <= self.length_m:
            for point, _source_index, progress in self._compact:
                if start_progress_m < progress < unwrapped_finish:
                    append(point)
            finish_pose = self.pose_at(unwrapped_finish)
        else:
            for point, _source_index, progress in self._compact:
                if progress > start_progress_m:
                    append(point)
            wrapped_finish = unwrapped_finish - self.length_m
            for point, _source_index, progress in self._compact:
                if 0.0 < progress < wrapped_finish:
                    append(point)
            finish_pose = self.pose_at(wrapped_finish)
        append(Point2D(finish_pose.x, finish_pose.y))
        if len(values) < 2:
            raise ValueError("global path slice has fewer than two distinct points")
        return tuple(values), finish_pose

    def forward_slice(
        self,
        ego: Pose2D,
        context_progress_m: float,
        context_route_length_m: float,
        nearest_source_index: int,
        previous_progress_m: Optional[float],
        preferred_distance_m: float,
        minimum_distance_m: float,
        final_end_margin_m: float,
        pose_is_valid: Callable[[Pose2D], bool],
        config: GlobalRouteTrackingConfig,
    ) -> GlobalRouteSlice:
        numeric = (
            context_progress_m,
            context_route_length_m,
            preferred_distance_m,
            minimum_distance_m,
            final_end_margin_m,
        )
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("global route slice contains a non-finite input")
        if abs(context_route_length_m - self.length_m) > config.length_tolerance_m:
            raise ValueError("route-context length disagrees with global path")
        if not 0 <= int(nearest_source_index) < len(self.raw_points):
            raise ValueError("route-context source index is outside global path")
        raw_progress = float(context_progress_m)
        if (
            raw_progress < -config.length_tolerance_m
            or raw_progress > self.length_m + config.length_tolerance_m
        ):
            raise ValueError("route-context progress is outside global path")
        progress = min(max(raw_progress, 0.0), self.length_m)
        if abs(self.source_progress_m[int(nearest_source_index)] - progress) > (
            config.maximum_index_progress_error_m
        ):
            raise ValueError("route-context index disagrees with progress")
        if previous_progress_m is not None:
            previous = float(previous_progress_m)
            if not math.isfinite(previous) or not 0.0 <= previous <= self.length_m:
                raise ValueError("previous global route progress is invalid")
            if progress + config.maximum_progress_regression_m < previous:
                raise ValueError("global route progress regressed beyond tolerance")
            progress = max(progress, previous)

        projection = self._project_local(
            ego,
            int(nearest_source_index),
            progress,
            config,
        )
        if projection.lateral_distance_m > config.matching_tube_radius_m:
            raise ValueError("ego pose is outside the global-path matching tube")
        if (
            abs(projection.progress_m - progress)
            > config.maximum_projection_progress_error_m
        ):
            raise ValueError("ego projection disagrees with route progress")
        if (
            projection.heading_error_rad
            > config.maximum_projection_heading_error_rad
        ):
            raise ValueError("ego heading disagrees with global path")

        # The competition artifact is a verified closed loop. A bounded local
        # slice may cross L->0; it never searches farther than the configured
        # local goal distance. RouteContext still owns lap/finish semantics.
        preferred = preferred_distance_m
        probe_distances = [0.0]
        probe = config.goal_scan_step_m
        while probe < preferred:
            probe_distances.append(probe)
            probe += config.goal_scan_step_m
        probe_distances.append(preferred)
        route_heading = self.pose_at(progress).yaw
        maximum_heading_change = max(
            angular_distance(
                route_heading,
                self.pose_at((progress + offset) % self.length_m).yaw,
            )
            for offset in probe_distances
        )
        if maximum_heading_change > config.turn_detection_heading_change_rad:
            preferred = min(preferred, config.turn_goal_distance_m)
        preferred = max(preferred, minimum_distance_m)
        candidates = [preferred]
        # Never scan beyond the configured local horizon.  A far-away point on
        # this closed route can be spatially inside the small current corridor
        # at a crossing and must not become an accidental local goal.  If the
        # preferred official point is unsafe, try shorter official goals only;
        # the ROS adapter may then use its existing safe MGeo fallback.
        value = preferred - config.goal_scan_step_m
        while value > minimum_distance_m:
            candidates.append(value)
            value -= config.goal_scan_step_m
        if candidates[-1] > minimum_distance_m:
            candidates.append(minimum_distance_m)

        for forward_distance in candidates:
            reference, goal = self._reference_forward(progress, forward_distance)
            if pose_is_valid(goal):
                return GlobalRouteSlice(
                    reference_path=reference,
                    goal=goal,
                    accepted_progress_m=progress,
                    ego_projection=projection,
                )
        raise ValueError("no safe goal exists on the competition global path")
