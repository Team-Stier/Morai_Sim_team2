"""Pure-Python reference-route model and fail-closed progress matcher.

This module intentionally has no ROS or third-party dependency so its geometry and
safety behavior can be unit-tested outside a ROS installation.
"""

from __future__ import division

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple


class RouteFormatError(ValueError):
    """Raised when the configured reference route cannot be trusted."""


@dataclass(frozen=True)
class RoutePoint:
    x_m: float
    y_m: float
    z_m: float
    source_index: int


@dataclass(frozen=True)
class RouteSegment:
    index: int
    start: RoutePoint
    end: RoutePoint
    start_s_m: float
    length_m: float
    heading_rad: float


@dataclass(frozen=True)
class ReferenceRoute:
    source_path: str
    source_sha256: str
    raw_points: Tuple[RoutePoint, ...]
    points: Tuple[RoutePoint, ...]
    segments: Tuple[RouteSegment, ...]
    length_m: float
    closed: bool
    duplicate_count: int

    @property
    def source_point_count(self):
        return len(self.raw_points)

    def raw_headings(self):
        """Return a finite yaw for every source point, including duplicates."""
        headings = []
        count = len(self.raw_points)
        for index, point in enumerate(self.raw_points):
            heading = None
            for other_index in range(index + 1, count):
                other = self.raw_points[other_index]
                dx = other.x_m - point.x_m
                dy = other.y_m - point.y_m
                if math.hypot(dx, dy) > 1.0e-9:
                    heading = math.atan2(dy, dx)
                    break
            if heading is None:
                for other_index in range(index - 1, -1, -1):
                    other = self.raw_points[other_index]
                    dx = point.x_m - other.x_m
                    dy = point.y_m - other.y_m
                    if math.hypot(dx, dy) > 1.0e-9:
                        heading = math.atan2(dy, dx)
                        break
            headings.append(0.0 if heading is None else heading)
        return tuple(headings)


@dataclass(frozen=True)
class MatchResult:
    valid: bool
    reason: str
    progress_m: float
    lateral_distance_m: float
    segment_index: int
    source_index: int
    projected_x_m: float
    projected_y_m: float


@dataclass(frozen=True)
class LinkSpan:
    link_id: str
    start_m: float
    end_m: float


@dataclass(frozen=True)
class RouteContextState:
    valid: bool
    reason: str
    progress_m: float
    route_length_m: float
    nearest_route_index: int
    current_link_id: str
    horizon_link_ids: Tuple[str, ...]
    speed_limit_exempt_zone: bool
    lateral_distance_m: float


def _finite(value):
    return math.isfinite(value)


def _fixed_numeric_values(values, expected_length):
    """Return a finite float tuple or a stable validation error category."""
    try:
        converted = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError):
        return tuple(), "malformed_odometry"
    if len(converted) != int(expected_length):
        return tuple(), "malformed_odometry"
    if not all(_finite(value) for value in converted):
        return converted, "non_finite"
    return converted, ""


def odometry_payload_invalid_reason(
        position_xyz,
        orientation_xyzw,
        linear_velocity_xyz,
        angular_velocity_xyz,
        pose_covariance,
        twist_covariance):
    """Validate every numeric field carried by ``nav_msgs/Odometry``.

    Arguments are plain iterables so this safety gate remains independent of a
    ROS installation and can be regression-tested with the standard library.
    Unknown ROS covariance is represented by finite ``-1`` values and remains
    valid; NaN/Inf is never used as an invalid sentinel.
    """
    groups = (
        (position_xyz, 3, "non_finite_pose"),
        (orientation_xyzw, 4, "non_finite_orientation"),
        (linear_velocity_xyz, 3, "non_finite_twist"),
        (angular_velocity_xyz, 3, "non_finite_twist"),
        (pose_covariance, 36, "non_finite_covariance"),
        (twist_covariance, 36, "non_finite_covariance"),
    )
    converted_groups = []
    for values, expected_length, non_finite_reason in groups:
        converted, reason = _fixed_numeric_values(values, expected_length)
        if reason == "malformed_odometry":
            return reason
        if reason:
            return non_finite_reason
        converted_groups.append(converted)

    orientation = converted_groups[1]
    norm = math.sqrt(sum(value * value for value in orientation))
    if not _finite(norm) or norm <= 1.0e-9:
        return "invalid_orientation"
    return ""


def gate_observation_stamp(
        last_accepted_stamp_sec, candidate_stamp_sec, validation_reason):
    """Apply monotonic ordering without letting invalid stamps poison state.

    The caller computes ``validation_reason`` first. This function never moves
    the accepted watermark for an invalid observation. An older callback that
    completes after a newer accepted callback is classified as out-of-order and
    can be ignored without invalidating the newer sample.
    """
    try:
        candidate = float(candidate_stamp_sec)
    except (TypeError, ValueError, OverflowError):
        candidate = float("nan")
    if last_accepted_stamp_sec is not None:
        last_accepted = float(last_accepted_stamp_sec)
        if _finite(candidate) and candidate < last_accepted:
            return "out_of_order_odometry", last_accepted
    if validation_reason:
        return str(validation_reason), last_accepted_stamp_sec
    if not _finite(candidate):
        return "non_finite_timestamp", last_accepted_stamp_sec
    return "", candidate


def _angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def load_reference_route(
        path,
        expected_point_count=None,
        expected_sha256=None,
        expected_length_m=None,
        length_tolerance_m=0.1,
        closure_tolerance_m=0.05):
    """Load whitespace-separated ``x y [z]`` points without altering the source.

    Consecutive XY duplicates are retained in ``raw_points`` and removed only from
    the geometry used for matching. Each filtered point keeps its original source
    index so ``nearest_route_index`` remains meaningful against the 4,430-point
    competition file.
    """
    source = Path(path).expanduser().resolve()
    try:
        payload = source.read_bytes()
    except OSError as exc:
        raise RouteFormatError("cannot read route: {}".format(exc))

    digest = hashlib.sha256(payload).hexdigest()
    if expected_sha256 and digest.lower() != str(expected_sha256).lower():
        raise RouteFormatError(
            "route SHA-256 mismatch: expected {}, got {}".format(
                expected_sha256, digest))

    raw_points = []
    text = payload.decode("utf-8-sig")
    for line_number, raw_line in enumerate(text.splitlines(), 1):
        line = raw_line.strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) not in (2, 3):
            raise RouteFormatError(
                "line {} must contain x y [z], got {} fields".format(
                    line_number, len(fields)))
        try:
            values = [float(value) for value in fields]
        except ValueError as exc:
            raise RouteFormatError(
                "line {} contains a non-numeric value: {}".format(
                    line_number, exc))
        if not all(_finite(value) for value in values):
            raise RouteFormatError(
                "line {} contains a non-finite coordinate".format(line_number))
        z_m = values[2] if len(values) == 3 else 0.0
        raw_points.append(RoutePoint(
            x_m=values[0], y_m=values[1], z_m=z_m,
            source_index=len(raw_points)))

    if expected_point_count is not None and len(raw_points) != int(expected_point_count):
        raise RouteFormatError(
            "route point count mismatch: expected {}, got {}".format(
                int(expected_point_count), len(raw_points)))
    if len(raw_points) < 2:
        raise RouteFormatError("route requires at least two points")

    points = []
    duplicate_count = 0
    for point in raw_points:
        if points and math.hypot(
                point.x_m - points[-1].x_m,
                point.y_m - points[-1].y_m) <= 1.0e-9:
            duplicate_count += 1
            continue
        points.append(point)
    if len(points) < 2:
        raise RouteFormatError("route has no non-zero XY segment")

    segments = []
    progress_m = 0.0
    for index in range(len(points) - 1):
        start = points[index]
        end = points[index + 1]
        dx = end.x_m - start.x_m
        dy = end.y_m - start.y_m
        length_m = math.hypot(dx, dy)
        if length_m <= 1.0e-9:
            continue
        segments.append(RouteSegment(
            index=len(segments),
            start=start,
            end=end,
            start_s_m=progress_m,
            length_m=length_m,
            heading_rad=math.atan2(dy, dx),
        ))
        progress_m += length_m

    if expected_length_m is not None:
        error_m = abs(progress_m - float(expected_length_m))
        if error_m > float(length_tolerance_m):
            raise RouteFormatError(
                "route length mismatch: expected {:.6f} m, got {:.6f} m".format(
                    float(expected_length_m), progress_m))

    closed = math.hypot(
        raw_points[0].x_m - raw_points[-1].x_m,
        raw_points[0].y_m - raw_points[-1].y_m,
    ) <= float(closure_tolerance_m)
    return ReferenceRoute(
        source_path=str(source),
        source_sha256=digest,
        raw_points=tuple(raw_points),
        points=tuple(points),
        segments=tuple(segments),
        length_m=progress_m,
        closed=closed,
        duplicate_count=duplicate_count,
    )


class RouteMatcher(object):
    """Stateful, bounded-search projection with nondecreasing route progress."""

    def __init__(
            self,
            route,
            max_lateral_distance_m=6.0,
            search_backward_segments=20,
            search_forward_segments=200,
            max_heading_error_rad=math.radians(100.0),
            heading_weight_m2_per_rad2=1.0):
        if not route.segments:
            raise ValueError("route has no usable segments")
        if max_lateral_distance_m <= 0.0:
            raise ValueError("max_lateral_distance_m must be positive")
        self.route = route
        self.max_lateral_distance_m = float(max_lateral_distance_m)
        self.search_backward_segments = max(0, int(search_backward_segments))
        self.search_forward_segments = max(1, int(search_forward_segments))
        self.max_heading_error_rad = float(max_heading_error_rad)
        self.heading_weight_m2_per_rad2 = float(heading_weight_m2_per_rad2)
        self._last_segment_index = None
        self._last_progress_m = None
        self._last_source_index = None

    @property
    def last_progress_m(self):
        return self._last_progress_m

    @property
    def last_source_index(self):
        return self._last_source_index

    def reset(self):
        self._last_segment_index = None
        self._last_progress_m = None
        self._last_source_index = None

    def _candidate_indices(self):
        if self._last_segment_index is None:
            return range(len(self.route.segments))
        start = max(0, self._last_segment_index - self.search_backward_segments)
        stop = min(
            len(self.route.segments),
            self._last_segment_index + self.search_forward_segments + 1)
        return range(start, stop)

    def match(self, x_m, y_m, yaw_rad=None):
        if not _finite(x_m) or not _finite(y_m):
            return self._invalid("non_finite_pose")
        if yaw_rad is not None and not _finite(yaw_rad):
            return self._invalid("non_finite_yaw")

        best = None
        for index in self._candidate_indices():
            segment = self.route.segments[index]
            dx = segment.end.x_m - segment.start.x_m
            dy = segment.end.y_m - segment.start.y_m
            length_sq = segment.length_m * segment.length_m
            projection = (
                (x_m - segment.start.x_m) * dx
                + (y_m - segment.start.y_m) * dy) / length_sq
            t = min(1.0, max(0.0, projection))
            projected_x = segment.start.x_m + t * dx
            projected_y = segment.start.y_m + t * dy
            distance_sq = (
                (x_m - projected_x) ** 2 + (y_m - projected_y) ** 2)
            heading_error = 0.0
            if yaw_rad is not None:
                heading_error = abs(_angle_difference(yaw_rad, segment.heading_rad))
                if heading_error > self.max_heading_error_rad:
                    continue
            score = (
                distance_sq
                + self.heading_weight_m2_per_rad2 * heading_error * heading_error)
            progress_m = segment.start_s_m + t * segment.length_m
            candidate = (
                score,
                distance_sq,
                progress_m,
                index,
                t,
                projected_x,
                projected_y,
            )
            # Tuple ordering deliberately prefers the earliest progress on exact
            # ties. At the closed route's coincident start/end this initializes
            # the first lap at s=0 instead of s=route_length.
            if best is None or candidate[:4] < best[:4]:
                best = candidate

        if best is None:
            return self._invalid("heading_mismatch")

        _, distance_sq, progress_m, segment_index, t, projected_x, projected_y = best
        distance_m = math.sqrt(distance_sq)
        if distance_m > self.max_lateral_distance_m:
            return self._invalid("off_route", distance_m)

        if self._last_progress_m is not None:
            progress_m = max(self._last_progress_m, progress_m)
        segment = self.route.segments[segment_index]
        source_index = (
            segment.start.source_index if t < 0.5 else segment.end.source_index)
        if self._last_source_index is not None and progress_m == self._last_progress_m:
            source_index = max(self._last_source_index, source_index)

        self._last_segment_index = max(
            segment_index,
            self._last_segment_index if self._last_segment_index is not None else 0)
        self._last_progress_m = min(progress_m, self.route.length_m)
        self._last_source_index = source_index
        return MatchResult(
            valid=True,
            reason="ok",
            progress_m=self._last_progress_m,
            lateral_distance_m=distance_m,
            segment_index=segment_index,
            source_index=source_index,
            projected_x_m=projected_x,
            projected_y_m=projected_y,
        )

    def _invalid(self, reason, lateral_distance_m=float("inf")):
        return MatchResult(
            valid=False,
            reason=reason,
            progress_m=0.0 if self._last_progress_m is None else self._last_progress_m,
            lateral_distance_m=lateral_distance_m,
            segment_index=-1 if self._last_segment_index is None else self._last_segment_index,
            source_index=0 if self._last_source_index is None else self._last_source_index,
            projected_x_m=0.0,
            projected_y_m=0.0,
        )


class RouteTopology(object):
    """Map a scalar reference-route progress to canonical HD-map link context."""

    def __init__(
            self,
            spans,
            route_length_m,
            high_speed_start_link_id,
            high_speed_end_link_id,
            continuity_tolerance_m=1.0):
        self.spans = tuple(spans)
        self.route_length_m = float(route_length_m)
        continuity_tolerance_m = float(continuity_tolerance_m)
        if (not _finite(self.route_length_m)
                or self.route_length_m <= 0.0):
            raise ValueError("route_length_m must be finite and positive")
        if (not _finite(continuity_tolerance_m)
                or continuity_tolerance_m < 0.0):
            raise ValueError(
                "continuity_tolerance_m must be finite and nonnegative")
        if not self.spans:
            raise ValueError("at least one link span is required")
        previous_end = None
        previous_start = None
        for span in self.spans:
            if (not span.link_id
                    or not _finite(span.start_m)
                    or not _finite(span.end_m)
                    or span.start_m < 0.0
                    or span.end_m > (
                        self.route_length_m + continuity_tolerance_m)
                    or span.end_m < span.start_m):
                raise ValueError("invalid link span: {}".format(span))
            if previous_end is not None:
                if (span.start_m < previous_start
                        or span.end_m < previous_end):
                    raise ValueError(
                        "route link spans must be nondecreasing")
                gap = abs(span.start_m - previous_end)
                if gap > continuity_tolerance_m:
                    raise ValueError(
                        "route link spans are not continuous near {} (gap {:.3f} m)".format(
                            span.link_id, gap))
            previous_end = span.end_m
            previous_start = span.start_m
        if abs(self.spans[0].start_m) > continuity_tolerance_m:
            raise ValueError("route link spans do not cover route start")
        if abs(self.spans[-1].end_m - self.route_length_m) > continuity_tolerance_m:
            raise ValueError("route link spans do not cover route end")

        start_indices = [
            index for index, span in enumerate(self.spans)
            if span.link_id == high_speed_start_link_id]
        end_indices = [
            index for index, span in enumerate(self.spans)
            if span.link_id == high_speed_end_link_id]
        if len(start_indices) != 1 or len(end_indices) != 1:
            raise ValueError("high-speed boundary links must each occur exactly once")
        self.high_speed_start_index = start_indices[0]
        self.high_speed_end_index = end_indices[0]
        if self.high_speed_start_index > self.high_speed_end_index:
            raise ValueError("high-speed link interval is reversed")
        self.high_speed_start_m = self.spans[self.high_speed_start_index].start_m
        self.high_speed_end_m = self.spans[self.high_speed_end_index].end_m

    @classmethod
    def from_dicts(
            cls,
            link_dicts,
            route_length_m,
            high_speed_start_link_id,
            high_speed_end_link_id,
            continuity_tolerance_m=1.0):
        spans = [LinkSpan(
            link_id=str(item["id"]),
            start_m=float(item["start_m"]),
            end_m=float(item["end_m"]),
        ) for item in link_dicts]
        return cls(
            spans=spans,
            route_length_m=route_length_m,
            high_speed_start_link_id=high_speed_start_link_id,
            high_speed_end_link_id=high_speed_end_link_id,
            continuity_tolerance_m=continuity_tolerance_m,
        )

    def span_index_at(self, progress_m):
        progress_m = min(max(float(progress_m), 0.0), self.route_length_m)
        containing = [
            index for index, span in enumerate(self.spans)
            if span.start_m <= progress_m <= span.end_m]
        if containing:
            # At a shared endpoint, advance to the successor. This makes the
            # regulatory interval begin with the first sample on its start link.
            return max(containing)
        # Rounded audit spans may leave sub-metre gaps. Use the nearest span edge,
        # with the later span winning ties so context advances rather than regresses.
        return min(
            range(len(self.spans)),
            key=lambda index: (
                min(abs(progress_m - self.spans[index].start_m),
                    abs(progress_m - self.spans[index].end_m)),
                -index,
            ))

    def context_for_match(self, match, horizon_link_count=6):
        if not match.valid:
            return RouteContextState(
                valid=False,
                reason=match.reason,
                progress_m=match.progress_m,
                route_length_m=self.route_length_m,
                nearest_route_index=match.source_index,
                current_link_id="",
                horizon_link_ids=tuple(),
                speed_limit_exempt_zone=False,
                lateral_distance_m=match.lateral_distance_m,
            )
        span_index = self.span_index_at(match.progress_m)
        horizon = []
        for offset in range(max(1, int(horizon_link_count))):
            candidate = self.spans[(span_index + offset) % len(self.spans)].link_id
            if not horizon or horizon[-1] != candidate:
                horizon.append(candidate)
        return RouteContextState(
            valid=True,
            reason="ok",
            progress_m=match.progress_m,
            route_length_m=self.route_length_m,
            nearest_route_index=match.source_index,
            current_link_id=self.spans[span_index].link_id,
            horizon_link_ids=tuple(horizon),
            speed_limit_exempt_zone=(
                self.high_speed_start_m <= match.progress_m <= self.high_speed_end_m),
            lateral_distance_m=match.lateral_distance_m,
        )

    def invalid_context(self, matcher, reason):
        match = matcher._invalid(reason)
        return self.context_for_match(match)


def observation_invalid_reason(
        stamp_sec,
        now_sec,
        timeout_sec,
        actual_frame,
        expected_frame,
        actual_child_frame=None,
        expected_child_frame=None):
    """Validate time/frame metadata before allowing a pose into the matcher."""
    values = (stamp_sec, now_sec, timeout_sec)
    if not all(_finite(float(value)) for value in values):
        return "non_finite_timestamp"
    if not actual_frame or actual_frame != expected_frame:
        return "frame_mismatch"
    if (expected_child_frame is not None
            and actual_child_frame != expected_child_frame):
        return "child_frame_mismatch"
    if stamp_sec <= 0.0:
        return "missing_timestamp"
    age_sec = now_sec - stamp_sec
    if age_sec < 0.0:
        return "future_timestamp"
    if age_sec > timeout_sec:
        return "stale_odometry"
    return ""
