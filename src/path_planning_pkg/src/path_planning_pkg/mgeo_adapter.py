"""KATRI MGeo-to-planner corridor adapter.

The adapter consumes the same materialized lane-side geometry as the validated
Lanelet2 exporter.  It never turns every line in the global map into an
obstacle: only the lateral sides of the selected route corridor become walls.
Selected topological successor mouths are the sole longitudinal openings.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple

from hd_map_pkg.coordinates import CoordinateTransformer
from hd_map_pkg.geometry import (
    cumulative_lengths,
    point_at_progress,
    polyline_length,
    progress_along_polyline,
    simplify_rdp,
    slice_polyline,
)
from hd_map_pkg.lanelet2_export import Lanelet2Exporter, boundary_tags
from hd_map_pkg.mgeo_v3 import MGeoV3Dataset

from .corridor import (
    BoundaryMarking,
    BoundarySegment,
    BoundarySide,
    DrivingCorridor,
    LanePolygon,
)
from .geometry import distance
from .models import Point2D, Pose2D


def _compact(points: Iterable[Point2D], tolerance_m: float = 1.0e-6):
    result = []
    for point in points:
        if not result or distance(result[-1], point) > tolerance_m:
            result.append(point)
    return tuple(result)


def _join_plan_side(plans, side):
    result = []
    source_ids = []
    synthetic = False
    subtypes = []
    for plan in plans:
        side_plan = plan[side]
        points = side_plan["points"]
        if result and points:
            if math.hypot(
                float(result[-1][0]) - float(points[0][0]),
                float(result[-1][1]) - float(points[0][1]),
            ) <= 1.0e-6:
                result.extend(points[1:])
            else:
                result.extend(points)
        else:
            result.extend(points)
        source_ids.extend(str(value) for value in side_plan["source_ids"])
        synthetic = synthetic or bool(side_plan["synthetic"])
        subtypes.append(str(side_plan["tags"].get("subtype", "unknown")))
    return tuple(result), tuple(dict.fromkeys(source_ids)), synthetic, tuple(subtypes)


def _boundary_marking(subtypes, synthetic):
    if synthetic or not subtypes:
        return BoundaryMarking.VIRTUAL
    normalized = set(subtypes)
    if normalized == {"dashed"}:
        return BoundaryMarking.DASHED
    if any("solid" in value for value in normalized):
        return BoundaryMarking.SOLID
    return BoundaryMarking.UNKNOWN


def _proper_intersection(first_start, first_end, second_start, second_end):
    """Return a strict segment intersection, excluding endpoint-only contact."""
    rx = first_end.x - first_start.x
    ry = first_end.y - first_start.y
    sx = second_end.x - second_start.x
    sy = second_end.y - second_start.y
    denominator = rx * sy - ry * sx
    if abs(denominator) <= 1.0e-12:
        return None
    qx = second_start.x - first_start.x
    qy = second_start.y - first_start.y
    first_ratio = (qx * sy - qy * sx) / denominator
    second_ratio = (qx * ry - qy * rx) / denominator
    endpoint_epsilon = 1.0e-7
    if not (
        endpoint_epsilon < first_ratio < 1.0 - endpoint_epsilon
        and endpoint_epsilon < second_ratio < 1.0 - endpoint_epsilon
    ):
        return None
    return (
        Point2D(
            first_start.x + first_ratio * rx,
            first_start.y + first_ratio * ry,
        ),
        first_ratio,
        second_ratio,
    )


def _trim_crossed_sides(left, right):
    """Trim tiny boundary tails after/before a lane converges to a point.

    A few KATRI source sides cross by centimetres immediately before a shared
    routing endpoint.  Keeping the tails creates a bow-tie polygon.  Trimming
    at the first physical intersection only removes the inverted, non-driveable
    sliver; it never expands the corridor.
    """
    left_values = list(left)
    right_values = list(right)
    left_lengths = cumulative_lengths([(p.x, p.y) for p in left_values])
    right_lengths = cumulative_lengths([(p.x, p.y) for p in right_values])
    if not left_lengths or not right_lengths:
        return tuple(left_values), tuple(right_values)
    intersections = []
    for left_index, (left_start, left_end) in enumerate(
        zip(left_values, left_values[1:])
    ):
        for right_index, (right_start, right_end) in enumerate(
            zip(right_values, right_values[1:])
        ):
            crossing = _proper_intersection(
                left_start, left_end, right_start, right_end
            )
            if crossing is None:
                continue
            point, left_ratio, right_ratio = crossing
            left_progress = left_lengths[left_index] + left_ratio * distance(
                left_start, left_end
            )
            right_progress = right_lengths[right_index] + right_ratio * distance(
                right_start, right_end
            )
            normalized = 0.5 * (
                left_progress / max(left_lengths[-1], 1.0e-9)
                + right_progress / max(right_lengths[-1], 1.0e-9)
            )
            intersections.append(
                (normalized, left_index, right_index, point)
            )
    start_crossings = [value for value in intersections if value[0] < 0.5]
    end_crossings = [value for value in intersections if value[0] >= 0.5]
    if start_crossings:
        _, left_index, right_index, point = max(start_crossings)
        left_values = [point] + left_values[left_index + 1 :]
        right_values = [point] + right_values[right_index + 1 :]
    if end_crossings:
        _, left_index, right_index, point = min(end_crossings)
        # Indices refer to the original lists.  End-crossing trims are expected
        # at the opposite end and remain valid after a start trim only when no
        # start crossing was present; reject pathological double-cross lanes.
        if start_crossings:
            raise ValueError("lane sides cross at both upstream and downstream ends")
        left_values = left_values[: left_index + 1] + [point]
        right_values = right_values[: right_index + 1] + [point]
    return _compact(left_values), _compact(right_values)


@dataclass(frozen=True)
class MapLane:
    link_id: str
    centerline: Tuple[Point2D, ...]
    left: Tuple[Point2D, ...]
    right: Tuple[Point2D, ...]
    left_source_ids: Tuple[str, ...]
    right_source_ids: Tuple[str, ...]
    left_marking: BoundaryMarking
    right_marking: BoundaryMarking
    max_speed_kph: float
    related_signal: str

    def __post_init__(self):
        if min(len(self.centerline), len(self.left), len(self.right)) < 2:
            raise ValueError("lane {} has incomplete geometry".format(self.link_id))

    @property
    def length_m(self):
        return polyline_length([(point.x, point.y) for point in self.centerline])

    def nearest_progress(self, point):
        progress, squared = progress_along_polyline(
            (point.x, point.y), [(value.x, value.y) for value in self.centerline]
        )
        return float(progress), math.sqrt(max(0.0, float(squared)))

    def pose_at(self, progress_m):
        points = [(point.x, point.y) for point in self.centerline]
        total = polyline_length(points)
        progress = max(0.0, min(total, float(progress_m)))
        point = point_at_progress(points, progress)
        delta = min(0.25, max(total, 0.25))
        before = point_at_progress(points, max(0.0, progress - delta))
        after = point_at_progress(points, min(total, progress + delta))
        yaw = math.atan2(after[1] - before[1], after[0] - before[0])
        return Pose2D(float(point[0]), float(point[1]), yaw)

    def polygon(self, left=None, right=None):
        left_side = self.left if left is None else tuple(left)
        right_side = self.right if right is None else tuple(right)
        return LanePolygon(
            self.link_id,
            _compact(tuple(left_side) + tuple(reversed(right_side))),
        )

    def sliced(self, start_progress_m, end_progress_m):
        total = self.length_m
        start = max(0.0, min(total, float(start_progress_m)))
        end = max(start, min(total, float(end_progress_m)))
        if end - start <= 0.25:
            raise ValueError("lane slice is too short for {}".format(self.link_id))

        def sliced_points(points):
            raw = [(point.x, point.y) for point in points]
            raw_length = polyline_length(raw)
            values = slice_polyline(
                raw,
                start / total * raw_length,
                end / total * raw_length,
            )
            return _compact(Point2D(float(value[0]), float(value[1])) for value in values)

        return MapLane(
            link_id=self.link_id,
            centerline=sliced_points(self.centerline),
            left=sliced_points(self.left),
            right=sliced_points(self.right),
            left_source_ids=self.left_source_ids,
            right_source_ids=self.right_source_ids,
            left_marking=self.left_marking,
            right_marking=self.right_marking,
            max_speed_kph=self.max_speed_kph,
            related_signal=self.related_signal,
        )


@dataclass(frozen=True)
class LaneChangePair:
    route_link_id: str
    adjacent_link_id: str
    direction: str
    shared_boundary_id: str
    merge_route_link_id: str
    branch_by_route_link: Mapping[str, str]


@dataclass(frozen=True)
class CorridorBuild:
    corridor: DrivingCorridor
    open_boundary_ids: Tuple[str, ...]
    current_lane: MapLane
    target_lane: MapLane
    topology_verified: bool
    shared_marking: BoundaryMarking
    guidance_points: Tuple[Point2D, ...]
    map_lanes: Tuple[MapLane, ...]


class MGeoPlannerMap:
    """Immutable, route-local planner view of one validated MGeo snapshot."""

    def __init__(
        self, dataset, transformer, map_config, simplification_tolerance_m=0.0
    ):
        self.dataset = dataset
        self.transformer = transformer
        self.map_config = map_config
        self.simplification_tolerance_m = float(simplification_tolerance_m)
        if (
            not math.isfinite(self.simplification_tolerance_m)
            or self.simplification_tolerance_m < 0.0
        ):
            raise ValueError("map simplification tolerance must be non-negative")
        exporter = Lanelet2Exporter(dataset, transformer, map_config)
        self._raw_plans = exporter.build_lanelet_segment_geometry()
        self._lanes = {}

    @classmethod
    def from_files(
        cls,
        map_source_directory,
        map_config_file,
        simplification_tolerance_m=0.0,
    ):
        import yaml

        with Path(map_config_file).open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        dataset = MGeoV3Dataset(
            map_source_directory,
            expected_major=int(config["source"]["expected_mgeo_major"]),
            deduplicate_verified_suffix_clones=bool(
                config.get("conversion", {}).get(
                    "deduplicate_verified_suffix_clones", True
                )
            ),
        )
        coordinates = config["coordinates"]
        transformer = CoordinateTransformer(
            dataset.local_origin_utm,
            coordinates["simulator_scene_origin_utm"],
            int(coordinates["utm_zone"]),
            bool(coordinates["northern_hemisphere"]),
        )
        return cls(
            dataset,
            transformer,
            config,
            simplification_tolerance_m=simplification_tolerance_m,
        )

    def _scene_points(self, values):
        transformed = [self.transformer.mgeo_to_sim(value) for value in values]
        if self.simplification_tolerance_m > 0.0 and len(transformed) > 2:
            transformed = simplify_rdp(
                transformed, self.simplification_tolerance_m
            )
        return _compact(Point2D(*value[:2]) for value in transformed)

    def _materialize_lane(self, link_id):
        plans = self._raw_plans.get(link_id) or ()
        if not plans:
            raise KeyError("MGeo link has no planner lane geometry: {}".format(link_id))
        left, left_ids, left_synthetic, left_subtypes = _join_plan_side(
            plans, "left"
        )
        right, right_ids, right_synthetic, right_subtypes = _join_plan_side(
            plans, "right"
        )
        link = self.dataset.links[link_id]
        scene_left, scene_right = _trim_crossed_sides(
            self._scene_points(left), self._scene_points(right)
        )
        return MapLane(
            link_id=link_id,
            centerline=self._scene_points(link.get("points") or []),
            left=scene_left,
            right=scene_right,
            left_source_ids=left_ids,
            right_source_ids=right_ids,
            left_marking=_boundary_marking(left_subtypes, left_synthetic),
            right_marking=_boundary_marking(right_subtypes, right_synthetic),
            max_speed_kph=float(link.get("max_speed") or 0.0),
            related_signal=str(link.get("related_signal") or ""),
        )

    def lane(self, link_id):
        canonical = self.dataset.canonical_id(str(link_id))
        if canonical not in self._lanes:
            self._lanes[canonical] = self._materialize_lane(canonical)
        return self._lanes[canonical]

    def topology_has_successor(self, first, second):
        first_id = self.dataset.canonical_id(first)
        second_id = self.dataset.canonical_id(second)
        return second_id in self.dataset.successors.get(first_id, ())

    @staticmethod
    def _lateral_boundaries(lane):
        return (
            BoundarySegment(
                "{}:left".format(lane.link_id),
                lane.left,
                BoundarySide.LEFT,
                lane.left_marking,
                frozenset((lane.link_id,)),
            ),
            BoundarySegment(
                "{}:right".format(lane.link_id),
                lane.right,
                BoundarySide.RIGHT,
                lane.right_marking,
                frozenset((lane.link_id,)),
            ),
        )

    @staticmethod
    def _end_cap(identifier, lane, downstream, lane_ids=None, side=None):
        if downstream:
            points = (lane.right[-1], lane.left[-1])
            terminal_side = BoundarySide.DOWNSTREAM_END
        else:
            points = (lane.left[0], lane.right[0])
            terminal_side = BoundarySide.UPSTREAM_END
        return BoundarySegment(
            identifier,
            points,
            terminal_side if side is None else side,
            BoundaryMarking.VIRTUAL,
            frozenset(lane_ids or (lane.link_id,)),
        )

    @classmethod
    def _append_end_cap(
        cls, boundaries, identifier, lane, downstream, lane_ids=None, side=None
    ):
        points = (
            (lane.right[-1], lane.left[-1])
            if downstream
            else (lane.left[0], lane.right[0])
        )
        if distance(points[0], points[1]) <= 1.0e-6:
            return False
        boundaries.append(
            cls._end_cap(
                identifier,
                lane,
                downstream,
                lane_ids=lane_ids,
                side=side,
            )
        )
        return True

    @staticmethod
    def _transition_needs_bridge(first, second, minimum_width_m=2.50):
        first_width = distance(first.left[-1], first.right[-1])
        second_width = distance(second.left[0], second.right[0])
        left_gap = distance(first.left[-1], second.left[0])
        right_gap = distance(first.right[-1], second.right[0])
        return (
            first_width < minimum_width_m
            or second_width < minimum_width_m
            or left_gap > 0.25
            or right_gap > 0.25
        )

    def build_route_corridor(self, ordered_link_ids, source_link_id):
        canonical = []
        for value in ordered_link_ids:
            identifier = self.dataset.canonical_id(value)
            if not canonical or canonical[-1] != identifier:
                canonical.append(identifier)
        if not canonical or source_link_id not in canonical:
            raise ValueError("route corridor must contain its source link")
        lanes = [self.lane(identifier) for identifier in canonical]
        for first, second in zip(canonical, canonical[1:]):
            if not self.topology_has_successor(first, second):
                raise ValueError("unverified MGeo transition {} -> {}".format(first, second))

        start_trim = [0.0 for _ in lanes]
        end_trim = [0.0 for _ in lanes]
        bridged = []
        for index, (first, second) in enumerate(zip(lanes, lanes[1:])):
            needs_bridge = self._transition_needs_bridge(first, second)
            bridged.append(needs_bridge)
            if needs_bridge:
                first_trim = min(5.0, max(1.0, 0.20 * first.length_m))
                second_trim = min(5.0, max(1.0, 0.20 * second.length_m))
                end_trim[index] = max(end_trim[index], first_trim)
                start_trim[index + 1] = max(start_trim[index + 1], second_trim)
        selected = []
        for index, lane in enumerate(lanes):
            selected.append(
                lane.sliced(start_trim[index], lane.length_m - end_trim[index])
                if start_trim[index] > 0.0 or end_trim[index] > 0.0
                else lane
            )

        polygons = []
        for lane in selected:
            polygons.append(lane.polygon())
        boundaries = []
        for lane in selected:
            boundaries.extend(self._lateral_boundaries(lane))
        self._append_end_cap(boundaries, "route:upstream", selected[0], False)
        opened = []
        for index, (first, second) in enumerate(zip(selected, selected[1:])):
            if not bridged[index]:
                identifier = "route:seam:{}:{}".format(
                    first.link_id, second.link_id
                )
                if not self._append_end_cap(
                    boundaries,
                    identifier,
                    first,
                    True,
                    lane_ids=(first.link_id, second.link_id),
                    side=BoundarySide.CONNECTOR_MOUTH,
                ):
                    raise ValueError(
                        "zero-width route seam {} -> {}".format(
                            first.link_id, second.link_id
                        )
                    )
                opened.append(identifier)
                continue

            bridge_id = "bridge:{}:{}".format(first.link_id, second.link_id)
            bridge = LanePolygon(
                bridge_id,
                (
                    first.left[-1],
                    second.left[0],
                    second.right[0],
                    first.right[-1],
                ),
            )
            polygons.append(bridge)
            boundaries.append(
                BoundarySegment(
                    "{}:left".format(bridge_id),
                    (first.left[-1], second.left[0]),
                    BoundarySide.LEFT,
                    BoundaryMarking.VIRTUAL,
                    frozenset((bridge_id,)),
                )
            )
            boundaries.append(
                BoundarySegment(
                    "{}:right".format(bridge_id),
                    (first.right[-1], second.right[0]),
                    BoundarySide.RIGHT,
                    BoundaryMarking.VIRTUAL,
                    frozenset((bridge_id,)),
                )
            )
            first_seam = "route:seam:{}:{}:enter".format(
                first.link_id, second.link_id
            )
            second_seam = "route:seam:{}:{}:exit".format(
                first.link_id, second.link_id
            )
            boundaries.append(
                BoundarySegment(
                    first_seam,
                    (first.right[-1], first.left[-1]),
                    BoundarySide.CONNECTOR_MOUTH,
                    BoundaryMarking.VIRTUAL,
                    frozenset((first.link_id, bridge_id)),
                )
            )
            boundaries.append(
                BoundarySegment(
                    second_seam,
                    (second.right[0], second.left[0]),
                    BoundarySide.CONNECTOR_MOUTH,
                    BoundaryMarking.VIRTUAL,
                    frozenset((bridge_id, second.link_id)),
                )
            )
            opened.extend((first_seam, second_seam))
        self._append_end_cap(boundaries, "route:downstream", selected[-1], True)
        corridor = DrivingCorridor(
            source_link_id,
            tuple(canonical),
            tuple(polygons),
            tuple(boundaries),
        )
        source = self.lane(source_link_id)
        guidance = _compact(
            point for lane in selected for point in lane.centerline
        )
        return CorridorBuild(
            corridor=corridor,
            open_boundary_ids=tuple(opened),
            current_lane=source,
            target_lane=source,
            topology_verified=True,
            shared_marking=BoundaryMarking.VIRTUAL,
            guidance_points=guidance,
            map_lanes=tuple(selected),
        )

    def build_lane_change_corridor(self, pair):
        current = self.lane(pair.route_link_id)
        adjacent = self.lane(pair.adjacent_link_id)
        link = self.dataset.links[current.link_id]
        if pair.direction == "left":
            current_side_name = "left"
            adjacent_side_name = "right"
            destination = link.get("left_lane_change_dst_link_idx")
            allowed = bool(link.get("can_move_left_lane"))
            current_shared = current.left
            adjacent_shared = adjacent.right
            shared_marking = current.left_marking
            current_outer = ("right", current.right, current.right_marking)
            adjacent_outer = ("left", adjacent.left, adjacent.left_marking)
        elif pair.direction == "right":
            current_side_name = "right"
            adjacent_side_name = "left"
            destination = link.get("right_lane_change_dst_link_idx")
            allowed = bool(link.get("can_move_right_lane"))
            current_shared = current.right
            adjacent_shared = adjacent.left
            shared_marking = current.right_marking
            current_outer = ("left", current.left, current.left_marking)
            adjacent_outer = ("right", adjacent.right, adjacent.right_marking)
        else:
            raise ValueError("lane-change direction must be left or right")
        destination = self.dataset.canonical_id(str(destination or ""))
        topology_verified = allowed and destination == adjacent.link_id

        # When a source dashed line covers only part of a link, restrict the
        # entire lane-change corridor to that measured overlap.  Synthetic
        # tails remain closed instead of inheriting a permissive label.
        current_candidates = [
            plan
            for plan in self._raw_plans[current.link_id]
            if pair.shared_boundary_id
            in plan[current_side_name]["source_ids"]
            and not plan[current_side_name]["synthetic"]
        ]
        adjacent_candidates = [
            plan
            for plan in self._raw_plans[adjacent.link_id]
            if pair.shared_boundary_id
            in plan[adjacent_side_name]["source_ids"]
            and not plan[adjacent_side_name]["synthetic"]
        ]
        measured_overlap = None
        for current_plan in current_candidates:
            for adjacent_plan in adjacent_candidates:
                current_start_fraction = current_plan["start"] / current.length_m
                current_end_fraction = current_plan["end"] / current.length_m
                adjacent_start_fraction = adjacent_plan["start"] / adjacent.length_m
                adjacent_end_fraction = adjacent_plan["end"] / adjacent.length_m
                start_fraction = max(current_start_fraction, adjacent_start_fraction)
                end_fraction = min(current_end_fraction, adjacent_end_fraction)
                span = (end_fraction - start_fraction) * min(
                    current.length_m, adjacent.length_m
                )
                if span <= 0.25:
                    continue
                candidate = (
                    span,
                    start_fraction,
                    end_fraction,
                    current_plan,
                    adjacent_plan,
                )
                if measured_overlap is None or candidate[0] > measured_overlap[0]:
                    measured_overlap = candidate
        if measured_overlap is not None:
            _, start_fraction, end_fraction, current_plan, adjacent_plan = measured_overlap
            current = current.sliced(
                start_fraction * current.length_m,
                end_fraction * current.length_m,
            )
            adjacent = adjacent.sliced(
                start_fraction * adjacent.length_m,
                end_fraction * adjacent.length_m,
            )
            current_side = current_plan[current_side_name]
            raw_shared = current_side["points"]
            raw_shared_length = polyline_length(raw_shared)
            plan_span_fraction = (
                current_plan["end"] - current_plan["start"]
            ) / self.lane(pair.route_link_id).length_m
            relative_start = max(
                0.0,
                (start_fraction - current_plan["start"] / self.lane(pair.route_link_id).length_m)
                / max(plan_span_fraction, 1.0e-9),
            )
            relative_end = min(
                1.0,
                (end_fraction - current_plan["start"] / self.lane(pair.route_link_id).length_m)
                / max(plan_span_fraction, 1.0e-9),
            )
            shared = self._scene_points(
                slice_polyline(
                    raw_shared,
                    relative_start * raw_shared_length,
                    relative_end * raw_shared_length,
                )
            )
            subtype_values = (
                str(current_side["tags"].get("subtype", "unknown")),
                str(adjacent_plan[adjacent_side_name]["tags"].get("subtype", "unknown")),
            )
            shared_marking = _boundary_marking(subtype_values, False)
            if pair.direction == "left":
                current = replace(current, left=shared, left_marking=shared_marking)
                adjacent = replace(adjacent, right=shared, right_marking=shared_marking)
            else:
                current = replace(current, right=shared, right_marking=shared_marking)
                adjacent = replace(adjacent, left=shared, left_marking=shared_marking)
            current_shared = shared
            adjacent_shared = shared

            # The lane slices above replace the geometry.  Refresh the outer
            # side references as well so the explicit hard-wall polylines do
            # not retain portions outside the measured dashed overlap.
            if pair.direction == "left":
                current_outer = (
                    "right",
                    current.right,
                    current.right_marking,
                )
                adjacent_outer = (
                    "left",
                    adjacent.left,
                    adjacent.left_marking,
                )
            else:
                current_outer = (
                    "left",
                    current.left,
                    current.left_marking,
                )
                adjacent_outer = (
                    "right",
                    adjacent.right,
                    adjacent.right_marking,
                )

        # Exact shared topology is mandatory.  A synthetic or merely nearby
        # boundary cannot be erased because that would fabricate driveable map.
        shared_ids = set(current.left_source_ids + current.right_source_ids)
        adjacent_ids = set(adjacent.left_source_ids + adjacent.right_source_ids)
        source_shared = (
            bool(pair.shared_boundary_id)
            and pair.shared_boundary_id in shared_ids
            and pair.shared_boundary_id in adjacent_ids
        )
        topology_verified = topology_verified and source_shared

        current_polygon = current.polygon()
        adjacent_polygon = adjacent.polygon()
        lane_ids = frozenset((current.link_id, adjacent.link_id))
        boundaries = [
            BoundarySegment(
                "{}:{}".format(current.link_id, current_outer[0]),
                current_outer[1],
                BoundarySide.RIGHT if current_outer[0] == "right" else BoundarySide.LEFT,
                current_outer[2],
                frozenset((current.link_id,)),
            ),
            BoundarySegment(
                "{}:{}".format(adjacent.link_id, adjacent_outer[0]),
                adjacent_outer[1],
                BoundarySide.LEFT if adjacent_outer[0] == "left" else BoundarySide.RIGHT,
                adjacent_outer[2],
                frozenset((adjacent.link_id,)),
            ),
            BoundarySegment(
                pair.shared_boundary_id or "synthetic:shared",
                current_shared,
                BoundarySide.SHARED,
                shared_marking,
                lane_ids,
            ),
        ]
        self._append_end_cap(
            boundaries, "lane_change:current:upstream", current, False
        )
        self._append_end_cap(
            boundaries, "lane_change:current:downstream", current, True
        )
        self._append_end_cap(
            boundaries, "lane_change:adjacent:upstream", adjacent, False
        )
        self._append_end_cap(
            boundaries, "lane_change:adjacent:downstream", adjacent, True
        )
        # Require the two sides to be the same physical polyline at millimetre
        # scale.  The policy stays closed if independent synthetic offsets only
        # happen to look adjacent.
        if len(current_shared) != len(adjacent_shared):
            topology_verified = False
        elif any(
            distance(first, second) > 0.001
            for first, second in zip(current_shared, adjacent_shared)
        ):
            topology_verified = False
        corridor = DrivingCorridor(
            current.link_id,
            (current.link_id, adjacent.link_id),
            (current_polygon, adjacent_polygon),
            tuple(boundaries),
        )
        return CorridorBuild(
            corridor=corridor,
            open_boundary_ids=(
                (pair.shared_boundary_id,) if pair.shared_boundary_id else ()
            ),
            current_lane=current,
            target_lane=adjacent,
            topology_verified=topology_verified,
            shared_marking=shared_marking,
            guidance_points=(),
            map_lanes=(current, adjacent),
        )


def parse_lane_change_pairs(values):
    if values is None:
        return {}
    if not isinstance(values, Mapping):
        raise TypeError("highway_lane_change_pairs must be a mapping")
    result = {}
    for route_link_id, raw in values.items():
        if not isinstance(raw, Mapping):
            raise TypeError(
                "lane-change pair {} must be a mapping".format(route_link_id)
            )
        enabled = raw.get("enabled", False)
        if type(enabled) is not bool:
            raise TypeError(
                "lane-change pair {} enabled must be boolean".format(
                    route_link_id
                )
            )
        if not enabled:
            continue
        pair = LaneChangePair(
            route_link_id=str(route_link_id),
            adjacent_link_id=str(raw["adjacent_link_id"]),
            direction=str(raw["direction"]),
            shared_boundary_id=str(raw.get("shared_boundary_id") or ""),
            merge_route_link_id=str(raw["merge_route_link_id"]),
            branch_by_route_link={
                str(key): str(value)
                for key, value in (raw.get("branch_by_route_link") or {}).items()
            },
        )
        result[pair.route_link_id] = pair
    return result
