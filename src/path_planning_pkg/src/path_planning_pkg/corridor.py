"""Fail-closed lane-corridor and virtual-wall policy.

Every polygon edge and every declared boundary starts as a hard wall.  A wall
can disappear only through one of the explicit policy gates below.  Unknown or
partially specified map metadata therefore makes the planner more restrictive,
never more permissive.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import FrozenSet, Iterable, Mapping, Optional, Sequence, Tuple

from .geometry import (
    GEOMETRY_EPSILON,
    distance,
    edge_matches_polyline,
    is_finite_point,
    polygon_edges,
    validate_simple_polygon,
)
from .models import Point2D


class BoundaryMarking(Enum):
    SOLID = "solid"
    DASHED = "dashed"
    SOLID_DASHED = "solid_dashed"
    CURB = "curb"
    VIRTUAL = "virtual"
    UNKNOWN = "unknown"


class BoundarySide(Enum):
    LEFT = "left"
    RIGHT = "right"
    SHARED = "shared"
    CONNECTOR_MOUTH = "connector_mouth"
    UPSTREAM_END = "upstream_end"
    DOWNSTREAM_END = "downstream_end"
    OTHER = "other"


class CorridorMode(Enum):
    KEEP_LANE = "keep_lane"
    TURN_CONNECTOR = "turn_connector"
    HIGHWAY_OVERTAKE = "highway_overtake"


@dataclass(frozen=True)
class LanePolygon:
    lane_id: str
    vertices: Tuple[Point2D, ...]

    def __post_init__(self) -> None:
        if not self.lane_id:
            raise ValueError("lane_id must not be empty")
        compact = _compact_points(self.vertices, close_ring=True)
        validate_simple_polygon(compact, "lane polygon {}".format(self.lane_id))
        object.__setattr__(self, "vertices", compact)


@dataclass(frozen=True)
class BoundarySegment:
    boundary_id: str
    points: Tuple[Point2D, ...]
    side: BoundarySide = BoundarySide.OTHER
    marking: BoundaryMarking = BoundaryMarking.UNKNOWN
    lane_ids: FrozenSet[str] = field(default_factory=frozenset)
    directional_lane_change_allowed: bool = False

    def __post_init__(self) -> None:
        if not self.boundary_id:
            raise ValueError("boundary_id must not be empty")
        compact = _compact_points(self.points, close_ring=False)
        if len(compact) < 2:
            raise ValueError("boundary {} needs at least two points".format(self.boundary_id))
        object.__setattr__(self, "points", compact)
        object.__setattr__(self, "lane_ids", frozenset(self.lane_ids))


def _compact_points(
    points: Iterable[Point2D], close_ring: bool
) -> Tuple[Point2D, ...]:
    compact = []
    for point in points:
        if not is_finite_point(point):
            raise ValueError("geometry contains a non-finite point")
        if not compact or distance(point, compact[-1]) > GEOMETRY_EPSILON:
            compact.append(point)
    if close_ring and len(compact) > 1:
        if distance(compact[0], compact[-1]) <= GEOMETRY_EPSILON:
            compact.pop()
    return tuple(compact)


@dataclass(frozen=True)
class DrivingCorridor:
    source_link_id: str
    route_link_ids: Tuple[str, ...]
    lanes: Tuple[LanePolygon, ...]
    boundaries: Tuple[BoundarySegment, ...]

    def __post_init__(self) -> None:
        if not self.source_link_id:
            raise ValueError("source_link_id must not be empty")
        route_ids = tuple(self.route_link_ids)
        lanes = tuple(self.lanes)
        boundaries = tuple(self.boundaries)
        if not route_ids or self.source_link_id not in route_ids:
            raise ValueError("ordered route_link_ids must contain source_link_id")
        if len(set(route_ids)) != len(route_ids):
            raise ValueError("route_link_ids must be unique and ordered")
        if not lanes:
            raise ValueError("driving corridor must contain at least one lane polygon")
        lane_ids = [lane.lane_id for lane in lanes]
        if len(set(lane_ids)) != len(lane_ids):
            raise ValueError("lane ids must be unique")
        boundary_ids = [boundary.boundary_id for boundary in boundaries]
        if len(set(boundary_ids)) != len(boundary_ids):
            raise ValueError("boundary ids must be unique")
        known_lanes = set(lane_ids)
        for boundary in boundaries:
            if not boundary.lane_ids.issubset(known_lanes):
                raise ValueError(
                    "boundary {} refers to an unknown lane".format(boundary.boundary_id)
                )
        object.__setattr__(self, "route_link_ids", route_ids)
        object.__setattr__(self, "lanes", lanes)
        object.__setattr__(self, "boundaries", boundaries)

    @classmethod
    def from_polygon(
        cls,
        source_link_id: str,
        lane_id: str,
        vertices: Sequence[Point2D],
    ) -> "DrivingCorridor":
        lane = LanePolygon(lane_id, tuple(vertices))
        boundaries = tuple(
            BoundarySegment(
                "{}:edge:{}".format(lane_id, index),
                (start, finish),
                BoundarySide.OTHER,
                BoundaryMarking.UNKNOWN,
                frozenset((lane_id,)),
            )
            for index, (start, finish) in enumerate(polygon_edges(lane.vertices))
        )
        return cls(source_link_id, (source_link_id,), (lane,), boundaries)


@dataclass(frozen=True)
class CorridorPolicyInput:
    mode: CorridorMode = CorridorMode.KEEP_LANE
    current_lane_id: str = ""
    requested_open_boundary_ids: FrozenSet[str] = field(default_factory=frozenset)
    turn_connector_verified: bool = False
    overtake_requested: bool = False
    high_speed_zone: bool = False
    adjacent_lane_verified: bool = False
    adjacent_lane_id: str = ""
    lead_vehicle_distance_m: Optional[float] = None
    lane_change_latched: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "requested_open_boundary_ids", frozenset(self.requested_open_boundary_ids)
        )


@dataclass(frozen=True)
class CorridorPolicyDecision:
    mode: CorridorMode
    opened_boundary_ids: FrozenSet[str]
    hard_boundary_ids: FrozenSet[str]
    lane_change_enabled: bool
    fail_closed: bool
    diagnostics: Tuple[str, ...]


@dataclass(frozen=True)
class HardBoundaryEdge:
    boundary_id: str
    start: Point2D
    finish: Point2D
    implicit_polygon_edge: bool = False


@dataclass(frozen=True)
class EffectiveCorridor:
    corridor: DrivingCorridor
    decision: CorridorPolicyDecision
    hard_edges: Tuple[HardBoundaryEdge, ...]
    polygon_bounds: Tuple[Tuple[float, float, float, float], ...]
    hard_edge_bounds: Tuple[Tuple[float, float, float, float], ...]
    hard_edge_spatial_index: Mapping[Tuple[int, int], Tuple[int, ...]]
    hard_edge_grid_resolution_m: float

    @property
    def polygons(self) -> Tuple[Tuple[Point2D, ...], ...]:
        return tuple(lane.vertices for lane in self.corridor.lanes)

    def nearby_hard_edge_indices(
        self,
        minimum_x: float,
        minimum_y: float,
        maximum_x: float,
        maximum_y: float,
        margin_m: float,
    ) -> Tuple[int, ...]:
        resolution = self.hard_edge_grid_resolution_m
        first_x = int(math.floor((minimum_x - margin_m) / resolution))
        last_x = int(math.floor((maximum_x + margin_m) / resolution))
        first_y = int(math.floor((minimum_y - margin_m) / resolution))
        last_y = int(math.floor((maximum_y + margin_m) / resolution))
        indices = set()
        for x_index in range(first_x, last_x + 1):
            for y_index in range(first_y, last_y + 1):
                indices.update(
                    self.hard_edge_spatial_index.get((x_index, y_index), ())
                )
        return tuple(sorted(indices))


class CorridorPolicy:
    """Resolve virtual walls without silently granting lane-change authority."""

    def resolve(
        self, corridor: DrivingCorridor, policy_input: CorridorPolicyInput
    ) -> CorridorPolicyDecision:
        boundaries = {boundary.boundary_id: boundary for boundary in corridor.boundaries}
        all_boundary_ids = frozenset(boundaries)
        requested = policy_input.requested_open_boundary_ids
        diagnostics = []

        if policy_input.mode is CorridorMode.KEEP_LANE:
            if requested:
                diagnostics.append("KEEP_LANE ignores every wall-opening request")
            diagnostics.append("solid and dashed lateral boundaries remain hard walls")
            return CorridorPolicyDecision(
                policy_input.mode,
                frozenset(),
                all_boundary_ids,
                False,
                False,
                tuple(diagnostics),
            )

        missing_ids = requested.difference(all_boundary_ids)
        if missing_ids:
            diagnostics.append(
                "requested boundary ids are absent: {}".format(",".join(sorted(missing_ids)))
            )
        if not requested:
            diagnostics.append("no boundary was explicitly selected for opening")

        if policy_input.mode is CorridorMode.TURN_CONNECTOR:
            if not policy_input.turn_connector_verified:
                diagnostics.append("turn connector was not verified")
            invalid = []
            for boundary_id in requested.intersection(all_boundary_ids):
                boundary = boundaries[boundary_id]
                matched_lane_ids = frozenset(
                    lane.lane_id
                    for lane in corridor.lanes
                    if any(
                        edge_matches_polyline(start, finish, boundary.points)
                        for start, finish in polygon_edges(lane.vertices)
                    )
                )
                if (
                    boundary.side is not BoundarySide.CONNECTOR_MOUTH
                    or boundary.marking is not BoundaryMarking.VIRTUAL
                    or len(boundary.points) != 2
                    or len(boundary.lane_ids) != 2
                    or matched_lane_ids != boundary.lane_ids
                ):
                    invalid.append(boundary_id)
            if invalid:
                diagnostics.append(
                    "turn policy may open verified virtual connector mouths only: {}".format(
                        ",".join(sorted(invalid))
                    )
                )
            qualified = (
                policy_input.turn_connector_verified
                and bool(requested)
                and not missing_ids
                and not invalid
            )
            opened = requested if qualified else frozenset()
            if qualified:
                diagnostics.append("verified turn-connector mouth opened")
            else:
                diagnostics.append("turn opening denied; all boundaries stay hard")
            return CorridorPolicyDecision(
                policy_input.mode,
                frozenset(opened),
                all_boundary_ids.difference(opened),
                False,
                not qualified,
                tuple(diagnostics),
            )

        # HIGHWAY_OVERTAKE is intentionally an atomic gate.  One missing fact
        # rejects the whole opening instead of opening the subset that happened
        # to pass validation.
        lane_ids = {lane.lane_id for lane in corridor.lanes}
        if not policy_input.overtake_requested:
            diagnostics.append("overtake was not explicitly requested")
        if not policy_input.high_speed_zone:
            diagnostics.append("current route context is not an approved high-speed zone")
        if not policy_input.adjacent_lane_verified:
            diagnostics.append("adjacent lane topology was not verified")
        if not policy_input.current_lane_id or policy_input.current_lane_id not in lane_ids:
            diagnostics.append("current lane id is absent from the corridor")
        if (
            not policy_input.adjacent_lane_id
            or policy_input.adjacent_lane_id not in lane_ids
            or policy_input.adjacent_lane_id == policy_input.current_lane_id
        ):
            diagnostics.append("adjacent lane id is absent, equal to current, or unknown")
        lead_distance = policy_input.lead_vehicle_distance_m
        lead_gap_valid = (
            lead_distance is not None
            and math.isfinite(lead_distance)
            and 0.0 <= lead_distance <= 10.0
        )
        if not lead_gap_valid and not policy_input.lane_change_latched:
            diagnostics.append("lead vehicle must be longitudinally 0..10 m ahead")
        elif not lead_gap_valid and policy_input.lane_change_latched:
            diagnostics.append(
                "stale/out-of-range lead gap ignored only for the latched crossing"
            )

        invalid_boundaries = []
        for boundary_id in requested.intersection(all_boundary_ids):
            boundary = boundaries[boundary_id]
            connects_selected_lanes = {
                policy_input.current_lane_id,
                policy_input.adjacent_lane_id,
            }.issubset(boundary.lane_ids)
            # User policy is stricter than directional MGeo permission: a
            # double/mixed line still contains a physical solid component, so
            # only an entirely dashed shared marking can disappear.
            marking_allows_crossing = boundary.marking is BoundaryMarking.DASHED
            if (
                boundary.side is not BoundarySide.SHARED
                or not marking_allows_crossing
                or not connects_selected_lanes
            ):
                invalid_boundaries.append(boundary_id)
        if invalid_boundaries:
            diagnostics.append(
                "only a pure-dashed shared boundary between selected lanes may open: {}".format(
                    ",".join(sorted(invalid_boundaries))
                )
            )

        qualified = (
            policy_input.overtake_requested
            and policy_input.high_speed_zone
            and policy_input.adjacent_lane_verified
            and policy_input.current_lane_id in lane_ids
            and policy_input.adjacent_lane_id in lane_ids
            and policy_input.adjacent_lane_id != policy_input.current_lane_id
            and (lead_gap_valid or policy_input.lane_change_latched)
            and bool(requested)
            and not missing_ids
            and not invalid_boundaries
        )
        opened = requested if qualified else frozenset()
        if qualified:
            diagnostics.append(
                "high-speed overtake gate enabled; selected internal wall opened"
            )
        else:
            diagnostics.append("overtake opening denied; all boundaries stay hard")
        return CorridorPolicyDecision(
            policy_input.mode,
            frozenset(opened),
            all_boundary_ids.difference(opened),
            qualified,
            not qualified,
            tuple(diagnostics),
        )

    def apply(
        self, corridor: DrivingCorridor, policy_input: CorridorPolicyInput
    ) -> EffectiveCorridor:
        decision = self.resolve(corridor, policy_input)
        boundaries = {boundary.boundary_id: boundary for boundary in corridor.boundaries}
        opened_polylines = tuple(
            boundaries[boundary_id].points
            for boundary_id in decision.opened_boundary_ids
        )
        hard_edges = []
        seen_geometry = set()

        def append_edge(boundary_id, start, finish, implicit):
            first = (round(start.x, 9), round(start.y, 9))
            second = (round(finish.x, 9), round(finish.y, 9))
            key = tuple(sorted((first, second)))
            if key in seen_geometry:
                return
            seen_geometry.add(key)
            hard_edges.append(HardBoundaryEdge(boundary_id, start, finish, implicit))

        # Polygon edges are implicit walls.  This protects against incomplete
        # boundary metadata and makes policy failure conservative.
        for lane in corridor.lanes:
            for index, (start, finish) in enumerate(polygon_edges(lane.vertices)):
                if any(
                    edge_matches_polyline(start, finish, polyline)
                    for polyline in opened_polylines
                ):
                    continue
                append_edge(
                    "implicit:{}:{}".format(lane.lane_id, index),
                    start,
                    finish,
                    True,
                )

        # Declared hard polylines remain walls even if they lie inside one large
        # combined polygon (the common representation for a multi-lane road).
        for boundary_id in decision.hard_boundary_ids:
            boundary = boundaries[boundary_id]
            for index in range(len(boundary.points) - 1):
                append_edge(
                    boundary_id,
                    boundary.points[index],
                    boundary.points[index + 1],
                    False,
                )
        polygon_bounds = tuple(
            (
                min(point.x for point in lane.vertices),
                min(point.y for point in lane.vertices),
                max(point.x for point in lane.vertices),
                max(point.y for point in lane.vertices),
            )
            for lane in corridor.lanes
        )
        hard_edge_bounds = tuple(
            (
                min(edge.start.x, edge.finish.x),
                min(edge.start.y, edge.finish.y),
                max(edge.start.x, edge.finish.x),
                max(edge.start.y, edge.finish.y),
            )
            for edge in hard_edges
        )
        grid_resolution_m = 5.0
        spatial_values = {}
        for edge_index, bounds in enumerate(hard_edge_bounds):
            minimum_x, minimum_y, maximum_x, maximum_y = bounds
            first_x = int(math.floor(minimum_x / grid_resolution_m))
            last_x = int(math.floor(maximum_x / grid_resolution_m))
            first_y = int(math.floor(minimum_y / grid_resolution_m))
            last_y = int(math.floor(maximum_y / grid_resolution_m))
            for x_index in range(first_x, last_x + 1):
                for y_index in range(first_y, last_y + 1):
                    spatial_values.setdefault((x_index, y_index), []).append(
                        edge_index
                    )
        spatial_index = {
            key: tuple(indices) for key, indices in spatial_values.items()
        }
        return EffectiveCorridor(
            corridor,
            decision,
            tuple(hard_edges),
            polygon_bounds,
            hard_edge_bounds,
            spatial_index,
            grid_resolution_m,
        )
