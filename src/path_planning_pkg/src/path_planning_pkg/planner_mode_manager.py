"""Choose the planner for the current ego position projected onto the map route.

No forward-only history is imposed: moving back or relocating selects the
zone at the current position. The manager owns no ROS interface.
"""

from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Optional, Tuple


HYBRID_ASTAR = "hybrid_astar"
FRENET = "frenet"
VALID_MODES = frozenset((HYBRID_ASTAR, FRENET))


@dataclass(frozen=True)
class PlannerZone:
    zone_id: str
    start_s: float
    end_s: float
    planner: str

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "PlannerZone":
        return cls(
            zone_id=str(value["id"]),
            start_s=float(value["start_s"]),
            end_s=float(value["end_s"]),
            planner=str(value["planner"]),
        )


@dataclass(frozen=True)
class PlannerModeSelection:
    zone_id: str
    planner: str
    progress_s: float
    changed: bool


class PlannerModeManager:
    """Select the zone at the current projected ego position."""

    def __init__(self, route_length_m: float, zones: Iterable[PlannerZone]) -> None:
        self.route_length_m = float(route_length_m)
        self.zones: Tuple[PlannerZone, ...] = tuple(zones)
        self._validate()
        self._progress_s: Optional[float] = None
        self._zone_index: Optional[int] = None

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "PlannerModeManager":
        return cls(
            route_length_m=float(value["route_length_m"]),
            zones=(PlannerZone.from_mapping(zone) for zone in value["zones"]),
        )

    def _validate(self) -> None:
        if not math.isfinite(self.route_length_m) or self.route_length_m <= 0.0:
            raise ValueError("route_length_m must be finite and positive")
        if not self.zones:
            raise ValueError("at least one planner zone is required")
        tolerance = 1.0e-6
        expected_start = 0.0
        identifiers = set()
        for zone in self.zones:
            if not zone.zone_id or zone.zone_id in identifiers:
                raise ValueError("planner zone ids must be non-empty and unique")
            identifiers.add(zone.zone_id)
            if zone.planner not in VALID_MODES:
                raise ValueError("unsupported planner mode: {}".format(zone.planner))
            if not all(math.isfinite(v) for v in (zone.start_s, zone.end_s)):
                raise ValueError("planner zone bounds must be finite")
            if zone.end_s <= zone.start_s:
                raise ValueError("planner zone end must be after its start")
            if abs(zone.start_s - expected_start) > tolerance:
                raise ValueError("planner zones must be contiguous and ordered")
            expected_start = zone.end_s
        if abs(expected_start - self.route_length_m) > tolerance:
            raise ValueError("planner zones must cover the complete route")

    def reset(self) -> None:
        self._progress_s = None
        self._zone_index = None

    def select(self, progress_s: float) -> PlannerModeSelection:
        progress = float(progress_s)
        if not math.isfinite(progress):
            raise ValueError("route progress must be finite")
        progress = min(max(progress, 0.0), self.route_length_m)
        index = len(self.zones) - 1
        for candidate, zone in enumerate(self.zones[:-1]):
            if progress < zone.end_s:
                index = candidate
                break

        changed = (self._zone_index is not None and
                   self.zones[index].planner != self.zones[self._zone_index].planner)
        self._progress_s = progress
        self._zone_index = index
        zone = self.zones[index]
        return PlannerModeSelection(zone.zone_id, zone.planner, progress, changed)

    @property
    def current(self) -> Optional[PlannerModeSelection]:
        if self._zone_index is None or self._progress_s is None:
            return None
        zone = self.zones[self._zone_index]
        return PlannerModeSelection(zone.zone_id, zone.planner, self._progress_s, False)
