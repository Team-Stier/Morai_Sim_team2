"""Official-route loading and monotonic progress tracking."""

from .reference_route import (
    LinkSpan,
    MatchResult,
    ReferenceRoute,
    RouteContextState,
    RouteFormatError,
    RouteMatcher,
    RouteTopology,
    gate_observation_stamp,
    load_reference_route,
    odometry_payload_invalid_reason,
)

__all__ = [
    "LinkSpan",
    "MatchResult",
    "ReferenceRoute",
    "RouteContextState",
    "RouteFormatError",
    "RouteMatcher",
    "RouteTopology",
    "gate_observation_stamp",
    "load_reference_route",
    "odometry_payload_invalid_reason",
]
