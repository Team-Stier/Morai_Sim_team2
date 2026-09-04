"""Dependency-free path-planning core for ROS1 adapters."""

from .collision import FootprintCollisionChecker, PoseValidity
from .corridor import (
    BoundaryMarking,
    BoundarySegment,
    BoundarySide,
    CorridorMode,
    CorridorPolicy,
    CorridorPolicyDecision,
    CorridorPolicyInput,
    DrivingCorridor,
    EffectiveCorridor,
    LanePolygon,
)
from .hybrid_astar import (
    DiscreteStateKey,
    HybridAStarConfig,
    HybridAStarPlanner,
    PathPoint,
    PlanningDiagnostics,
    PlanningRequest,
    PlanResult,
    PlanStatus,
)
from .models import BoxObstacle, CircleObstacle, Point2D, Pose2D, VehicleGeometry

__all__ = (
    "BoundaryMarking",
    "BoundarySegment",
    "BoundarySide",
    "BoxObstacle",
    "CircleObstacle",
    "CorridorMode",
    "CorridorPolicy",
    "CorridorPolicyDecision",
    "CorridorPolicyInput",
    "DiscreteStateKey",
    "DrivingCorridor",
    "EffectiveCorridor",
    "FootprintCollisionChecker",
    "HybridAStarConfig",
    "HybridAStarPlanner",
    "LanePolygon",
    "PathPoint",
    "PlanningDiagnostics",
    "PlanningRequest",
    "PlanResult",
    "PlanStatus",
    "Point2D",
    "Pose2D",
    "PoseValidity",
    "VehicleGeometry",
)
