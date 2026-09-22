"""ROS-independent World Model geometry and tracking core."""

from .tracking import Detection, MultiObjectTracker, TrackerConfig, TrackView
from .transforms import transform_points

__all__ = [
    "Detection",
    "MultiObjectTracker",
    "TrackerConfig",
    "TrackView",
    "transform_points",
]
