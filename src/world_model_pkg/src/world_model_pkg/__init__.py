"""ROS-independent World Model geometry and tracking core."""

from .tracking import Detection, MultiObjectTracker, TrackerConfig, TrackView
from .transforms import MapAlignedBox, transform_aabb

__all__ = [
    "Detection",
    "MapAlignedBox",
    "MultiObjectTracker",
    "TrackerConfig",
    "TrackView",
    "transform_aabb",
]
