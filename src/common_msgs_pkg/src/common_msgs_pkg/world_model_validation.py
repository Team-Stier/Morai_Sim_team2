"""Pure validation for the centrally approved World Model wire contract."""

import math


def _stamp_sec(stamp):
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    return float(stamp.secs) + float(stamp.nsecs) * 1.0e-9


def _finite(*values):
    return all(math.isfinite(float(value)) for value in values)


def _valid_quaternion(value):
    values = (value.x, value.y, value.z, value.w)
    return _finite(*values) and abs(sum(float(item) ** 2 for item in values) - 1.0) <= 1.0e-5


def validate_world_model(message, for_planning=False):
    """Reject malformed, temporally inconsistent, or over-claimed scenes."""

    if message.header.frame_id != "map" or _stamp_sec(message.header.stamp) <= 0.0:
        raise ValueError("WorldModel needs a positive map-frame fusion stamp")
    if not message.objects_valid and message.objects:
        raise ValueError("invalid object geometry cannot carry objects")
    if message.tracking_valid and not message.objects_valid:
        raise ValueError("tracking validity requires valid object geometry")
    if message.objects_verified and not (message.objects_valid and message.tracking_valid):
        raise ValueError("verified objects require valid geometry and tracking")
    required_capabilities = (
        message.objects_verified,
        message.map_context_valid,
        message.route_context_valid,
    )
    if message.planner_ready and not all(required_capabilities):
        raise ValueError("planner-ready scene is missing a required capability")
    if for_planning and not message.planner_ready:
        raise ValueError("WorldModel is diagnostic-only and not planner ready")

    seen = set()
    fusion_stamp = _stamp_sec(message.header.stamp)
    for item in message.objects:
        if item.track_id == 0 or item.track_id in seen:
            raise ValueError("track IDs must be nonzero and unique")
        seen.add(item.track_id)
        source_stamp = _stamp_sec(item.source_stamp)
        if source_stamp <= 0.0 or source_stamp > fusion_stamp + 1.0e-9:
            raise ValueError("object source stamp is zero or after fusion time")
        expected_age = fusion_stamp - source_stamp
        if not _finite(item.age_since_observation_sec) or item.age_since_observation_sec < 0.0:
            raise ValueError("object age must be finite and non-negative")
        if abs(item.age_since_observation_sec - expected_age) > 1.0e-6:
            raise ValueError("object age disagrees with source and fusion stamps")
        if not item.source_frame_id or not item.timestamp_provenance or not item.calibration_id:
            raise ValueError("object source provenance must be complete")
        if not _valid_quaternion(item.pose.orientation):
            raise ValueError("object pose quaternion must be finite and normalized")
        if not _finite(item.pose.position.x, item.pose.position.y, item.pose.position.z):
            raise ValueError("object position must be finite")
        if not item.points or any(not _finite(p.x, p.y, p.z) for p in item.points):
            raise ValueError("cluster points must be nonempty and finite")
        if item.track_state not in (item.TRACK_TENTATIVE, item.TRACK_CONFIRMED, item.TRACK_COASTING):
            raise ValueError("unknown track state")
        if item.semantic_class not in (
            item.CLASS_UNKNOWN,
            item.CLASS_PEDESTRIAN,
            item.CLASS_VEHICLE,
            item.CLASS_OTHER,
        ):
            raise ValueError("unknown semantic class")
        if not _finite(item.confidence) or not (-1.0 <= item.confidence <= 1.0):
            raise ValueError("confidence must be -1 or a probability")
        if not _finite(item.position_stddev_m, item.velocity_stddev_mps):
            raise ValueError("uncertainty sentinels must be finite")
        if item.position_stddev_m < -1.0 or item.velocity_stddev_mps < -1.0:
            raise ValueError("uncertainty must be non-negative or -1")
        linear = item.twist.linear
        angular = item.twist.angular
        if not _finite(linear.x, linear.y, linear.z, angular.x, angular.y, angular.z):
            raise ValueError("object twist must be finite")
        if item.velocity_valid and item.velocity_stddev_mps < 0.0:
            raise ValueError("valid velocity requires bounded uncertainty")
        if item.calibration_verified is False and message.objects_verified:
            raise ValueError("unverified calibration cannot yield verified objects")
    return True
