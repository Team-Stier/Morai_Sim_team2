"""World Model boundary validation for the object-only LiDAR baseline.

Use validate_lidar(..., for_fusion=True) before accepting geometry into a scene.
Caller additionally enforces measured age, source reset and status freshness.
"""
import math
from .validation import require, validate_header


def validate_lidar(message, for_fusion=False):
    validate_header(message.header, 'lidar_link')
    require(bool(message.calibration_id), 'missing calibration identity')
    require(message.timestamp_provenance == 'ingress_fallback', 'unexpected clock provenance')
    require(not any((message.ground_valid, message.free_space_valid,
                     message.occupancy_valid, message.velocity_valid)),
            'unsupported layers in object-only schema')
    require(message.objects_valid or not message.objects, 'invalid scan carries objects')
    ids = set()
    for obj in message.objects:
        require(obj.scan_local_id not in ids, 'duplicate scan-local ID')
        ids.add(obj.scan_local_id)
        require(obj.point_count > 0, 'empty cluster')
        for value in (obj.center.x, obj.center.y, obj.center.z):
            require(math.isfinite(value), 'nonfinite center')
        for value in (obj.size.x, obj.size.y, obj.size.z):
            require(math.isfinite(value) and value >= 0, 'invalid box size')
        require(obj.confidence == -1 or (math.isfinite(obj.confidence) and
                                        0 <= obj.confidence <= 1), 'invalid confidence')
    if for_fusion:
        require(message.objects_valid and message.calibration_verified and
                message.freshness_verified, 'development observation cannot enter live scene')
