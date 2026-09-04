"""Prepare immutable MGeo boundary geometry for ROS visualization.

This module deliberately contains no ROS imports so the coordinate and
classification rules can be unit-tested without a ROS installation.
"""

from collections import defaultdict

from .lanelet2_export import boundary_tags


LAYER_NAMES = ("solid", "dashed", "mixed", "road_border", "other")


def boundary_layer(tags):
    """Return a conservative visual category for one normalized boundary."""
    if tags.get("type") == "road_border":
        return "road_border"
    subtype = str(tags.get("subtype", "unknown"))
    if subtype == "dashed":
        return "dashed"
    if subtype == "solid":
        return "solid"
    if "solid" in subtype and "dashed" in subtype:
        return "mixed"
    return "other"


def build_boundary_layers(dataset, transformer, config):
    """Return layer -> flat line-segment endpoint tuples in scene-local ENU."""
    layers = defaultdict(list)
    mapping = config.get("lane_boundary", {})
    for boundary_id in sorted(dataset.lane_boundaries):
        boundary = dataset.lane_boundaries[boundary_id]
        points = boundary.get("points") or []
        if len(points) < 2:
            continue
        tags = boundary_tags(boundary, mapping)
        layer = boundary_layer(tags)
        transformed = [transformer.mgeo_to_sim(point) for point in points]
        for start, finish in zip(transformed, transformed[1:]):
            if start[:2] == finish[:2]:
                continue
            layers[layer].extend((start, finish))
    return {name: tuple(layers.get(name, ())) for name in LAYER_NAMES}
