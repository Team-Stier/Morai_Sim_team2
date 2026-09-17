"""Small, dependency-free rigid transform helpers for LiDAR AABBs."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MapAlignedBox:
    center: tuple
    size: tuple


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def _rotation_matrix(quaternion):
    x, y, z, w = (float(value) for value in quaternion)
    if not _finite((x, y, z, w)):
        raise ValueError("transform quaternion is non-finite")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError("transform quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def transform_aabb(center, size, translation, quaternion):
    """Transform a source-frame AABB into a conservative map-aligned AABB."""

    center = tuple(float(value) for value in center)
    size = tuple(float(value) for value in size)
    translation = tuple(float(value) for value in translation)
    if len(center) != 3 or len(size) != 3 or len(translation) != 3:
        raise ValueError("center, size and translation must contain three values")
    if not _finite(center + size + translation) or min(size) <= 0.0:
        raise ValueError("box geometry must be finite with positive size")
    rotation = _rotation_matrix(quaternion)
    mapped_center = tuple(
        translation[row] + sum(rotation[row][column] * center[column] for column in range(3))
        for row in range(3)
    )
    half = tuple(0.5 * value for value in size)
    mapped_half = tuple(
        sum(abs(rotation[row][column]) * half[column] for column in range(3))
        for row in range(3)
    )
    return MapAlignedBox(mapped_center, tuple(2.0 * value for value in mapped_half))
