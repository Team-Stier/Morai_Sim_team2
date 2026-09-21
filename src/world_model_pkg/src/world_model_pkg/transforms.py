"""Small, dependency-free rigid transform helpers for LiDAR cluster points."""

import math


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


def transform_points(points, translation, quaternion):
    """Rigidly transform each measured point, preserving order and cardinality."""
    translation = tuple(float(v) for v in translation)
    points = tuple(tuple(float(v) for v in p) for p in points)
    if len(translation) != 3 or not _finite(translation):
        raise ValueError("invalid translation")
    if not points or any(len(p) != 3 or not _finite(p) for p in points):
        raise ValueError("empty or nonfinite cluster")
    rotation = _rotation_matrix(quaternion)
    return tuple(tuple(translation[row] + sum(rotation[row][col] * p[col]
                       for col in range(3)) for row in range(3)) for p in points)
