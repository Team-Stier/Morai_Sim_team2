"""Small, dependency-free geometry helpers used by the offline pipeline."""

import math


def distance_2d(a, b):
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def squared_distance_2d(a, b):
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    return dx * dx + dy * dy


def polyline_length(points):
    return sum(distance_2d(a, b) for a, b in zip(points, points[1:]))


def cumulative_lengths(points):
    """Return monotonically increasing 2-D arclengths for a polyline."""
    result = [0.0]
    for first, second in zip(points, points[1:]):
        result.append(result[-1] + distance_2d(first, second))
    return result


def interpolate_point(first, second, ratio):
    """Linearly interpolate a 2-D/3-D point without mutating either input."""
    ratio = max(0.0, min(1.0, float(ratio)))
    dimension = max(len(first), len(second), 3)
    values = []
    for index in range(dimension):
        first_value = float(first[index]) if index < len(first) else 0.0
        second_value = float(second[index]) if index < len(second) else 0.0
        values.append(first_value + ratio * (second_value - first_value))
    return values


def point_at_progress(points, progress, lengths=None):
    """Interpolate the point at a clamped 2-D arclength along *points*."""
    if not points:
        raise ValueError("cannot sample an empty polyline")
    if len(points) == 1:
        return list(points[0])
    lengths = cumulative_lengths(points) if lengths is None else lengths
    target = max(0.0, min(float(progress), lengths[-1]))
    for index, (start, end) in enumerate(zip(lengths, lengths[1:])):
        if target <= end or index == len(lengths) - 2:
            span = end - start
            ratio = 0.0 if span <= 1.0e-12 else (target - start) / span
            return interpolate_point(points[index], points[index + 1], ratio)
    return list(points[-1])


def slice_polyline(points, start_progress, end_progress):
    """Clip a polyline to an arclength interval, retaining interior vertices."""
    if len(points) < 2:
        return [list(point) for point in points]
    lengths = cumulative_lengths(points)
    start = max(0.0, min(float(start_progress), lengths[-1]))
    end = max(0.0, min(float(end_progress), lengths[-1]))
    if end < start:
        start, end = end, start
    result = [point_at_progress(points, start, lengths)]
    for point, progress in zip(points[1:-1], lengths[1:-1]):
        if start + 1.0e-9 < progress < end - 1.0e-9:
            result.append(list(point))
    result.append(point_at_progress(points, end, lengths))
    return result


def _segment_distance_squared(point, start, end):
    vx = float(end[0]) - float(start[0])
    vy = float(end[1]) - float(start[1])
    wx = float(point[0]) - float(start[0])
    wy = float(point[1]) - float(start[1])
    denom = vx * vx + vy * vy
    if denom <= 1.0e-18:
        return squared_distance_2d(point, start)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
    dx = float(point[0]) - (float(start[0]) + t * vx)
    dy = float(point[1]) - (float(start[1]) + t * vy)
    return dx * dx + dy * dy


def _orientation(a, b, c):
    return ((float(b[0]) - float(a[0])) * (float(c[1]) - float(a[1])) -
            (float(b[1]) - float(a[1])) * (float(c[0]) - float(a[0])))


def segments_intersect_2d(first_start, first_end, second_start, second_end,
                          epsilon=1.0e-9):
    first_a = _orientation(first_start, first_end, second_start)
    first_b = _orientation(first_start, first_end, second_end)
    second_a = _orientation(second_start, second_end, first_start)
    second_b = _orientation(second_start, second_end, first_end)
    if ((first_a > epsilon and first_b < -epsilon) or
            (first_a < -epsilon and first_b > epsilon)) and (
            (second_a > epsilon and second_b < -epsilon) or
            (second_a < -epsilon and second_b > epsilon)):
        return True
    return min(
        _segment_distance_squared(first_start, second_start, second_end),
        _segment_distance_squared(first_end, second_start, second_end),
        _segment_distance_squared(second_start, first_start, first_end),
        _segment_distance_squared(second_end, first_start, first_end),
    ) <= epsilon * epsilon


def segment_distance_2d(first_start, first_end, second_start, second_end):
    if segments_intersect_2d(first_start, first_end, second_start, second_end):
        return 0.0
    return math.sqrt(min(
        _segment_distance_squared(first_start, second_start, second_end),
        _segment_distance_squared(first_end, second_start, second_end),
        _segment_distance_squared(second_start, first_start, first_end),
        _segment_distance_squared(second_end, first_start, first_end),
    ))


def point_segment_distance_2d(point, start, end):
    return math.sqrt(_segment_distance_squared(point, start, end))


def polyline_distance_2d(first, second):
    if not first or not second:
        return float("inf")
    if len(first) == 1:
        return closest_polyline_distance(first[0], second)
    if len(second) == 1:
        return closest_polyline_distance(second[0], first)
    return min(segment_distance_2d(a, b, c, d)
               for a, b in zip(first, first[1:])
               for c, d in zip(second, second[1:]))


def simplify_rdp(points, tolerance):
    """Iterative Ramer-Douglas-Peucker simplification preserving endpoints."""
    if len(points) <= 2 or tolerance <= 0.0:
        return [list(point) for point in points]
    keep = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    threshold = float(tolerance) ** 2
    while stack:
        first, last = stack.pop()
        farthest_index = None
        farthest_distance = -1.0
        for index in range(first + 1, last):
            value = _segment_distance_squared(points[index], points[first], points[last])
            if value > farthest_distance:
                farthest_distance = value
                farthest_index = index
        if farthest_index is not None and farthest_distance > threshold:
            keep.add(farthest_index)
            stack.append((first, farthest_index))
            stack.append((farthest_index, last))
    return [list(points[index]) for index in sorted(keep)]


def progress_along_polyline(point, line):
    """Return projected distance along *line* and squared lateral distance."""
    if len(line) < 2:
        return 0.0, squared_distance_2d(point, line[0]) if line else float("inf")
    best_progress = 0.0
    best_distance = float("inf")
    progress = 0.0
    for start, end in zip(line, line[1:]):
        vx = float(end[0]) - float(start[0])
        vy = float(end[1]) - float(start[1])
        length_squared = vx * vx + vy * vy
        segment_length = math.sqrt(length_squared)
        if length_squared <= 1.0e-18:
            continue
        wx = float(point[0]) - float(start[0])
        wy = float(point[1]) - float(start[1])
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / length_squared))
        projected = [float(start[0]) + t * vx, float(start[1]) + t * vy]
        candidate = squared_distance_2d(point, projected)
        if candidate < best_distance:
            best_distance = candidate
            best_progress = progress + t * segment_length
        progress += segment_length
    return best_progress, best_distance


def orient_and_stitch(polylines, reference):
    """Orient and order boundary fragments in reference-line travel direction."""
    parts = []
    for source_id, points in polylines:
        if len(points) < 2:
            continue
        start_progress, _ = progress_along_polyline(points[0], reference)
        end_progress, _ = progress_along_polyline(points[-1], reference)
        oriented = list(points)
        if end_progress < start_progress:
            oriented.reverse()
            start_progress, end_progress = end_progress, start_progress
        parts.append((start_progress, end_progress, source_id, oriented))
    parts.sort(key=lambda value: (value[0], value[1], value[2]))
    result = []
    max_gap = 0.0
    source_ids = []
    for _, _, source_id, part in parts:
        source_ids.append(source_id)
        if not result:
            result.extend(part)
            continue
        gap = distance_2d(result[-1], part[0])
        max_gap = max(max_gap, gap)
        if gap <= 0.001:
            result.extend(part[1:])
        else:
            result.extend(part)
    return result, source_ids, max_gap


def offset_polyline(points, offset):
    """Offset a 2D/3D centerline to its left (positive) using averaged normals."""
    if len(points) < 2:
        return [list(point) for point in points]
    normals = []
    for start, end in zip(points, points[1:]):
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        length = math.hypot(dx, dy)
        normals.append((0.0, 0.0) if length <= 1.0e-12 else (-dy / length, dx / length))
    result = []
    for index, point in enumerate(points):
        if index == 0:
            nx, ny = normals[0]
        elif index == len(points) - 1:
            nx, ny = normals[-1]
        else:
            nx = normals[index - 1][0] + normals[index][0]
            ny = normals[index - 1][1] + normals[index][1]
            normal_length = math.hypot(nx, ny)
            if normal_length > 1.0e-12:
                nx, ny = nx / normal_length, ny / normal_length
        shifted = [float(point[0]) + float(offset) * nx, float(point[1]) + float(offset) * ny]
        if len(point) > 2:
            shifted.append(float(point[2]))
        result.append(shifted)
    return result


def convex_hull(points):
    """Return a closed XY convex hull while retaining a representative Z value."""
    unique = {}
    for point in points:
        key = (round(float(point[0]), 6), round(float(point[1]), 6))
        unique[key] = [float(point[0]), float(point[1]), float(point[2]) if len(point) > 2 else 0.0]
    ordered = sorted(unique.values(), key=lambda point: (point[0], point[1]))
    if len(ordered) < 3:
        return []

    def cross(origin, first, second):
        return ((first[0] - origin[0]) * (second[1] - origin[1]) -
                (first[1] - origin[1]) * (second[0] - origin[0]))

    lower = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    return hull + [list(hull[0])]


def closest_polyline_distance(point, line):
    if not line:
        return float("inf")
    if len(line) == 1:
        return distance_2d(point, line[0])
    return math.sqrt(min(_segment_distance_squared(point, a, b)
                         for a, b in zip(line, line[1:])))
