"""Small, dependency-free geometry helpers used by the planner core.

The functions in this module deliberately treat contact as intersection.  That
convention is important for the competition rule: a zero-distance contact with
a lane marking must never be reported as safe.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import Iterable, Sequence, Tuple

from .models import Point2D, Pose2D


GEOMETRY_EPSILON = 1.0e-9


class PointLocation(Enum):
    OUTSIDE = "outside"
    INSIDE = "inside"
    BOUNDARY = "boundary"


def is_finite_number(value: float) -> bool:
    return math.isfinite(value)


def is_finite_point(point: Point2D) -> bool:
    return math.isfinite(point.x) and math.isfinite(point.y)


def is_finite_pose(pose: Pose2D) -> bool:
    return (
        math.isfinite(pose.x)
        and math.isfinite(pose.y)
        and math.isfinite(pose.yaw)
    )


def normalize_yaw(yaw: float) -> float:
    if not math.isfinite(yaw):
        return yaw
    wrapped = math.fmod(yaw + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def angular_distance(lhs: float, rhs: float) -> float:
    return abs(normalize_yaw(lhs - rhs))


def squared_distance(lhs: Point2D, rhs: Point2D) -> float:
    dx = lhs.x - rhs.x
    dy = lhs.y - rhs.y
    return dx * dx + dy * dy


def distance(lhs: Point2D, rhs: Point2D) -> float:
    return math.sqrt(squared_distance(lhs, rhs))


def transform_local_point(pose: Pose2D, local_x: float, local_y: float) -> Point2D:
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return Point2D(
        pose.x + cosine * local_x - sine * local_y,
        pose.y + sine * local_x + cosine * local_y,
    )


def point_segment_distance(
    point: Point2D, start: Point2D, finish: Point2D
) -> float:
    dx = finish.x - start.x
    dy = finish.y - start.y
    length_squared = dx * dx + dy * dy
    if length_squared <= GEOMETRY_EPSILON * GEOMETRY_EPSILON:
        return distance(point, start)
    ratio = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared
    ratio = max(0.0, min(1.0, ratio))
    projection = Point2D(start.x + ratio * dx, start.y + ratio * dy)
    return distance(point, projection)


def _orientation(a: Point2D, b: Point2D, c: Point2D) -> int:
    value = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    if value > GEOMETRY_EPSILON:
        return 1
    if value < -GEOMETRY_EPSILON:
        return -1
    return 0


def _point_on_segment(point: Point2D, start: Point2D, finish: Point2D) -> bool:
    return point_segment_distance(point, start, finish) <= GEOMETRY_EPSILON


def segments_intersect(
    a: Point2D, b: Point2D, c: Point2D, d: Point2D
) -> bool:
    abc = _orientation(a, b, c)
    abd = _orientation(a, b, d)
    cda = _orientation(c, d, a)
    cdb = _orientation(c, d, b)
    if abc * abd < 0 and cda * cdb < 0:
        return True
    return (
        (abc == 0 and _point_on_segment(c, a, b))
        or (abd == 0 and _point_on_segment(d, a, b))
        or (cda == 0 and _point_on_segment(a, c, d))
        or (cdb == 0 and _point_on_segment(b, c, d))
    )


def segment_distance(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> float:
    if segments_intersect(a, b, c, d):
        return 0.0
    return min(
        point_segment_distance(a, c, d),
        point_segment_distance(b, c, d),
        point_segment_distance(c, a, b),
        point_segment_distance(d, a, b),
    )


def polygon_edges(vertices: Sequence[Point2D]) -> Tuple[Tuple[Point2D, Point2D], ...]:
    return tuple(
        (vertices[index], vertices[(index + 1) % len(vertices)])
        for index in range(len(vertices))
    )


def signed_area(vertices: Sequence[Point2D]) -> float:
    twice_area = 0.0
    for start, finish in polygon_edges(vertices):
        twice_area += start.x * finish.y - finish.x * start.y
    return 0.5 * twice_area


def validate_simple_polygon(vertices: Sequence[Point2D], name: str) -> None:
    if len(vertices) < 3:
        raise ValueError("{} must contain at least three vertices".format(name))
    if any(not is_finite_point(point) for point in vertices):
        raise ValueError("{} contains a non-finite vertex".format(name))
    for start, finish in polygon_edges(vertices):
        if distance(start, finish) <= GEOMETRY_EPSILON:
            raise ValueError("{} contains duplicate adjacent vertices".format(name))
    if abs(signed_area(vertices)) <= GEOMETRY_EPSILON:
        raise ValueError("{} has zero area".format(name))
    edges = polygon_edges(vertices)
    for first_index, first in enumerate(edges):
        for second_index in range(first_index + 1, len(edges)):
            if second_index in (
                first_index,
                (first_index + 1) % len(edges),
                (first_index - 1) % len(edges),
            ):
                continue
            second = edges[second_index]
            if segments_intersect(first[0], first[1], second[0], second[1]):
                raise ValueError("{} self-intersects".format(name))


def locate_point_in_polygon(point: Point2D, vertices: Sequence[Point2D]) -> PointLocation:
    if not is_finite_point(point):
        return PointLocation.OUTSIDE
    inside = False
    for start, finish in polygon_edges(vertices):
        if _point_on_segment(point, start, finish):
            return PointLocation.BOUNDARY
        crosses_y = (start.y > point.y) != (finish.y > point.y)
        if crosses_y:
            crossing_x = start.x + (
                (point.y - start.y) * (finish.x - start.x) / (finish.y - start.y)
            )
            if point.x < crossing_x:
                inside = not inside
    return PointLocation.INSIDE if inside else PointLocation.OUTSIDE


def point_in_polygon_union(
    point: Point2D, polygons: Iterable[Sequence[Point2D]]
) -> bool:
    return any(
        locate_point_in_polygon(point, vertices) is not PointLocation.OUTSIDE
        for vertices in polygons
    )


def point_polyline_distance(point: Point2D, points: Sequence[Point2D]) -> float:
    return min(
        point_segment_distance(point, points[index], points[index + 1])
        for index in range(len(points) - 1)
    )


def edge_matches_polyline(
    start: Point2D,
    finish: Point2D,
    points: Sequence[Point2D],
    tolerance_m: float = 1.0e-6,
) -> bool:
    """Return true when a polygon edge is represented by an opened polyline.

    Endpoints plus two interior samples are checked.  This supports an opened
    polyline made of several shorter collinear segments while refusing to erase
    a merely crossing boundary.
    """

    for ratio in (0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0):
        sample = Point2D(
            start.x + ratio * (finish.x - start.x),
            start.y + ratio * (finish.y - start.y),
        )
        if point_polyline_distance(sample, points) > tolerance_m:
            return False
    return True


def polygon_segment_distance(
    vertices: Sequence[Point2D], start: Point2D, finish: Point2D
) -> float:
    if locate_point_in_polygon(start, vertices) is not PointLocation.OUTSIDE:
        return 0.0
    if locate_point_in_polygon(finish, vertices) is not PointLocation.OUTSIDE:
        return 0.0
    return min(
        segment_distance(edge_start, edge_finish, start, finish)
        for edge_start, edge_finish in polygon_edges(vertices)
    )


def _intersection_parameters(
    start: Point2D, finish: Point2D, edge_start: Point2D, edge_finish: Point2D
) -> Tuple[float, ...]:
    """Return useful parameters where two segments meet, including collinear ends."""

    rx = finish.x - start.x
    ry = finish.y - start.y
    sx = edge_finish.x - edge_start.x
    sy = edge_finish.y - edge_start.y
    denominator = rx * sy - ry * sx
    qx = edge_start.x - start.x
    qy = edge_start.y - start.y
    if abs(denominator) > GEOMETRY_EPSILON:
        ratio = (qx * sy - qy * sx) / denominator
        other_ratio = (qx * ry - qy * rx) / denominator
        if (
            -GEOMETRY_EPSILON <= ratio <= 1.0 + GEOMETRY_EPSILON
            and -GEOMETRY_EPSILON <= other_ratio <= 1.0 + GEOMETRY_EPSILON
        ):
            return (max(0.0, min(1.0, ratio)),)
        return ()
    length_squared = rx * rx + ry * ry
    if length_squared <= GEOMETRY_EPSILON * GEOMETRY_EPSILON:
        return (0.0,) if _point_on_segment(start, edge_start, edge_finish) else ()
    if abs(qx * ry - qy * rx) > GEOMETRY_EPSILON:
        return ()
    values = []
    for point in (edge_start, edge_finish):
        ratio = ((point.x - start.x) * rx + (point.y - start.y) * ry) / length_squared
        if -GEOMETRY_EPSILON <= ratio <= 1.0 + GEOMETRY_EPSILON:
            values.append(max(0.0, min(1.0, ratio)))
    return tuple(values)


def segment_is_covered_by_polygon_union(
    start: Point2D,
    finish: Point2D,
    polygons: Iterable[Sequence[Point2D]],
) -> bool:
    """Check an entire segment, not only its endpoints, against a polygon union."""

    polygon_tuple = tuple(polygons)
    if not point_in_polygon_union(start, polygon_tuple):
        return False
    if not point_in_polygon_union(finish, polygon_tuple):
        return False
    parameters = [0.0, 1.0]
    for vertices in polygon_tuple:
        for edge_start, edge_finish in polygon_edges(vertices):
            parameters.extend(
                _intersection_parameters(start, finish, edge_start, edge_finish)
            )
    parameters.sort()
    unique = []
    for value in parameters:
        if not unique or abs(value - unique[-1]) > GEOMETRY_EPSILON:
            unique.append(value)
    for first, second in zip(unique, unique[1:]):
        if second - first <= GEOMETRY_EPSILON:
            continue
        ratio = 0.5 * (first + second)
        midpoint = Point2D(
            start.x + ratio * (finish.x - start.x),
            start.y + ratio * (finish.y - start.y),
        )
        if not point_in_polygon_union(midpoint, polygon_tuple):
            return False
    return True


def propagate_bicycle(
    start: Pose2D, steering_rad: float, distance_m: float, wheelbase_m: float
) -> Pose2D:
    curvature = math.tan(steering_rad) / wheelbase_m
    if abs(curvature) <= GEOMETRY_EPSILON:
        return Pose2D(
            start.x + distance_m * math.cos(start.yaw),
            start.y + distance_m * math.sin(start.yaw),
            normalize_yaw(start.yaw),
        )
    finish_yaw = start.yaw + curvature * distance_m
    return Pose2D(
        start.x + (math.sin(finish_yaw) - math.sin(start.yaw)) / curvature,
        start.y + (math.cos(start.yaw) - math.cos(finish_yaw)) / curvature,
        normalize_yaw(finish_yaw),
    )
