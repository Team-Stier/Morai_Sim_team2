"""Static signal stop-line associations shared by export, ROS map and display."""
import math
from .geometry import segment_distance_2d, distance_2d, simplify_rdp
from .lane_rddf import SegmentIndex


def build_signal_stop_lines(dataset, transform, route, config):
    """Keep same-direction course approaches, not nearby opposing/pedestrian signals."""
    from .lanelet2_export import boundary_tags
    settings = config['conversion']
    radius = settings['stop_line_search_radius_m']
    tolerance = settings['stop_line_intersection_tolerance_m']
    boundaries = {key: simplify_rdp(item['points'], settings['geometry_simplification_m'])
                  for key, item in dataset.lane_boundaries.items()
                  if boundary_tags(item, config['lane_boundary'])['type'] == 'stop_line'
                  and len(item['points']) >= 2}
    boundaries = {key: points for key, points in boundaries.items()
                  if any(distance_2d(a, b) > 0 for a, b in zip(points, points[1:]))}
    index = SegmentIndex({'route': route})
    lane_settings = config['lane_rddf']
    allowed = set(lane_settings['allowed_link_ids']) - set(lane_settings['excluded_link_ids'])
    excluded = set(lane_settings['excluded_link_ids'])
    cache, records, course_approaches = {}, {}, {}

    def aligned(a, b):
        mid = transform.mgeo_to_sim([(x+y)/2 for x, y in zip(a, b)])
        hit = index.nearest(mid, lane_settings['route_match_m'])
        length = distance_2d(a, b)
        return hit and length > 0 and sum((y-x)*t/length for x, y, t in
            zip(a[:2], b[:2], hit[2])) >= lane_settings['minimum_heading_dot']

    for signal, links in sorted(dataset.traffic_light_link_ids().items()):
        if str(dataset.traffic_lights[signal].get('type', '')).lower() not in ('car', 'bus'):
            continue
        for link in links:
            if link in excluded:
                continue
            points = dataset.links[link]['points']
            if link not in course_approaches:
                course_approaches[link] = link in allowed or any(
                    aligned(a, b) for a, b in zip(points, points[1:]))
            if not course_approaches[link]:
                continue
            for stop_id, _, _ in match_stop_lines([link], dataset.links, boundaries,
                                                  radius, tolerance, cache):
                geometry = boundaries[stop_id]
                on_course = link in allowed
                for a, b in zip(points, points[1:]):
                    if min(segment_distance_2d(a, b, x, y)
                           for x, y in zip(geometry, geometry[1:])) > tolerance:
                        continue
                    if aligned(a, b):
                        on_course = True
                        break
                if not on_course:
                    continue
                row = records.setdefault(stop_id, dict(id=stop_id,
                    points=[list(transform.mgeo_to_sim(p)) for p in dataset.lane_boundaries[stop_id]['points']],
                    approach_link_ids=set(), signal_ids=set()))
                row['approach_link_ids'].add(link)
                row['signal_ids'].add(signal)
    for row in records.values():
        row['approach_link_ids'] = sorted(row['approach_link_ids'])
        row['signal_ids'] = sorted(row['signal_ids'])
    return [records[key] for key in sorted(records)]


def match_stop_lines(link_ids, links, boundaries, radius, tolerance, cache):
    """Match stop bars crossing the downstream portion of each approach."""
    matches = {}
    for link_id in link_ids:
        if link_id in cache:
            cached = cache[link_id]
            if cached is not None:
                boundary_id, lateral_distance, upstream_distance = cached
                existing = matches.get(boundary_id)
                value = (lateral_distance, upstream_distance)
                if existing is None or value < existing:
                    matches[boundary_id] = value
            continue
        link = links.get(link_id)
        points = link.get("points") if link else None
        if not points or len(points) < 2:
            continue
        downstream_segments = []
        distance_from_end = 0.0
        for upstream, downstream in reversed(list(zip(points, points[1:]))):
            segment_length = math.hypot(
                downstream[0] - upstream[0], downstream[1] - upstream[1])
            if distance_from_end > radius:
                break
            downstream_segments.append((upstream, downstream, distance_from_end))
            distance_from_end += segment_length
        best = None
        for boundary_id in sorted(boundaries):
            geometry = boundaries[boundary_id]
            closest = min(
                (min(segment_distance_2d(upstream, downstream, start, end)
                     for start, end in zip(geometry, geometry[1:])), progress)
                for upstream, downstream, progress in downstream_segments)
            if closest[0] <= tolerance:
                score = (closest[1], closest[0], boundary_id)
                if best is None or score < best[0]:
                    best = (score, boundary_id, closest[0], closest[1])
        if best is not None:
            _, boundary_id, lateral_distance, upstream_distance = best
            cache[link_id] = (
                boundary_id, lateral_distance, upstream_distance)
            existing = matches.get(boundary_id)
            value = (lateral_distance, upstream_distance)
            if existing is None or value < existing:
                matches[boundary_id] = value
        else:
            cache[link_id] = None
    return [(boundary_id, values[0], values[1])
            for boundary_id, values in sorted(matches.items())]
