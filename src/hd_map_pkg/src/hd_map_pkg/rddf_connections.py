"""Explicit derived course alternatives, preserving the immutable MGeo source."""
from bisect import bisect_right
import math

from .geometry import cumulative_lengths, distance_2d


def interpolate(stations, values, station):
    i = max(0, min(len(stations)-2, bisect_right(stations, station)-1))
    t = max(0.0, min(1.0, (station-stations[i])/(stations[i+1]-stations[i])))
    return [a+(b-a)*t for a, b in zip(values[i], values[i+1])]


def smooth_connection(route, source, route_index, spacing, entry, exit_length, radius,
                      start=None, end=None):
    """Join source and reference with quintics matching position and tangent."""
    source_s, source_points = [], []
    for point in source:
        hit = route_index.nearest(point, radius)
        if hit is None:
            raise ValueError('Configured RDDF chain leaves the course corridor')
        station = hit[1]
        if source_s and station < source_s[-1]-0.05:
            raise ValueError('Configured RDDF chain reverses route progress')
        if not source_s or station > source_s[-1]+1e-6:
            source_s.append(station)
            source_points.append(point)
    start = source_s[0] if start is None else start
    end = source_s[-1] if end is None else end
    if end-start <= entry+exit_length:
        raise ValueError('RDDF chain is too short for its entry and exit blends')
    # Drop duplicate reference rows only in this interpolation view.
    reference_s, reference = [], []
    for s, p in zip(cumulative_lengths(route), route):
        if not reference_s or s > reference_s[-1]+1e-9:
            reference_s.append(s)
            reference.append(p)

    def tangent(stations, points, s):
        i = max(0, min(len(stations)-2, bisect_right(stations, s)-1))
        return [(b-a)/(stations[i+1]-stations[i])
                for a, b in zip(points[i], points[i+1])]

    def coefficients(a, b, va, vb, length):
        # Zero second derivatives at both ends; interior source is piecewise linear.
        result = []
        for x, y, u, v in zip(a, b, va, vb):
            u, v, delta = u*length, v*length, y-x
            result.append((x, u, 10*delta-6*u-4*v,
                           -15*delta+8*u+7*v, 6*delta-3*u-3*v))
        return result

    entry_curve = coefficients(
        interpolate(reference_s, reference, start),
        interpolate(source_s, source_points, start+entry),
        tangent(reference_s, reference, start),
        tangent(source_s, source_points, start+entry), entry)
    exit_curve = coefficients(
        interpolate(source_s, source_points, end-exit_length),
        interpolate(reference_s, reference, end),
        tangent(source_s, source_points, end-exit_length),
        tangent(reference_s, reference, end), exit_length)

    def point(s):
        if s <= start or s >= end:
            return interpolate(reference_s, reference, s)
        if s < start+entry:
            curve, t = entry_curve, (s-start)/entry
        elif s > end-exit_length:
            curve, t = exit_curve, (s-end+exit_length)/exit_length
        else:
            return interpolate(source_s, source_points, s)
        return [a+b*t+c*t**3+d*t**4+e*t**5 for a, b, c, d, e in curve]

    n = math.ceil((end-start)/spacing)
    stations = [start+(end-start)*i/n for i in range(n+1)]
    stations = sorted(set(stations + [start+entry, end-exit_length]))
    points, result_s = [point(start)], [start]
    for s in stations[1:]:
        p = point(s)
        count = max(1, math.ceil(distance_2d(points[-1], p)/spacing))
        previous_s = result_s[-1]
        for j in range(1, count+1):
            q = previous_s+(s-previous_s)*j/count
            result_s.append(q)
            points.append(point(q))
    return points, result_s


def connect_course_lanes(result, dataset, lines, route, route_index, settings):
    """Replace configured source chains; remap only unchanged legal windows."""
    config = settings.get('course_connections')
    if config is None:
        return result
    entry, exit_length = config['entry_blend_m'], config['exit_blend_m']
    if any(not isinstance(x, (int, float)) or not math.isfinite(x) or x <= 0
           for x in (entry, exit_length)):
        raise ValueError('RDDF blend lengths must be finite and positive')
    old = {lane['id']: lane for lane in result['lanes']}
    start_link = config['start_reference_link']
    end_link = config['end_reference_link']
    if start_link not in lines or end_link not in lines:
        raise ValueError('Unknown RDDF connection reference link')
    start_hit = route_index.nearest(lines[start_link][0], settings['route_match_m'])
    end_hit = route_index.nearest(lines[end_link][-1], settings['route_match_m'])
    if start_hit is None or end_hit is None:
        raise ValueError('RDDF connection anchors must match the global route')
    replacements, used, mapping = [], set(), {}
    for chain in config['chains']:
        ids = chain['link_ids']
        if not ids or len(ids) != len(set(ids)) or used.intersection(ids):
            raise ValueError('RDDF chains must contain distinct source links')
        if any(key not in lines or key not in settings['allowed_link_ids'] for key in ids):
            raise ValueError('RDDF chain contains an unavailable or unapproved link')
        if chain['id'] == 'global_route' or chain['id'] in old or any(
                lane['id'] == chain['id'] for lane in replacements):
            raise ValueError('Duplicate derived RDDF lane ID')
        for a, b in zip(ids, ids[1:]):
            if (b not in dataset.successors.get(a, []) or
                    distance_2d(lines[a][-1], lines[b][0]) > settings['successor_gap_m']):
                raise ValueError('RDDF chain lacks an explicit continuous MGeo successor')
        source = [p for key in ids for p in lines[key]]
        points, stations = smooth_connection(
            route, source, route_index, settings['sample_spacing_m'], entry,
            exit_length, settings['course_corridor_m'], start_hit[1], end_hit[1])
        lane = dict(id=chain['id'], link_id=ids[0], source_link_ids=ids,
                    points=points, route_s=stations,
                    source_indices=list(range(len(points))), successors=['global_route'],
                    geometry_origin='derived_course_connection',
                    unchanged_route_s=[stations[0]+entry, stations[-1]-exit_length])
        replacements.append(lane)
        used.update(ids)
        for key, previous in old.items():
            if previous['link_id'] in ids:
                mapping[key] = lane

    def remap_window(window):
        updated = dict(window)
        for side in ('source', 'target'):
            key = window[side+'_lane']
            if key not in mapping:
                continue
            previous, lane = old[key], mapping[key]
            lengths = cumulative_lengths(previous['points'])
            endpoints = [interpolate(lengths, [[s] for s in previous['route_s']],
                                     window[side+'_s_'+suffix])[0]
                         for suffix in ('start', 'end')]
            low, high = lane['unchanged_route_s']
            if endpoints[0] < low or endpoints[1] > high:
                return None
            updated[side+'_lane'] = lane['id']
            new_lengths = [[s] for s in cumulative_lengths(lane['points'])]
            for suffix, station in zip(('start', 'end'), endpoints):
                updated[side+'_s_'+suffix] = interpolate(lane['route_s'], new_lengths, station)[0]
        return updated

    graph = result['graph']
    graph['lane_changes'] = [updated for w in graph['lane_changes']
                             for updated in [remap_window(w)] if updated is not None]
    graph['longitudinal_connections'] = [edge for edge in graph['longitudinal_connections']
                                        if edge['source_lane'] not in mapping and
                                        edge['target_lane'] not in mapping]
    for lane in replacements:
        length = cumulative_lengths(lane['points'])[-1]
        for a, b, s, t in [('global_route', lane['id'], lane['route_s'][0], 0.0),
                            (lane['id'], 'global_route', length, lane['route_s'][-1])]:
            graph['longitudinal_connections'].append(dict(
                source_lane=a, target_lane=b, source_s=s, target_s=t,
                geometry_origin='derived_course_connection'))
    result['lanes'] = [lane for lane in result['lanes'] if lane['id'] not in mapping]+replacements
    for lane in result['lanes']:
        lane['successors'] = list(dict.fromkeys(mapping[key]['id'] if key in mapping else key
                                                for key in lane['successors']))
    # Draw only crossing samples whose original positions survive unchanged.
    intervals = {key: lane['unchanged_route_s'] for lane in replacements
                 for key in lane['source_link_ids']}
    result['crossings'] = [crossing for crossing in result['crossings']
                           if all(key not in intervals or intervals[key][0] <=
                                  route_index.nearest(p, settings['course_corridor_m'])[1] <=
                                  intervals[key][1]
                                  for (key, _), p in zip(
                                      [crossing['source'], crossing['target']], crossing['points']))]
    result['counts'].update(lanes=len(result['lanes']), crossing_samples=len(result['crossings']),
                            lane_change_windows=len(graph['lane_changes']))
    return result
