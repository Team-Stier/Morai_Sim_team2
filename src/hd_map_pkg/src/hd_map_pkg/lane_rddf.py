"""Conservative static lane alternatives; XYZ polylines, never control trajectories."""
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path

from .geometry import (cumulative_lengths, point_at_progress, progress_along_polyline,
                       distance_2d, segments_intersect_2d)


class SegmentIndex:
    """Exact nearest segment within a bounded search radius, using a spatial grid."""
    def __init__(self, lines, cell=10.0):
        self.cell = cell
        self.grid = defaultdict(list)
        for key, points in lines.items():
            lengths = cumulative_lengths(points)
            for i, (a, b) in enumerate(zip(points, points[1:])):
                length = distance_2d(a, b)
                if length < 1e-9:
                    continue
                record = (key, i, a, b, lengths[i], length)
                for x in range(math.floor(min(a[0], b[0])/cell), math.floor(max(a[0], b[0])/cell)+1):
                    for y in range(math.floor(min(a[1], b[1])/cell), math.floor(max(a[1], b[1])/cell)+1):
                        self.grid[x, y].append(record)

    def nearest(self, p, radius):
        best = None
        seen = set()
        for x in range(math.floor((p[0]-radius)/self.cell), math.floor((p[0]+radius)/self.cell)+1):
            for y in range(math.floor((p[1]-radius)/self.cell), math.floor((p[1]+radius)/self.cell)+1):
                for key, i, a, b, start, length in self.grid[x, y]:
                    if (key, i) in seen:
                        continue
                    seen.add((key, i))
                    s, d2 = progress_along_polyline(p, [a, b])
                    if d2 <= radius*radius and (best is None or d2 < best[0]):
                        best = (d2, start+s, ((b[0]-a[0])/length, (b[1]-a[1])/length), key, i)
        return best


def read_route(path):
    points = [list(map(float, line.split())) for line in Path(path).read_text().splitlines() if line.strip()]
    if len(points) < 2 or any(len(p) != 3 or not all(map(math.isfinite, p)) for p in points):
        raise ValueError('RDDF reference requires finite XYZ rows')
    return points


def _walk(seeds, edges):
    found = set(seeds)
    pending = list(seeds)
    while pending:
        for other in edges.get(pending.pop(), ()):
            if other not in found:
                found.add(other)
                pending.append(other)
    return found


def build_lane_rddf(dataset, transform, route, settings):
    cfg = settings
    excluded = cfg.get('excluded_link_ids', [])
    if not isinstance(excluded, list) or any(not isinstance(key, str) for key in excluded):
        raise ValueError('excluded_link_ids must be a list of MGeo link IDs')
    unknown = set(excluded) - set(dataset.links)
    if unknown:
        raise ValueError('Unknown excluded link IDs: '+', '.join(sorted(unknown)))
    for key, value in cfg.items():
        if key in ('excluded_link_ids', 'allowed_link_ids', 'course_connections'):
            continue
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Invalid lane RDDF setting: '+key)
    if cfg['minimum_heading_dot'] > 1:
        raise ValueError('minimum_heading_dot must not exceed one')
    step = cfg['sample_spacing_m']
    radius = cfg['course_corridor_m']
    route_index = SegmentIndex({'route': route})
    lines = {key: [transform.mgeo_to_sim(p) for p in link['points']]
             for key, link in dataset.links.items()
             if not link.get('opp_traffic') and key not in excluded}
    samples, distances, headings, route_progress = {}, {}, {}, {}
    totals = {key: cumulative_lengths(line)[-1] for key, line in lines.items()}
    bounds = (min(p[0] for p in route)-radius, max(p[0] for p in route)+radius,
              min(p[1] for p in route)-radius, max(p[1] for p in route)+radius)
    for key, line in lines.items():
        if (max(p[0] for p in line) < bounds[0] or min(p[0] for p in line) > bounds[1]
                or max(p[1] for p in line) < bounds[2] or min(p[1] for p in line) > bounds[3]):
            continue
        lengths = cumulative_lengths(line)
        if lengths[-1] < step:
            continue
        n = math.ceil(lengths[-1]/step)
        points = [point_at_progress(line, lengths[-1]*i/n, lengths) for i in range(n+1)]
        for i, p in enumerate(points):
            hit = route_index.nearest(p, radius)
            a, b = points[max(0, i-1)], points[min(n, i+1)]
            length = distance_2d(a, b)
            if hit is None or length < 1e-9:
                continue
            ra, rb = route[hit[4]], route[hit[4]+1]
            along, _ = progress_along_polyline(p, [ra, rb])
            route_z = ra[2] + (rb[2]-ra[2])*along/distance_2d(ra, rb)
            if abs(p[2]-route_z) > cfg['maximum_height_difference_m']:
                continue
            direction = ((b[0]-a[0])/length, (b[1]-a[1])/length)
            if sum(x*y for x,y in zip(direction, hit[2])) < cfg['minimum_heading_dot']:
                continue
            samples[key, i] = p
            distances[key, i] = math.sqrt(hit[0])
            headings[key, i] = direction
            route_progress[key, i] = hit[1]
    edges, reverse = defaultdict(set), defaultdict(set)
    def connect(a, b):
        if a in samples and b in samples:
            edges[a].add(b)
            reverse[b].add(a)
    by_link = defaultdict(list)
    for key, i in samples:
        by_link[key].append(i)
        connect((key, i), (key, i+1))
    for key, indices in by_link.items():
        end = max(indices)
        if distance_2d(samples[key, end], lines[key][-1]) > 1e-6:
            continue
        for other in dataset.successors.get(key, []):
            if (other, 0) in samples and distance_2d(samples[key, end], samples[other, 0]) <= cfg['successor_gap_m']:
                connect((key, end), (other, 0))
    # Keep longitudinal source topology separate from permitted lateral edges.
    longitudinal = {node: set(targets) for node, targets in edges.items()}
    boundaries = {key: [transform.mgeo_to_sim(p) for p in b['points']]
                  for key, b in dataset.lane_boundaries.items()}
    boundary_lengths = {key: cumulative_lengths(line)[-1] for key, line in boundaries.items()}
    changes = []
    for key, indices in sorted(by_link.items()):
        link = dataset.links[key]
        for side, opposite in [('left', 'right'), ('right', 'left')]:
            other = link.get(side+'_lane_change_dst_link_idx')
            if link.get('can_move_'+side+'_lane') is not True or other not in by_link:
                continue
            target = dataset.links[other]
            shared = set(link.get('lane_mark_'+side) or []) & set(target.get('lane_mark_'+opposite) or [])
            allowed = []
            for bid in sorted(shared):
                b = dataset.lane_boundaries[bid]
                if (b.get('lane_type') == [503] and b.get('lane_shape') == ['broken']
                        and b.get('lane_color') == ['white'] and not b.get('pass_restr')):
                    allowed.append(bid)
            if not allowed:
                continue
            target_points = {i: samples[other, i] for i in by_link[other]}
            target_index = SegmentIndex({other: lines[other]})
            for i in indices:
                p = samples[key, i]
                hit = target_index.nearest(p, cfg['maximum_lane_width_m'])
                if hit is None:
                    continue
                total = totals[other]
                j = round(hit[1]/total * math.ceil(total/step))
                if j not in target_points:
                    continue
                q = target_points[j]
                if abs(p[2]-q[2]) > cfg['maximum_height_difference_m']:
                    continue
                if sum(a*b for a,b in zip(headings[key,i], headings[other,j])) < cfg['minimum_heading_dot']:
                    continue
                lateral = headings[key,i][0]*(q[1]-p[1])-headings[key,i][1]*(q[0]-p[0])
                if (side == 'left' and lateral <= 0) or (side == 'right' and lateral >= 0):
                    continue
                crossed = []
                for bid in set(link.get('lane_mark_'+side) or []) | set(target.get('lane_mark_'+opposite) or []):
                    if bid in boundaries and any(segments_intersect_2d(p,q,a,b) for a,b in zip(boundaries[bid],boundaries[bid][1:])):
                        crossed.append(bid)
                if len(crossed) != 1 or crossed[0] not in allowed:
                    continue
                bid = crossed[0]
                boundary = boundaries[bid]
                s, _ = progress_along_polyline(p, boundary)
                if min(s, boundary_lengths[bid]-s) < cfg['boundary_end_clearance_m']:
                    continue
                connect((key,i),(other,j))
                changes.append({'source': [key,i], 'target': [other,j], 'boundary': bid,
                                'side': side, 'points': [p,q]})
    seeds = {node for node, distance in distances.items() if distance <= cfg['route_match_m']}
    retained = _walk(seeds, edges) & _walk(seeds, reverse)
    lanes = []
    for key, indices in sorted(by_link.items()):
        run = []
        def finish():
            if len(run) >= 2 and distance_2d(samples[key,run[0]], samples[key,run[-1]]) >= cfg['minimum_run_m']:
                lanes.append({'id': '{}_{}'.format(key,run[0]), 'link_id': key,
                              'points': [samples[key,i] for i in run],
                              'source_indices': list(run),
                              'route_s': [route_progress[key,i] for i in run],
                              'source_successors': list(dataset.successors.get(key, []))})
        for i in indices:
            if (key,i) in retained and distances[key,i] > cfg['route_exclusion_m']:
                if run and i != run[-1]+1:
                    finish()
                    run = []
                run.append(i)
            else:
                finish()
                run = []
        finish()
    changes = [c for c in changes if tuple(c['source']) in retained and tuple(c['target']) in retained]
    if 'allowed_link_ids' in cfg:
        lanes = [lane for lane in lanes if lane['link_id'] in cfg['allowed_link_ids']]
    graph = _lane_graph(lanes, changes, longitudinal, retained, distances,
                        route_progress, boundaries, cfg['route_exclusion_m'])
    forbidden_ids = [key for key, boundary in sorted(dataset.lane_boundaries.items())
                     if len(boundaries[key]) > 1 and forbidden_lateral_boundary(boundary)]
    result = {'format': 'hd_map_pkg.lane_rddf.v2', 'frame': 'map', 'units': 'm',
            'purpose': 'static_lane_alternatives_and_crossing_windows_not_driving_trajectories',
            'settings': dict(cfg), 'lanes': lanes, 'crossings': changes,
            'graph': graph,
            'forbidden_boundary_ids': forbidden_ids,
            'forbidden_boundaries': [boundaries[key] for key in forbidden_ids],
            'counts': {'lanes': len(lanes), 'crossing_samples': len(changes),
                       'route_seed_samples': len(seeds), 'retained_samples': len(retained),
                       'lane_change_windows': len(graph['lane_changes'])}}
    from .rddf_connections import connect_course_lanes
    return connect_course_lanes(result, dataset, lines, route, route_index, cfg)


def forbidden_lateral_boundary(boundary):
    """Stop lines/bike symbols are separate rules; unknown lateral lines are not crossed."""
    if boundary.get('lane_type') in ([530], [535]):
        return False
    # Match the existing Lanelet conversion's ordinary dashed-line categories.
    # In particular, junction guide line 525 is not a solid lateral boundary.
    return not (boundary.get('lane_type') in ([503], [504], [506], [515], [525])
                and boundary.get('lane_shape') == ['broken']
                and boundary.get('lane_color') == ['white']
                and not boundary.get('pass_restr'))


def _lane_graph(lanes, crossings, longitudinal, retained, distances,
                route_progress, boundaries, route_exclusion):
    """Project retained source samples to exported lane IDs without proximity links."""
    owners = {node: ('global_route', route_progress[node]) for node in retained
              if distances[node] <= route_exclusion}
    for lane in lanes:
        lengths = cumulative_lengths(lane['points'])
        owners.update({(lane['link_id'], index): (lane['id'], s)
                       for index, s in zip(lane['source_indices'], lengths)})
        lane['successors'] = []
    by_id = {lane['id']: lane for lane in lanes}
    connections = []
    for source, targets in sorted(longitudinal.items()):
        if source not in owners:
            continue
        for target in sorted(targets):
            if target not in owners or owners[source][0] == owners[target][0]:
                continue
            source_id, source_s = owners[source]
            target_id, target_s = owners[target]
            connections.append(dict(source_lane=source_id, target_lane=target_id,
                                    source_s=source_s, target_s=target_s,
                                    source_sample=list(source), target_sample=list(target)))
            if source_id in by_id and target_id not in by_id[source_id]['successors']:
                by_id[source_id]['successors'].append(target_id)
    grouped = defaultdict(list)
    for crossing in crossings:
        source, target = tuple(crossing['source']), tuple(crossing['target'])
        if source not in owners or target not in owners:
            continue
        source_id, source_s = owners[source]
        target_id, target_s = owners[target]
        if source_id != target_id:
            grouped[source_id, target_id, crossing['boundary'], crossing['side']].append(
                (source, target, source_s, target_s))
    windows = []
    for key, values in sorted(grouped.items()):
        run = []

        def finish():
            if len(run) < 2:
                return
            source_id, target_id, boundary, side = key
            windows.append(dict(source_lane=source_id, target_lane=target_id,
                                boundary_id=boundary, side=side,
                                source_s_start=run[0][2], source_s_end=run[-1][2],
                                target_s_start=run[0][3], target_s_end=run[-1][3],
                                boundary=boundaries[boundary],
                                source_samples=[list(row[0]) for row in run],
                                target_samples=[list(row[1]) for row in run]))

        for row in sorted(values):
            if run and (row[0][0] != run[-1][0][0] or row[0][1] != run[-1][0][1]+1
                        or row[1][0] != run[-1][1][0] or not 0 <= row[1][1]-run[-1][1][1] <= 2
                        or row[2] <= run[-1][2] or row[3] < run[-1][3]):
                finish()
                run = []
            run.append(row)
        finish()
    return dict(reference_lane_id='global_route',
                longitudinal_connections=connections, lane_changes=windows)


def write_lane_rddf(result, directory, reference_path, dataset):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    result = dict(result)
    result['reference_sha256'] = hashlib.sha256(Path(reference_path).read_bytes()).hexdigest()
    result['source_hashes'] = dataset.source_hashes()
    for lane in result['lanes']:
        name = lane['id']+'.txt'
        (directory/name).write_text(''.join('{:.6f} {:.6f} {:.6f}\n'.format(*p) for p in lane['points']))
        lane['file'] = name
    path = directory/'manifest.json'
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return path
