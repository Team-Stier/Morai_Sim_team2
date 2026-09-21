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
    for key, value in cfg.items():
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Invalid lane RDDF setting: '+key)
    if cfg['minimum_heading_dot'] > 1:
        raise ValueError('minimum_heading_dot must not exceed one')
    step = cfg['sample_spacing_m']
    radius = cfg['course_corridor_m']
    route_index = SegmentIndex({'route': route})
    lines = {key: [transform.mgeo_to_sim(p) for p in link['points']]
             for key, link in dataset.links.items() if not link.get('opp_traffic')}
    samples, distances, headings = {}, {}, {}
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
                              'points': [samples[key,i] for i in run]})
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
    return {'format': 'hd_map_pkg.lane_rddf.v1', 'frame': 'map', 'units': 'm',
            'purpose': 'static_lane_alternatives_and_crossing_windows_not_driving_trajectories',
            'settings': dict(cfg), 'lanes': lanes, 'crossings': changes,
            'counts': {'lanes': len(lanes), 'crossing_samples': len(changes),
                       'route_seed_samples': len(seeds), 'retained_samples': len(retained)}}


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
