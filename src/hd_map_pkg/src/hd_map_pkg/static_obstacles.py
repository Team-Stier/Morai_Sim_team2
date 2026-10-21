"""Route-cropped occupied samples from immutable MGeo fixed structures."""
import math

from .lane_rddf import SegmentIndex
from .geometry import cumulative_lengths


SUPPORTED = {'wall', 'guardrail', 'concrete_barrier', 'light_pole', 'sign_pole'}


def build_static_obstacles(dataset, transformer, route, config):
    cfg = config['static_obstacles']
    kinds = set(cfg['kinds'])
    if not kinds or not kinds <= SUPPORTED:
        raise ValueError('Unsupported static obstacle kind')
    for key in ('route_distance_m', 'maximum_height_difference_m', 'sample_spacing_m', 'padding_m'):
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('Invalid static obstacle setting: ' + key)
    step, padding = cfg['sample_spacing_m'], cfg['padding_m']
    index = SegmentIndex({'route': route})
    radius = cfg['route_distance_m']
    route_s = cumulative_lengths(route)
    n = math.ceil(2 * padding / step)
    offsets = [-padding + 2 * padding * i / n for i in range(n + 1)] + [0.0]
    result = []
    for row in sorted(dataset.data.get('object_set', []), key=lambda row: str(row['idx'])):
        if row.get('name') not in kinds:
            continue
        raw = row.get('points', [])
        if not raw or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in raw):
            raise ValueError('Invalid static obstacle geometry: ' + str(row['idx']))
        points = [transformer.mgeo_to_sim(p) for p in raw]
        if row.get('is_closed') and len(points) > 2 and points[0] != points[-1]:
            points.append(points[0])
        samples = [points[0]]
        for a, b in zip(points, points[1:]):
            count = max(1, math.ceil(math.dist(a[:2], b[:2]) / step))
            samples.extend([a[j] + (b[j]-a[j])*i/count for j in range(3)]
                           for i in range(1, count + 1))
        occupied = []
        for p in samples:
            hit = index.nearest(p, radius)
            if hit is None:
                continue
            a, b = route[hit[4]], route[hit[4]+1]
            fraction = (hit[1] - route_s[hit[4]]) / math.dist(a[:2], b[:2])
            z = a[2] + fraction * (b[2]-a[2])
            if abs(p[2]-z) > cfg['maximum_height_difference_m']:
                continue
            occupied.extend([p[0]+dx, p[1]+dy, p[2]] for dx in offsets for dy in offsets)
        if occupied:
            result.append(dict(id=str(row['idx']), kind=row['name'], points=occupied))
    return result
