"""Validation of the centrally approved immutable StaticWallMap boundary."""
import math
import re


def validate_static_walls(message):
    if message.header.frame_id != 'map' or message.header.stamp.to_nsec() <= 0:
        raise ValueError('wall map needs map frame and nonzero load stamp')
    if not message.map_id or not re.fullmatch('[0-9a-f]{64}', message.source_sha256):
        raise ValueError('invalid wall map identity')
    ids, lines = message.wall_ids, message.baselines
    if not 2 <= len(ids) <= 32 or len(ids) != len(lines) or len(set(ids)) != len(ids) or any(not i for i in ids):
        raise ValueError('wall IDs and baselines must be nonempty unique parallel arrays')
    if not math.isfinite(message.horizontal_stddev_m) or message.horizontal_stddev_m <= 0:
        raise ValueError('invalid wall uncertainty')
    for line in lines:
        if not 2 <= len(line.points) <= 2048:
            raise ValueError('invalid wall vertex count')
        for p in line.points:
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                raise ValueError('nonfinite wall geometry')
        if any(math.hypot(a.x-b.x, a.y-b.y) < 1e-4 for a,b in zip(line.points, line.points[1:])):
            raise ValueError('degenerate wall segment')
    return message
