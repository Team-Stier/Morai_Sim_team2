"""PIDNet foreground selection and class-separated voxel connectivity.

Fusion and grouping belong to World Model. No raw-only fallback is used.
"""
from itertools import product
import numpy as np
from ros_numpy.point_cloud2 import array_to_pointcloud2
from .pointpainting import cloud_records


def foreground_mask(records, policy):
    required = ['x', 'y', 'z', 'painted', 'class_id', 'confidence'] + ['score_' + str(i) for i in range(6)]
    if any(name not in records.dtype.names for name in required):
        raise ValueError('missing PIDNet point fields')
    xyz = np.column_stack([records[n] for n in ('x', 'y', 'z')])
    scores = np.column_stack([records['score_' + str(i)] for i in range(6)])
    threshold = float(policy['minimum_probability'])
    if not np.isfinite(threshold) or not 0 <= threshold <= 1:
        raise ValueError('invalid foreground probability threshold')
    observed = records['painted'] == 1
    valid_scores = (np.isfinite(scores).all(1) & (scores >= 0).all(1) & (scores <= 1).all(1)
                    & np.isclose(scores.sum(1), 1, atol=1e-3))
    if np.any(observed & ~valid_scores):
        raise ValueError('invalid observed PIDNet probabilities')
    labels = scores.argmax(1)
    confidence = scores.max(1)
    if np.any(observed & ((records['class_id'] != labels)
                         | ~np.isclose(records['confidence'], confidence, atol=1e-5))):
        raise ValueError('PIDNet label/confidence disagrees with probabilities')
    return (observed & np.isfinite(xyz).all(1) & valid_scores
            & np.isin(labels, policy['classes']) & (confidence >= threshold))


def foreground_cloud(cloud, policy, geometry_mask=None):
    records = cloud_records(cloud)
    mask = foreground_mask(records, policy)
    if geometry_mask is not None:
        if geometry_mask.shape != mask.shape:
            raise ValueError("surface selection size does not match cloud")
        mask &= geometry_mask
    output = array_to_pointcloud2(records[mask])
    output.header = cloud.header
    return output


def semantic_groups(records, policy, config):
    selected = records[foreground_mask(records, policy)]
    if len(selected) > int(config['maximum_points']):
        raise ValueError('PIDNet foreground exceeds configured point budget')
    cell_size = float(config['cell_size_m'])
    minimum = int(config['minimum_points'])
    if not np.isfinite(cell_size) or cell_size <= 0 or minimum < 1:
        raise ValueError('invalid semantic grouping configuration')
    xyz = np.column_stack([selected[n] for n in ('x', 'y', 'z')])
    # Group each semantic class separately. Preserve every original return in accepted groups.
    cells = {}
    for index, cell in enumerate(np.floor(xyz / cell_size).astype(np.int64)):
        key = (int(selected['class_id'][index]),) + tuple(int(v) for v in cell)
        cells.setdefault(key, []).append(index)
    groups = []
    offsets = tuple(product((-1, 0, 1), repeat=3))
    while cells:
        seed = next(iter(cells))
        pending = [seed]
        indices = cells.pop(seed)
        label = seed[0]
        while pending:
            current = pending.pop()
            for offset in offsets:
                key = (label,) + tuple(current[i + 1] + offset[i] for i in range(3))
                if key in cells:
                    indices.extend(cells.pop(key))
                    pending.append(key)
        if len(indices) >= minimum:
            groups.append((label, xyz[sorted(indices)]))
    return groups
