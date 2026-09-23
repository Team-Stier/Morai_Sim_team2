"""Source-backed static wall polylines, independent of vehicle pose and ROS."""
import math


def build_static_walls(dataset, transformer, config):
    requested = config.get('static_walls', {}).get('source_object_ids', [])
    if len(requested) != len(set(requested)):
        raise ValueError('Duplicate static wall source ID')
    if not requested:
        return []
    objects = {str(item['idx']): item for item in dataset.data.get('object_set', [])}
    result = []
    for identifier in requested:
        record = objects.get(identifier)
        if record is None or record.get('name') != 'wall':
            raise ValueError('Missing or non-wall source object: ' + identifier)
        points = record.get('points', [])
        if len(points) < 2 or any(len(p) != 3 or not all(math.isfinite(float(v)) for v in p) for p in points):
            raise ValueError('Invalid static wall geometry: ' + identifier)
        result.append(dict(id=identifier, p=[list(transformer.mgeo_to_sim(p)) for p in points],
                           source='object_set.json', source_type=record.get('type'),
                           source_sub_type=record.get('sub_type'),
                           height_m=None, physical_alignment_verified=False))
    return result
