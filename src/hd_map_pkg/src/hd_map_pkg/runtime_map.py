"""Static course graph preparation shared by the ROS map server and tests."""
import hashlib
import json
from pathlib import Path

import yaml

from .coordinates import CoordinateTransformer
from .course_speed import CourseSpeedZones
from .geometry import cumulative_lengths
from .lane_rddf import build_lane_rddf, read_route
from .mgeo_v3 import MGeoV3Dataset
from .validation import validate_source


def build_static_map(raw_route, rddf, checkpoints, policy, reference_sha256):
    """Keep the official route and explicit MGeo edges as distinct graph lanes."""
    indices = [i for i, point in enumerate(raw_route)
               if i == 0 or point != raw_route[i-1]]
    route = [raw_route[i] for i in indices]
    zones = CourseSpeedZones(raw_route, policy)
    reference = dict(id='global_route', link_id='', points=route,
                     source_indices=indices, route_s=cumulative_lengths(route),
                     successors=['global_route'],
                     speed_limits_mps=[-1.0 if zones.unlimited(i)
                                       else zones.limit_kph(i)/3.6 for i in indices])
    # Only an explicit retained longitudinal edge may create a route fork.
    for edge in rddf['graph']['longitudinal_connections']:
        if edge['source_lane'] == 'global_route' and edge['target_lane'] not in reference['successors']:
            reference['successors'].append(edge['target_lane'])
    lanes = [reference]
    for lane in rddf['lanes']:
        lane = dict(lane)
        lane['speed_limits_mps'] = [
            -1.0 if zones.unlimited(zones.nearest_index(point))
            else zones.limit_kph(zones.nearest_index(point))/3.6
            for point in lane['points']]
        lanes.append(lane)
    result = dict(frame='map', reference_sha256=reference_sha256,
                  lanes=lanes, lane_changes=rddf['graph']['lane_changes'],
                  checkpoints=checkpoints['points'], checkpoint_radius_m=checkpoints['radius_m'],
                  checkpoint_source=checkpoints['source'],
                  forbidden_boundaries=rddf['forbidden_boundaries'],
                  forbidden_boundary_ids=rddf['forbidden_boundary_ids'],
                  source_hashes=rddf['source_hashes'], course_speed_policy=policy)
    result['map_id'] = 'katri-rddf-' + hashlib.sha256(
        json.dumps(result, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return result


def load_static_map(config_file, source, reference_path, checkpoints_file, policy_file):
    config = yaml.safe_load(Path(config_file).read_text())
    dataset = MGeoV3Dataset(source,
                           expected_major=config['source']['expected_mgeo_major'],
                           deduplicate_verified_suffix_clones=True)
    failed = [check for check in validate_source(dataset, config) if check['status'] == 'fail']
    if failed:
        raise ValueError('; '.join(check['name'] + ': ' + check['summary'] for check in failed))
    policy = yaml.safe_load(Path(policy_file).read_text())
    digest = hashlib.sha256(Path(reference_path).read_bytes()).hexdigest()
    if digest != policy['reference']['sha256']:
        raise ValueError('global route differs from the centrally approved reference SHA-256')
    coordinates = config['coordinates']
    transform = CoordinateTransformer(dataset.local_origin_utm,
                                      coordinates['simulator_scene_origin_utm'],
                                      coordinates['utm_zone'], coordinates['northern_hemisphere'])
    route = read_route(reference_path)
    rddf = build_lane_rddf(dataset, transform, route, config['lane_rddf'])
    rddf['source_hashes'] = dataset.source_hashes()
    checkpoints = yaml.safe_load(Path(checkpoints_file).read_text())
    return build_static_map(route, rddf, checkpoints, policy, digest)
