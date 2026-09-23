"""Read-only HD-map geometry for display, using the central map projection."""
import math
from .coordinates import mgeo_local_to_sim_local
from .geometry import simplify_rdp
from .mgeo_v3 import MGeoV3Dataset


def display_layers(dataset, projection, tolerance=0.2):
    if projection['epsg'] != 32652 or projection['map_frame'] != 'map':
        raise ValueError('HD map display requires the central UTM 52N map projection')
    crs = dataset.global_info['global_coordinate_system'].split()
    if not {'+proj=utm', '+zone=52', '+datum=WGS84', '+units=m'}.issubset(crs) or '+south' in crs:
        raise ValueError('HD map CRS differs from localization projection')
    if not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError('Map display simplification must be finite and nonnegative')
    origin = dataset.global_info['local_origin_in_global']
    layers = {}
    for name, records in [('lane_boundaries', dataset.lane_boundaries), ('centerlines', dataset.links)]:
        lines = []
        for record in records.values():
            points = [mgeo_local_to_sim_local(p, origin, projection['origin_utm_m'])
                      for p in record['points']]
            if len(points) >= 2:
                lines.append(simplify_rdp(points, tolerance))
        layers[name] = lines
    return layers


def load_display_layers(source, projection, tolerance=0.2):
    return display_layers(MGeoV3Dataset(source), projection, tolerance)


def load_route_display_layers(source, projection, config, reference_path):
    """Use exactly the existing HTML preview's route crop and northern anchors."""
    from .coordinates import CoordinateTransformer
    from .viewer import build_viewer_data, load_global_route
    if not load_global_route(reference_path)['p']:
        raise ValueError('Route-cropped HD map requires a nonempty reference route')
    dataset = MGeoV3Dataset(source)
    # Enforce the same CRS checks as the full-map display.
    display_layers(dataset, projection, config['conversion']['viewer_simplification_m'])
    transform = CoordinateTransformer(dataset.global_info['local_origin_in_global'],
                                      projection['origin_utm_m'])
    preview = build_viewer_data(dataset, transform, config, reference_path=reference_path)
    layers = {
        'lane_boundaries': [item['p'] for item in preview['boundaries']],
        'centerlines': [item['p'] for item in preview['centerlines']],
        'global_route': [preview['globalRoute']['p']],
        'static_walls': [item['p'] for item in preview['staticWalls']],
    }
    if 'lane_rddf' in config:
        from .lane_rddf import build_lane_rddf, read_route
        alternatives = (preview['laneRddf'] if config.get('course_speed_policy') else
                        build_lane_rddf(dataset, transform, read_route(reference_path), config['lane_rddf']))
        layers['lane_rddf'] = [lane['points'] for lane in alternatives['lanes']]
        # Crossbars indicate allowed boundary crossing locations, not a steering trajectory.
        layers['lane_change_windows'] = [c['points'] for c in alternatives['crossings']
                                        if c['source'][1] % 20 == 0]
        preview['metadata']['lane_rddf'] = alternatives['counts']
    if config.get('course_speed_policy'):
        sections = preview['speedSections']
        layers['global_route'] = [s['p'] for s in sections if s['speed_limit_kph'] is not None]
        layers['global_route_unlimited'] = [s['p'] for s in sections if s['speed_limit_kph'] is None]
        if 'lane_rddf' in config:
            lane_sections = [s for lane in alternatives['lanes'] for s in lane['speed_sections']]
            layers['lane_rddf'] = [s['p'] for s in lane_sections if s['speed_limit_kph'] is not None]
            layers['lane_rddf_unlimited'] = [s['p'] for s in lane_sections if s['speed_limit_kph'] is None]
    return layers, preview['metadata']
