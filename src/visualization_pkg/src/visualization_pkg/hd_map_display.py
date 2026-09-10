"""Static planar HD-map markers; no pose estimation or TF publication."""
from pathlib import Path
import math
import rospkg
import rospy
import yaml
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from hd_map_pkg.display_geometry import load_route_display_layers


def map_markers(layers, plane_z, line_width):
    if not math.isfinite(plane_z) or not math.isfinite(line_width) or line_width <= 0:
        raise ValueError('Invalid HD map display height or line width')
    result = MarkerArray()
    for index, (layer, lines) in enumerate(layers.items()):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time(0)
        marker.ns = layer
        marker.id = index
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = line_width
        color = (0.80, 0.83, 0.88, 0.95) if layer == 'lane_boundaries' else (0.22, 0.55, 0.65, 0.5)
        if layer == "global_route":
            color = (0.2, 1.0, 0.35, 1.0)
            marker.scale.x = line_width * 2
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        # Flatten display only; authoritative source geometry and localization stay intact.
        for line in lines:
            for a, b in zip(line, line[1:]):
                marker.points.extend([Point(a[0], a[1], plane_z), Point(b[0], b[1], plane_z)])
        result.markers.append(marker)
    return result


def load_map_markers():
    packages = rospkg.RosPack()
    map_root = Path(packages.get_path('hd_map_pkg'))
    architecture = Path(packages.get_path('ros_architecture_pkg'))
    projection = yaml.safe_load((architecture / 'config/tf/map_projection.yaml').read_text())
    config = yaml.safe_load((map_root / 'config/map_conversion.yaml').read_text())
    source = rospy.get_param('~hd_map_source', str(map_root / config['source']['relative_path']))
    reference = map_root / config['references']['simulator_global_path']
    layers, metadata = load_route_display_layers(source, projection, config, reference)
    rospy.loginfo('Route-cropped HD map: %s; bounds=%s', metadata['counts'], metadata['bounds'])
    return map_markers(layers, rospy.get_param('~hd_map_plane_z_m', -0.10),
                       rospy.get_param('~hd_map_line_width_m', 0.10))
