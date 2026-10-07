"""Static planar HD-map markers; no pose estimation or TF publication."""
from pathlib import Path
import math
import rospkg
import rospy
import yaml
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from hd_map_pkg.display_geometry import load_route_display_layers
from hd_map_pkg.course_speed import load_course_speed_policy


def map_markers(layers, plane_z, line_width, wall_display_height=5.0):
    if not math.isfinite(wall_display_height) or wall_display_height <= 0:
        raise ValueError("Wall display height must be positive and finite")
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
        if layer == 'lane_rddf':
            color = (0.1, 0.75, 1.0, 1.0)
            marker.scale.x = line_width * 2
        elif layer == 'lane_change_windows':
            color = (1.0, 0.65, 0.1, 0.85)
        elif layer in ('global_route_unlimited', 'lane_rddf_unlimited'):
            color = (1.0, 0.25, 0.86, 1.0)
            marker.scale.x = line_width * 3
        elif layer == 'signal_stop_lines':
            color = (1.0, 0.1, 0.1, 1.0)
            marker.scale.x = line_width * 4
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        # Flatten display only; authoritative source geometry and localization stay intact.
        for line in lines:
            for a, b in zip(line, line[1:]):
                marker.points.extend([Point(a[0], a[1], plane_z), Point(b[0], b[1], plane_z)])
        if layer == 'static_walls':
            # Preserve source XYZ. The extrusion is cosmetic, not surveyed height.
            marker.type = Marker.TRIANGLE_LIST
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.65, 0.15, 0.45)
            marker.points = []
            for line in lines:
                for a, b in zip(line, line[1:]):
                    low_a, low_b = Point(*a), Point(*b)
                    high_a = Point(a[0], a[1], a[2]+wall_display_height)
                    high_b = Point(b[0], b[1], b[2]+wall_display_height)
                    marker.points.extend([low_a, low_b, high_b, low_a, high_b, high_a])
        result.markers.append(marker)
    if 'global_route_unlimited' in layers:
        high = layers['global_route_unlimited'][0]
        policy = load_course_speed_policy()
        for index, (point, label, color) in enumerate([
                (high[0], 'NO LIMIT | cruise %g km/h' % policy['high_speed']['cruise_kph'], (1.0, 0.25, 0.86)),
                (high[-1], 'MAX %g km/h' % policy['normal_limit_kph'], (0.2, 1.0, 0.35))]):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns = 'course_speed_labels'
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.pose.position = Point(point[0]+25, point[1], plane_z+0.2)
            marker.scale.z = 2.0
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            marker.text = label
            result.markers.append(marker)
    return result


def stop_line_labels(stops, plane_z, label_height):
    if not math.isfinite(label_height) or label_height <= 0:
        raise ValueError('Invalid stop-line label height')
    labels = []
    for index, row in enumerate(stops):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time(0)
        marker.ns = 'signal_stop_line_labels'
        marker.id = index
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        a, b = row['points'][0], row['points'][-1]
        marker.pose.position = Point((a[0]+b[0])/2, (a[1]+b[1])/2, plane_z+label_height)
        marker.scale.z = label_height
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.1, 0.1, 1.0)
        marker.text = 'STOP %d' % (index+1)
        labels.append(marker)
    return labels


def checkpoint_markers(points, plane_z, diameter, label_height):
    """Display central ENU checkpoints; the final point is START/END."""
    if (not all(math.isfinite(v) for v in (plane_z, diameter, label_height))
            or diameter <= 0 or label_height <= 0):
        raise ValueError('Invalid checkpoint display size or height')
    if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('Checkpoint coordinates must be finite XYZ rows')
    result = MarkerArray()
    dots = Marker()
    dots.header.frame_id = 'map'
    dots.header.stamp = rospy.Time(0)
    dots.ns = 'checkpoints'
    dots.id = 0
    dots.type = Marker.SPHERE_LIST
    dots.action = Marker.ADD
    dots.pose.orientation.w = 1.0
    dots.scale.x = dots.scale.y = dots.scale.z = diameter
    dots.color.r, dots.color.g, dots.color.b, dots.color.a = (1.0, 0.9, 0.1, 1.0)
    for index, point in enumerate(points):
        # The map is planar in RViz; source heights stay untouched.
        dots.points.append(Point(point[0], point[1], plane_z+diameter/2))
        label = Marker()
        label.header = dots.header
        label.ns = 'checkpoint_labels'
        label.id = index
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.orientation.w = 1.0
        label.pose.position = Point(point[0]+diameter, point[1], plane_z+diameter)
        label.scale.z = label_height
        label.color = dots.color
        label.text = 'START / END' if index == len(points)-1 else 'CP %d' % (index+1)
        result.markers.append(label)
    result.markers.insert(0, dots)
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
    plane_z = rospy.get_param('~hd_map_plane_z_m', -0.10)
    result = map_markers(layers, plane_z, rospy.get_param('~hd_map_line_width_m', 0.10),
        rospy.get_param('~hd_map_wall_display_height_m', 5.0))
    result.markers.extend(stop_line_labels(metadata.get('signal_stop_lines', []), plane_z,
        rospy.get_param('~stop_line_label_height_m', 1.5)))
    checkpoints = yaml.safe_load((architecture / 'config/map/checkpoints.yaml').read_text())
    result.markers.extend(checkpoint_markers(checkpoints['points'], plane_z,
        rospy.get_param('~checkpoint_point_diameter_m', 1.5),
        rospy.get_param('~checkpoint_label_height_m', 1.5)).markers)
    return result
