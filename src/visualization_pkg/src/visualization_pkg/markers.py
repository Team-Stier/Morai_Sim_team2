"""Convert a checked display state into RViz markers without publishing TF."""

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def _stamp(value):
    return rospy.Time(value // 1000000000, value % 1000000000)


def _marker(state, marker_id, marker_type, stamp, lifetime):
    marker = Marker()
    marker.header.frame_id = state.frame_id
    marker.header.stamp = _stamp(stamp)
    marker.ns = 'vehicle'
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.lifetime = rospy.Duration.from_sec(lifetime)
    marker.frame_locked = False
    return marker


def _assign_pose(marker, pose):
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pose.position
    (marker.pose.orientation.x, marker.pose.orientation.y,
     marker.pose.orientation.z, marker.pose.orientation.w) = pose.quaternion


def render_markers(state, config, now_ns):
    """Keep geometry stamps unchanged; always explicitly delete invalid geometry."""
    array = MarkerArray()
    if not state.valid:
        for marker_id in (0, 1):
            marker = _marker(state, marker_id, Marker.CUBE, now_ns, 0)
            marker.action = Marker.DELETE
            array.markers.append(marker)
        label = _marker(state, 2, Marker.TEXT_VIEW_FACING, now_ns, 0)
        label.pose.position.z = 2.0
        label.scale.z = 0.45
        label.color.r, label.color.g, label.color.b, label.color.a = (1.0, 0.65, 0.1, 1.0)
        label.text = 'WAITING FOR LOCALIZATION\n' + state.reason
        array.markers.append(label)
        return array

    height = (config.footprint_thickness_m if config.display_mode == 'footprint'
              else config.vehicle_height_m)
    body = _marker(state, 0, Marker.CUBE, state.stamp_ns, config.display_timeout_sec)
    _assign_pose(body, state.pose)
    body.scale.x, body.scale.y, body.scale.z = (
        config.vehicle_length_m, config.vehicle_width_m, height)
    amber = state.stop_required or state.frame_id == 'odom'
    color = (1.0, 0.55, 0.1, 0.6) if amber else (0.1, 0.55, 1.0, 0.6)
    body.color.r, body.color.g, body.color.b, body.color.a = color
    array.markers.append(body)

    arrow = _marker(state, 1, Marker.ARROW, state.stamp_ns, config.display_timeout_sec)
    _assign_pose(arrow, state.pose)
    z = height / 2 + 0.15
    arrow.points = [Point(0.0, 0.0, z), Point(config.vehicle_length_m / 2 + 0.7, 0.0, z)]
    arrow.scale.x, arrow.scale.y, arrow.scale.z = (0.12, 0.28, 0.45)
    arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (1.0, 0.85, 0.15, 1.0)
    array.markers.append(arrow)

    label = _marker(state, 2, Marker.TEXT_VIEW_FACING, state.stamp_ns, config.display_timeout_sec)
    _assign_pose(label, state.pose)
    # Keep the caption clear of the footprint in the default top-down view.
    label.pose.position.y -= max(config.vehicle_length_m, config.vehicle_width_m) / 2 + 1.0
    label.pose.position.z += height / 2 + 0.7
    label.scale.z = 0.4
    label.color.r, label.color.g, label.color.b, label.color.a = (1.0, 1.0, 1.0, 1.0)
    parts = ['MAP ESTIMATE' if state.frame_id == 'map' else 'LOCAL ODOMETRY']
    if state.stop_required:
        parts.append('STOP REQUIRED')
    if state.projected:
        parts.append('PLANAR PROJECTION')
    if not config.body_center_offset_verified:
        parts.append('MODEL OFFSET PROVISIONAL')
    label.text = '\n'.join(parts)
    array.markers.append(label)
    return array
