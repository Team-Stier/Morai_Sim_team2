"""Read-only scan geometry, resolved by RViz at the original measurement stamp."""
import copy

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from common_msgs_pkg.lidar_validation import validate_lidar


class LidarDisplay:
    def __init__(self, config, tf_buffer, publisher):
        self.config, self.tf_buffer, self.publisher = config, tf_buffer, publisher
        self.pending = None
        self.received = 0.0
        self.last_stamp = None
        self.last_clock = None
        self.clock_changed = 0.0
        self.shown_stamp = None
        self.visible = False

    def clear(self):
        self.pending = None
        self.shown_stamp = None
        if self.visible:
            self.publisher.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        self.visible = False

    def update(self, now, wall):
        if self.last_clock is None or now != self.last_clock:
            if self.last_clock is not None and now < self.last_clock:
                self.clear()
                self.last_stamp = None
            self.last_clock, self.clock_changed = now, wall
        if wall - self.clock_changed > self.config.clock_stall_sec:
            self.clear()
        msg = self.pending
        if msg is None:
            return
        age = (now - msg.header.stamp).to_sec()
        if not 0 <= age <= self.config.display_timeout_sec or wall - self.received > self.config.display_timeout_sec:
            self.clear()
            return
        if self.shown_stamp == msg.header.stamp:
            return
        # No latest-time fallback: a moving vehicle must use the scan-time pose.
        if not self.tf_buffer.can_transform(self.config.reference_frame, msg.header.frame_id,
                                            msg.header.stamp, rospy.Duration(0)):
            rospy.logwarn_throttle(2, 'LiDAR display waiting for scan-time TF %s <- %s',
                                   self.config.reference_frame, msg.header.frame_id)
            return
        markers = [Marker(action=Marker.DELETEALL)]
        for index, obj in enumerate(msg.objects):
            marker = Marker()
            marker.header = copy.deepcopy(msg.header)
            marker.ns = 'lidar_object_geometry_unverified'
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = copy.deepcopy(obj.center)
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(obj.size.x, 0.03)
            marker.scale.y = max(obj.size.y, 0.03)
            marker.scale.z = max(obj.size.z, 0.03)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.3, 0.7, 0.45
            marker.lifetime = rospy.Duration(self.config.display_timeout_sec)
            marker.frame_locked = False
            markers.append(marker)
        self.publisher.publish(MarkerArray(markers=markers))
        self.shown_stamp, self.visible = msg.header.stamp, True

    def ingest(self, message, now, wall):
        self.update(now, wall)
        try:
            validate_lidar(message, for_fusion=False)
            if self.last_stamp is not None and message.header.stamp <= self.last_stamp:
                raise ValueError('duplicate or regressing scan')
            if not 0 <= (now - message.header.stamp).to_sec() <= self.config.display_timeout_sec:
                raise ValueError('future or stale scan')
        except (ValueError, TypeError, AttributeError) as error:
            self.clear()
            rospy.logwarn_throttle(2, 'LiDAR display rejected observation: %s', error)
            return
        self.last_stamp = message.header.stamp
        if not message.objects_valid:
            self.clear()
            return
        self.pending, self.received = message, wall
        self.update(now, wall)
