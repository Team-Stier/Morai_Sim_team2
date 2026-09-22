"""Read-only RViz markers for map-frame World Model tracks."""

import copy
import colorsys

import rospy
from visualization_msgs.msg import Marker, MarkerArray

from common_msgs_pkg.world_model_validation import validate_world_model


class WorldModelDisplay:
    def __init__(self, config, publisher):
        self.config = config
        self.publisher = publisher
        self.pending = None
        self.received = 0.0
        self.last_stamp = None
        self.last_reset_id = None
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

    def _publish(self, message):
        markers = [Marker(action=Marker.DELETEALL)]
        for obj in message.objects:
            if obj.source_stamp != message.header.stamp:
                continue  # Do not display coasted points as a new scan.
            marker = Marker()
            marker.header = copy.deepcopy(message.header)
            marker.ns = "world_model_track_unverified" if not message.objects_verified else "world_model_track"
            marker.id = int(obj.track_id & 0x7FFFFFFF)
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.points = copy.deepcopy(obj.points)
            marker.scale.x = marker.scale.y = self.config.cluster_point_size_m
            marker.color.r, marker.color.g, marker.color.b = colorsys.hsv_to_rgb((obj.track_id * 0.61803398875) % 1, 0.75, 1)
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(self.config.display_timeout_sec)
            marker.frame_locked = False
            markers.append(marker)
        self.publisher.publish(MarkerArray(markers=markers))
        self.shown_stamp = message.header.stamp
        self.visible = bool(message.objects)

    def update(self, now, wall):
        if self.last_clock is None or now != self.last_clock:
            if self.last_clock is not None and now < self.last_clock:
                self.clear()
                self.last_stamp = None
                self.last_reset_id = None
            self.last_clock, self.clock_changed = now, wall
        if wall - self.clock_changed > self.config.clock_stall_sec:
            self.clear()
        message = self.pending
        if message is None:
            return
        age = (now - message.header.stamp).to_sec()
        if not 0.0 <= age <= self.config.display_timeout_sec or wall - self.received > self.config.display_timeout_sec:
            self.clear()
            return
        if self.shown_stamp == message.header.stamp:
            return
        self._publish(message)

    def ingest(self, message, now, wall):
        self.update(now, wall)
        try:
            validate_world_model(message, for_planning=False)
            if self.last_reset_id is not None and message.localization_reset_id != self.last_reset_id:
                self.clear()
                self.last_stamp = None
            if self.last_stamp is not None and message.header.stamp <= self.last_stamp:
                raise ValueError("duplicate or regressing World Model scene")
            if not 0.0 <= (now - message.header.stamp).to_sec() <= self.config.display_timeout_sec:
                raise ValueError("future or stale World Model scene")
        except (ValueError, TypeError, AttributeError) as error:
            self.clear()
            rospy.logwarn_throttle(2, "World Model display rejected scene: %s", error)
            return
        self.last_stamp = message.header.stamp
        self.last_reset_id = message.localization_reset_id
        if not message.objects_valid:
            self.clear()
            return
        self.pending, self.received = message, wall
        self.update(now, wall)
