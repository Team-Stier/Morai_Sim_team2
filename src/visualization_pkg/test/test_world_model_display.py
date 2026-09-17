#!/usr/bin/env python3
"""Map-frame World Model marker rendering and watchdog behavior."""

import unittest
from unittest.mock import Mock, patch

import rospy
from common_msgs_pkg.msg import TrackedObject, WorldModel
from visualization_msgs.msg import Marker

from visualization_pkg.vehicle_display import DisplayConfig
from visualization_pkg.world_model_display import WorldModelDisplay


class WorldModelDisplayTest(unittest.TestCase):
    def setUp(self):
        self.publisher = Mock()
        self.display = WorldModelDisplay(DisplayConfig(), self.publisher)
        patch("rospy.logwarn_throttle").start()
        self.addCleanup(patch.stopall)

    @staticmethod
    def scene(seconds=10.0, reset_id=1):
        message = WorldModel()
        message.header.stamp = rospy.Time.from_sec(seconds)
        message.header.frame_id = "map"
        message.localization_reset_id = reset_id
        message.objects_valid = True
        message.tracking_valid = True
        message.reason = "diagnostic"
        obj = TrackedObject()
        obj.track_id = 42
        obj.source_stamp = message.header.stamp
        obj.source_frame_id = "lidar_link"
        obj.source_local_id = 7
        obj.timestamp_provenance = "ingress_fallback"
        obj.calibration_id = "development-test"
        obj.pose.position.x = 17.0
        obj.pose.orientation.w = 1.0
        obj.size.x, obj.size.y, obj.size.z = 2.0, 1.0, 1.0
        obj.confidence = -1.0
        obj.track_state = TrackedObject.TRACK_CONFIRMED
        obj.observation_count = 2
        obj.position_stddev_m = -1.0
        obj.velocity_stddev_mps = -1.0
        message.objects = [obj]
        return message

    def test_map_pose_and_track_id_are_rendered_without_tf(self):
        message = self.scene()
        self.display.ingest(message, rospy.Time(10), 1.0)
        marker = self.publisher.publish.call_args[0][0].markers[-1]
        self.assertEqual(marker.header.frame_id, "map")
        self.assertEqual(marker.header.stamp, message.header.stamp)
        self.assertEqual(marker.pose.position.x, 17.0)
        self.assertEqual(marker.id, 42)
        self.assertFalse(marker.frame_locked)

    def test_stale_invalid_and_reset_clear_existing_tracks(self):
        self.display.ingest(self.scene(), rospy.Time(10), 1.0)
        self.assertTrue(self.display.visible)
        self.display.update(rospy.Time(11), 2.0)
        self.assertFalse(self.display.visible)
        self.assertEqual(self.publisher.publish.call_args[0][0].markers[0].action, Marker.DELETEALL)

        self.display.ingest(self.scene(11.1), rospy.Time.from_sec(11.1), 2.1)
        self.display.ingest(self.scene(11.2, reset_id=2), rospy.Time.from_sec(11.2), 2.2)
        self.assertEqual(self.display.last_reset_id, 2)
        self.assertTrue(self.display.visible)

        invalid = self.scene(11.3, reset_id=2)
        invalid.objects_valid = False
        invalid.tracking_valid = False
        invalid.objects = []
        self.display.ingest(invalid, rospy.Time.from_sec(11.3), 2.3)
        self.assertFalse(self.display.visible)


if __name__ == "__main__":
    unittest.main()
