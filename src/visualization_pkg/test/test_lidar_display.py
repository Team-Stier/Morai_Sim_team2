#!/usr/bin/env python3
"""Scan-time TF gate and display watchdog using actual generated ROS messages."""
import unittest
from unittest.mock import Mock, patch
import rospy
from common_msgs_pkg.msg import LidarObservationArray, LidarObjectObservation
from visualization_msgs.msg import Marker
from visualization_pkg.lidar_display import LidarDisplay
from visualization_pkg.vehicle_display import DisplayConfig


class LidarDisplayTest(unittest.TestCase):
    def setUp(self):
        self.publisher, self.tf = Mock(), Mock()
        self.tf.can_transform.return_value = True
        self.display = LidarDisplay(DisplayConfig(), self.tf, self.publisher)
        self.log = patch('rospy.logwarn_throttle').start()
        self.addCleanup(patch.stopall)

    def scan(self, seconds=10):
        msg = LidarObservationArray()
        msg.header.frame_id = 'lidar_link'
        msg.header.stamp = rospy.Time.from_sec(seconds)
        msg.calibration_id = 'test'
        msg.timestamp_provenance = 'ingress_fallback'
        msg.objects_valid = True
        obj = LidarObjectObservation(point_count=20, confidence=-1)
        obj.center.x, obj.center.y, obj.center.z = 5, 2, -0.2
        obj.size.x, obj.size.y, obj.size.z = 1, 2, 0.5
        msg.objects = [obj]
        return msg

    def test_scan_time_tf_no_offset_double_application(self):
        msg = self.scan()
        self.display.ingest(msg, rospy.Time(10), 1)
        self.tf.can_transform.assert_called_with('map', 'lidar_link', msg.header.stamp, rospy.Duration(0))
        marker = self.publisher.publish.call_args[0][0].markers[-1]
        self.assertEqual(marker.header, msg.header)
        self.assertEqual(marker.pose.position, msg.objects[0].center)
        self.assertFalse(marker.frame_locked)
        self.assertEqual(marker.pose.orientation.w, 1)

    def test_wait_for_exact_tf_then_retry_and_delete_stale(self):
        self.tf.can_transform.return_value = False
        self.display.ingest(self.scan(), rospy.Time(10), 1)
        self.publisher.publish.assert_not_called()
        self.tf.can_transform.return_value = True
        self.display.update(rospy.Time.from_sec(10.1), 1.1)
        self.assertTrue(self.display.visible)
        self.display.update(rospy.Time(11), 2)
        self.assertFalse(self.display.visible)
        self.assertEqual(self.publisher.publish.call_args[0][0].markers[0].action, Marker.DELETEALL)

    def test_invalid_geometry_time_and_frame_clear(self):
        for mutation in ('nan', 'frame', 'invalid', 'future', 'duplicate'):
            self.display = LidarDisplay(DisplayConfig(), self.tf, self.publisher)
            self.display.ingest(self.scan(), rospy.Time(10), 1)
            msg = self.scan(10.1)
            if mutation == 'nan': msg.objects[0].center.x = float('nan')
            if mutation == 'frame': msg.header.frame_id = 'map'
            if mutation == 'invalid': msg.objects_valid = False
            if mutation == 'future': msg.header.stamp = rospy.Time(12)
            if mutation == 'duplicate': msg.header.stamp = rospy.Time(10)
            self.display.ingest(msg, rospy.Time.from_sec(10.1), 1.1)
            self.assertFalse(self.display.visible, mutation)

    def test_paused_clock_and_reset(self):
        self.display.ingest(self.scan(), rospy.Time(10), 1)
        self.display.update(rospy.Time(10), 2)
        self.assertFalse(self.display.visible)
        self.display.update(rospy.Time(5), 3)
        self.display.ingest(self.scan(5), rospy.Time(5), 3)
        self.assertTrue(self.display.visible)

    def test_empty_valid_scan_removes_previous_boxes(self):
        self.display.ingest(self.scan(), rospy.Time(10), 1)
        msg = self.scan(10.1)
        msg.objects = []
        self.display.ingest(msg, rospy.Time.from_sec(10.1), 1.1)
        self.assertEqual(len(self.publisher.publish.call_args[0][0].markers), 1)


if __name__ == '__main__':
    unittest.main()
