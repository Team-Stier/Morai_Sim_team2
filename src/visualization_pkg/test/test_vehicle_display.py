#!/usr/bin/env python3
"""Exercise local-frame geometry and clock discontinuities without a ROS master."""

import copy
import math
import unittest

import rospy
from common_msgs_pkg.msg import LocalizationStatus
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from visualization_pkg.markers import render_markers
from visualization_pkg.vehicle_display import DisplayConfig, VehicleDisplay, stamp_ns


class LocalVehicleDisplayTest(unittest.TestCase):
    def setUp(self):
        self.config = DisplayConfig(
            reference_frame="odom", body_center_offset_m=(1.0, 0.0, 0.5),
            display_timeout_sec=10.0, clock_stall_sec=0.3)
        self.display = VehicleDisplay(self.config)
        self.now = rospy.Time(20, 200000000)
        self.odometry = Odometry()
        self.odometry.header.frame_id = "odom"
        self.odometry.header.stamp = rospy.Time(20, 123456789)
        self.odometry.child_frame_id = "base_link"
        self.odometry.pose.pose.position.x = 2.0
        self.odometry.pose.pose.position.y = 4.0
        self.odometry.pose.pose.position.z = 1.0
        self.odometry.pose.pose.orientation.z = math.sqrt(0.5)
        self.odometry.pose.pose.orientation.w = math.sqrt(0.5)
        self.odometry.pose.covariance = [0.04 if i % 7 == 0 else 0.0 for i in range(36)]
        self.status = LocalizationStatus()
        self.status.header.stamp = self.now
        self.status.mode = LocalizationStatus.DEAD_RECKONING
        self.status.local_odometry_valid = True
        self.status.stop_required = True
        self.status.local_odometry_stamp = self.odometry.header.stamp
        self.status.reset_id = 7
        self.status.gps_age_sec = -1.0
        self.status.map_position_stddev_m = -1.0
        self.status.local_position_stddev_m = math.sqrt(0.08)
        self.status.yaw_stddev_rad = 0.2

    def _ready(self):
        now_ns = stamp_ns(self.now)
        self.assertTrue(self.display.ingest_status(self.status, now_ns, 1.0))
        self.assertTrue(self.display.ingest_odometry(self.odometry, now_ns, 1.0))
        state = self.display.evaluate(now_ns, 1.01)
        self.assertTrue(state.valid, state.reason)
        return state

    def test_local_pose_and_footprint_keep_odom_frame_and_nanosecond_stamp(self):
        state = self._ready()
        self.assertEqual(state.frame_id, "odom")
        self.assertEqual(state.stamp_ns, 20123456789)
        for actual, expected in zip(state.pose.position, (2.0, 5.0, 1.5)):
            self.assertAlmostEqual(actual, expected)
        markers = render_markers(state, self.config, stamp_ns(self.now))
        body = next(marker for marker in markers.markers if marker.id == 0)
        label = next(marker for marker in markers.markers if marker.id == 2)
        self.assertEqual(body.action, Marker.ADD)
        self.assertEqual(body.header.frame_id, "odom")
        self.assertEqual(body.header.stamp, self.odometry.header.stamp)
        self.assertAlmostEqual(body.scale.x, 4.635)
        self.assertAlmostEqual(body.scale.y, 1.892)
        self.assertAlmostEqual(body.scale.z, 0.06)
        self.assertIn("LOCAL ODOMETRY", label.text)

    def test_reset_invalidates_cached_local_pose_until_new_pair_arrives(self):
        self._ready()
        now = rospy.Time(20, 300000000)
        reset = copy.deepcopy(self.status)
        reset.header.stamp = now
        reset.reset_id += 1
        self.assertTrue(self.display.ingest_status(reset, stamp_ns(now), 1.1))
        self.assertFalse(self.display.evaluate(stamp_ns(now), 1.11).valid)

        fresh = copy.deepcopy(self.odometry)
        fresh.header.stamp = rospy.Time(20, 250000000)
        self.assertTrue(self.display.ingest_odometry(fresh, stamp_ns(now), 1.12))
        reset.header.stamp = rospy.Time(20, 310000000)
        reset.local_odometry_stamp = fresh.header.stamp
        self.assertTrue(self.display.ingest_status(reset, stamp_ns(reset.header.stamp), 1.13))
        state = self.display.evaluate(stamp_ns(reset.header.stamp), 1.14)
        self.assertTrue(state.valid, state.reason)
        self.assertEqual(state.stamp_ns, stamp_ns(fresh.header.stamp))

    def test_status_one_nanosecond_away_does_not_match_another_sample(self):
        self._ready()
        self.status.header.stamp = rospy.Time(20, 300000000)
        self.status.local_odometry_stamp = rospy.Time(20, 123456790)
        now_ns = stamp_ns(self.status.header.stamp)
        self.assertTrue(self.display.ingest_status(self.status, now_ns, 1.1))
        self.assertFalse(self.display.evaluate(now_ns, 1.11).valid)

    def test_wrong_child_frame_or_covariance_invalidates_local_drawing(self):
        for corruption in ("child_frame", "covariance", "position_uncertainty", "yaw_uncertainty"):
            with self.subTest(corruption=corruption):
                self.display = VehicleDisplay(self.config)
                self._ready()
                now = rospy.Time(20, 300000000)
                bad_odometry = copy.deepcopy(self.odometry)
                bad_odometry.header.stamp = rospy.Time(20, 250000000)
                bad_status = copy.deepcopy(self.status)
                bad_status.header.stamp = now
                bad_status.local_odometry_stamp = bad_odometry.header.stamp
                if corruption == "child_frame":
                    bad_odometry.child_frame_id = "imu_link"
                elif corruption == "covariance":
                    bad_odometry.pose.covariance[0] = -1.0
                elif corruption == "position_uncertainty":
                    bad_status.local_position_stddev_m = 0.0
                else:
                    bad_status.yaw_stddev_rad = 0.0
                accepted = self.display.ingest_odometry(bad_odometry, stamp_ns(now), 1.1)
                if corruption in ("child_frame", "covariance"):
                    self.assertFalse(accepted)
                self.display.ingest_status(bad_status, stamp_ns(now), 1.11)
                self.assertFalse(self.display.evaluate(stamp_ns(now), 1.12).valid)

    def test_clock_stall_and_rewind_require_new_estimates(self):
        self._ready()
        now_ns = stamp_ns(self.now)
        stalled = self.display.evaluate(now_ns, 1.31)
        self.assertFalse(stalled.valid)
        markers = render_markers(stalled, self.config, now_ns)
        self.assertEqual({m.id for m in markers.markers if m.action == Marker.DELETE}, {0, 1})
        self.assertFalse(self.display.evaluate(now_ns + 100000000, 1.32).valid)

        # A fresh instance establishes a visible pose before the independent
        # backward-clock case, avoiding a false positive from the prior stall.
        self.display = VehicleDisplay(self.config)
        self._ready()
        self.assertFalse(self.display.evaluate(now_ns - 100000000, 1.1).valid)
        self.assertFalse(self.display.evaluate(now_ns + 100000000, 1.2).valid)


if __name__ == "__main__":
    unittest.main()
