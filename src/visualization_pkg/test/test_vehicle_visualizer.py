#!/usr/bin/env python3
"""Observe RViz marker behavior using only an isolated rostest ROS graph."""

import copy
import math
import threading
import time
import unittest

import rosgraph
import rospy
import rostest
from common_msgs_pkg.msg import EgoState, LocalizationStatus
from common_msgs_pkg.validation import validate_pair
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker, MarkerArray


MARKERS = "/molit/internal/visualization/vehicle_markers"
EGO = "/molit/localization/ego_state"
ODOMETRY = "/molit/localization/local/odometry"
STATUS = "/molit/localization/status"


class VehicleVisualizerRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("vehicle_visualizer_test", anonymous=True)
        cls.condition = threading.Condition()
        cls.sequence = 0
        cls.latest = None
        cls.epoch = 0
        cls.marker_sub = rospy.Subscriber(
            MARKERS, MarkerArray, cls._on_markers, queue_size=100)
        cls.clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
        cls.ego_pub = rospy.Publisher(EGO, EgoState, queue_size=10)
        cls.odom_pub = rospy.Publisher(ODOMETRY, Odometry, queue_size=10)
        cls.status_pub = rospy.Publisher(STATUS, LocalizationStatus, queue_size=10)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if all(pub.get_num_connections() for pub in (
                    cls.clock_pub, cls.ego_pub, cls.odom_pub, cls.status_pub)):
                break
            time.sleep(0.02)
        else:
            raise AssertionError("Visualizer did not connect to fixture publishers")
        cls._clock(100.0)

    @classmethod
    def tearDownClass(cls):
        for transport in (cls.marker_sub, cls.clock_pub, cls.ego_pub,
                          cls.odom_pub, cls.status_pub):
            transport.unregister()

    @classmethod
    def _on_markers(cls, message):
        with cls.condition:
            cls.sequence += 1
            cls.latest = message
            cls.condition.notify_all()

    @classmethod
    def _snapshot(cls):
        with cls.condition:
            return cls.sequence, cls.latest

    @classmethod
    def _wait(cls, after, predicate, timeout=4.0):
        # Callback order, not increasing ROS stamps: deletion must work while
        # /clock is frozen or has gone backwards.
        deadline = time.monotonic() + timeout
        with cls.condition:
            while True:
                if cls.sequence > after and cls.latest is not None and predicate(cls.latest):
                    return cls.latest
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise AssertionError("No matching marker callback; last: {}".format(cls.latest))
                cls.condition.wait(min(0.1, remaining))

    @classmethod
    def _clock(cls, seconds):
        stamp = rospy.Time.from_sec(seconds)
        cls.clock_pub.publish(Clock(clock=stamp))
        deadline = time.monotonic() + 2.0
        while rospy.Time.now() != stamp:
            if time.monotonic() >= deadline:
                raise AssertionError("Fixture clock did not advance")
            time.sleep(0.01)
        # Allow the node's independent /clock callback to observe this update.
        time.sleep(0.06)
        return stamp

    @staticmethod
    def _marker(message, marker_id, action):
        return next((m for m in message.markers
                     if m.ns == "vehicle" and m.id == marker_id and m.action == action), None)

    @classmethod
    def _hidden(cls, message):
        return (cls._marker(message, 0, Marker.DELETE) is not None
                and cls._marker(message, 1, Marker.DELETE) is not None
                and cls._marker(message, 0, Marker.ADD) is None)

    @classmethod
    def _visible(cls, message, stamp=None):
        body = cls._marker(message, 0, Marker.ADD)
        return body is not None and (stamp is None or body.header.stamp == stamp)

    @classmethod
    def _unavailable_status(cls):
        status = LocalizationStatus()
        status.header.stamp = rospy.Time.now()
        status.mode = LocalizationStatus.INITIALIZING
        status.stop_required = True
        status.reset_id = cls.epoch
        status.gps_age_sec = -1.0
        status.map_position_stddev_m = -1.0
        status.local_position_stddev_m = -1.0
        status.yaw_stddev_rad = -1.0
        status.reason = "Test estimator has not produced a valid pose"
        return status

    @classmethod
    def _pair(cls):
        ego = EgoState()
        ego.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(0.05)
        ego.header.frame_id = "map"
        ego.child_frame_id = "base_link"
        ego.pose.pose.position.x = 10.0
        ego.pose.pose.position.y = 20.0
        ego.pose.pose.position.z = 3.0
        ego.pose.pose.orientation.z = math.sqrt(0.5)
        ego.pose.pose.orientation.w = math.sqrt(0.5)
        ego.pose_valid = [True] * 6
        ego.twist_valid = [True] * 6
        ego.pose.covariance = [0.04 if i % 7 == 0 else 0.0 for i in range(36)]
        ego.twist.covariance = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]
        ego.reset_id = cls.epoch
        status = cls._unavailable_status()
        status.mode = LocalizationStatus.TRACKING
        status.map_pose_valid = True
        status.gps_fix_valid = True
        status.gps_age_sec = 0.05
        status.ego_state_stamp = ego.header.stamp
        status.map_position_stddev_m = math.sqrt(0.08)
        # A stop request does not erase an otherwise valid estimated position.
        status.stop_required = True
        validate_pair(ego, status)
        return ego, status

    def _show_pair(self, ego=None, status=None):
        if ego is None:
            self._clock(rospy.Time.now().to_sec() + 0.1)
            ego, status = self._pair()
        before, _ = self._snapshot()
        self.ego_pub.publish(ego)
        self.status_pub.publish(status)
        markers = self._wait(before, lambda msg: self._visible(msg, ego.header.stamp))
        return ego, status, markers

    def setUp(self):
        self._clock(rospy.Time.now().to_sec() + 10.0)
        type(self).epoch += 1
        before, _ = self._snapshot()
        self.status_pub.publish(self._unavailable_status())
        self._wait(before, self._hidden)

    def test_rotated_box_preserves_pose_time_and_read_only_ros_boundary(self):
        ego, _, markers = self._show_pair()
        body = self._marker(markers, 0, Marker.ADD)
        arrow = self._marker(markers, 1, Marker.ADD)
        label = self._marker(markers, 2, Marker.ADD)
        self.assertEqual(body.type, Marker.CUBE)
        self.assertEqual(arrow.type, Marker.ARROW)
        self.assertEqual(label.type, Marker.TEXT_VIEW_FACING)
        self.assertEqual(body.header.frame_id, "map")
        self.assertEqual(body.header.stamp, ego.header.stamp)
        self.assertEqual(arrow.header.stamp, ego.header.stamp)
        self.assertAlmostEqual(body.pose.position.x, 10.0)
        self.assertAlmostEqual(body.pose.position.y, 21.0)
        self.assertAlmostEqual(body.pose.position.z, 3.5)
        self.assertAlmostEqual(body.pose.orientation.z, math.sqrt(0.5))
        self.assertAlmostEqual(body.pose.orientation.w, math.sqrt(0.5))
        self.assertAlmostEqual(arrow.pose.position.x, body.pose.position.x)
        self.assertAlmostEqual(arrow.pose.position.y, body.pose.position.y)
        self.assertAlmostEqual(arrow.pose.orientation.z, body.pose.orientation.z)
        self.assertAlmostEqual(arrow.pose.orientation.w, body.pose.orientation.w)
        self.assertEqual(len(arrow.points), 2)
        self.assertGreater(arrow.points[1].x, arrow.points[0].x)
        self.assertAlmostEqual(arrow.points[1].y, arrow.points[0].y)
        self.assertAlmostEqual(body.scale.x, 4.635)
        self.assertAlmostEqual(body.scale.y, 1.892)
        self.assertAlmostEqual(body.scale.z, 2.434)
        self.assertIn("STOP REQUIRED", label.text)
        self.assertIn("MODEL OFFSET PROVISIONAL", label.text)

        publishers, subscribers, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        node = "/vehicle_visualizer_node"
        emitted = {topic for topic, nodes in publishers if node in nodes}
        consumed = {topic for topic, nodes in subscribers if node in nodes and topic.startswith("/molit/")}
        self.assertEqual(emitted, {MARKERS, "/molit/internal/visualization/hd_map_markers", "/rosout"})
        self.assertEqual(consumed, {EGO, ODOMETRY, STATUS})

    def test_missing_height_and_tilt_are_labelled_as_planar_projection(self):
        self._clock(rospy.Time.now().to_sec() + 0.1)
        ego, status = self._pair()
        ego.pose_valid = [True, True, False, False, False, True]
        ego.pose.pose.position.z = float("nan")
        validate_pair(ego, status)
        _, _, markers = self._show_pair(ego, status)
        body = self._marker(markers, 0, Marker.ADD)
        label = self._marker(markers, 2, Marker.ADD)
        self.assertAlmostEqual(body.pose.position.x, 10.0)
        self.assertAlmostEqual(body.pose.position.y, 21.0)
        self.assertAlmostEqual(body.pose.position.z, 0.5)
        self.assertIn("PLANAR PROJECTION", label.text)

    def test_unavailable_and_wrong_epoch_status_remove_previous_box(self):
        self._show_pair()
        self._clock(rospy.Time.now().to_sec() + 0.1)
        before, _ = self._snapshot()
        self.status_pub.publish(self._unavailable_status())
        markers = self._wait(before, self._hidden)
        self.assertIn("WAITING FOR LOCALIZATION", self._marker(markers, 2, Marker.ADD).text)

        _, status, _ = self._show_pair()
        self._clock(rospy.Time.now().to_sec() + 0.1)
        status.header.stamp = rospy.Time.now()
        status.reset_id += 1
        before, _ = self._snapshot()
        self.status_pub.publish(status)
        self._wait(before, self._hidden)

    def test_invalid_frame_future_stamp_and_missing_yaw_remove_box(self):
        for corruption in ("frame", "future", "yaw", "nonfinite"):
            with self.subTest(corruption=corruption):
                ego, _, _ = self._show_pair()
                bad = copy.deepcopy(ego)
                bad.header.stamp += rospy.Duration.from_sec(0.01)
                if corruption == "frame":
                    bad.header.frame_id = "odom"
                elif corruption == "future":
                    bad.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(1.0)
                elif corruption == "yaw":
                    bad.pose_valid[5] = False
                else:
                    bad.pose.pose.position.x = float("nan")
                before, _ = self._snapshot()
                self.ego_pub.publish(bad)
                self._wait(before, self._hidden)

    def test_status_must_refer_to_the_exact_estimate(self):
        ego, status, _ = self._show_pair()
        self._clock(rospy.Time.now().to_sec() + 0.1)
        status.header.stamp = rospy.Time.now()
        status.ego_state_stamp = ego.header.stamp + rospy.Duration.from_sec(0.01)
        before, _ = self._snapshot()
        self.status_pub.publish(status)
        self._wait(before, self._hidden)

    def test_stale_and_frozen_or_regressing_clock_remove_box(self):
        self._show_pair()
        before, _ = self._snapshot()
        self._clock(rospy.Time.now().to_sec() + 1.0)
        self._wait(before, self._hidden)

        self._show_pair()
        before, _ = self._snapshot()
        self._wait(before, self._hidden, timeout=4.0)

        self._show_pair()
        before, _ = self._snapshot()
        self._clock(rospy.Time.now().to_sec() - 0.2)
        self._wait(before, self._hidden)

    def test_map_view_does_not_relabel_local_odometry_as_map(self):
        self._clock(rospy.Time.now().to_sec() + 0.1)
        odometry = Odometry()
        odometry.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(0.05)
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.orientation.w = 1.0
        status = self._unavailable_status()
        status.mode = LocalizationStatus.DEAD_RECKONING
        status.local_odometry_valid = True
        status.local_odometry_stamp = odometry.header.stamp
        status.local_position_stddev_m = 0.1
        status.yaw_stddev_rad = 0.1
        before, _ = self._snapshot()
        self.odom_pub.publish(odometry)
        self.status_pub.publish(status)
        self._wait(before, self._hidden)
        # Permit receipt of both inputs and several publication cycles.
        time.sleep(0.25)
        _, markers = self._snapshot()
        self.assertTrue(self._hidden(markers))


if __name__ == "__main__":
    rostest.rosrun("visualization_pkg", "vehicle_visualizer", VehicleVisualizerRuntimeTest)
