#!/usr/bin/env python3
"""Exercise the diagnostic node on rostest's isolated simulated-time master."""

import re
import threading
import time
import unittest

import rosgraph
import rospy
import rostest
from common_msgs_pkg.msg import LocalizationStatus
from common_msgs_pkg.validation import validate_localization
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


class LocalizationDiagnosticNodeRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("localization_diagnostic_node_test", anonymous=True)
        cls.condition = threading.Condition()
        cls.sequence = 0
        cls.status = None
        cls.contract_errors = []
        cls.status_sub = rospy.Subscriber(
            "/molit/localization/status", LocalizationStatus,
            cls._on_status, queue_size=100)
        cls.clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
        cls.gps_pub = rospy.Publisher(
            "/molit/sensors/gps/fix", NavSatFix, queue_size=10)
        cls.imu_pub = rospy.Publisher(
            "/molit/sensors/imu/data", Imu, queue_size=10)

        # A non-latched first clock/sample must not disappear during connection setup.
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if all(pub.get_num_connections() for pub in (
                    cls.clock_pub, cls.gps_pub, cls.imu_pub)):
                break
            time.sleep(0.02)
        else:
            raise AssertionError("Diagnostic node did not connect to test publishers")
        cls.clock_time = 10.0
        cls._set_clock(cls.clock_time)

    @classmethod
    def tearDownClass(cls):
        for transport in (cls.status_sub, cls.clock_pub, cls.gps_pub, cls.imu_pub):
            transport.unregister()

    @classmethod
    def _on_status(cls, message):
        try:
            validate_localization(message)
            if (message.gps_fix_valid or message.map_pose_valid
                    or message.local_odometry_valid or not message.stop_required):
                raise ValueError("Diagnostic output unlocked a validity/stop gate")
        except ValueError as error:
            with cls.condition:
                cls.contract_errors.append(str(error))
        with cls.condition:
            cls.sequence += 1
            cls.status = message
            cls.condition.notify_all()

    @classmethod
    def _snapshot(cls):
        with cls.condition:
            return cls.sequence, cls.status

    @classmethod
    def _wait_status(cls, after, predicate=lambda message: True, timeout=4.0):
        """A new callback may legally carry an equal or lower ROS stamp."""
        deadline = time.monotonic() + timeout
        with cls.condition:
            while True:
                if cls.sequence > after and cls.status is not None and predicate(cls.status):
                    return cls.status
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise AssertionError(
                        "No matching new status; last status: {}".format(cls.status))
                cls.condition.wait(min(remaining, 0.1))

    @classmethod
    def _set_clock(cls, seconds, predicate=lambda message: True):
        before, _ = cls._snapshot()
        cls.clock_time = float(seconds)
        expected = rospy.Time.from_sec(cls.clock_time)
        cls.clock_pub.publish(Clock(clock=expected))
        return cls._wait_status(
            before, lambda message: message.header.stamp == expected and predicate(message))

    @staticmethod
    def _counter(message, key):
        match = re.search(r"\b" + re.escape(key) + r"=(\d+)\b", message.reason)
        if match is None:
            raise AssertionError("Missing {} in diagnostic reason: {}".format(key, message.reason))
        return int(match.group(1))

    @staticmethod
    def _gps(stamp, frame="gps_link"):
        message = NavSatFix()
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.header.frame_id = frame
        message.status.status = NavSatStatus.STATUS_FIX
        message.latitude, message.longitude, message.altitude = 37.0, 127.0, 10.0
        message.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        return message

    @staticmethod
    def _imu(stamp, frame="imu_link"):
        message = Imu()
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.header.frame_id = frame
        message.orientation.w = 1.0
        message.linear_acceleration.z = 9.80665
        return message

    @classmethod
    def _send_and_count(cls, publisher, sample, counter):
        before, previous = cls._snapshot()
        expected = cls._counter(previous, counter) + 1
        publisher.publish(sample)
        return cls._wait_status(
            before, lambda message: cls._counter(message, counter) == expected)

    def setUp(self):
        # Observe the lower clock before advancing again so a reset cannot be hidden.
        high = self._set_clock(self.clock_time + 10.0)
        reset = self._set_clock(
            self.clock_time - 1.0,
            lambda message: message.reset_id == high.reset_id + 1)
        self.assertEqual(reset.gps_age_sec, -1.0)
        for sensor in ("gps", "imu"):
            self.assertEqual(self._counter(reset, sensor + "_accepted"), 0)
        ready = self._set_clock(
            self.clock_time + 0.1,
            lambda message: message.mode == LocalizationStatus.UNINITIALIZED)
        self.assertEqual(ready.reset_id, reset.reset_id)

    def tearDown(self):
        with self.condition:
            self.assertEqual(self.contract_errors, [])

    def test_status_contract_and_only_approved_advertisement(self):
        _, status = self._snapshot()
        validate_localization(status)
        self.assertEqual(status.header.frame_id, "")
        self.assertFalse(status.gps_fix_valid)
        self.assertFalse(status.map_pose_valid)
        self.assertFalse(status.local_odometry_valid)
        self.assertTrue(status.stop_required)
        self.assertEqual(status.gps_age_sec, -1.0)
        self.assertEqual(status.ego_state_stamp, rospy.Time(0))
        self.assertEqual(status.local_odometry_stamp, rospy.Time(0))
        self.assertEqual(status.map_position_stddev_m, -1.0)
        self.assertEqual(status.local_position_stddev_m, -1.0)
        self.assertEqual(status.yaw_stddev_rad, -1.0)

        master = rosgraph.Master(rospy.get_name())
        publishers, _, _ = master.getSystemState()
        node_topics = {topic for topic, nodes in publishers if "/localization_node" in nodes}
        self.assertEqual(node_topics, {"/rosout", "/molit/localization/status"})
        types = dict(master.getPublishedTopics("/molit/localization"))
        self.assertEqual(types["/molit/localization/status"], "common_msgs_pkg/LocalizationStatus")

        # Status must continue at approximately 10 Hz even when ROS time freezes.
        before, _ = self._snapshot()
        time.sleep(1.0)
        after, _ = self._snapshot()
        self.assertGreaterEqual(after - before, 6)
        self.assertLessEqual(after - before, 15)

    def test_bad_gps_preserves_last_accepted_measurement_age(self):
        accepted_stamp = self.clock_time - 0.5
        accepted = self._send_and_count(
            self.gps_pub, self._gps(accepted_stamp), "gps_accepted")
        accepted_count = self._counter(accepted, "gps_accepted")
        epoch = accepted.reset_id
        samples = [self._gps(self.clock_time, ""),
                   self._gps(self.clock_time, "wrong_link"),
                   self._gps(0.0), self._gps(self.clock_time + 1.0)]
        nonfinite = self._gps(self.clock_time)
        nonfinite.latitude = float("nan")
        samples.append(nonfinite)
        out_of_range = self._gps(self.clock_time)
        out_of_range.latitude = 91.0
        samples.append(out_of_range)
        no_fix = self._gps(self.clock_time)
        no_fix.status.status = NavSatStatus.STATUS_NO_FIX
        samples.append(no_fix)
        bad_covariance = self._gps(self.clock_time)
        bad_covariance.position_covariance[0] = float("nan")
        samples.append(bad_covariance)

        for sample in samples:
            with self.subTest(frame=sample.header.frame_id, stamp=sample.header.stamp,
                              latitude=sample.latitude, fix=sample.status.status):
                status = self._send_and_count(self.gps_pub, sample, "gps_rejected")
                self.assertEqual(status.reset_id, epoch)
                self.assertEqual(self._counter(status, "gps_accepted"), accepted_count)
                self.assertAlmostEqual(status.gps_age_sec, 0.5, places=6)
                self.assertFalse(status.gps_fix_valid)
        aged = self._set_clock(self.clock_time + 0.25)
        self.assertAlmostEqual(aged.gps_age_sec, 0.75, places=6)

    def test_imu_validation_and_source_ordering_do_not_reset_clock_epoch(self):
        samples = [self._imu(self.clock_time, ""),
                   self._imu(self.clock_time, "wrong_link"),
                   self._imu(0.0), self._imu(self.clock_time + 1.0)]
        nonfinite = self._imu(self.clock_time)
        nonfinite.linear_acceleration.x = float("nan")
        samples.append(nonfinite)
        for sample in samples:
            with self.subTest(frame=sample.header.frame_id, stamp=sample.header.stamp):
                self._send_and_count(self.imu_pub, sample, "imu_rejected")

        for source, publisher, factory in (("gps", self.gps_pub, self._gps),
                                           ("imu", self.imu_pub, self._imu)):
            with self.subTest(source=source):
                stamp = self.clock_time - 0.25
                accepted = self._send_and_count(publisher, factory(stamp), source + "_accepted")
                count = self._counter(accepted, source + "_accepted")
                duplicates = self._counter(accepted, source + "_duplicates")
                regressions = self._counter(accepted, source + "_regressions")
                rejected = self._counter(accepted, source + "_rejected")
                duplicate = self._send_and_count(publisher, factory(stamp), source + "_duplicates")
                self.assertEqual(self._counter(duplicate, source + "_duplicates"), duplicates + 1)
                late = self._send_and_count(publisher, factory(stamp - 0.1), source + "_regressions")
                self.assertEqual(self._counter(late, source + "_regressions"), regressions + 1)
                self.assertEqual(self._counter(late, source + "_rejected"), rejected + 2)
                self.assertEqual(self._counter(late, source + "_accepted"), count)
                self.assertEqual(late.reset_id, accepted.reset_id)
                newer = self._send_and_count(publisher, factory(stamp + 0.1), source + "_accepted")
                self.assertEqual(newer.reset_id, accepted.reset_id)

    def test_frozen_clock_recovery_and_reset_to_zero(self):
        self._send_and_count(self.gps_pub, self._gps(self.clock_time - 0.1), "gps_accepted")
        accepted = self._send_and_count(self.imu_pub, self._imu(self.clock_time - 0.1), "imu_accepted")
        before, _ = self._snapshot()
        stalled = self._wait_status(
            before, lambda message: message.mode == LocalizationStatus.LOST)
        self.assertEqual(stalled.header.stamp, accepted.header.stamp)
        self.assertEqual(stalled.reset_id, accepted.reset_id)
        self.assertTrue(stalled.stop_required)
        resumed = self._set_clock(
            self.clock_time + 0.1,
            lambda message: message.mode == LocalizationStatus.INITIALIZING)
        self.assertEqual(resumed.reset_id, accepted.reset_id)

        self.clock_pub.publish(Clock(clock=rospy.Time(0)))
        # Drain an already queued status before checking that zero-time publication stops.
        time.sleep(0.25)
        before, _ = self._snapshot()
        time.sleep(0.35)
        after, _ = self._snapshot()
        self.assertEqual(after, before, "Status must not publish with zero ROS time")
        reset = self._set_clock(
            1.0, lambda message: message.reset_id == resumed.reset_id + 1)
        self.assertEqual(reset.gps_age_sec, -1.0)
        self.assertEqual(self._counter(reset, "gps_accepted"), 0)
        self.assertEqual(self._counter(reset, "imu_accepted"), 0)

        # Stamps from the new epoch must be accepted after buffers were cleared.
        self._send_and_count(self.gps_pub, self._gps(0.9), "gps_accepted")
        fresh = self._send_and_count(self.imu_pub, self._imu(0.9), "imu_accepted")
        self.assertEqual(fresh.reset_id, reset.reset_id)


if __name__ == "__main__":
    rostest.rosrun("localization_pkg", "localization_node_diagnostic",
                  LocalizationDiagnosticNodeRuntimeTest)
