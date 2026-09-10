#!/usr/bin/env python3
"""Check the real producer against TF and its visualization consumer in isolation.

Run only with rostest's fresh master, never --reuse-master against MORAI.
The synthetic GPS samples are PROJ reference coordinates, not simulator truth.
"""

import math
import threading
import time
import unittest
from collections import OrderedDict

import numpy as np
import rosgraph
import rospy
import rostest
from common_msgs_pkg.msg import EgoState, LocalizationStatus
from common_msgs_pkg.validation import validate_ego, validate_localization, validate_pair
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage
from visualization_pkg.vehicle_display import DisplayConfig, VehicleDisplay


def rotation(q):
    """Independent Hamilton quaternion-to-matrix oracle; xyzw convention."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def matrix(transform):
    result = np.eye(4)
    q = transform.rotation
    result[:3, :3] = rotation((q.x, q.y, q.z, q.w))
    p = transform.translation
    result[:3, 3] = (p.x, p.y, p.z)
    return result


class LocalizationEstimatorRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('localization_estimator_contract_test', anonymous=True)
        cls.condition = threading.Condition()
        cls.ego = OrderedDict()
        cls.odom = OrderedDict()
        cls.transforms = OrderedDict()
        cls.status = None
        cls.errors = []
        cls.relocalizing_epochs = set()
        cls.sequence = 0
        cls.clock = 100.0
        cls.clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
        cls.gps_pub = rospy.Publisher('/molit/sensors/gps/fix', NavSatFix, queue_size=10)
        cls.imu_pub = rospy.Publisher('/molit/sensors/imu/data', Imu, queue_size=10)
        cls.subscribers = [
            rospy.Subscriber('/molit/localization/ego_state', EgoState, cls._on_ego, queue_size=100),
            rospy.Subscriber('/molit/localization/local/odometry', Odometry, cls._on_odom, queue_size=100),
            rospy.Subscriber('/molit/localization/status', LocalizationStatus, cls._on_status, queue_size=100),
            rospy.Subscriber('/tf', TFMessage, cls._on_tf, queue_size=100),
        ]
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            if all(pub.get_num_connections() for pub in (cls.clock_pub, cls.gps_pub, cls.imu_pub)):
                break
            time.sleep(0.02)
        else:
            raise AssertionError('Estimator did not connect to isolated sensor publishers')
        cls._advance(cls.clock)

    @classmethod
    def tearDownClass(cls):
        for transport in cls.subscribers + [cls.clock_pub, cls.gps_pub, cls.imu_pub]:
            transport.unregister()

    @classmethod
    def _on_ego(cls, message):
        with cls.condition:
            try:
                validate_ego(message)
            except ValueError as error:
                cls.errors.append(str(error))
            cls.ego[message.header.stamp.to_nsec()] = message
            cls.condition.notify_all()

    @classmethod
    def _on_odom(cls, message):
        with cls.condition:
            cls.odom[message.header.stamp.to_nsec()] = message
            cls.condition.notify_all()

    @classmethod
    def _on_tf(cls, message):
        with cls.condition:
            for transform in message.transforms:
                key = (transform.header.stamp.to_nsec(), transform.child_frame_id)
                cls.transforms[key] = transform
            cls.condition.notify_all()

    @classmethod
    def _on_status(cls, message):
        with cls.condition:
            try:
                validate_localization(message)
            except ValueError as error:
                cls.errors.append(str(error))
            if message.mode == LocalizationStatus.RELOCALIZING:
                cls.relocalizing_epochs.add(message.reset_id)
                cls.relocalizing_status = message
            cls.status = message
            cls.sequence += 1
            cls.condition.notify_all()

    @classmethod
    def _wait(cls, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        with cls.condition:
            while not predicate():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise AssertionError('Timed out; last status: {}'.format(cls.status))
                cls.condition.wait(min(remaining, 0.1))

    @classmethod
    def _advance(cls, seconds):
        cls.clock = float(seconds)
        value = rospy.Time.from_sec(cls.clock)
        before = cls.sequence
        cls.clock_pub.publish(Clock(clock=value))
        cls._wait(lambda: cls.sequence > before and cls.status.header.stamp == value)

    @staticmethod
    def _imu(stamp):
        message = Imu()
        message.header.frame_id = 'imu_link'
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.orientation.z = math.sin(math.pi / 4)
        message.orientation.w = math.cos(math.pi / 4)
        message.orientation_covariance = [0.0001, 0., 0., 0., 0.0001, 0., 0., 0., 0.0001]
        message.angular_velocity_covariance = [0.0001, 0., 0., 0., 0.0001, 0., 0., 0., 0.0001]
        message.linear_acceleration_covariance = [0.01, 0., 0., 0., 0.01, 0., 0., 0., 0.01]
        message.linear_acceleration.z = 9.80665
        return message

    @staticmethod
    def _gps(stamp):
        message = NavSatFix()
        message.header.frame_id = 'gps_link'
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.status.status = NavSatStatus.STATUS_FIX
        message.latitude, message.longitude, message.altitude = 37.23956789, 126.77456789, 30.0
        message.position_covariance = [0.01, 0., 0., 0., 0.01, 0., 0., 0., 0.01]
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        return message

    @classmethod
    def _initialize(cls):
        # A distinct clock reset gives each scenario independent temporal buffers.
        previous_epoch = cls.status.reset_id
        cls._advance(cls.clock + 10.0)
        cls._advance(cls.clock - 1.0)
        cls._wait(lambda: cls.status.reset_id > previous_epoch)
        for index in range(8):
            cls._advance(cls.clock + 0.02)
            cls.imu_pub.publish(cls._imu(cls.clock))
            if index % 3 == 0:
                time.sleep(0.02)
                cls.gps_pub.publish(cls._gps(cls.clock))
            time.sleep(0.02)
        cls._wait(lambda: cls.status.map_pose_valid and cls.status.local_odometry_valid)
        cls._wait(lambda: cls.status.ego_state_stamp.to_nsec() in cls.ego
                  and cls.status.local_odometry_stamp.to_nsec() in cls.odom)
        return cls.status

    def tearDown(self):
        self.assertEqual(self.errors, [])

    def test_live_contract_tf_composition_and_visualization_consumer(self):
        status = self._initialize()
        stamp = status.ego_state_stamp.to_nsec()
        ego = self.ego[stamp]
        odom = self.odom[status.local_odometry_stamp.to_nsec()]
        validate_pair(ego, status)
        self.assertEqual(ego.header.stamp, odom.header.stamp)
        self.assertEqual(odom.header.frame_id, 'odom')
        self.assertEqual(odom.child_frame_id, 'base_link')
        self.assertTrue(status.stop_required, 'Development estimates do not authorize driving')
        # Independent PROJ reference UTM minus the approved translated map origin.
        self.assertAlmostEqual(ego.pose.pose.position.x, 4.349874600, delta=0.05)
        self.assertAlmostEqual(ego.pose.pose.position.y, -375.461369939, delta=0.05)
        self.assertAlmostEqual(ego.pose.pose.position.z, 28.7, delta=0.05)

        self._wait(lambda: (stamp, 'odom') in self.transforms and (stamp, 'base_link') in self.transforms)
        map_odom = self.transforms[(stamp, 'odom')]
        odom_base = self.transforms[(stamp, 'base_link')]
        self.assertEqual(map_odom.header.frame_id, 'map')
        self.assertEqual(odom_base.header.frame_id, 'odom')
        composed = matrix(map_odom.transform).dot(matrix(odom_base.transform))
        p, q = ego.pose.pose.position, ego.pose.pose.orientation
        np.testing.assert_allclose(composed[:3, 3], [p.x, p.y, p.z], atol=1e-8)
        np.testing.assert_allclose(composed[:3, :3], rotation((q.x, q.y, q.z, q.w)), atol=1e-8)

        # Exercise the actual consumer, including exact stamp/reset and covariance checks.
        for frame in ('map', 'odom'):
            display = VehicleDisplay(DisplayConfig(reference_frame=frame, display_timeout_sec=1.0))
            now_ns = status.header.stamp.to_nsec()
            if frame == 'map':
                self.assertTrue(display.ingest_ego(ego, now_ns, 1.0))
            else:
                self.assertTrue(display.ingest_odometry(odom, now_ns, 1.0))
            self.assertTrue(display.ingest_status(status, now_ns, 1.0))
            rendered = display.evaluate(now_ns, 1.0)
            self.assertTrue(rendered.valid, rendered.reason)
            self.assertEqual(rendered.stamp_ns, stamp)

        publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        topics = {name for name, nodes in publishers if '/localization_node' in nodes}
        self.assertEqual(topics, {'/rosout', '/tf', '/molit/localization/ego_state',
                                 '/molit/localization/local/odometry', '/molit/localization/status'})

    def test_invalid_and_out_of_order_input_never_produces_bad_estimate(self):
        status = self._initialize()
        epoch = status.reset_id
        last_stamp = status.ego_state_stamp.to_nsec()
        cases = []
        wrong_frame = self._imu(self.clock + 0.02)
        wrong_frame.header.frame_id = 'map'
        cases.append(wrong_frame)
        zero_quaternion = self._imu(self.clock + 0.04)
        zero_quaternion.orientation.z = zero_quaternion.orientation.w = 0.0
        cases.append(zero_quaternion)
        nonfinite = self._imu(self.clock + 0.06)
        nonfinite.linear_acceleration.x = float('nan')
        cases.append(nonfinite)
        for sample in cases:
            self._advance(sample.header.stamp.to_sec())
            self.imu_pub.publish(sample)
            time.sleep(0.08)
            self.assertNotIn(sample.header.stamp.to_nsec(), self.ego)
        # Replayed and earlier measurements must not create another reset epoch.
        self.imu_pub.publish(self._imu((last_stamp * 1e-9)))
        self.imu_pub.publish(self._imu((last_stamp * 1e-9) - 0.01))
        time.sleep(0.12)
        self.assertEqual(self.status.reset_id, epoch)
        self.assertEqual(self.errors, [])

    def test_gps_blackout_and_recovery_keep_local_epoch(self):
        status = self._initialize()
        epoch = status.reset_id
        for index in range(40):
            self._advance(self.clock + 0.025)
            self.imu_pub.publish(self._imu(self.clock))
            time.sleep(0.01)
        self._wait(lambda: self.status.mode == LocalizationStatus.DEAD_RECKONING)
        self.assertTrue(self.status.local_odometry_valid)
        self.assertTrue(self.status.stop_required)
        self.assertEqual(self.status.reset_id, epoch)
        for index in range(10):
            self._advance(self.clock + 0.02)
            self.imu_pub.publish(self._imu(self.clock))
            if index % 3 == 0:
                self.gps_pub.publish(self._gps(self.clock))
            time.sleep(0.02)
        self._wait(lambda: self.status.mode == LocalizationStatus.TRACKING)
        self.assertEqual(self.status.reset_id, epoch)

    def test_sensor_relocation_updates_epoch_and_consumer_without_odom_jump(self):
        status = self._initialize()
        epoch = status.reset_id
        old_x = self.ego[status.ego_state_stamp.to_nsec()].pose.pose.position.x
        old_local = self.odom[status.local_odometry_stamp.to_nsec()].pose.pose.position.x
        display = VehicleDisplay(DisplayConfig(reference_frame='map', display_timeout_sec=1.0))
        display.ingest_ego(self.ego[status.ego_state_stamp.to_nsec()], status.header.stamp.to_nsec(), 1.0)
        display.ingest_status(status, status.header.stamp.to_nsec(), 1.0)
        self.assertTrue(display.evaluate(status.header.stamp.to_nsec(), 1.0).valid)
        for index in range(65):
            self._advance(self.clock + 0.02)
            self.imu_pub.publish(self._imu(self.clock))
            if index % 10 == 0:
                shifted = self._gps(self.clock)
                shifted.longitude += 0.002
                self.gps_pub.publish(shifted)
            time.sleep(0.015)
        self._wait(lambda: self.status.reset_id == epoch + 1 and self.status.map_pose_valid)
        self._wait(lambda: self.status.ego_state_stamp.to_nsec() in self.ego
                  and self.status.local_odometry_stamp.to_nsec() in self.odom)
        current = self.status
        ego = self.ego[current.ego_state_stamp.to_nsec()]
        odom = self.odom[current.local_odometry_stamp.to_nsec()]
        self.assertIn(epoch, self.relocalizing_epochs)
        self.assertEqual(ego.reset_id, epoch + 1)
        self.assertGreater(ego.pose.pose.position.x - old_x, 100.)
        self.assertAlmostEqual(odom.pose.pose.position.x, old_local, delta=0.05)
        self.assertAlmostEqual(odom.twist.twist.linear.x, 0., delta=0.05)
        validate_pair(ego, current)
        self.assertTrue(current.stop_required)
        pending = self.relocalizing_status
        self.assertTrue(display.ingest_status(pending, pending.header.stamp.to_nsec(), 1.1))
        self.assertFalse(display.evaluate(pending.header.stamp.to_nsec(), 1.1).valid)
        self.assertTrue(display.ingest_status(current, current.header.stamp.to_nsec(), 1.2))
        self.assertFalse(display.evaluate(current.header.stamp.to_nsec(), 1.2).valid)
        self.assertTrue(display.ingest_ego(ego, current.header.stamp.to_nsec(), 1.2))
        self.assertTrue(display.evaluate(current.header.stamp.to_nsec(), 1.2).valid)

    def test_nanoseconds_and_late_gps_preserve_source_time(self):
        self._initialize()
        self._advance(self.clock + 0.023456789)
        sample = self._imu(self.clock)
        original_ns = sample.header.stamp.to_nsec()
        self.imu_pub.publish(sample)
        for index in range(5):
            self._advance(self.clock + 0.02)
            self.imu_pub.publish(self._imu(self.clock))
            time.sleep(0.02)
        self._wait(lambda: original_ns in self.ego)
        self.assertEqual(self.ego[original_ns].header.stamp, sample.header.stamp)
        accepted_gps = self.status.header.stamp.to_sec() - self.status.gps_age_sec
        self.gps_pub.publish(self._gps(self.clock - 0.4))
        time.sleep(0.15)
        self.assertAlmostEqual(self.status.header.stamp.to_sec() - self.status.gps_age_sec,
                               accepted_gps, places=6)

    def test_clock_stall_and_regression_clear_validity_and_restart_epoch(self):
        status = self._initialize()
        previous_epoch = status.reset_id
        self._wait(lambda: self.status.mode == LocalizationStatus.LOST, timeout=5.0)
        self.assertFalse(self.status.map_pose_valid)
        self.assertFalse(self.status.local_odometry_valid)
        self.assertTrue(self.status.stop_required)
        time.sleep(0.1)  # Drain already published packets before checking no repeated pose/TF.
        counts = (len(self.ego), len(self.odom), len(self.transforms))
        time.sleep(0.25)
        self.assertEqual(counts, (len(self.ego), len(self.odom), len(self.transforms)))
        self._advance(1.0)
        self._wait(lambda: self.status.reset_id > previous_epoch)
        self.assertFalse(self.status.map_pose_valid)
        self.assertEqual(self.status.gps_age_sec, -1.0)
        for index in range(8):
            self._advance(1.02 + 0.02 * index)
            self.imu_pub.publish(self._imu(self.clock))
            if index % 3 == 0:
                time.sleep(0.02)
                self.gps_pub.publish(self._gps(self.clock))
            time.sleep(0.02)
        self._wait(lambda: self.status.map_pose_valid)
        self.assertGreater(self.status.reset_id, previous_epoch)


if __name__ == '__main__':
    rostest.rosrun('localization_pkg', 'localization_estimator', LocalizationEstimatorRuntimeTest)
