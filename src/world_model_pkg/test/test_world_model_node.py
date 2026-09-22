#!/usr/bin/env python3
"""ROS producer-consumer check for scan-time map anchoring and persistent IDs."""

import time
import unittest

import rospy
import rostest
import tf2_ros
from common_msgs_pkg.msg import (
    ComponentStatus,
    LidarObjectObservation,
    LidarObservationArray,
    LocalizationStatus,
    WorldModel,
)
from geometry_msgs.msg import TransformStamped, Point


class WorldModelPipelineTest(unittest.TestCase):
    def setUp(self):
        self.scenes = []
        self.health = []
        self.scene_subscriber = rospy.Subscriber(
            "/molit/world_model/scene", WorldModel, self.scenes.append, queue_size=10
        )
        self.health_subscriber = rospy.Subscriber(
            "/molit/world_model/status", ComponentStatus, self.health.append, queue_size=10
        )
        self.observation_publisher = rospy.Publisher(
            "/molit/perception/lidar/observations", LidarObservationArray, queue_size=2
        )
        self.localization_publisher = rospy.Publisher(
            "/molit/localization/status", LocalizationStatus, queue_size=10, latch=True
        )
        self.lidar_status_publisher = rospy.Publisher(
            "/molit/perception/lidar/status", ComponentStatus, queue_size=2, latch=True
        )
        self.dynamic_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    @staticmethod
    def transform(parent, child, stamp, x):
        value = TransformStamped()
        value.header.frame_id = parent
        value.header.stamp = stamp
        value.child_frame_id = child
        value.transform.translation.x = x
        value.transform.rotation.w = 1.0
        return value

    @staticmethod
    def observation(stamp, relative_x):
        message = LidarObservationArray()
        message.header.stamp = stamp
        message.header.frame_id = "lidar_link"
        message.calibration_id = "development-test"
        message.calibration_verified = False
        message.timestamp_provenance = "ingress_fallback"
        message.freshness_verified = False
        message.objects_valid = True
        detected = LidarObjectObservation()
        detected.scan_local_id = 7
        detected.points = [Point(relative_x-.1,0,.5), Point(relative_x+.1,0,.5)]
        detected.point_count = 2
        detected.confidence = -1.0
        message.objects = [detected]
        return message

    def wait_for(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            values = [item for item in self.scenes if predicate(item)]
            if values:
                return values[-1]
            rospy.sleep(0.01)
        self.fail("timed out waiting for World Model scene: " + str([m.reason for m in self.health]))

    def test_vehicle_motion_leaves_static_object_at_same_map_position(self):
        deadline = time.monotonic() + 5.0
        while (
            self.observation_publisher.get_num_connections() == 0
            or self.localization_publisher.get_num_connections() == 0
        ) and time.monotonic() < deadline:
            rospy.sleep(0.02)
        self.assertGreater(self.observation_publisher.get_num_connections(), 0)
        self.assertGreater(self.localization_publisher.get_num_connections(), 0)

        # Publish both poses only after every subscriber is connected, then feed
        # the observations while they are still well inside the 0.5 s contract.
        now = rospy.Time.now()
        first_stamp = now - rospy.Duration(0.05)
        second_stamp = now
        mount = self.transform("base_link", "lidar_link", first_stamp, 2.0)
        self.static_broadcaster.sendTransform(mount)
        first_pose = self.transform("map", "base_link", first_stamp, 10.0)
        second_pose = self.transform("map", "base_link", second_stamp, 11.0)
        self.dynamic_broadcaster.sendTransform([first_pose, second_pose])
        rospy.sleep(0.05)

        localization = LocalizationStatus()
        localization.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
        localization.mode = LocalizationStatus.TRACKING
        localization.map_pose_valid = True
        localization.local_odometry_valid = True
        localization.stop_required = True
        localization.ego_state_stamp = second_stamp
        localization.local_odometry_stamp = second_stamp
        localization.reset_id = 1
        localization.map_position_stddev_m = 0.2
        localization.local_position_stddev_m = 0.1
        localization.yaw_stddev_rad = 0.05
        self.localization_publisher.publish(localization)
        upstream = ComponentStatus()
        upstream.header.stamp = rospy.Time.now()
        upstream.component = "lidar_perception_pkg"
        upstream.state = ComponentStatus.DEGRADED
        upstream.stop_required = True
        upstream.data_stamp = second_stamp
        upstream.data_age_sec = 0.0
        upstream.processing_latency_sec = 0.01
        self.lidar_status_publisher.publish(upstream)
        rospy.sleep(0.02)

        self.observation_publisher.publish(self.observation(first_stamp, 5.0))
        rospy.sleep(0.05)
        self.assertFalse(any(item.objects_valid for item in self.scenes))
        localization.header.stamp = rospy.Time.now()
        self.localization_publisher.publish(localization)
        first = self.wait_for(lambda item: item.objects_valid and len(item.objects) == 1)
        self.assertAlmostEqual(first.objects[0].pose.position.x, 17.0, places=5)
        self.assertAlmostEqual(first.objects[0].points[0].x, 16.9, places=5)
        self.assertAlmostEqual(first.objects[0].points[1].x, 17.1, places=5)
        first_id = first.objects[0].track_id
        self.assertEqual(first.objects[0].track_state, first.objects[0].TRACK_TENTATIVE)

        self.observation_publisher.publish(self.observation(second_stamp, 4.0))
        second = self.wait_for(
            lambda item: item.objects_valid
            and len(item.objects) == 1
            and item.header.stamp == second_stamp
        )
        self.assertEqual(second.objects[0].track_id, first_id)
        self.assertAlmostEqual(second.objects[0].pose.position.x, 17.0, places=5)
        self.assertEqual(second.objects[0].track_state, second.objects[0].TRACK_CONFIRMED)
        self.assertFalse(second.objects_verified)
        self.assertFalse(second.planner_ready)


if __name__ == "__main__":
    rospy.init_node("world_model_pipeline_test")
    rostest.rosrun("world_model_pkg", "world_model_pipeline", WorldModelPipelineTest)
