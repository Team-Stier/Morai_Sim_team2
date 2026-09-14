#!/usr/bin/env python3
"""Actual PointCloud2 -> C++ detector -> visualization with measured-time TF."""
import time
import unittest
import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, Bool
from common_msgs_pkg.msg import LidarObservationArray
from visualization_msgs.msg import Marker, MarkerArray


class PipelineTest(unittest.TestCase):
    def test_geometry_tf_and_timeout(self):
        observations, markers = [], []
        subs = [rospy.Subscriber('/molit/perception/lidar/observations', LidarObservationArray, observations.append),
                rospy.Subscriber('/molit/internal/visualization/lidar_markers', MarkerArray, markers.append)]
        points = rospy.Publisher('/molit/sensors/lidar/points', PointCloud2, queue_size=1)
        transport = rospy.Publisher('/molit/sensors/lidar/status', Bool, queue_size=1, latch=True)
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        end = time.monotonic() + 5
        while points.get_num_connections() == 0 and time.monotonic() < end:
            time.sleep(.02)
        self.assertGreater(points.get_num_connections(), 0)
        transport.publish(Bool(data=True))
        time.sleep(.2)
        stamp = rospy.Time.now()
        blob = [(2+i*.05, j*.05, k*.05) for i in range(4) for j in range(4) for k in range(2)]
        points.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id='lidar_link'), blob))
        end = time.monotonic()+.4
        while not observations and time.monotonic() < end:
            time.sleep(.01)
        self.assertTrue(observations)
        self.assertTrue(observations[-1].objects_valid)
        self.assertEqual(len(observations[-1].objects), 1)
        self.assertFalse(any(m.action == Marker.ADD for a in markers for m in a.markers))
        # Supply missing TF before the pending observation expires.
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = 'base_link', 'lidar_link'
        mount.header.stamp = stamp
        mount.transform.translation.x, mount.transform.translation.z = 2.0, 1.5
        mount.transform.rotation.w = 1
        pose = TransformStamped()
        pose.header.frame_id, pose.child_frame_id = 'map', 'base_link'
        pose.header.stamp = stamp
        pose.transform.translation.x, pose.transform.translation.y = 10, 20
        pose.transform.rotation.z = pose.transform.rotation.w = 2**-.5
        broadcaster.sendTransform([mount, pose])
        end = time.monotonic()+.5
        shown = []
        while not shown and time.monotonic() < end:
            shown = [m for a in markers for m in a.markers if m.action == Marker.ADD]
            time.sleep(.01)
        self.assertTrue(shown)
        box = shown[-1]
        self.assertEqual(box.header, observations[-1].header)
        self.assertEqual(box.pose.position, observations[-1].objects[0].center)
        self.assertFalse(box.frame_locked)
        time.sleep(1.2)
        self.assertTrue(all(m.action == Marker.DELETEALL for m in markers[-1].markers))
        self.assertTrue(subs)


if __name__ == '__main__':
    rospy.init_node('lidar_pipeline_test')
    rostest.rosrun('visualization_pkg', 'lidar_pipeline', PipelineTest)
