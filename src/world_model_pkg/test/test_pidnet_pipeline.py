#!/usr/bin/env python3
"""Sensor + semantic scores -> foreground cloud -> scan-time tracked scene."""
import time
import unittest
import numpy as np
import rospy
import rostest
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, PointCloud2
from common_msgs_pkg.msg import LocalizationStatus, WorldModel, ComponentStatus
from ros_numpy.point_cloud2 import array_to_pointcloud2
from world_model_pkg.pointpainting import cloud_records


class PidnetPipelineTest(unittest.TestCase):
    def wait_for(self, check):
        end = time.monotonic() + 5
        while time.monotonic() < end:
            if check(): return
            time.sleep(.01)
        self.fail('pipeline timed out')

    def test_foreground_scene_and_camera_loss(self):
        painted, scenes, status = [], [], []
        subs = [rospy.Subscriber('/molit/world_model/painted_points', PointCloud2, painted.append),
                rospy.Subscriber('/molit/world_model/scene', WorldModel, scenes.append),
                rospy.Subscriber('/molit/world_model/status', ComponentStatus, status.append)]
        clouds = rospy.Publisher('/molit/sensors/lidar/points', PointCloud2, queue_size=1)
        scores = rospy.Publisher('/molit/perception/camera/front/semantic_scores', Image, queue_size=1)
        loc = rospy.Publisher('/molit/localization/status', LocalizationStatus, queue_size=1, latch=True)
        self.wait_for(lambda: clouds.get_num_connections() and scores.get_num_connections() and loc.get_num_connections())
        # World Model must consume only painted clouds, never old raw-only observations.
        import rosgraph
        subscriptions = dict(rosgraph.Master(rospy.get_name()).getSystemState()[1])
        self.assertIn('/world_model_node', subscriptions['/molit/internal/world_model/pidnet_points'])
        self.assertNotIn('/world_model_node', subscriptions.get('/molit/perception/lidar/observations', []))
        tf = TransformStamped(); tf.header.frame_id = 'map'; tf.child_frame_id = 'lidar_link'
        tf.transform.translation.x = 10; tf.transform.rotation.w = 1
        broadcaster = tf2_ros.StaticTransformBroadcaster(); broadcaster.sendTransform(tf)
        time.sleep(.1)

        def send(label):
            stamp = rospy.Time.now() - rospy.Duration(.02)
            localization = LocalizationStatus(); localization.header.stamp = rospy.Time.now()
            localization.map_pose_valid = True; localization.reset_id = 1
            loc.publish(localization)
            data = np.array([(5., 0., 0., 41.), (5.1, 0., 0., 42.), (5.2, 0., 0., 43.),
                             (-5., 0., 0., 99.)], dtype=[(n, '<f4') for n in ('x','y','z','intensity')])
            cloud = array_to_pointcloud2(data, stamp=stamp, frame_id='lidar_link')
            probabilities = np.zeros((18,32,6), np.float32); probabilities[:,:,label] = 1
            image = CvBridge().cv2_to_imgmsg(probabilities); image.header.stamp = stamp
            image.header.frame_id = 'camera_front_optical_frame'
            clouds.publish(cloud); scores.publish(image)
            self.wait_for(lambda: any(m.header.stamp == stamp for m in scenes))
            return stamp, next(m for m in scenes if m.header.stamp == stamp)

        stamp, scene = send(1)
        output = next(m for m in painted if m.header.stamp == stamp)
        data = cloud_records(output)
        np.testing.assert_array_equal(data['intensity'], [41,42,43])
        self.assertEqual(output.header.frame_id, 'lidar_link')
        self.assertTrue(scene.objects_valid); self.assertFalse(scene.planner_ready)
        self.assertEqual(len(scene.objects), 1)
        obj = scene.objects[0]
        self.assertEqual(obj.source_stamp, stamp)
        self.assertEqual(obj.semantic_class, obj.CLASS_PEDESTRIAN)
        self.assertEqual(obj.confidence, -1)
        np.testing.assert_allclose([p.x for p in obj.points], [15,15.1,15.2], atol=1e-5)
        self.assertEqual(scene.header.frame_id, 'map')
        stamp, _ = send(0)
        self.assertEqual(next(m for m in painted if m.header.stamp == stamp).width, 0)
        # Keep raw LiDAR arriving without semantic images. No raw-only fallback or cached pair.
        before = len(painted)
        for _ in range(12):
            cloud = array_to_pointcloud2(np.array([(5.,0.,0.)], dtype=[(n,'<f4') for n in ('x','y','z')]),
                                         stamp=rospy.Time.now(), frame_id='lidar_link')
            clouds.publish(cloud); time.sleep(.1)
        self.assertEqual(len(painted), before)
        self.wait_for(lambda: status and status[-1].state == ComponentStatus.FAULT)
        self.assertTrue(status[-1].stop_required)
        for sub in subs: sub.unregister()


if __name__ == '__main__':
    rospy.init_node('pidnet_pipeline_test')
    rostest.rosrun('world_model_pkg', 'pidnet_pipeline', PidnetPipelineTest)
