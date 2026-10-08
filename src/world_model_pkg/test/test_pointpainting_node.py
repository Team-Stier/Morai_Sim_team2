#!/usr/bin/env python3
import time
import unittest
import numpy as np
import rospy
import rostest
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image
from ros_numpy.point_cloud2 import array_to_pointcloud2
from world_model_pkg.pointpainting import cloud_records


class PaintingTransportTest(unittest.TestCase):
    def test_real_transport_sync_fields_and_stale_pair(self):
        received = []
        sub = rospy.Subscriber('/molit/world_model/painted_points', PointCloud2, received.append)
        clouds = rospy.Publisher('/molit/sensors/lidar/points', PointCloud2, queue_size=1)
        images = rospy.Publisher('/molit/perception/camera/front/semantic_scores', Image, queue_size=1)
        deadline = time.monotonic() + 10
        while not all([clouds.get_num_connections(), images.get_num_connections(), sub.get_num_connections()]) and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertTrue(all([clouds.get_num_connections(), images.get_num_connections(), sub.get_num_connections()]))
        records = np.array([(5., 0., 0., 42.), (-5., 0., 0., 99.)],
                           dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])
        cloud = array_to_pointcloud2(records, frame_id='lidar_link')
        scores = np.zeros((18, 32, 6), np.float32); scores[:, :, 1] = 1
        image = CvBridge().cv2_to_imgmsg(scores)
        image.header.frame_id = 'camera_front_optical_frame'
        cloud.header.stamp = rospy.Time.now() - rospy.Duration(0.03)
        image.header.stamp = cloud.header.stamp + rospy.Duration(0.01)
        clouds.publish(cloud); images.publish(image)
        deadline = time.monotonic() + 5
        while not received and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual(len(received), 1)
        result = cloud_records(received[0])
        self.assertEqual(received[0].header, cloud.header)
        np.testing.assert_array_equal(result['intensity'], records['intensity'])
        np.testing.assert_array_equal(result['painted'], [1, 0])
        np.testing.assert_array_equal(result['score_1'], [1, 0])
        np.testing.assert_array_equal(result['image_stamp_nsec'], image.header.stamp.nsecs)
        cloud.header.stamp = rospy.Time.now() - rospy.Duration(2)
        image.header.stamp = cloud.header.stamp
        clouds.publish(cloud); images.publish(image)
        time.sleep(0.4)
        self.assertEqual(len(received), 1)
        sub.unregister()


if __name__ == '__main__':
    rospy.init_node('pointpainting_transport_test')
    rostest.rosrun('world_model_pkg', 'pointpainting_transport', PaintingTransportTest)
