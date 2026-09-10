#!/usr/bin/env python3
"""Synthetic ROS producer -> detector -> consumer test; no UDP/GT input."""
import time
import unittest
import rospy
import rostest
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from common_msgs_pkg.msg import LidarObservationArray, ComponentStatus


class DetectionTest(unittest.TestCase):
    def test_detection_empty_invalid_and_watchdog(self):
        received, statuses = [], []
        rospy.Subscriber('/molit/perception/lidar/observations', LidarObservationArray,
                         received.append)
        rospy.Subscriber('/molit/perception/lidar/status', ComponentStatus, statuses.append)
        points = rospy.Publisher('/molit/sensors/lidar/points', PointCloud2, queue_size=1)
        transport = rospy.Publisher('/molit/sensors/lidar/status', Bool, queue_size=1, latch=True)
        deadline = time.monotonic()+5
        while points.get_num_connections()==0 and time.monotonic()<deadline: time.sleep(.02)
        self.assertGreater(points.get_num_connections(),0)

        def send(xyz):
            transport.publish(Bool(data=True)); time.sleep(.1)
            header=Header(stamp=rospy.Time.now(), frame_id='lidar_link')
            points.publish(create_cloud_xyz32(header, xyz))
            end=time.monotonic()+3
            while time.monotonic()<end:
                for msg in received:
                    if msg.header.stamp==header.stamp: return msg
                time.sleep(.01)
            self.fail('no matching scan output')

        blob=[(2+i*.05,j*.05,k*.05) for i in range(4) for j in range(4) for k in range(2)]
        msg=send(blob)
        self.assertTrue(msg.objects_valid); self.assertEqual(len(msg.objects),1)
        self.assertFalse(msg.calibration_verified)
        self.assertFalse(msg.freshness_verified)
        msg=send([(-10,0,0)])
        self.assertTrue(msg.objects_valid); self.assertEqual(len(msg.objects),0)
        msg=send([])
        self.assertFalse(msg.objects_valid); self.assertEqual(len(msg.objects),0)
        time.sleep(1.6)
        self.assertTrue(statuses[-1].stop_required)
        self.assertFalse(statuses[-1].ready)
        self.assertEqual(statuses[-1].state, ComponentStatus.FAULT)
        # Invalid time/frame never produces a misleading stamped observation.
        for header in (Header(stamp=rospy.Time(),frame_id='lidar_link'),
                       Header(stamp=rospy.Time.now()+rospy.Duration(30),frame_id='lidar_link'),
                       Header(stamp=rospy.Time.now(),frame_id='velodyne')):
            before=len(received)
            points.publish(create_cloud_xyz32(header,blob))
            time.sleep(.2)
            self.assertEqual(len(received),before)


if __name__=='__main__':
    rospy.init_node('lidar_test')
    rostest.rosrun('lidar_perception_pkg','lidar_node_contract',DetectionTest)
