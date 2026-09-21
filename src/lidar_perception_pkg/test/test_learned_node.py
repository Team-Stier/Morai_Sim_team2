#!/usr/bin/env python3
"""Exercise actual ROS ingress, worker failure, deadline and original-stamp output."""
import threading
import time
import unittest
import numpy as np
import rospy
import rostest
from common_msgs_pkg.msg import LidarObservationArray, ComponentStatus
from common_msgs_pkg.lidar_validation import validate_lidar
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.point_cloud2 import create_cloud
from std_msgs.msg import Header, Bool
from lidar_perception_pkg.learned_node import LearnedNode


class FakeModel:
    classes = ['pedestrian','truck']
    mode = 'ok'
    entered = threading.Event()
    release = threading.Event()
    def __init__(self, *args): pass
    def infer(self, points):
        if self.mode == 'fail': raise RuntimeError('injected CUDA failure')
        if self.mode == 'slow': time.sleep(.5)
        if self.mode == 'blocked':
            self.entered.set()
            if not self.release.wait(2): raise RuntimeError('test release timeout')
        return np.array([[5,0,0,.8,.8,1.8,0],[10,2,0,5,2,3,0]]), np.array([.8,.9]), np.array([1,2])


class NodeTest(unittest.TestCase):
    def test_live_boundary_and_failures(self):
        observations, statuses = [], []
        node = LearnedNode(FakeModel)
        subs = [rospy.Subscriber('/molit/perception/lidar/observations',LidarObservationArray,observations.append),
                rospy.Subscriber('/molit/perception/lidar/status',ComponentStatus,statuses.append)]
        points = rospy.Publisher('/molit/sensors/lidar/points',PointCloud2,queue_size=1)
        transport = rospy.Publisher('/molit/sensors/lidar/status',Bool,queue_size=1)
        stop = threading.Event()
        def transport_loop():
            while not stop.wait(.05): transport.publish(Bool(True))
        threading.Thread(target=transport_loop,daemon=True).start()
        self.addCleanup(node.shutdown); self.addCleanup(stop.set)
        def wait(predicate,seconds=3):
            end = time.monotonic()+seconds
            while not predicate() and time.monotonic()<end: time.sleep(.01)
            self.assertTrue(predicate())
        wait(lambda: points.get_num_connections()>0 and node.transport_ok and node.model_ready)
        time.sleep(.15)
        fields = [PointField(n,i*4,7,1) for i,n in enumerate(('x','y','z','intensity'))]
        def send():
            msg = create_cloud(Header(stamp=rospy.Time.now(),frame_id='lidar_link'), fields,
                                [(5,0,0,20),(10,2,0,30)])
            points.publish(msg)
            return msg
        msg = send()
        wait(lambda: len(observations)>0)
        output = observations[-1]
        self.assertEqual(output.header,msg.header)
        self.assertEqual([o.semantic_class for o in output.objects],[1,2])
        validate_lidar(output)
        with self.assertRaises(ValueError): validate_lidar(output,for_fusion=True)
        for mode in ('fail','slow'):
            FakeModel.mode = mode
            count = len(observations); send()
            wait(lambda: len(observations)>count)
            self.assertFalse(observations[-1].objects_valid)
            self.assertEqual(observations[-1].objects,[])
        FakeModel.mode = 'ok'
        count = len(observations)
        points.publish(msg)  # older than the accepted failed/slow input stamps
        wait(lambda: any('regressing' in s.reason for s in statuses))
        self.assertEqual(len(observations),count)
        # A reset while CUDA is in flight must discard that epoch's result.
        FakeModel.mode = 'blocked'
        FakeModel.entered.clear(); FakeModel.release.clear()
        send()
        wait(FakeModel.entered.is_set)
        with node.condition:
            node.clock(rospy.Time(1),time.monotonic())
            self.assertEqual(node.accepted_stamp,rospy.Time())
            self.assertFalse(node.transport_ok)
        FakeModel.release.set()
        wait(lambda: node.inference_start is None)
        self.assertEqual(len(observations),count)
        FakeModel.mode = 'ok'
        wait(lambda: node.transport_ok)
        send()
        wait(lambda: len(observations)>count)
        self.assertTrue(observations[-1].objects_valid)
        wait(lambda: any(s.state==s.FAULT for s in statuses))
        self.assertTrue(all(not s.ready and s.stop_required for s in statuses))
        stop.set()
        wait(lambda: statuses and 'no recent' in statuses[-1].reason,
             seconds=node.watchdog + 2 * node.period + .2)
        self.assertTrue(subs)

    def test_model_load_failure_has_fault_heartbeat(self):
        def fail(*args): raise RuntimeError('test missing checkpoint')
        node = LearnedNode(fail)
        self.addCleanup(node.shutdown)
        end = time.monotonic()+2
        while node.health.state != ComponentStatus.FAULT and time.monotonic()<end:
            time.sleep(.01)
        self.assertFalse(node.model_ready)
        self.assertIn('model load failed',node.health.reason)
        time.sleep(node.period*1.2)
        self.assertNotEqual(node.health.header.stamp,rospy.Time())
        self.assertFalse(node.health.ready)

if __name__ == '__main__':
    rospy.init_node('learned_lidar_test')
    rostest.rosrun('lidar_perception_pkg','learned_lidar_test',NodeTest)
