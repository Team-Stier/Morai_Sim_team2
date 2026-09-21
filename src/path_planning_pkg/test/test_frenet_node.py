import importlib.util
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

import numpy as np
import rospy
import yaml
from common_msgs_pkg.msg import EgoState
from nav_msgs.msg import Odometry
from path_planning_pkg.frenet import Candidate


spec = importlib.util.spec_from_file_location('frenet_node', Path(__file__).parents[1]/'src/frenet_planner_node.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Output:
    def publish(self, message):
        self.message = message


class FrenetOutputTest(unittest.TestCase):
    @patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(100.75))
    def test_plan_survives_input_age_but_expires_after_retention(self, _):
        node = self.node()
        self.assertEqual(node.c['input_age_sec'], 0.5)
        node.publish(None)
        self.assertFalse(node.trajectory.message.stop_required)
        with patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(101.01)):
            node.publish(None)
        self.assertTrue(node.trajectory.message.stop_required)

    def node(self):
        node = module.Node.__new__(module.Node)
        node.c = yaml.safe_load((Path(__file__).parents[1]/'config/frenet_planner.yaml').read_text())
        ego, odom = EgoState(), Odometry()
        ego.reset_id = 12
        ego.pose.pose.position.x = 10.
        ego.pose.pose.orientation.w = 1.
        odom.pose.pose.position.x = 100.
        odom.pose.pose.position.y = 200.
        odom.pose.pose.orientation.z = odom.pose.pose.orientation.w = np.sqrt(.5)
        node.state = ego, odom
        node.lock = threading.Lock()
        node.trajectory = Output()
        node.path_markers = Output()
        node.selected_stamp = rospy.Time(100)
        node.selected = Candidate('keep', 'global_route', np.array([[10.,0.,0.],[11.,0.,0.],[12.,0.,0.],[13.,0.,0.]]),
                                  np.arange(4.), np.full(4, 2.), speed=np.array([2.,1.,0.,0.]),
                                  times=np.array([0.,2/3,8/3,np.inf]))
        return node

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_future_stop_preserves_frame_epoch_and_monotonic_time(self, _):
        node = self.node()
        node.publish(None)
        message = node.trajectory.message
        self.assertEqual(message.header.frame_id, 'odom')
        self.assertEqual(message.reset_id, 12)
        self.assertTrue(message.valid)
        self.assertFalse(message.stop_required)
        self.assertEqual(message.speed_mps[-1], 0.)
        self.assertAlmostEqual(message.poses[0].position.x, 100.)
        self.assertAlmostEqual(message.poses[0].position.y, 200.)
        self.assertAlmostEqual(message.poses[-1].position.y, 202.)
        self.assertEqual(len(message.poses), len(message.speed_mps))
        self.assertEqual(len(message.poses), len(message.time_from_start))
        self.assertTrue(np.all(np.diff([x.to_sec() for x in message.time_from_start])>0))

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_no_candidate_stops_at_current_odometry_pose(self, _):
        node = self.node()
        node.selected = None
        node.publish(None)
        message = node.trajectory.message
        self.assertTrue(message.valid and message.stop_required)
        self.assertEqual(message.speed_mps, [0., 0.])
        self.assertEqual(message.poses[0], node.state[1].pose.pose)


if __name__ == '__main__':
    unittest.main()
