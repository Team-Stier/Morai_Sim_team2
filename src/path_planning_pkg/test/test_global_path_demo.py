import importlib.util
import math
import unittest
from pathlib import Path
from unittest.mock import patch

import rospy
from common_msgs_pkg.msg import EgoState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from hd_map_pkg.course_speed import CourseSpeedZones

SOURCE = Path(__file__).resolve().parents[1] / 'src/global_path_demo.py'
spec = importlib.util.spec_from_file_location('global_path_demo', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Output:
    def publish(self, value):
        self.value = value


class GlobalPathDemoTest(unittest.TestCase):
    def planner(self):
        node = module.Planner.__new__(module.Planner)
        node.speeds = [2.0]*20
        node.forward_points = 5
        node.behind_points = 2
        node.trajectory = Output()
        node.status = Output()
        node.path = []
        for i in range(20):
            p = PoseStamped()
            p.pose.position.x = float(i)
            p.pose.orientation.w = 1.0
            node.path.append(p)
        node.zones = CourseSpeedZones([[float(i), 0] for i in range(20)], {
            'normal_limit_kph': 50, 'high_speed': {'start_map_xy': [10, 0], 'end_map_xy': [15, 0]}})
        ego = EgoState()
        ego.pose.pose.position.x = 10.0
        ego.pose.pose.orientation.w = 1.0
        ego.reset_id = 7
        odom = Odometry()
        odom.pose.pose.position.x = 100.0
        odom.pose.pose.position.y = 200.0
        odom.pose.pose.orientation.z = math.sqrt(0.5)
        odom.pose.pose.orientation.w = math.sqrt(0.5)
        node.state = ego, odom
        return node

    def test_map_route_becomes_odom_trajectory_at_matching_ego_pose(self):
        node = self.planner()
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(30)):
            node.update(None)
        value = node.trajectory.value
        self.assertEqual(value.header.frame_id, 'odom')
        self.assertEqual(value.reset_id, 7)
        self.assertEqual(len(value.poses), 7)
        self.assertEqual(value.speed_mps, [2.0]*7)
        for i, pose in enumerate(value.poses):
            self.assertAlmostEqual(pose.position.x, 100.0)
            self.assertAlmostEqual(pose.position.y, 198.0+i)
            self.assertAlmostEqual(module.yaw(pose.orientation), math.pi/2)
            self.assertAlmostEqual(value.time_from_start[i].to_sec(), i/2)
        self.assertTrue(value.valid)
        self.assertTrue(node.status.value.ready)
        self.assertEqual(node.status.value.data_stamp, value.header.stamp)

    def test_closed_route_selects_last_points_before_first_point(self):
        node = self.planner()
        node.state[0].pose.pose.position.x = 0.0
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(30)):
            node.update(None)
        self.assertAlmostEqual(node.trajectory.value.poses[0].position.y, 218.0)
        self.assertAlmostEqual(node.trajectory.value.poses[2].position.y, 200.0)

    def test_current_speed_at_exit_is_not_taken_from_points_behind_vehicle(self):
        node = self.planner()
        node.speeds = [100/3.6]*10 + [48/3.6]*10
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(30)):
            node.update(None)
        value = node.trajectory.value
        self.assertEqual(value.speed_mps[:3], [48/3.6]*3)
        self.assertEqual(value.time_from_start[0], rospy.Duration(0))
        self.assertTrue(all(b > a for a, b in zip(value.time_from_start, value.time_from_start[1:])))


if __name__ == '__main__':
    unittest.main()
