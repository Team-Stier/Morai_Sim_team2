#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest
from common_msgs_pkg.msg import ActuatorCommand, ComponentStatus, ControllerStatus, LocalizationStatus, Trajectory
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


class ControllerPipelineTest(unittest.TestCase):
    def setUp(self):
        self.command = None
        self.status = None
        self.lock = threading.Lock()
        self.odom_pub = rospy.Publisher('/molit/localization/local/odometry', Odometry, queue_size=2)
        self.loc_pub = rospy.Publisher('/molit/localization/status', LocalizationStatus, queue_size=2)
        self.traj_pub = rospy.Publisher('/molit/planning/trajectory', Trajectory, queue_size=2)
        self.plan_pub = rospy.Publisher('/molit/planning/status', ComponentStatus, queue_size=2)
        self.command_sub = rospy.Subscriber('/molit/control/nominal_command', ActuatorCommand, self.on_command)
        self.status_sub = rospy.Subscriber('/molit/control/status', ControllerStatus, self.on_status)

    def on_command(self, message):
        with self.lock:
            self.command = message

    def on_status(self, message):
        with self.lock:
            self.status = message

    def inputs(self, speed=1.0, target=2.0, stop=False, localization_stop=False):
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = speed
        loc = LocalizationStatus()
        loc.header.stamp = stamp
        loc.local_odometry_stamp = stamp
        loc.local_odometry_valid = True
        loc.mode = LocalizationStatus.DEAD_RECKONING
        loc.gps_fix_valid = False
        loc.stop_required = localization_stop
        loc.reset_id = 4
        traj = Trajectory()
        traj.header.stamp = stamp
        traj.header.frame_id = 'odom'
        traj.reset_id = 4
        traj.valid = True
        traj.valid_for = rospy.Duration(0.5)
        traj.stop_required = stop
        if not stop:
            for i in range(21):
                pose = Pose()
                pose.position.x = i
                pose.orientation.w = 1.0
                traj.poses.append(pose)
                traj.speed_mps.append(target)
                traj.time_from_start.append(rospy.Duration(i*0.1))
        plan = ComponentStatus()
        plan.header.stamp = stamp
        plan.component = 'path_planning_pkg'
        plan.state = ComponentStatus.READY
        plan.ready = True
        plan.data_stamp = stamp
        self.odom_pub.publish(odom)
        self.loc_pub.publish(loc)
        self.traj_pub.publish(traj)
        self.plan_pub.publish(plan)

    def wait_for(self, predicate, **inputs):
        deadline = time.monotonic()+6
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.inputs(**inputs)
            with self.lock:
                if self.command and self.status and predicate(self.command, self.status):
                    return self.command, self.status
            time.sleep(0.03)
        self.fail('Expected controller output not received')

    def test_ros_inputs_to_nominal_command_and_status(self):
        command, status = self.wait_for(lambda c, s: c.valid and c.accel > 0 and s.ready)
        self.assertEqual(command.header.frame_id, 'base_link')
        self.assertEqual(command.gear, ActuatorCommand.GEAR_DRIVE)
        self.assertEqual(command.brake, 0.0)
        self.assertEqual(status.reset_id, 4)
        self.assertEqual(status.mode, ControllerStatus.PURE_PURSUIT)
        self.assertEqual(status.target_speed_mps, 2.0)
        self.assertFalse(status.odometry_stamp.is_zero())
        self.assertFalse(status.trajectory_stamp.is_zero())
        self.assertGreaterEqual(command.header.stamp, status.trajectory_stamp)
        self.wait_for(lambda c, s: c.valid and c.brake > 0 and c.accel == 0,
                      speed=3.0, target=1.0)
        self.wait_for(lambda c, s: c.valid and c.brake >= 0.35 and s.stop_required,
                      speed=0.0, stop=True)
        self.wait_for(lambda c, s: not c.valid and c.accel == 0 and s.reason == 'upstream_stop_required',
                      localization_stop=True)


if __name__ == '__main__':
    rospy.init_node('controller_pipeline_test')
    rostest.rosrun('vehicle_control_pkg', 'controller_pipeline', ControllerPipelineTest)
