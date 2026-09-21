#!/usr/bin/env python3
"""Route-only trajectory adapter. Uses real Localization estimates, not route-derived ego pose."""
import math
import message_filters
import rospy
from common_msgs_pkg.msg import ComponentStatus, EgoState, Trajectory
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry, Path


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Planner:
    def __init__(self):
        self.path = None
        self.state = None
        self.speed = rospy.get_param('~speed_mps')
        self.forward_points = rospy.get_param('~forward_points')
        self.behind_points = rospy.get_param('~behind_points')
        self.path_sub = rospy.Subscriber('/molit/route/global_path', Path, self.on_path, queue_size=1)
        self.ego_sub = message_filters.Subscriber('/molit/localization/ego_state', EgoState)
        self.odom_sub = message_filters.Subscriber('/molit/localization/local/odometry', Odometry)
        self.sync = message_filters.TimeSynchronizer([self.ego_sub, self.odom_sub], 30)
        self.sync.registerCallback(self.on_state)
        self.trajectory = rospy.Publisher('/molit/planning/trajectory', Trajectory, queue_size=2)
        self.status = rospy.Publisher('/molit/planning/status', ComponentStatus, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update)

    def on_path(self, message):
        self.path = message.poses

    def on_state(self, ego, odom):
        self.state = ego, odom

    def update(self, _):
        if self.path is None or self.state is None:
            return
        ego, odom = self.state
        position = ego.pose.pose.position
        nearest = min(range(len(self.path)), key=lambda i:
                      (self.path[i].pose.position.x-position.x)**2 +
                      (self.path[i].pose.position.y-position.y)**2)
        rotation = yaw(odom.pose.pose.orientation)-yaw(ego.pose.pose.orientation)
        cosine, sine = math.cos(rotation), math.sin(rotation)
        output = Trajectory()
        output.header.stamp = rospy.Time.now()
        output.header.frame_id = 'odom'
        output.reset_id = ego.reset_id
        output.valid = True
        output.valid_for = rospy.Duration(0.5)
        distance = 0.0
        previous = None
        for offset in range(-self.behind_points, self.forward_points):
            source = self.path[(nearest+offset) % len(self.path)].pose
            dx, dy = source.position.x-position.x, source.position.y-position.y
            pose = Pose()
            pose.position.x = odom.pose.pose.position.x+cosine*dx-sine*dy
            pose.position.y = odom.pose.pose.position.y+sine*dx+cosine*dy
            pose.position.z = odom.pose.pose.position.z+source.position.z-position.z
            heading = yaw(source.orientation)+rotation
            pose.orientation.z, pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
            if previous is not None:
                distance += math.hypot(pose.position.x-previous.x, pose.position.y-previous.y)
            previous = pose.position
            output.poses.append(pose)
            output.speed_mps.append(self.speed)
            output.time_from_start.append(rospy.Duration(distance/self.speed))
        self.trajectory.publish(output)
        status = ComponentStatus()
        status.header.stamp = output.header.stamp
        status.component = 'path_planning_pkg'
        status.state = ComponentStatus.READY
        status.ready = True
        status.data_stamp = output.header.stamp
        status.reason = 'global_path_only development demonstration; nearest_index=%d' % nearest
        self.status.publish(status)


if __name__ == '__main__':
    rospy.init_node('path_planner_node')
    planner = Planner()
    rospy.spin()
