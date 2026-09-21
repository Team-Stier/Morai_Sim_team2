#!/usr/bin/env python3
"""Global path source for the centrally declared global_path_only simulator profile."""
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

rospy.init_node('global_route_manager_node')
path = Path()
path.header.frame_id = 'map'
path.header.stamp = rospy.Time.now()
with open(rospy.get_param('~path_file')) as stream:
    points = [tuple(map(float, line.split())) for line in stream if line.strip()]
# The supplied file repeats points; retain one copy when forming path segments.
points = [point for index, point in enumerate(points) if index == 0 or point != points[index-1]]
if points[-1] == points[0]:
    points.pop()
for index, point in enumerate(points):
    following = points[(index+1) % len(points)]
    yaw = math.atan2(following[1]-point[1], following[0]-point[0])
    pose = PoseStamped()
    pose.header = path.header
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
    pose.pose.orientation.z = math.sin(yaw/2)
    pose.pose.orientation.w = math.cos(yaw/2)
    path.poses.append(pose)
publisher = rospy.Publisher('/molit/route/global_path', Path, queue_size=1, latch=True)
publisher.publish(path)
rospy.loginfo('Global path demo: %d points from %s', len(points), rospy.get_param('~path_file'))
rospy.spin()
