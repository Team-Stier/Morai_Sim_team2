#!/usr/bin/env python3
import rospy
from lidar_perception_pkg.learned_node import LearnedNode

if __name__ == '__main__':
    rospy.init_node('lidar_perception_node')
    node = LearnedNode()
    rospy.spin()
