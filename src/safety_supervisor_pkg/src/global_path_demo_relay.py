#!/usr/bin/env python3
"""Transparent relay used ONLY by the user-authorized global_path_only simulator profile."""
import rospy
from common_msgs_pkg.msg import ActuatorCommand

rospy.init_node('safety_supervisor_node')
publisher = rospy.Publisher('/molit/safety/final_command', ActuatorCommand, queue_size=2)
subscriber = rospy.Subscriber('/molit/control/nominal_command', ActuatorCommand, publisher.publish, queue_size=2)
rospy.spin()
