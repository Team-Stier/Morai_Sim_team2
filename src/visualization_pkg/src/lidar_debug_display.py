#!/usr/bin/env python3
"""Package-internal, read-only lidar_link view without requiring localization."""
import threading
import time
from types import SimpleNamespace

import rospy
import tf2_ros
from common_msgs_pkg.msg import LidarObservationArray
from visualization_msgs.msg import MarkerArray
from visualization_pkg.lidar_display import LidarDisplay


def main():
    rospy.init_node('lidar_debug_display')
    config = SimpleNamespace(reference_frame='lidar_link',
                             display_timeout_sec=float(rospy.get_param('~display_timeout_sec')),
                             clock_stall_sec=float(rospy.get_param('~clock_stall_sec')))
    if not 0 < config.display_timeout_sec <= 10 or not 0 < config.clock_stall_sec <= 10:
        raise ValueError('invalid debug display timeouts')
    publisher = rospy.Publisher('/molit/internal/visualization/lidar_markers',MarkerArray,queue_size=2,latch=True)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    display = LidarDisplay(config,buffer,publisher)
    lock = threading.Lock()
    def ingest(msg):
        with lock:
            display.ingest(msg,rospy.Time.now(),time.monotonic())
    subscriber = rospy.Subscriber('/molit/perception/lidar/observations',LidarObservationArray,ingest,queue_size=2)
    try:
        while not rospy.is_shutdown():
            with lock:
                display.update(rospy.Time.now(),time.monotonic())
            time.sleep(.05)
    finally:
        with lock:
            display.clear()
        subscriber.unregister()
    return listener


if __name__ == '__main__':
    main()
