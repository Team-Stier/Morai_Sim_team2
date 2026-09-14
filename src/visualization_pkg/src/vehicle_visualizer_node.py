#!/usr/bin/env python3
"""Display approved localization in RViz; never estimate pose or broadcast TF."""

import threading
import time
from dataclasses import fields

import rospy
import tf2_ros
from common_msgs_pkg.msg import LidarObservationArray
from visualization_pkg.lidar_display import LidarDisplay
from common_msgs_pkg.msg import EgoState, LocalizationStatus
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

from visualization_pkg.markers import render_markers
from visualization_pkg.vehicle_display import DisplayConfig, DisplayState, VehicleDisplay, stamp_ns


class VehicleVisualizerNode:
    def __init__(self):
        defaults = DisplayConfig()
        params = {field.name: rospy.get_param('~' + field.name, getattr(defaults, field.name))
                  for field in fields(DisplayConfig)}
        self.config = DisplayConfig(**params)
        self.display = VehicleDisplay(self.config)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._last_render_key = None
        self._last_valid_display = None
        self.publisher = rospy.Publisher('/molit/internal/visualization/vehicle_markers',
                                         MarkerArray, queue_size=100, latch=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.lidar_publisher = rospy.Publisher(
            "/molit/internal/visualization/lidar_markers", MarkerArray, queue_size=2, latch=True)
        self.lidar_display = LidarDisplay(self.config, self.tf_buffer, self.lidar_publisher)
        self._lidar_reset_id = None
        self.map_publisher = None
        if rospy.get_param("~show_hd_map", True):
            from visualization_pkg.hd_map_display import load_map_markers
            self.map_publisher = rospy.Publisher("/molit/internal/visualization/hd_map_markers",
                                                MarkerArray, queue_size=1, latch=True)
            rospy.loginfo("Loading HD map for RViz in the central map frame")
            try:
                self.map_publisher.publish(load_map_markers())
            except (OSError, ValueError, KeyError, TypeError) as error:
                rospy.logerr("HD map display unavailable: %s", error)
        self.subscribers = [
            rospy.Subscriber("/molit/perception/lidar/observations", LidarObservationArray,
                             self._lidar, queue_size=2),
            rospy.Subscriber('/molit/localization/ego_state', EgoState,
                             self._ego, queue_size=100, tcp_nodelay=True),
            rospy.Subscriber('/molit/localization/local/odometry', Odometry,
                             self._odometry, queue_size=100, tcp_nodelay=True),
            rospy.Subscriber('/molit/localization/status', LocalizationStatus,
                             self._status, queue_size=100, tcp_nodelay=True),
        ]
        # A wall worker keeps deleting stale geometry even when /clock is paused.
        self._worker = threading.Thread(target=self._run, name='vehicle_display_wall_watchdog')
        self._worker.daemon = True
        rospy.on_shutdown(self.shutdown)
        self._worker.start()
        rospy.loginfo('Vehicle display frame=%s; waiting for valid localization/status pair',
                      self.config.reference_frame)

    def _ingest(self, method, message):
        with self._lock:
            method(message, stamp_ns(rospy.Time.now()), time.monotonic())
            self._publish_changed()

    def _lidar(self, message):
        with self._lock:
            self.lidar_display.ingest(message, rospy.Time.now(), time.monotonic())

    def _ego(self, message):
        self._ingest(self.display.ingest_ego, message)

    def _odometry(self, message):
        self._ingest(self.display.ingest_odometry, message)

    def _status(self, message):
        with self._lock:
            if self._lidar_reset_id is not None and message.reset_id != self._lidar_reset_id:
                self.lidar_display.clear()
            self._lidar_reset_id = message.reset_id
            self.display.ingest_status(message, stamp_ns(rospy.Time.now()), time.monotonic())
            self._publish_changed()

    def _publish_changed(self):
        now_ns = stamp_ns(rospy.Time.now())
        self.lidar_display.update(rospy.Time.now(), time.monotonic())
        state = self.display.evaluate(now_ns, time.monotonic())
        wall = time.monotonic()
        previous = self._last_valid_display
        if (state.reason == 'awaiting exact estimate/status stamp and reset match'
                and previous is not None):
            old_state, epoch, received = previous
            if (epoch == self.display._epoch
                    and 0 <= (now_ns - old_state.stamp_ns) * 1e-9 <= self.config.display_timeout_sec
                    and 0 <= wall - received <= self.config.display_timeout_sec):
                return
        if state.valid:
            if previous is None or previous[0].stamp_ns != state.stamp_ns:
                self._last_valid_display = (state, self.display._epoch, wall)
        else:
            self._last_valid_display = None
        key = (state.valid, state.stamp_ns, state.reason, state.stop_required)
        if key == self._last_render_key:
            return
        self.publisher.publish(render_markers(state, self.config, now_ns))
        self._last_render_key = key

    def _run(self):
        # This timer detects stale input; valid pose updates are callback-driven.
        interval = 1.0 / self.config.marker_publish_rate_hz
        while not self._stop.is_set() and not rospy.is_shutdown():
            with self._lock:
                self._publish_changed()
            self._stop.wait(interval)

    def shutdown(self):
        self._stop.set()
        if threading.current_thread() is not self._worker:
            self._worker.join(timeout=1.0)
        # A final explicit delete also works when RViz's ROS clock is paused.
        try:
            self.lidar_display.clear()
            state = DisplayState(False, 'visualizer stopped', self.config.reference_frame)
            self.publisher.publish(render_markers(state, self.config, stamp_ns(rospy.Time.now())))
        except rospy.ROSException:
            pass


def main():
    rospy.init_node('vehicle_visualizer_node', anonymous=False)
    try:
        VehicleVisualizerNode()
    except (ValueError, TypeError) as error:
        rospy.logfatal('Invalid vehicle display configuration: %s', error)
        raise
    rospy.spin()


if __name__ == '__main__':
    main()
