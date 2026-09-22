#!/usr/bin/env python3
"""Publish route geometry and ordered progress from approved map/localization I/O."""
import math

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from common_msgs_pkg.msg import ComponentStatus, EgoState, HdMap, LocalizationStatus, RouteContext

from global_route_manager_pkg.progress import RouteProgress


class RouteManagerNode:
    def __init__(self):
        self.map = self.map_status = self.localization_status = self.ego = None
        self.progress = None
        self.context = None
        self.last_stamp = None
        self.reset_id = None
        self.config = {key: rospy.get_param('~'+key) for key in (
            'initialize_from_current_position', 'matching_backward_m', 'matching_forward_m',
            'rddf_geometry_only', 'comparison_distance_m', 'route_completion_tolerance_m', 'loop_route')}
        self.path_publisher = rospy.Publisher('/molit/route/global_path', Path, queue_size=1, latch=True)
        self.context_publisher = rospy.Publisher('/molit/route/context', RouteContext, queue_size=2)
        self.status_publisher = rospy.Publisher('/molit/route/status', ComponentStatus, queue_size=1, latch=True)
        rospy.Subscriber('/molit/map/hd_map', HdMap, self.on_map, queue_size=1)
        rospy.Subscriber('/molit/map/status', ComponentStatus, self.on_map_status, queue_size=1)
        rospy.Subscriber('/molit/localization/ego_state', EgoState, self.on_ego, queue_size=2)
        rospy.Subscriber('/molit/localization/status', LocalizationStatus, self.on_localization_status, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/rospy.get_param('~context_rate_hz')), self.publish_context)
        rospy.Timer(rospy.Duration(0.5), self.publish_status)

    def on_map(self, message):
        lanes = [dict(id=lane.id, points=[(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                                        for pose in lane.centerline.poses], route_s=lane.route_s)
                 for lane in message.lanes]
        if self.config['rddf_geometry_only']:
            checkpoints, radius = [], 0.
        else:
            checkpoints = [(point.x, point.y, point.z) for point in message.checkpoints]
            radius = message.checkpoint_radius_m
        self.progress = RouteProgress(lanes, checkpoints, radius, self.config)
        self.map = message
        self.last_stamp = None
        path = next(lane.centerline for lane in message.lanes if lane.id == 'global_route')
        self.path_publisher.publish(path)

    def on_map_status(self, message):
        self.map_status = message

    def on_localization_status(self, message):
        self.localization_status = message

    def on_ego(self, message):
        self.ego = message

    def inputs_ready(self):
        return (self.map is not None and self.map_status is not None and self.map_status.ready
                and self.ego is not None and all(self.ego.pose_valid[index] for index in (0, 1, 5))
                and self.localization_status is not None and self.localization_status.local_odometry_valid)

    def publish_context(self, _event):
        if not self.inputs_ready() or self.ego.header.stamp == self.last_stamp:
            return
        ego = self.ego
        if self.reset_id != ego.reset_id:
            self.progress.previous_point = None
            if self.config['rddf_geometry_only']:
                self.progress.progress = None
            self.reset_id = ego.reset_id
        pose = ego.pose.pose
        q = pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        state = self.progress.update((pose.position.x, pose.position.y, pose.position.z), yaw)
        message = RouteContext(header=ego.header, map_id=self.map.map_id,
                               reference_sha256=self.map.reference_sha256)
        for field in ('current_lane', 'progress', 'next_checkpoint', 'comparison_goal_s', 'route_complete'):
            setattr(message, field, state[field])
        message.comparison_goal = Point(*state['comparison_goal'])
        self.context = message
        self.last_stamp = ego.header.stamp
        self.context_publisher.publish(message)

    def publish_status(self, _event):
        ready = self.inputs_ready() and self.context is not None and not self.progress.missed_checkpoint
        status = ComponentStatus(component='global_route_manager_pkg', ready=ready,
                                 state=ComponentStatus.READY if ready else ComponentStatus.INITIALIZING,
                                 stop_required=not ready, processing_latency_sec=-1.0,
                                 data_age_sec=-1.0)
        status.header.stamp = rospy.Time.now()
        if self.context is not None:
            status.data_stamp = self.context.header.stamp
            status.data_age_sec = (status.header.stamp-status.data_stamp).to_sec()
            status.processed_count = len(self.progress.passed_checkpoints)
            if self.config['rddf_geometry_only']:
                status.reason = 'rddf_geometry_only; lane={}; progress={:.2f}; comparison_s={:.2f}; checkpoint_enforcement=off'.format(
                    self.context.current_lane, self.context.progress, self.context.comparison_goal_s)
            else:
                status.reason = 'lane={}; progress={:.2f}; rddf={}; windows={}; next_checkpoint={}; development_start_checkpoint={}; passed={}'.format(
                    self.context.current_lane, self.context.progress,
                    len(self.map.lanes)-1, len(self.map.lane_changes), self.context.next_checkpoint,
                    self.progress.start_checkpoint_index, self.progress.passed_checkpoints)
            if self.progress.missed_checkpoint:
                status.state = ComponentStatus.FAULT
                status.reason += '; required checkpoint was not crossed within its radius'
        else:
            status.reason = 'waiting for map and usable localization estimates'
        self.status_publisher.publish(status)


def main():
    rospy.init_node('global_route_manager_node')
    RouteManagerNode()
    rospy.spin()


if __name__ == '__main__':
    main()
