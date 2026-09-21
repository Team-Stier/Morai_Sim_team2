#!/usr/bin/env python3
"""Approved Route/World Model -> odom trajectory, without raw sensor input."""
import copy
import json
import math
import time
import threading

import message_filters
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from common_msgs_pkg.msg import ComponentStatus, EgoState, LocalizationStatus, RouteContext, Trajectory, WorldModel
from path_planning_pkg.frenet import Planner, Lane, Window, Obstacle, Candidate, geometry


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Node:
    def __init__(self):
        self.c = rospy.get_param('~')
        self.planner = Planner(self.c)
        self.route = self.world = self.state = self.selected = None
        self.selected_stamp = None
        self.epoch = None
        self.map_id = None
        self.boundaries = []
        self.route_status = self.world_status = self.localization_status = None
        self.lock = threading.Lock()
        self.trajectory = rospy.Publisher('/molit/planning/trajectory', Trajectory, queue_size=2)
        self.status = rospy.Publisher('/molit/planning/status', ComponentStatus, queue_size=1, latch=True)
        self.audit = rospy.Publisher('~candidate_costs', String, queue_size=1)
        rospy.Subscriber('/molit/route/context', RouteContext, lambda m:setattr(self, 'route', m), queue_size=1)
        rospy.Subscriber('/molit/world_model/scene', WorldModel, lambda m:setattr(self, 'world', m), queue_size=1)
        rospy.Subscriber('/molit/route/status', ComponentStatus, lambda m:setattr(self,'route_status',m),queue_size=1)
        rospy.Subscriber('/molit/world_model/status', ComponentStatus, lambda m:setattr(self,'world_status',m),queue_size=1)
        rospy.Subscriber('/molit/localization/status', LocalizationStatus, lambda m:setattr(self,'localization_status',m),queue_size=1)
        self.ego_sub = message_filters.Subscriber('/molit/localization/ego_state', EgoState)
        self.odom_sub = message_filters.Subscriber('/molit/localization/local/odometry', Odometry)
        self.sync = message_filters.TimeSynchronizer([self.ego_sub, self.odom_sub], 30)
        self.sync.registerCallback(self.on_state)
        rospy.Timer(rospy.Duration(1/self.c['decision_rate_hz']), self.plan)
        rospy.Timer(rospy.Duration(1/self.c['trajectory_rate_hz']), self.publish)

    def on_state(self, ego, odom):
        self.state = ego, odom

    def report(self, reason, ready=False, latency=0.):
        m = ComponentStatus(component='path_planning_pkg', state=ComponentStatus.READY if ready else ComponentStatus.DEGRADED,
                            ready=ready, stop_required=not ready, processing_latency_sec=latency, reason=reason)
        m.header.stamp = rospy.Time.now()
        if self.state:
            m.data_stamp = self.state[0].header.stamp
            m.data_age_sec = (m.header.stamp-m.data_stamp).to_sec()
        self.status.publish(m)

    def plan(self, _):
        started = time.monotonic()
        route, world, state = self.route, self.world, self.state
        if route is None or world is None or state is None:
            self.report('waiting_for_route_world_localization')
            return
        ego, odom = state
        now = rospy.Time.now()
        if self.route_status is not None and self.route_status.state == ComponentStatus.FAULT:
            with self.lock:
                self.selected = None
            self.report('required_checkpoint_missed',True)
            return
        if (not world.objects_valid or world.localization_reset_id != ego.reset_id or
                any(not 0 <= (now-stamp).to_sec() <= self.c['input_age_sec'] for stamp in
                    (ego.header.stamp, world.header.stamp, route.header.stamp))):
            with self.lock:
                self.selected = None
            self.report('unusable_or_stale_planning_inputs')
            return
        if self.epoch != ego.reset_id:
            self.planner.committed = self.planner.pending = None
            self.epoch = ego.reset_id
        lanes = {}
        for lane in route.lanes:
            xyz = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in lane.centerline.poses])
            s = np.asarray(lane.route_s)
            keep = np.r_[True, np.diff(s) > 1e-6]
            lanes[lane.id] = Lane(lane.id, xyz[keep], s[keep], np.asarray(lane.speed_limits_mps)[keep], list(lane.successors))
        windows = []
        for w in route.lane_changes:
            lane = lanes[w.source_lane]
            local = np.r_[0., np.cumsum(np.linalg.norm(np.diff(lane.xy[:, :2], axis=0), axis=1))]
            begin, end = ((w.source_s_start, w.source_s_end) if w.source_lane == 'global_route' else
                          np.interp([w.source_s_start, w.source_s_end], local, lane.s))
            windows.append(Window(w.source_lane, w.target_lane, begin, end))
        p = ego.pose.pose.position
        position = np.array([p.x, p.y, p.z])
        speed = max(0., odom.twist.twist.linear.x)
        if route.route_complete or lanes['global_route'].s[-1]-route.progress < 2*self.c['spatial_step_m']:
            with self.lock:
                self.selected = None
            self.report('route_endpoint_stop', True)
            return
        goal_s = max(route.comparison_goal_s, route.progress+1.)
        candidates = self.planner.candidates(lanes, windows, route.current_lane, route.progress,
                                              goal_s, position, yaw(ego.pose.pose.orientation), speed)
        committed = self.planner.committed
        if committed is not None:
            end_s = committed.change_end
            if route.progress < end_s:
                i = min(np.searchsorted(committed.route_s, route.progress), len(committed.route_s)-3)
                held = Candidate('committed', committed.target, committed.xy[i:].copy(), committed.route_s[i:].copy(),
                                 committed.limits[i:].copy(), committed.changes, committed.change_end, committed.return_start)
                held.xy[0] = position
                candidates.append(held)
            else:
                self.planner.committed = None
        objects = [Obstacle(np.array([[p.x, p.y, p.z] for p in o.points]),
                            np.array([o.twist.linear.x, o.twist.linear.y]), o.velocity_valid,
                            (now-o.source_stamp).to_sec()) for o in world.objects]
        if self.map_id != route.map_id:
            self.map_id = route.map_id
            self.boundaries = [np.array([[p.x,p.y,p.z] for p in line.points]) for line in route.forbidden_boundaries]
        boundaries = self.boundaries
        for candidate in candidates:
            self.planner.evaluate(candidate, speed, objects, boundaries, goal_s)
            index = min(np.searchsorted(candidate.route_s,goal_s),len(candidate.xy)-1)
            goal = route.comparison_goal
            if np.linalg.norm(candidate.xy[index,:2]-[goal.x,goal.y]) > self.c['checkpoint_radius_m']:
                candidate.cost = candidate.eta = math.inf
                candidate.feasible = False
                candidate.reason = 'does_not_reach_common_checkpoint'
        chosen = self.planner.select(candidates, now.to_sec(), route.progress)
        with self.lock:
            self.selected, self.selected_stamp = chosen, now
        audit = [{'key':x.key, 'target':x.target, 'feasible':x.feasible, 'reason':x.reason,
                  'eta':x.eta if math.isfinite(x.eta) else None,
                  'cost':x.cost if math.isfinite(x.cost) else None,
                  'comfort':x.comfort, 'changes':x.changes} for x in candidates]
        self.audit.publish(String(data=json.dumps(audit)))
        self.report('frenet; selected=%s; candidates=%d; observed_clusters_only; unverified_development'%
                    (chosen.key if chosen else 'stop', len(candidates)), True, time.monotonic()-started)

    def publish(self, _):
        if self.state is None:
            return
        ego, odom = self.state
        with self.lock:
            chosen, stamp = self.selected, self.selected_stamp
        now = rospy.Time.now()
        output = Trajectory()
        output.header.stamp, output.header.frame_id = now, 'odom'
        output.reset_id = ego.reset_id
        output.valid_for = rospy.Duration(self.c['trajectory_valid_for_sec'])
        if chosen is None or stamp is None or (now-stamp).to_sec() > self.c['plan_retention_sec']:
            output.valid = True
            output.stop_required = True
            output.poses = [copy.deepcopy(odom.pose.pose), copy.deepcopy(odom.pose.pose)]
            output.speed_mps = [0., 0.]
            output.time_from_start = [rospy.Duration(0), rospy.Duration(1)]
            self.trajectory.publish(output)
            return
        p = ego.pose.pose.position
        position = np.array([p.x, p.y, p.z])
        i = int(np.argmin(np.linalg.norm(chosen.xy[:, :2]-position[:2], axis=1)))
        finite = np.flatnonzero(np.isfinite(chosen.times))
        end = max(i+2, min(len(chosen.xy), int(finite[-1])+1))
        xyz, speeds = chosen.xy[i:end].copy(), chosen.speed[i:end].copy()
        xyz[0] = position
        # Relative map->odom transform from a synchronized estimate pair.
        rotation = yaw(odom.pose.pose.orientation)-yaw(ego.pose.pose.orientation)
        co, si = math.cos(rotation), math.sin(rotation)
        ds, headings, _ = geometry(xyz)
        headings[0] = yaw(ego.pose.pose.orientation)
        times = np.zeros(len(xyz))
        for j in range(1, len(xyz)):
            times[j] = times[j-1]+2*(ds[j]-ds[j-1])/max(speeds[j]+speeds[j-1], .01)
        for j, point in enumerate(xyz):
            dx, dy = point[:2]-position[:2]
            pose = Pose()
            pose.position.x = odom.pose.pose.position.x+co*dx-si*dy
            pose.position.y = odom.pose.pose.position.y+si*dx+co*dy
            pose.position.z = odom.pose.pose.position.z+point[2]-position[2]
            angle = headings[j]+rotation
            pose.orientation.z, pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
            output.poses.append(pose)
        output.speed_mps = list(speeds)
        output.time_from_start = [rospy.Duration(float(t)) for t in times]
        output.valid = True
        output.stop_required = bool(np.max(speeds) < .01)
        self.trajectory.publish(output)


if __name__ == '__main__':
    rospy.init_node('path_planner_node')
    Node()
    rospy.spin()
