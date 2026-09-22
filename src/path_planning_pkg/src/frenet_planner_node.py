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
from visualization_msgs.msg import Marker, MarkerArray
from common_msgs_pkg.msg import ComponentStatus, EgoState, HdMap, LocalizationStatus, RouteContext, Trajectory, WorldModel
from path_planning_pkg.frenet import Planner, Lane, Window, Obstacle, ObstacleGrid, Candidate, geometry, geometry_windows
from hd_map_pkg.course_speed import CourseSpeedZones, load_course_speed_policy


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Node:
    def __init__(self):
        self.c = rospy.get_param('~')
        self.planner = Planner(self.c)
        self.route = self.world = self.state = self.selected = None
        self.selected_stamp = None
        self.pending_selection = None
        self.pending_selection_set = False
        self.pending_selection_stamp = None
        self.epoch = None
        self.static_map = None
        self.route_status = self.world_status = self.localization_status = None
        self.last_lane_change_evaluation = -math.inf
        self.lock = threading.Lock()
        self.trajectory = rospy.Publisher('/molit/planning/trajectory', Trajectory, queue_size=2)
        self.status = rospy.Publisher('/molit/planning/status', ComponentStatus, queue_size=1, latch=True)
        self.audit = rospy.Publisher('~candidate_costs', String, queue_size=1)
        self.path_markers = rospy.Publisher('~trajectory_markers', MarkerArray, queue_size=1, latch=True)
        rospy.Subscriber('/molit/map/hd_map', HdMap, self.on_map, queue_size=1)
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

    def on_map(self, message):
        if self.static_map is not None and self.static_map[0] == message.map_id:
            return
        lanes = {}
        geometry_only = self.c['rddf_geometry_only']
        if geometry_only:
            reference = next(lane for lane in message.lanes if lane.id == 'global_route')
            reference_points = [[p.pose.position.x,p.pose.position.y] for p in reference.centerline.poses]
            zones = CourseSpeedZones(reference_points, load_course_speed_policy())
            reference_s = np.asarray(reference.route_s)
            high_start, high_end = reference_s[zones.start], reference_s[zones.end]
            route_length = reference_s[-1]
        for lane in message.lanes:
            xyz = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in lane.centerline.poses])
            s = np.asarray(lane.route_s)
            keep = np.r_[True, np.diff(s) > 1e-6]
            if geometry_only:
                unlimited = (s-high_start) % route_length < (high_end-high_start) % route_length
                limits = np.where(unlimited,-1.,zones.policy['normal_limit_kph']/3.6)
                successors = []
            else:
                limits, successors = np.asarray(lane.speed_limits_mps), list(lane.successors)
            lanes[lane.id] = Lane(lane.id, xyz[keep], s[keep], limits[keep], successors)
        if geometry_only:
            self.static_map = (message.map_id, lanes, geometry_windows(lanes,self.c), [])
            return
        windows = []
        for w in message.lane_changes:
            lane = lanes[w.source_lane]
            local = np.r_[0., np.cumsum(np.linalg.norm(np.diff(lane.xy[:, :2], axis=0), axis=1))]
            begin, end = ((w.source_s_start, w.source_s_end) if w.source_lane == 'global_route' else
                          np.interp([w.source_s_start, w.source_s_end], local, lane.s))
            windows.append(Window(w.source_lane, w.target_lane, begin, end))
        # Cache exactly the same route-envelope boundary set previously sent by Route.
        points = np.array([[p.pose.position.x, p.pose.position.y]
                           for lane in message.lanes for p in lane.centerline.poses])
        low = points.min(axis=0)-self.c['map_boundary_margin_m']
        high = points.max(axis=0)+self.c['map_boundary_margin_m']
        boundaries = []
        for line in message.forbidden_boundaries:
            xyz = np.array([[p.x, p.y, p.z] for p in line.points])
            if np.all(xyz[:, :2].max(axis=0) >= low) and np.all(xyz[:, :2].min(axis=0) <= high):
                boundaries.append(xyz)
        self.static_map = (message.map_id, lanes, windows, boundaries)

    def report(self, reason, ready=False, latency=0.):
        m = ComponentStatus(component='path_planning_pkg', state=ComponentStatus.READY if ready else ComponentStatus.DEGRADED,
                            ready=ready, stop_required=not ready, processing_latency_sec=latency, reason=reason)
        m.header.stamp = rospy.Time.now()
        if self.state:
            m.data_stamp = self.state[0].header.stamp
            m.data_age_sec = (m.header.stamp-m.data_stamp).to_sec()
        self.status.publish(m)

    def offer_selection(self, candidate, completed):
        """Hold an activated result for one second and retain only the latest offer."""
        with self.lock:
            if candidate is None and self.selected is not None:
                if not self.pending_selection_set or self.pending_selection is not None:
                    self.pending_selection_stamp = completed
                self.pending_selection = None
                self.pending_selection_set = True
                return
            if ((self.selected is None and candidate is not None) or
                    self.selected_stamp is None or
                    (completed-self.selected_stamp).to_sec() >= self.c['minimum_path_hold_sec']):
                self.selected, self.selected_stamp = candidate, completed
                self.pending_selection = None
                self.pending_selection_set = False
                self.pending_selection_stamp = None
            else:
                self.pending_selection = candidate
                self.pending_selection_set = True
                self.pending_selection_stamp = completed

    def defer_stop(self, reason):
        now = rospy.Time.now()
        with self.lock:
            holding = (self.selected is not None and self.selected_stamp is not None and
                       (now-self.selected_stamp).to_sec() < self.c['minimum_path_hold_sec'])
        self.offer_selection(None,now)
        self.report(reason+('; holding_active_path' if holding else ''),holding)

    def plan(self, _):
        started = time.monotonic()
        route, world, state, static_map = self.route, self.world, self.state, self.static_map
        if route is None or world is None or state is None or static_map is None:
            self.report('waiting_for_map_route_world_localization')
            return
        map_id, lanes, windows, boundaries = static_map
        ego, odom = state
        now = rospy.Time.now()
        if (not self.c['rddf_geometry_only'] and self.route_status is not None
                and self.route_status.state == ComponentStatus.FAULT):
            self.defer_stop('required_checkpoint_missed')
            return
        if (map_id != route.map_id or not world.objects_valid or world.localization_reset_id != ego.reset_id or
                any(not 0 <= (now-stamp).to_sec() <= self.c['input_age_sec'] for stamp in
                    (ego.header.stamp, world.header.stamp, route.header.stamp))):
            self.defer_stop('unusable_or_stale_planning_inputs')
            return
        if self.epoch != ego.reset_id:
            self.planner.committed = self.planner.pending = None
            self.pending_selection = None
            self.pending_selection_set = False
            self.pending_selection_stamp = None
            self.epoch = ego.reset_id
        p = ego.pose.pose.position
        position = np.array([p.x, p.y, p.z])
        speed = max(0., odom.twist.twist.linear.x)
        if (not self.c.get('loop_route',False) and
                (route.route_complete or lanes['global_route'].s[-1]-route.progress < 2*self.c['spatial_step_m'])):
            self.defer_stop('route_endpoint_stop')
            return
        goal_s = max(route.comparison_goal_s, route.progress+1.)
        if self.c.get('loop_route',False):
            if route.progress < getattr(self,'previous_progress',route.progress)-lanes['global_route'].s[-1]/2:
                self.planner.committed = self.planner.pending = None
            self.previous_progress = route.progress
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
        obstacle_grid = ObstacleGrid(objects,self.c)

        def evaluate(candidate):
            self.planner.evaluate(candidate, speed, obstacle_grid, boundaries, goal_s)
            index = min(np.searchsorted(candidate.route_s,goal_s),len(candidate.xy)-1)
            goal = route.comparison_goal
            if (not self.c['rddf_geometry_only'] and
                    np.linalg.norm(candidate.xy[index,:2]-[goal.x,goal.y]) > self.c['checkpoint_radius_m']):
                candidate.cost = candidate.eta = math.inf
                candidate.feasible = False
                candidate.reason = 'does_not_reach_common_checkpoint'
            return candidate

        # Refresh the currently-followed trajectory first. The 10 Hz publisher
        # can use this result while lateral alternatives continue evaluating.
        fast = next((x for x in candidates if x.key == 'committed'),candidates[0])
        evaluate(fast)
        completed = rospy.Time.now()
        self.offer_selection(fast if fast.feasible else None,completed)

        lateral_period = 1./self.c['lane_change_evaluation_rate_hz']
        if time.monotonic()-self.last_lane_change_evaluation < lateral_period and fast.feasible:
            self.audit.publish(String(data=json.dumps([{'key':fast.key,'target':fast.target,
                'feasible':fast.feasible,'reason':fast.reason,'eta':fast.eta if math.isfinite(fast.eta) else None,
                'cost':fast.cost if math.isfinite(fast.cost) else None,'comfort':fast.comfort,'changes':fast.changes}])))
            self.report('frenet; selected=%s; fast_keep; rddf_geometry_only=%s; observed_clusters_only; unverified_development'%
                        (fast.key,self.c['rddf_geometry_only']), True, time.monotonic()-started)
            return

        self.last_lane_change_evaluation = time.monotonic()
        for candidate in candidates:
            if candidate is not fast:
                evaluate(candidate)
        chosen = self.planner.select(candidates, now.to_sec(), route.progress)
        self.offer_selection(chosen,rospy.Time.now())
        audit = [{'key':x.key, 'target':x.target, 'feasible':x.feasible, 'reason':x.reason,
                  'eta':x.eta if math.isfinite(x.eta) else None,
                  'cost':x.cost if math.isfinite(x.cost) else None,
                  'comfort':x.comfort, 'changes':x.changes} for x in candidates]
        self.audit.publish(String(data=json.dumps(audit)))
        self.report('frenet; selected=%s; candidates=%d; rddf_geometry_only=%s; observed_clusters_only; unverified_development'%
                    (chosen.key if chosen else 'stop', len(candidates),self.c['rddf_geometry_only']), True, time.monotonic()-started)

    def publish(self, _):
        if self.state is None:
            return
        ego, odom = self.state
        now = rospy.Time.now()
        with self.lock:
            stop_mature = (self.pending_selection is None and self.pending_selection_stamp is not None and
                           (now-self.pending_selection_stamp).to_sec() >= self.c['minimum_path_hold_sec'])
            path_mature = (self.pending_selection is not None and self.selected_stamp is not None and
                           (now-self.selected_stamp).to_sec() >= self.c['minimum_path_hold_sec'])
            if self.pending_selection_set and (stop_mature or path_mature):
                self.selected = self.pending_selection
                self.selected_stamp = now
                self.pending_selection = None
                self.pending_selection_set = False
                self.pending_selection_stamp = None
            chosen, stamp = self.selected, self.selected_stamp
        output = Trajectory()
        output.header.stamp, output.header.frame_id = now, 'odom'
        output.reset_id = ego.reset_id
        output.valid_for = rospy.Duration(self.c['trajectory_valid_for_sec'])
        if chosen is None or stamp is None:
            output.valid = True
            output.stop_required = True
            output.poses = [copy.deepcopy(odom.pose.pose), copy.deepcopy(odom.pose.pose)]
            output.speed_mps = [0., 0.]
            output.time_from_start = [rospy.Duration(0), rospy.Duration(1)]
            self.emit(output)
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
        self.emit(output)

    def emit(self, output):
        self.trajectory.publish(output)
        line = Marker(header=output.header, ns='frenet_selected', id=0,
                      type=Marker.LINE_STRIP, action=Marker.ADD)
        line.pose.orientation.w = 1.
        line.scale.x = .18
        line.color.r, line.color.g, line.color.b, line.color.a = 0., 1., 1., 1.
        line.lifetime = output.valid_for
        line.points = [copy.deepcopy(p.position) for p in output.poses]
        for p in line.points:
            p.z += .25
        markers = [Marker(action=Marker.DELETEALL), line]
        stops = [i for i, speed in enumerate(output.speed_mps) if speed == 0. and (i > 0 or output.stop_required)]
        if stops:
            stop = Marker(header=output.header, ns='frenet_stop', id=1,
                          type=Marker.SPHERE, action=Marker.ADD)
            stop.pose.position = copy.deepcopy(line.points[stops[0]])
            stop.pose.orientation.w = 1.
            stop.scale.x = stop.scale.y = stop.scale.z = .7
            stop.color.r, stop.color.a = 1., 1.
            stop.lifetime = output.valid_for
            markers.append(stop)
        self.path_markers.publish(MarkerArray(markers=markers))


if __name__ == '__main__':
    rospy.init_node('path_planner_node')
    Node()
    rospy.spin()
