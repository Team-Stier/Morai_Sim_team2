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
from path_planning_pkg.frenet import Planner, Lane, Window, Obstacle, ObstacleGrid, Candidate, geometry, geometry_windows, project
from path_planning_pkg.hybrid_astar import HybridAStarConfig, HybridAStarPlanner, HybridPlanStatus, Pose2D
from path_planning_pkg.hybrid_runtime import build_hybrid_candidate, flatten_obstacle_points, hybrid_path_clear
from path_planning_pkg.planner_mode_manager import HYBRID_ASTAR, PlannerModeManager
from path_planning_pkg.transition_speed import TransitionSpeedPolicy
from hd_map_pkg.course_speed import CourseSpeedZones, load_course_speed_policy


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Node:
    def __init__(self):
        self.c = rospy.get_param('~')
        self.planner = Planner(self.c)
        self.mode_manager = PlannerModeManager.from_mapping(self.c['planner_mode'])
        self.transition_speed = TransitionSpeedPolicy(self.c)
        self.hybrid_planner = HybridAStarPlanner(HybridAStarConfig.from_mapping(self.c['hybrid_astar']))
        self.hybrid_runtime_config = self.c['hybrid_astar']
        self.active_planner_mode = None
        self.active_zone = None
        self.hybrid_fallback_deadline = None
        self.hybrid_checked_candidate = None
        self.hybrid_checked_scene_key = None
        self.handoff_candidate = None
        self.handoff_stamp = None
        self.handoff_deadline = None
        self.handoff_reset_id = None
        self.handoff_from_mode = None
        self.handoff_target_mode = None
        self.handoff_started = None
        self.handoff_checked_scene_key = None
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
        self.publish_lock = threading.Lock()
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
        with self.publish_lock:
            with self.lock:
                if self.state is not None and self.state[0].reset_id != ego.reset_id:
                    self.selected = self.selected_stamp = None
                    self.pending_selection = self.pending_selection_stamp = None
                    self.pending_selection_set = False
                    self.mode_manager.reset()
                    self.active_planner_mode = None
                    self.active_zone = None
                    self.hybrid_fallback_deadline = None
                    self.hybrid_checked_candidate = None
                    self.hybrid_checked_scene_key = None
                    self.clear_handoff_locked()
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

    def offer_selection(self, candidate, completed, reset_id=None):
        """Hold normal paths; stop results override and discard pending motion."""
        with self.lock:
            if reset_id is not None and self.state[0].reset_id != reset_id:
                return
            if candidate is not None and getattr(self, 'handoff_candidate', None) is not None:
                self.clear_handoff_locked()
                self.selected, self.selected_stamp = candidate, completed
                self.pending_selection = self.pending_selection_stamp = None
                self.pending_selection_set = False
                self.hybrid_fallback_deadline = None
                return
            stopping = candidate is None or bool(np.any(candidate.speed <= 0.))
            active_stop = self.selected is None or bool(np.any(self.selected.speed <= 0.))
            age = (completed-self.selected_stamp).to_sec() if self.selected_stamp is not None else math.inf
            if (not stopping and not active_stop and
                    getattr(self, 'hybrid_fallback_deadline', None) is None and
                    0. <= age < self.c['minimum_active_path_hold_sec']):
                self.pending_selection = candidate
                self.pending_selection_stamp = completed
                self.pending_selection_set = True
                return
            self.selected, self.selected_stamp = candidate, completed
            self.pending_selection = None
            self.pending_selection_set = False
            self.pending_selection_stamp = None
            self.hybrid_fallback_deadline = None

    def clear_handoff_locked(self):
        self.handoff_candidate = None
        self.handoff_stamp = None
        self.handoff_deadline = None
        self.handoff_reset_id = None
        self.handoff_from_mode = None
        self.handoff_target_mode = None
        self.handoff_started = None
        self.handoff_checked_scene_key = None

    def defer_stop(self, reason):
        with self.lock:
            self.clear_handoff_locked()
        self.offer_selection(None, rospy.Time.now())
        self.publish(None)
        self.report(reason, False)

    def hybrid_scene_key(self, candidate, world):
        """Key only observed points close enough to affect this selected path."""
        if candidate is None or world is None:
            return None
        points = flatten_obstacle_points(world.objects)
        if not np.isfinite(points).all():
            return None
        radius = (math.hypot(self.hybrid_planner.config.front_overhang_m,
                             self.hybrid_planner.config.vehicle_width_m / 2.) +
                  self.hybrid_planner.config.obstacle_margin_m)
        low = np.min(candidate.xy[:, :2], axis=0)-radius
        high = np.max(candidate.xy[:, :2], axis=0)+radius
        nearby = points[np.all((points >= low) & (points <= high), axis=1)]
        return nearby.tobytes()

    def aligned_path_suffix(self, candidate, ego, max_distance=None):
        """Return the forward map-frame path only when ego can follow its tangent."""
        if (candidate is None or len(candidate.xy) < 2 or
                len(candidate.route_s) != len(candidate.xy) or
                len(candidate.limits) != len(candidate.xy) or
                not np.isfinite(candidate.xy).all() or
                (len(candidate.speed) and (len(candidate.speed) != len(candidate.xy) or
                 not np.isfinite(candidate.speed).all() or np.any(candidate.speed < 0.)))):
            return None
        p = ego.pose.pose.position
        position = np.array([p.x, p.y, p.z])
        nearest = int(np.argmin(np.linalg.norm(candidate.xy[:, :2]-position[:2], axis=1)))
        max_error = float(self.hybrid_runtime_config.get('fallback_max_path_error_m', 1.5))
        if (nearest >= len(candidate.xy)-1 or
                np.linalg.norm(candidate.xy[nearest, :2]-position[:2]) > max_error):
            return None
        # Average over one primitive so a 0.3 m discretization kink is not
        # mistaken for a vehicle heading reversal.
        path_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(candidate.xy[:, :2], axis=0), axis=1))]
        half_window = self.hybrid_planner.config.primitive_length_m / 2.
        before = max(0, int(np.searchsorted(path_s, path_s[nearest]-half_window, side='right'))-1)
        after = min(len(path_s)-1, int(np.searchsorted(path_s, path_s[nearest]+half_window)))
        forward = candidate.xy[after, :2]-candidate.xy[before, :2]
        if np.linalg.norm(forward) <= 1.0e-6:
            return None
        path_heading = math.atan2(forward[1], forward[0])
        heading_error = math.atan2(math.sin(yaw(ego.pose.pose.orientation)-path_heading),
                                   math.cos(yaw(ego.pose.pose.orientation)-path_heading))
        maximum_heading_error = math.radians(float(
            self.hybrid_runtime_config.get('reuse_max_heading_error_deg', 35.0)))
        if not math.isfinite(maximum_heading_error) or abs(heading_error) > maximum_heading_error:
            return None
        start = max(0, nearest-1)
        end = len(candidate.xy)
        if max_distance is not None:
            end = min(end, max(start+2, int(np.searchsorted(
                path_s, path_s[nearest] + max_distance, side='right')) + 1))
        return Candidate(candidate.key, candidate.target, candidate.xy[start:end].copy(),
                         candidate.route_s[start:end].copy(), candidate.limits[start:end].copy(),
                         candidate.changes, candidate.change_end, candidate.return_start)

    def hybrid_path_usable(self, candidate, now, map_id, zone_id, lane, boundaries,
                           route, world, state, check_geometry=True):
        """Check a Hybrid result against inputs and ego pose current at acceptance."""
        if candidate is None or route is None or world is None or state is None:
            return False
        ego, _odom = state
        world_max_age = (self.c.get('world_model_input_age_sec', self.c['input_age_sec'])
                         if self.c['rddf_geometry_only'] else self.c['input_age_sec'])
        zone = next((item for item in self.mode_manager.zones if item.zone_id == zone_id), None)
        current_mode = self.mode_manager.current
        if (self.static_map is None or self.static_map[0] != map_id or
                self.active_planner_mode != HYBRID_ASTAR or zone is None or
                (current_mode is not None and current_mode.zone_id != zone_id) or
                (zone is not self.mode_manager.zones[-1] and route.progress >= zone.end_s) or
                route.map_id != map_id or not world.objects_valid or
                world.localization_reset_id != ego.reset_id or
                self.epoch != ego.reset_id or self.active_zone != zone_id or
                any(not 0. <= (now-stamp).to_sec() <= self.c['input_age_sec']
                    for stamp in (ego.header.stamp, route.header.stamp)) or
                not 0. <= (now-world.header.stamp).to_sec() <= world_max_age):
            return False
        if candidate.key != 'hybrid_astar':
            return False
        suffix = self.aligned_path_suffix(candidate, ego)
        if suffix is None:
            return False
        if not check_geometry:
            return True
        p = ego.pose.pose.position
        position = np.array([p.x, p.y, p.z])
        progress = project(lane.xy, lane.s, position[:2])[0]
        return hybrid_path_clear(self.hybrid_planner, lane, progress, suffix,
                                 flatten_obstacle_points(world.objects),
                                 self.hybrid_runtime_config,
                                 self.c.get('loop_route', False), boundaries)

    def reusable_hybrid_path(self, now, map_id, zone_id, lane, boundaries,
                             route, world, state, check_geometry=True):
        """Recheck a recent Hybrid path before using it after an interrupted search."""
        with self.lock:
            selected, selected_stamp = self.selected, self.selected_stamp
        hold_sec = min(float(self.hybrid_runtime_config.get('transient_path_hold_sec', 0.5)),
                       float(self.c['trajectory_valid_for_sec']))
        age = (now-selected_stamp).to_sec() if selected_stamp is not None else math.inf
        if (selected is None or selected.key != 'hybrid_astar' or not selected.feasible or
                not 0. <= age < hold_sec or len(selected.speed) != len(selected.xy)):
            return None
        if not self.hybrid_path_usable(selected, now, map_id, zone_id, lane, boundaries,
                                       route, world, state, check_geometry):
            return None
        return selected, selected_stamp

    def hybrid_pending_usable(self, candidate, stamp, now, map_id, zone_id, lane,
                              boundaries, route, world, state):
        """Recheck a queued Hybrid trajectory before replacing its active path."""
        return (candidate is not None and stamp is not None and candidate.feasible and
                len(candidate.speed) == len(candidate.xy) and
                len(candidate.times) == len(candidate.xy) and
                np.isfinite(candidate.times).all() and now >= stamp and
                self.hybrid_path_usable(candidate, now, map_id, zone_id, lane,
                                        boundaries, route, world, state))

    def handoff_usable(self, now, map_id, lane, boundaries, route, world, state,
                       check_geometry=True):
        """Check the old mode's path during one bounded planner transition."""
        with self.lock:
            candidate = self.handoff_candidate
            deadline = self.handoff_deadline
            started = self.handoff_started
            reset_id = self.handoff_reset_id
            target_mode = self.handoff_target_mode
        if (candidate is None or deadline is None or started is None or
                not started <= now < deadline or self.active_planner_mode != target_mode or
                route is None or world is None or state is None or
                self.static_map is None or self.static_map[0] != map_id or
                route.map_id != map_id or not world.objects_valid or
                state[0].reset_id != reset_id or world.localization_reset_id != reset_id or
                self.epoch != reset_id or not candidate.feasible or
                len(candidate.speed) != len(candidate.xy) or
                len(candidate.times) != len(candidate.xy) or
                not np.isfinite(candidate.speed).all() or
                not np.any(candidate.speed > .01) or
                np.count_nonzero(np.isfinite(candidate.times)) < 2):
            return False
        world_max_age = (self.c.get('world_model_input_age_sec', self.c['input_age_sec'])
                         if self.c['rddf_geometry_only'] else self.c['input_age_sec'])
        if (any(not 0. <= (now-stamp).to_sec() <= self.c['input_age_sec']
                for stamp in (state[0].header.stamp, route.header.stamp)) or
                not 0. <= (now-world.header.stamp).to_sec() <= world_max_age):
            return False
        suffix = self.aligned_path_suffix(candidate, state[0],
            float(self.hybrid_runtime_config['local_goal_distance_m']))
        if suffix is None:
            return False
        if not check_geometry:
            return True
        p = state[0].pose.pose.position
        progress = project(lane.xy, lane.s, np.array([p.x, p.y]))[0]
        return hybrid_path_clear(self.hybrid_planner, lane, progress, suffix,
            flatten_obstacle_points(world.objects), self.hybrid_runtime_config,
            self.c.get('loop_route', False), boundaries)

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
        world_max_age = (self.c.get('world_model_input_age_sec', self.c['input_age_sec']) if self.c['rddf_geometry_only']
                         else self.c['input_age_sec'])
        if (map_id != route.map_id or not world.objects_valid or world.localization_reset_id != ego.reset_id or
                any(not 0 <= (now-stamp).to_sec() <= self.c['input_age_sec'] for stamp in
                    (ego.header.stamp, route.header.stamp)) or
                not 0 <= (now-world.header.stamp).to_sec() <= world_max_age):
            self.defer_stop('unusable_or_stale_planning_inputs')
            return
        if self.epoch != ego.reset_id:
            self.planner.committed = self.planner.pending = None
            self.epoch = ego.reset_id
        p = ego.pose.pose.position
        position = np.array([p.x, p.y, p.z])
        progress = project(lanes['global_route'].xy, lanes['global_route'].s, position[:2])[0]
        speed = max(0., odom.twist.twist.linear.x)
        if (not self.c.get('loop_route',False) and
                (route.route_complete or lanes['global_route'].s[-1]-progress < 2*self.c['spatial_step_m'])):
            self.defer_stop('route_endpoint_stop')
            return
        selection = self.mode_manager.select(route.progress)
        mode_changed = selection.planner != self.active_planner_mode
        reset_selection = (mode_changed or
            (selection.planner == HYBRID_ASTAR and selection.zone_id != self.active_zone))
        handoff_ready = False
        if reset_selection:
            with self.publish_lock:
                with self.lock:
                    old_mode = self.active_planner_mode
                    old_candidate, old_stamp = self.selected, self.selected_stamp
                    self.selected = self.selected_stamp = None
                    self.pending_selection = self.pending_selection_stamp = None
                    self.pending_selection_set = False
                    self.hybrid_fallback_deadline = None
                    self.clear_handoff_locked()
                    if mode_changed and old_mode is not None and old_candidate is not None:
                        self.handoff_candidate = old_candidate
                        self.handoff_stamp = old_stamp
                        self.handoff_started = now
                        self.handoff_deadline = now + rospy.Duration(float(
                            self.c.get('mode_transition_handoff_sec', .8)))
                        self.handoff_reset_id = ego.reset_id
                        self.handoff_from_mode = old_mode
                        self.handoff_target_mode = selection.planner
                self.active_planner_mode = selection.planner
                self.active_zone = selection.zone_id
                if self.handoff_candidate is not None:
                    handoff_ready = self.handoff_usable(now, map_id, lanes['global_route'],
                        boundaries, route, world, state)
                    with self.lock:
                        if handoff_ready:
                            self.handoff_checked_scene_key = self.hybrid_scene_key(
                                self.handoff_candidate, world)
                        else:
                            self.clear_handoff_locked()
            self.planner.committed = self.planner.pending = None
            if handoff_ready:
                self.report('planner_mode=%s; zone=%s; selected=handoff; replanning' %
                            (selection.planner, selection.zone_id), True)
            elif old_mode is not None:
                self.publish(None)
                self.report('planner_mode=%s; zone=%s; selected=stop; replanning' %
                            (selection.planner, selection.zone_id), False)
        else:
            self.active_zone = selection.zone_id

        if selection.planner == HYBRID_ASTAR:
            with self.lock:
                previous_candidate = self.selected
            previous_usable = self.hybrid_path_usable(
                previous_candidate, now, map_id, selection.zone_id,
                lanes['global_route'], boundaries, route, world, state)
            if not previous_usable and self.handoff_usable(
                    now, map_id, lanes['global_route'], boundaries,
                    route, world, state):
                with self.lock:
                    previous_candidate = self.handoff_candidate
                previous_usable = previous_candidate is not None
            if not previous_usable:
                with self.lock:
                    pending = (self.pending_selection if self.selected is previous_candidate and
                               self.pending_selection_set else None)
                    pending_stamp = self.pending_selection_stamp
                current_route, current_world, current_state = self.route, self.world, self.state
                pending_usable = self.hybrid_pending_usable(
                    pending, pending_stamp, rospy.Time.now(), map_id, selection.zone_id,
                    lanes['global_route'], boundaries, current_route, current_world, current_state)
                had_active = False
                with self.lock:
                    if self.selected is previous_candidate:
                        if (pending_usable and self.pending_selection_set and
                                self.pending_selection is pending and
                                self.pending_selection_stamp == pending_stamp):
                            self.selected, self.selected_stamp = pending, pending_stamp
                            self.pending_selection = self.pending_selection_stamp = None
                            self.pending_selection_set = False
                            self.hybrid_fallback_deadline = None
                            previous_candidate, previous_usable = pending, True
                        else:
                            had_active = self.selected is not None
                            self.selected = self.selected_stamp = None
                            self.pending_selection = self.pending_selection_stamp = None
                            self.pending_selection_set = False
                            self.hybrid_fallback_deadline = None
                            previous_candidate = None
                    else:
                        # Another callback replaced the active path during validation.
                        previous_candidate = None
                if had_active:
                    self.publish(None)
                    self.report('planner_mode=hybrid_astar; zone=%s; selected=stop; replanning' %
                                selection.zone_id, False)
            hybrid = build_hybrid_candidate(
                self.hybrid_planner,
                lanes['global_route'],
                Pose2D(float(position[0]), float(position[1]), yaw(ego.pose.pose.orientation)),
                progress,
                flatten_obstacle_points(world.objects),
                self.hybrid_runtime_config,
                self.c.get('loop_route', False),
                previous_path=previous_candidate.xy if previous_usable else None,
                forbidden_boundaries=boundaries,
            )
            candidate = hybrid.candidate
            completed = rospy.Time.now()
            accepted = activated = held = handoff_held = False
            rejected_after_search = False
            current_route, current_world, current_state = self.route, self.world, self.state
            if candidate is not None and self.hybrid_path_usable(
                    candidate, completed, map_id, selection.zone_id, lanes['global_route'],
                    boundaries, current_route, current_world, current_state):
                with self.lock:
                    active_candidate = self.selected
                if not self.hybrid_path_usable(
                        active_candidate, completed, map_id, selection.zone_id,
                        lanes['global_route'], boundaries, current_route, current_world,
                        current_state):
                    with self.lock:
                        self.selected = self.selected_stamp = None
                        self.pending_selection = self.pending_selection_stamp = None
                        self.pending_selection_set = False
                        self.hybrid_fallback_deadline = None
                current_speed = max(0., current_state[1].twist.twist.linear.x)
                candidate.speed_cap_mps = self.transition_speed.caps(
                    candidate.route_s, selection.planner, lanes['global_route'])
                _distance, _heading, _curvature, speeds, times = self.planner.profile(
                    candidate, current_speed)
                candidate.speed = speeds
                candidate.times = times
                candidate.feasible = True
                candidate.eta = float(times[-1])
                candidate.cost = candidate.eta
                self.offer_selection(candidate, completed, current_state[0].reset_id)
                with self.lock:
                    accepted = self.selected is candidate or self.pending_selection is candidate
                    activated = self.selected is candidate
            elif candidate is not None:
                rejected_after_search = True
            if not accepted:
                if (candidate is not None or hybrid.plan.status in
                        (HybridPlanStatus.TIME_LIMIT, HybridPlanStatus.SEARCH_LIMIT)):
                    held_path = self.reusable_hybrid_path(
                        completed, map_id, selection.zone_id, lanes['global_route'],
                        boundaries, current_route, current_world, current_state)
                    if held_path is not None:
                        with self.lock:
                            if self.selected is held_path[0] and self.selected_stamp == held_path[1]:
                                held = True
                                self.hybrid_fallback_deadline = held_path[1] + rospy.Duration(
                                    min(float(self.hybrid_runtime_config.get('transient_path_hold_sec', 0.5)),
                                        float(self.c['trajectory_valid_for_sec'])))
                                self.pending_selection = self.pending_selection_stamp = None
                                self.pending_selection_set = False
                if not held:
                    handoff_held = self.handoff_usable(completed, map_id,
                        lanes['global_route'], boundaries, current_route, current_world,
                        current_state)
                    held = handoff_held
                if not held:
                    self.offer_selection(None, completed, ego.reset_id)
                    self.publish(None)
            diagnostic = {
                'planner': HYBRID_ASTAR,
                'zone': selection.zone_id,
                'status': hybrid.plan.status.value,
                'expanded_nodes': hybrid.plan.expanded_nodes,
                'generated_nodes': hybrid.plan.generated_nodes,
                'rejected_by_corridor': hybrid.plan.rejected_by_corridor,
                'rejected_by_obstacle': hybrid.plan.rejected_by_obstacle,
                'elapsed_sec': hybrid.plan.elapsed_sec,
                'accepted': accepted,
                'activated': activated,
                'held': held,
                'handoff_held': handoff_held,
                'rejected_after_search': rejected_after_search,
            }
            self.audit.publish(String(data=json.dumps([diagnostic])))
            self.report(
                'planner_mode=%s; zone=%s; status=%s; selected=%s; simulator_closed_loop_unverified' %
                (selection.planner, selection.zone_id, hybrid.plan.status.value,
                 'new' if activated else 'pending' if accepted else 'handoff' if handoff_held
                 else 'held' if held else 'stop'),
                accepted or held,
                time.monotonic()-started,
            )
            return
        goal_s = (progress+max(route.comparison_goal_s-route.progress,1.)
                  if self.c['rddf_geometry_only'] else max(route.comparison_goal_s,progress+1.))
        if self.c.get('loop_route',False):
            if progress < getattr(self,'previous_progress',progress)-lanes['global_route'].s[-1]/2:
                self.planner.committed = self.planner.pending = None
            self.previous_progress = progress
        candidates = self.planner.candidates(lanes, windows, route.current_lane, progress,
                                              goal_s, position, yaw(ego.pose.pose.orientation), speed)
        committed = self.planner.committed
        if committed is not None:
            end_s = committed.change_end
            if progress < end_s:
                i = max(0, min(np.searchsorted(committed.route_s, progress)-1, len(committed.route_s)-3))
                held = Candidate('committed', committed.target, committed.xy[i:].copy(), committed.route_s[i:].copy(),
                                 committed.limits[i:].copy(), committed.changes, committed.change_end, committed.return_start)
                candidates.append(held)
            else:
                self.planner.committed = None
        objects = [Obstacle(np.array([[p.x, p.y, p.z] for p in o.points]),
                            np.array([o.twist.linear.x, o.twist.linear.y]), o.velocity_valid,
                            (now-o.source_stamp).to_sec()) for o in world.objects]
        obstacle_grid = ObstacleGrid(objects,self.c)

        def evaluate(candidate):
            candidate.speed_cap_mps = self.transition_speed.caps(
                candidate.route_s, selection.planner, lanes['global_route'])
            self.planner.evaluate(candidate, speed, obstacle_grid, boundaries, goal_s)
            index = min(np.searchsorted(candidate.route_s,goal_s),len(candidate.xy)-1)
            goal = route.comparison_goal
            if (not self.c['rddf_geometry_only'] and
                    np.linalg.norm(candidate.xy[index,:2]-[goal.x,goal.y]) > self.c['checkpoint_radius_m']):
                candidate.cost = candidate.eta = math.inf
                candidate.feasible = False
                candidate.reason = 'does_not_reach_common_checkpoint'
            return candidate

        # Holding geometry does not hold its old collision verdict. Recheck the
        # actual active path against this scene while replacements are waiting.
        with self.lock:
            active, active_stamp = self.selected, self.selected_stamp
        if (active is not None and active_stamp is not None and
                0. <= (now-active_stamp).to_sec() < self.c['minimum_active_path_hold_sec']):
            nearest = int(np.argmin(np.linalg.norm(active.xy[:,:2]-position[:2],axis=1)))
            i = max(0, nearest-1)
            if len(active.xy)-i >= 3:
                checked = Candidate('committed', active.target, active.xy[i:].copy(),
                    active.route_s[i:].copy(), active.limits[i:].copy(), active.changes,
                    active.change_end, active.return_start)
                evaluate(checked)
                if not checked.feasible or np.any(checked.speed <= 0.):
                    self.offer_selection(checked if checked.feasible else None,now,ego.reset_id)
            else:
                self.offer_selection(None,now,ego.reset_id)

        # Refresh the currently-followed trajectory first. The 10 Hz publisher
        # can use this result while lateral alternatives continue evaluating.
        fast = next((x for x in candidates if x.key == 'committed'),candidates[0])
        evaluate(fast)
        completed = rospy.Time.now()
        # An unfinished search is not a stop result. Keep the published selection
        # until alternatives finish, unless this evaluation found a safety hazard.
        if fast.feasible:
            self.offer_selection(fast,completed,ego.reset_id)
        elif fast.reason in (
                'predicted_cluster_collision', 'insufficient_stopping_distance',
                'no_collision_free_stop', 'forbidden_boundary'):
            self.defer_stop('planner_mode=frenet; zone=%s; reason=%s' %
                            (selection.zone_id, fast.reason))

        lateral_period = 1./self.c['lane_change_evaluation_rate_hz']
        # A traversable committed manoeuvre stays selected regardless of other
        # costs. An unresolved stop must periodically search for recovery instead.
        if fast.feasible and ((fast.key == 'committed' and math.isfinite(fast.cost)) or
                time.monotonic()-self.last_lane_change_evaluation < lateral_period):
            self.audit.publish(String(data=json.dumps([{'key':fast.key,'target':fast.target,
                'feasible':fast.feasible,'reason':fast.reason,'eta':fast.eta if math.isfinite(fast.eta) else None,
                'cost':fast.cost if math.isfinite(fast.cost) else None,'comfort':fast.comfort,'changes':fast.changes}])))
            self.report('planner_mode=frenet; zone=%s; selected=%s; fast_keep; rddf_geometry_only=%s; observed_clusters_only; unverified_development'%
                        (selection.zone_id,fast.key,self.c['rddf_geometry_only']), True, time.monotonic()-started)
            return

        self.last_lane_change_evaluation = time.monotonic()
        # A blocked committed manoeuvre must allow checked recovery candidates.
        # Evaluate keep first so detours are generated only if it is also blocked.
        if candidates[0] is not fast:
            evaluate(candidates[0])
        if not math.isfinite(candidates[0].cost) and (self.planner.committed is None or
                not fast.feasible or not math.isfinite(fast.cost)):
            candidates.extend(self.planner.obstacle_detours(candidates[0], objects))
        for candidate in candidates:
            if candidate is not fast and candidate is not candidates[0]:
                evaluate(candidate)
        chosen = self.planner.select(candidates, now.to_sec(), progress)
        handoff_held = False
        if chosen is not None:
            self.offer_selection(chosen,rospy.Time.now(),ego.reset_id)
        else:
            handoff_held = self.handoff_usable(rospy.Time.now(), map_id,
                lanes['global_route'], boundaries, self.route, self.world, self.state)
            if not handoff_held:
                self.offer_selection(None,rospy.Time.now(),ego.reset_id)
                self.publish(None)
        audit = [{'key':x.key, 'target':x.target, 'feasible':x.feasible, 'reason':x.reason,
                  'eta':x.eta if math.isfinite(x.eta) else None,
                  'cost':x.cost if math.isfinite(x.cost) else None,
                  'comfort':x.comfort, 'changes':x.changes} for x in candidates]
        self.audit.publish(String(data=json.dumps(audit)))
        self.report('planner_mode=frenet; zone=%s; selected=%s; candidates=%d; rddf_geometry_only=%s; observed_clusters_only; unverified_development'%
                    (selection.zone_id,chosen.key if chosen else 'handoff' if handoff_held else 'stop',
                     len(candidates),self.c['rddf_geometry_only']),
                    chosen is not None or handoff_held, time.monotonic()-started)

    def publish(self, event):
        # Plan-triggered stop publication must be ordered after any in-flight
        # timer publication, so an older motion path cannot follow the stop.
        with self.publish_lock:
            self._publish_once(event)

    def _publish_once(self, _):
        if self.state is None:
            return
        now = rospy.Time.now()
        fallback_stop_reason = None
        with self.lock:
            deadline = getattr(self, 'hybrid_fallback_deadline', None)
            if deadline is not None and now >= deadline:
                self.selected = self.selected_stamp = None
                self.pending_selection = self.pending_selection_stamp = None
                self.pending_selection_set = False
                self.hybrid_fallback_deadline = None
                fallback_stop_reason = 'transient_hold_expired'
            if self.selected_stamp is not None and now < self.selected_stamp:
                self.selected = self.selected_stamp = None
                self.pending_selection = self.pending_selection_stamp = None
                self.pending_selection_set = False
            pending_due = (self.pending_selection_set and self.selected_stamp is not None and
                           (now-self.selected_stamp).to_sec() >= self.c['minimum_active_path_hold_sec'])
            hybrid_pending_due = pending_due and self.active_planner_mode == HYBRID_ASTAR
            if pending_due and not hybrid_pending_due:
                self.selected = self.pending_selection
                self.selected_stamp = now
                self.pending_selection = self.pending_selection_stamp = None
                self.pending_selection_set = False
            active_before, active_stamp = self.selected, self.selected_stamp
            pending = self.pending_selection if hybrid_pending_due else None
            pending_stamp = self.pending_selection_stamp if hybrid_pending_due else None
        if hybrid_pending_due:
            static_map = self.static_map
            pending_usable = (static_map is not None and 'global_route' in static_map[1] and
                self.hybrid_pending_usable(pending, pending_stamp, now, static_map[0],
                    self.active_zone, static_map[1]['global_route'], static_map[3],
                    self.route, self.world, self.state))
            with self.lock:
                if (self.selected is active_before and self.selected_stamp == active_stamp and
                        self.pending_selection_set and self.pending_selection is pending and
                        self.pending_selection_stamp == pending_stamp):
                    if pending_usable:
                        self.selected, self.selected_stamp = pending, pending_stamp
                        self.hybrid_fallback_deadline = None
                    self.pending_selection = self.pending_selection_stamp = None
                    self.pending_selection_set = False
        with self.lock:
            ego, odom = self.state
            chosen, stamp = self.selected, self.selected_stamp
            handoff = self.handoff_candidate if chosen is None else None
            handoff_stamp = self.handoff_stamp
            handoff_deadline = self.handoff_deadline
        using_handoff = False
        if handoff is not None:
            static_map = self.static_map
            scene_key = self.hybrid_scene_key(handoff, self.world)
            full_check = (scene_key is None or
                          self.handoff_checked_scene_key != scene_key)
            usable = (static_map is not None and 'global_route' in static_map[1] and
                self.handoff_usable(now, static_map[0], static_map[1]['global_route'],
                    static_map[3], self.route, self.world, self.state, full_check))
            if usable:
                chosen, stamp = handoff, handoff_stamp
                using_handoff = True
                if full_check:
                    self.handoff_checked_scene_key = scene_key
            else:
                with self.lock:
                    if self.handoff_candidate is handoff:
                        self.clear_handoff_locked()
                        fallback_stop_reason = ('mode_handoff_expired' if
                            handoff_deadline is not None and now >= handoff_deadline else
                            'mode_handoff_invalidated')
        if (self.active_planner_mode == HYBRID_ASTAR and chosen is not None and
                not using_handoff and fallback_stop_reason is None):
            static_map = self.static_map
            checked = False
            if static_map is not None and 'global_route' in static_map[1]:
                scene_key = self.hybrid_scene_key(chosen, self.world)
                full_check = (scene_key is None or self.hybrid_checked_candidate is not chosen or
                              self.hybrid_checked_scene_key != scene_key)
                args = (now, static_map[0], self.active_zone,
                        static_map[1]['global_route'], static_map[3], self.route,
                        self.world, self.state)
                if deadline is not None:
                    held_path = self.reusable_hybrid_path(*args, check_geometry=full_check)
                    checked = held_path is not None and held_path[0] is chosen
                else:
                    checked = self.hybrid_path_usable(chosen, *args,
                                                      check_geometry=full_check)
                if checked and full_check:
                    self.hybrid_checked_candidate = chosen
                    self.hybrid_checked_scene_key = scene_key
            if not checked:
                with self.lock:
                    pending = (self.pending_selection if self.selected is chosen and
                               self.pending_selection_set else None)
                    pending_stamp = self.pending_selection_stamp
                pending_usable = (static_map is not None and
                                  'global_route' in static_map[1] and
                                  self.hybrid_pending_usable(pending, pending_stamp, *args))
                with self.lock:
                    if self.selected is chosen:
                        if (pending_usable and self.pending_selection_set and
                                self.pending_selection is pending and
                                self.pending_selection_stamp == pending_stamp):
                            self.selected, self.selected_stamp = pending, pending_stamp
                            self.pending_selection = self.pending_selection_stamp = None
                            self.pending_selection_set = False
                            self.hybrid_fallback_deadline = None
                            chosen, stamp = pending, pending_stamp
                        else:
                            self.selected = self.selected_stamp = None
                            self.pending_selection = self.pending_selection_stamp = None
                            self.pending_selection_set = False
                            self.hybrid_fallback_deadline = None
                            chosen = stamp = None
                            fallback_stop_reason = ('transient_hold_invalidated' if deadline is not None
                                                    else 'active_path_invalidated')
                    else:
                        chosen, stamp = self.selected, self.selected_stamp
        output = Trajectory()
        output.header.stamp, output.header.frame_id = now, 'odom'
        output.reset_id = ego.reset_id
        output.valid_for = rospy.Duration(self.c['trajectory_valid_for_sec'])
        p = ego.pose.pose.position
        position = np.array([p.x, p.y, p.z])
        xyz = speeds = None
        if chosen is not None and stamp is not None:
            finite = np.flatnonzero(np.isfinite(chosen.times))
            if len(finite) >= 2:
                end = int(finite[-1])+1
                # Keep the approved geometry: an ego-position replacement of just
                # point zero hides tracking error and introduces artificial curvature.
                # Retain one point before the nearest point for a stable tangent.
                nearest = int(np.argmin(np.linalg.norm(chosen.xy[:end, :2]-position[:2], axis=1)))
                i = max(0, nearest-1)
                xyz, speeds = chosen.xy[i:end].copy(), chosen.speed[i:end].copy()
        if xyz is None or len(xyz) < 2:
            output.valid = True
            output.stop_required = True
            output.poses = [copy.deepcopy(odom.pose.pose), copy.deepcopy(odom.pose.pose)]
            output.speed_mps = [0., 0.]
            output.time_from_start = [rospy.Duration(0), rospy.Duration(1)]
            self.emit(output)
            if fallback_stop_reason:
                self.report('planner_mode=%s; status=%s; selected=stop' %
                            (self.active_planner_mode, fallback_stop_reason), False)
            return
        # Relative map->odom transform from a synchronized estimate pair.
        rotation = yaw(odom.pose.pose.orientation)-yaw(ego.pose.pose.orientation)
        co, si = math.cos(rotation), math.sin(rotation)
        ds, headings, _ = geometry(xyz)
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
        line = Marker(header=output.header, ns='planner_selected', id=0,
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
            stop = Marker(header=output.header, ns='planner_stop', id=1,
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
