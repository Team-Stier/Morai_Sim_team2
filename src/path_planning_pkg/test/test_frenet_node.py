import importlib.util
import io
import math
from pathlib import Path
import threading
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
import rospy
import yaml
from common_msgs_pkg.msg import EgoState, HdMap, RouteLane, RouteContext, WorldModel, ComponentStatus, TrackedObject
from geometry_msgs.msg import PoseStamped, Point, Point32, Polygon
from nav_msgs.msg import Odometry
from path_planning_pkg.frenet import Candidate, Lane, Planner
from path_planning_pkg.hybrid_astar import HybridAStarConfig, HybridAStarPlanner, HybridPlanResult, HybridPlanStatus
from path_planning_pkg.hybrid_runtime import HybridCandidateResult
from path_planning_pkg.planner_mode_manager import FRENET, HYBRID_ASTAR, PlannerModeManager, PlannerZone
from path_planning_pkg.transition_speed import TransitionSpeedPolicy


spec = importlib.util.spec_from_file_location('path_planner_node', Path(__file__).parents[1]/'src/path_planner_node.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Output:
    def publish(self, message):
        self.message = message


class TraceOutput(Output):
    def __init__(self):
        self.messages = []

    def publish(self, message):
        super().publish(message)
        self.messages.append(message)


class FrenetOutputTest(unittest.TestCase):
    def mode_boundary_node(self, boundary, from_mode):
        node = self.hybrid_node()
        config = dict(node.c)
        config['planner_mode'] = yaml.safe_load((Path(__file__).parents[1]/
            'config/planner_mode.yaml').read_text())['planner_mode']
        config['mode_transition_speed_kph'] = 30.
        config['mode_transition_min_distance_m'] = 20.
        config['mode_transition_reaction_sec'] = .5
        config['mode_transition_handoff_sec'] = .8
        config['rddf_geometry_only'] = True
        node.c = config
        node.planner = Planner(config)
        node.transition_speed = TransitionSpeedPolicy(config)
        node.mode_manager = PlannerModeManager.from_mapping(config['planner_mode'])
        node.mode_manager.select(boundary-.1)
        node.active_planner_mode = from_mode
        node.active_zone = node.mode_manager.current.zone_id
        x = np.arange(0., 2185.1, .5)
        lane = Lane('global_route', np.column_stack((x, np.zeros_like(x), np.zeros_like(x))),
                    x, np.full(len(x), 16.), [])
        node.static_map = ('map-a', {'global_route': lane}, [], [])
        node.route.progress = boundary+.1
        node.route.comparison_goal_s = boundary+30.
        node.state[0].pose.pose.position.x = boundary+.1
        x = np.arange(boundary-3., boundary+33.1, .5)
        node.selected = Candidate('hybrid_astar' if from_mode == HYBRID_ASTAR else 'keep',
            'global_route', np.column_stack((x, np.zeros_like(x), np.zeros_like(x))),
            x, np.full(len(x), 16.), speed=np.full(len(x), 4.),
            times=np.arange(len(x))*.125, feasible=True)
        node.selected_stamp = rospy.Time.from_sec(99.8)
        node.trajectory = TraceOutput()
        node.status = TraceOutput()
        return node

    def plan_without_frenet_candidate(self, node, reason='steering_limit'):
        base = node.selected if node.selected is not None else node.handoff_candidate
        rejected = Candidate('keep', 'global_route', base.xy.copy(),
            base.route_s.copy(), base.limits.copy())

        def reject(candidate, *_args):
            candidate.feasible = False
            candidate.reason = reason
            candidate.cost = candidate.eta = math.inf
            return candidate

        with patch.object(node.planner, 'candidates', return_value=[rejected]), \
                patch.object(node.planner, 'evaluate', side_effect=reject), \
                patch.object(node.planner, 'obstacle_detours', return_value=[]), \
                patch.object(node.planner, 'select', return_value=None):
            node.plan(None)
        return rejected

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_all_three_mode_boundaries_keep_valid_old_path_while_replanning(self, _):
        zones = yaml.safe_load((Path(__file__).parents[1]/
            'config/planner_mode.yaml').read_text())['planner_mode']['zones']
        boundaries = [(zones[0]['end_s'], FRENET, HYBRID_ASTAR),
                      (zones[1]['end_s'], HYBRID_ASTAR, FRENET),
                      (zones[3]['end_s'], FRENET, HYBRID_ASTAR)]
        for boundary, previous, target in boundaries:
            with self.subTest(boundary=boundary):
                node = self.mode_boundary_node(boundary, previous)
                if target == HYBRID_ASTAR:
                    with patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_failure(
                            HybridPlanStatus.TIME_LIMIT)):
                        node.plan(None)
                else:
                    rejected = self.plan_without_frenet_candidate(node)
                    self.assertEqual(len(rejected.speed_cap_mps), len(rejected.xy))
                    self.assertAlmostEqual(rejected.speed_cap_mps[6], 30./3.6, delta=.1)
                node.publish(None)
                self.assertEqual(node.active_planner_mode, target)
                self.assertIsNotNone(node.handoff_candidate)
                self.assertTrue(node.status.message.ready)
                self.assertTrue(all(not message.stop_required for message in node.trajectory.messages))
                self.assertAlmostEqual(node.handoff_deadline.to_sec(), 100.8)

    def test_mode_handoff_stops_when_new_object_blocks_old_path(self):
        boundary = 237.423
        node = self.mode_boundary_node(boundary, FRENET)
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(100)), \
                patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_failure(
                    HybridPlanStatus.TIME_LIMIT)):
            node.plan(None)
            node.publish(None)
        stamp = rospy.Time.from_sec(100.1)
        node.world.objects = [TrackedObject(points=[Point(boundary+10., 0., 0.)],
                                            source_stamp=stamp)]
        node.world.header.stamp = node.route.header.stamp = node.state[0].header.stamp = stamp
        with patch.object(rospy.Time, 'now', return_value=stamp):
            node.publish(None)
        self.assertIsNone(node.handoff_candidate)
        self.assertTrue(node.trajectory.message.stop_required)
        self.assertFalse(node.status.message.ready)
        self.assertIn('mode_handoff_invalidated', node.status.message.reason)

    def test_mode_handoff_expires_from_boundary_time_without_refresh(self):
        node = self.mode_boundary_node(237.423, FRENET)
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(100)), \
                patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_failure(
                    HybridPlanStatus.TIME_LIMIT)):
            node.plan(None)
            node.publish(None)
        stamp = rospy.Time.from_sec(100.81)
        node.world.header.stamp = node.route.header.stamp = node.state[0].header.stamp = stamp
        with patch.object(rospy.Time, 'now', return_value=stamp):
            node.publish(None)
        self.assertIsNone(node.handoff_candidate)
        self.assertTrue(node.trajectory.message.stop_required)
        self.assertFalse(node.status.message.ready)
        self.assertIn('mode_handoff_expired', node.status.message.reason)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_explicit_stop_discards_mode_handoff(self, _):
        node = self.mode_boundary_node(237.423, FRENET)
        with patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_failure(
                HybridPlanStatus.TIME_LIMIT)):
            node.plan(None)
        self.assertIsNotNone(node.handoff_candidate)
        node.defer_stop('route_endpoint_stop')
        node.publish(None)
        self.assertIsNone(node.handoff_candidate)
        self.assertTrue(node.trajectory.message.stop_required)
        self.assertFalse(node.status.message.ready)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_predicted_hazard_stops_frenet_handoff(self, _):
        node = self.mode_boundary_node(635.113, HYBRID_ASTAR)
        node.world.objects = [TrackedObject(points=[Point(645., 5., 0.)])]
        rejected = self.plan_without_frenet_candidate(
            node, reason='predicted_cluster_collision')
        self.assertEqual(rejected.reason, 'predicted_cluster_collision')
        self.assertIsNone(node.handoff_candidate)
        self.assertTrue(node.trajectory.message.stop_required)
        self.assertFalse(node.status.message.ready)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_new_frenet_candidate_replaces_handoff_immediately(self, _):
        node = self.mode_boundary_node(635.113, HYBRID_ASTAR)
        x = np.arange(635.213, 665.213, .5)
        new_path = Candidate('keep', 'global_route',
            np.column_stack((x, np.zeros_like(x), np.zeros_like(x))),
            x, np.full(len(x), 16.))

        def accept(candidate, *_args):
            candidate.feasible = True
            candidate.reason = 'ok'
            candidate.cost = candidate.eta = 1.
            candidate.speed = np.full(len(candidate.xy), 4.)
            candidate.times = np.arange(len(candidate.xy))*.125
            return candidate

        with patch.object(node.planner, 'candidates', return_value=[new_path]), \
                patch.object(node.planner, 'evaluate', side_effect=accept), \
                patch.object(node.planner, 'select', return_value=new_path):
            node.plan(None)
        self.assertIs(node.selected, new_path)
        self.assertIsNone(node.handoff_candidate)
        self.assertFalse(node.pending_selection_set)
        self.assertTrue(node.status.message.ready)
        self.assertTrue(all(not message.stop_required for message in node.trajectory.messages))
        self.assertAlmostEqual(new_path.speed_cap_mps[0], 30./3.6, delta=.1)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_new_hybrid_candidate_replaces_handoff_and_uses_transition_cap(self, _):
        boundary = 237.423
        node = self.mode_boundary_node(boundary, FRENET)
        x = np.arange(boundary+.1, boundary+30.1, .5)
        new_path = Candidate('hybrid_astar', 'global_route',
            np.column_stack((x, np.zeros_like(x), np.zeros_like(x))),
            x, np.full(len(x), 16.))
        with patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_success(new_path)):
            node.plan(None)
        self.assertIs(node.selected, new_path)
        self.assertIsNone(node.pending_selection)
        self.assertIsNone(node.handoff_candidate)
        self.assertTrue(node.status.message.ready)
        self.assertAlmostEqual(new_path.speed_cap_mps[0], 30./3.6, delta=.1)
        self.assertAlmostEqual(new_path.speed_cap_mps[-1], 20./3.6, delta=.01)

    def hybrid_node(self):
        node = self.node()
        node.mode_manager = PlannerModeManager(100., (PlannerZone('Ztest', 0., 100., HYBRID_ASTAR),))
        node.active_planner_mode = HYBRID_ASTAR
        node.active_zone = 'Ztest'
        node.hybrid_fallback_deadline = None
        node.hybrid_runtime_config = yaml.safe_load((Path(__file__).parents[1]/
            'config/hybrid_astar.yaml').read_text())['hybrid_astar']
        node.hybrid_planner = HybridAStarPlanner(HybridAStarConfig())
        node.hybrid_checked_candidate = None
        node.hybrid_checked_scene_key = None
        node.planner = Planner(node.c)
        x = np.arange(0., 101., .5)
        lane = Lane('global_route', np.column_stack((x, np.zeros_like(x), np.zeros_like(x))),
                    x, np.full(len(x), 16.), [])
        node.static_map = ('map-a', {'global_route': lane}, [], [])
        node.route = RouteContext(map_id='map-a', current_lane='global_route',
                                  progress=10., comparison_goal_s=40.)
        node.route.header.stamp = rospy.Time(100)
        node.route_status = None
        node.world = WorldModel(objects_valid=True, localization_reset_id=12)
        node.world.header.stamp = rospy.Time(100)
        node.state[0].header.stamp = rospy.Time(100)
        node.state[1].header.stamp = rospy.Time(100)
        node.epoch = 12
        node.audit = Output()
        node.status = Output()
        x = np.arange(10., 40.1, .5)
        node.selected = Candidate('hybrid_astar', 'global_route',
                                  np.column_stack((x, np.zeros_like(x), np.zeros_like(x))),
                                  x, np.full(len(x), 16.), speed=np.full(len(x), 4.),
                                  times=np.arange(len(x))*.125, feasible=True)
        node.selected_stamp = rospy.Time.from_sec(99.8)
        return node

    def hybrid_failure(self, status):
        plan = HybridPlanResult(status, (), 12, 20, 0, 0, .5)
        return HybridCandidateResult(None, plan)

    def hybrid_success(self, candidate):
        plan = HybridPlanResult(HybridPlanStatus.SUCCESS, (), 12, 20, 0, 0, .2)
        return HybridCandidateResult(candidate, plan)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_timeout_keeps_only_recent_rechecked_path(self, _):
        node = self.hybrid_node()
        selected = node.selected
        with patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_failure(
                HybridPlanStatus.TIME_LIMIT)) as build:
            node.plan(None)
        self.assertIs(node.selected, selected)
        self.assertEqual(node.selected_stamp, rospy.Time.from_sec(99.8))
        self.assertIsNotNone(build.call_args.kwargs['previous_path'])
        self.assertTrue(node.status.message.ready)
        self.assertFalse(node.status.message.stop_required)
        self.assertIn('selected=held', node.status.message.reason)
        node.publish(None)
        self.assertFalse(node.trajectory.message.stop_required)
        with patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(100.31)):
            node.publish(None)
        self.assertIsNone(node.selected)
        self.assertTrue(node.trajectory.message.stop_required)
        self.assertFalse(node.status.message.ready)
        self.assertIn('transient_hold_expired', node.status.message.reason)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_timeout_stops_if_previous_path_has_new_obstacle(self, _):
        node = self.hybrid_node()
        node.world.objects = [TrackedObject(points=[Point(15., 0., 0.)])]
        with patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_failure(
                HybridPlanStatus.TIME_LIMIT)) as build:
            node.plan(None)
        self.assertIsNone(build.call_args.kwargs['previous_path'])
        self.assertIsNone(node.selected)
        self.assertFalse(node.status.message.ready)
        self.assertTrue(node.status.message.stop_required)
        self.assertTrue(node.trajectory.message.valid)
        self.assertTrue(node.trajectory.message.stop_required)
        self.assertEqual(node.trajectory.message.speed_mps, [0., 0.])

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_held_path_is_rechecked_when_scene_changes(self, _):
        node = self.hybrid_node()
        with patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_failure(
                HybridPlanStatus.TIME_LIMIT)):
            node.plan(None)
        self.assertTrue(node.status.message.ready)
        node.world.objects = [TrackedObject(points=[Point(15., 0., 0.)])]
        with patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(100.1)):
            node.world.header.stamp = rospy.Time.from_sec(100.1)
            node.route.header.stamp = rospy.Time.from_sec(100.1)
            node.state[0].header.stamp = rospy.Time.from_sec(100.1)
            node.publish(None)
        self.assertIsNone(node.selected)
        self.assertFalse(node.status.message.ready)
        self.assertTrue(node.trajectory.message.stop_required)
        self.assertIn('transient_hold_invalidated', node.status.message.reason)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_timeout_stops_if_vehicle_departed_from_held_path(self, _):
        node = self.hybrid_node()
        node.state[0].pose.pose.position.y = 2.
        with patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_failure(
                HybridPlanStatus.TIME_LIMIT)) as build:
            node.plan(None)
        self.assertIsNone(build.call_args.kwargs['previous_path'])
        self.assertIsNone(node.selected)
        self.assertFalse(node.status.message.ready)
        self.assertTrue(node.trajectory.message.stop_required)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_success_replaces_obstructed_active_path_without_hold(self, _):
        node = self.hybrid_node()
        node.c['minimum_active_path_hold_sec'] = .5
        node.selected_stamp = rospy.Time.from_sec(99.9)
        node.world.objects = [TrackedObject(points=[Point(25., 0., 0.)])]
        old = node.selected
        x = np.arange(10., 40.1, .5)
        y = 2. * np.sin(np.pi/2 * np.clip((x-10.)/8., 0., 1.))
        detour = Candidate('hybrid_astar', 'global_route',
                           np.column_stack((x, y, np.zeros_like(x))),
                           x, np.full(len(x), 16.))
        with patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_success(detour)), \
                patch.object(module, 'hybrid_path_clear', side_effect=lambda *args: args[3].xy[:,1].max() > 1.):
            node.plan(None)
        self.assertIsNot(node.selected, old)
        self.assertIs(node.selected, detour)
        self.assertIsNone(node.pending_selection)
        self.assertTrue(node.status.message.ready)
        self.assertIn('selected=new', node.status.message.reason)

    def test_hybrid_success_is_rechecked_against_scene_at_completion(self):
        node = self.hybrid_node()
        clock = [rospy.Time(100)]
        candidate = Candidate('hybrid_astar', 'global_route', node.selected.xy.copy(),
                              node.selected.route_s.copy(), node.selected.limits.copy())

        def new_obstacle_during_search(*_args, **_kwargs):
            clock[0] = rospy.Time.from_sec(100.2)
            node.world.header.stamp = node.route.header.stamp = node.state[0].header.stamp = clock[0]
            node.world.objects = [TrackedObject(points=[Point(25., 0., 0.)])]
            return self.hybrid_success(candidate)

        with patch.object(rospy.Time, 'now', side_effect=lambda: clock[0]), \
                patch.object(module, 'build_hybrid_candidate', side_effect=new_obstacle_during_search):
            node.plan(None)
        self.assertIsNone(node.selected)
        self.assertFalse(node.status.message.ready)
        self.assertTrue(node.trajectory.message.stop_required)
        self.assertEqual(node.audit.message.data.find('"rejected_after_search": true') >= 0, True)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_no_path_does_not_reuse_previous_path(self, _):
        node = self.hybrid_node()
        with patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_failure(
                HybridPlanStatus.NO_PATH)):
            node.plan(None)
        self.assertIsNone(node.selected)
        self.assertFalse(node.status.message.ready)
        self.assertTrue(node.status.message.stop_required)
        self.assertTrue(node.trajectory.message.stop_required)
        self.assertIn('status=no_path; selected=stop', node.status.message.reason)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_timeout_does_not_refresh_expired_path_stamp(self, _):
        node = self.hybrid_node()
        node.selected_stamp = rospy.Time.from_sec(99.4)
        with patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_failure(
                HybridPlanStatus.TIME_LIMIT)) as build:
            node.plan(None)
        self.assertIsNotNone(build.call_args.kwargs['previous_path'])
        self.assertIsNone(node.selected)
        self.assertFalse(node.status.message.ready)
        self.assertTrue(node.trajectory.message.stop_required)

    def test_hybrid_pending_path_keeps_its_computation_stamp(self):
        import copy
        node = self.hybrid_node()
        node.c['minimum_active_path_hold_sec'] = .5
        node.selected_stamp = rospy.Time.from_sec(99.6)
        pending = copy.deepcopy(node.selected)
        node.pending_selection = pending
        node.pending_selection_stamp = rospy.Time.from_sec(99.95)
        node.pending_selection_set = True
        with patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(100.1)):
            node.publish(None)
        self.assertIs(node.selected, pending)
        self.assertEqual(node.selected_stamp, rospy.Time.from_sec(99.95))

    def test_hybrid_pending_path_is_rechecked_before_publication(self):
        import copy
        node = self.hybrid_node()
        node.c['minimum_active_path_hold_sec'] = .5
        node.selected_stamp = rospy.Time.from_sec(99.6)
        pending = copy.deepcopy(node.selected)
        x = pending.xy[:, 0]
        pending.xy[:, 1] = 2. * np.sin(np.pi/2 * np.clip((x-10.)/8., 0., 1.))
        node.pending_selection = pending
        node.pending_selection_stamp = rospy.Time.from_sec(100.)
        node.pending_selection_set = True
        node.world.objects = [TrackedObject(points=[Point(25., 2., 0.)])]
        with patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(100.1)):
            node.world.header.stamp = node.route.header.stamp = node.state[0].header.stamp = rospy.Time.from_sec(100.1)
            node.publish(None)
        self.assertIsNone(node.selected)
        self.assertTrue(node.trajectory.message.stop_required)
        self.assertFalse(node.status.message.ready)
        self.assertIn('active_path_invalidated', node.status.message.reason)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_normal_active_path_is_not_expired_by_hold_age(self, _):
        node = self.hybrid_node()
        node.selected_stamp = rospy.Time.from_sec(99.4)
        node.publish(None)
        self.assertIsNotNone(node.selected)
        self.assertFalse(node.trajectory.message.stop_required)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_near_path_with_opposite_heading_is_not_reused(self, _):
        node = self.hybrid_node()
        node.state[0].pose.pose.orientation.z = 1.
        node.state[0].pose.pose.orientation.w = 0.
        with patch.object(module, 'build_hybrid_candidate', return_value=self.hybrid_failure(
                HybridPlanStatus.TIME_LIMIT)) as build:
            node.plan(None)
        self.assertIsNone(build.call_args.kwargs['previous_path'])
        self.assertIsNone(node.selected)
        self.assertFalse(node.status.message.ready)
        self.assertTrue(node.trajectory.message.stop_required)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_heading_check_averages_small_path_kink(self, _):
        node = self.hybrid_node()
        node.selected.xy[1, 1] = .25
        heading = math.radians(20.)
        node.state[0].pose.pose.orientation.z = math.sin(heading / 2.)
        node.state[0].pose.pose.orientation.w = math.cos(heading / 2.)
        node.publish(None)
        self.assertIsNotNone(node.selected)
        self.assertFalse(node.trajectory.message.stop_required)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_route_progress_jitter_before_zone_start_keeps_path(self, _):
        node = self.hybrid_node()
        node.mode_manager = PlannerModeManager(100., (
            PlannerZone('prior', 0., 10., FRENET),
            PlannerZone('Ztest', 10., 100., HYBRID_ASTAR)))
        node.mode_manager.select(10.1)
        node.route.progress = 9.95
        node.publish(None)
        self.assertIsNotNone(node.selected)
        self.assertFalse(node.trajectory.message.stop_required)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_publish_skips_full_check_until_nearby_scene_changes(self, _):
        node = self.hybrid_node()
        with patch.object(module, 'hybrid_path_clear', wraps=module.hybrid_path_clear) as check:
            node.publish(None)
            node.publish(None)
            self.assertEqual(check.call_count, 1)
            node.world.objects = [TrackedObject(points=[Point(25., 0., 0.)])]
            node.publish(None)
            self.assertEqual(check.call_count, 2)
        self.assertTrue(node.trajectory.message.stop_required)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_hybrid_stop_publication_follows_in_flight_motion(self, _):
        node = self.hybrid_node()
        entered = threading.Event()
        release = threading.Event()
        published = []

        def slow_emit(output):
            if not published:
                entered.set()
                self.assertTrue(release.wait(2.))
            published.append(output.stop_required)

        node.emit = slow_emit
        timer = threading.Thread(target=node.publish, args=(None,))
        timer.start()
        self.assertTrue(entered.wait(2.))
        node.offer_selection(None, rospy.Time(100))
        stop = threading.Thread(target=node.publish, args=(None,))
        stop.start()
        release.set()
        timer.join(2.)
        stop.join(2.)
        self.assertFalse(timer.is_alive())
        self.assertFalse(stop.is_alive())
        self.assertEqual(published, [False, True])

    def test_hd_map_connected_rddf_survives_wire_and_planner_ingestion(self):
        from importlib.machinery import SourceFileLoader
        from hd_map_pkg.lane_rddf import SegmentIndex
        from hd_map_pkg.rddf_connections import smooth_connection
        from hd_map_pkg.runtime_map import build_static_map
        source = Path(__file__).parents[2]/'hd_map_pkg/scripts/hd_map_server_node'
        producer = SourceFileLoader('connected_map_producer', str(source)).load_module()
        reference = [[float(x), 0., 0.] for x in range(251)]
        points, stations = smooth_connection(reference, [[0., 3.5, 0.], [200., 3.5, 0.]],
            SegmentIndex({'route': reference}), .5, 80., 45., 15.)
        lane = dict(id='connected', link_id='source', points=points, route_s=stations,
                    source_indices=list(range(len(points))), successors=['global_route'])
        rddf = dict(lanes=[lane], source_hashes={}, forbidden_boundaries=[],
                    forbidden_boundary_ids=[], graph=dict(lane_changes=[],
                    longitudinal_connections=[dict(source_lane='global_route', target_lane='connected')]))
        policy = dict(normal_limit_kph=58., high_speed=dict(start_map_xy=[0., 0.], end_map_xy=[200., 0.]))
        data = build_static_map(reference, rddf, dict(points=[], radius_m=3., source='fixture'),
                                policy, 'fixture-sha')
        message = producer.map_message(data, rospy.Time(1))
        wire = io.BytesIO()
        message.serialize(wire)
        decoded = HdMap().deserialize(wire.getvalue())
        node = self.node()
        for geometry_only in (False, True):
            node.c['rddf_geometry_only'] = geometry_only
            node.static_map = None
            node.on_map(decoded)
            lanes = node.static_map[1]
            connected = lanes['connected']
            np.testing.assert_allclose(connected.xy[[0,-1]], [[0.,0.,0.],[200.,0.,0.]])
            self.assertTrue(np.all(np.diff(connected.s) > 0))
            if not geometry_only:
                chained = Planner(node.c).chain('connected', lanes, 230.)
                self.assertGreaterEqual(chained.s[-1],230.)
                self.assertLessEqual(np.max(np.linalg.norm(np.diff(chained.xy[:,:2],axis=0),axis=1)),1.001)

    def static_map(self):
        message = HdMap(map_id='map-a', reference_sha256='reference-a')
        lane = RouteLane(id='global_route', route_s=[0.,10.,20.,30.], speed_limits_mps=[16.]*4)
        for x in lane.route_s:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.
            lane.centerline.poses.append(pose)
        message.lanes = [lane]
        message.checkpoints = [Point(20.,0.,0.)]
        message.checkpoint_radius_m = 3.
        message.forbidden_boundaries = [Polygon(points=[Point32(0.,5.,0.),Point32(30.,5.,0.)]),
                                        Polygon(points=[Point32(0.,500.,0.),Point32(30.,500.,0.)])]
        return message

    def test_static_map_is_cached_until_version_changes(self):
        node = self.node()
        node.static_map = None
        message = self.static_map()
        node.on_map(message)
        cached = node.static_map
        self.assertEqual(len(cached[3]), 1)
        node.on_map(message)
        self.assertIs(node.static_map, cached)
        message.map_id = 'map-b'
        node.on_map(message)
        self.assertEqual(node.static_map[0], 'map-b')
        self.assertIsNot(node.static_map, cached)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_route_producer_sends_progress_and_planner_uses_static_cache(self, _):
        source = Path(__file__).parents[2]/'global_route_manager_pkg/src/route_manager_node.py'
        spec = importlib.util.spec_from_file_location('route_producer',source)
        producer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(producer)
        route = producer.RouteManagerNode.__new__(producer.RouteManagerNode)
        route.config = dict(initialize_from_current_position=True,matching_backward_m=15.,matching_forward_m=120.,
                            rddf_geometry_only=False,comparison_distance_m=100.)
        route.path_publisher = Output()
        route.context_publisher = Output()
        route.reset_id = None
        route.inputs_ready = lambda: True
        node = self.node()
        route.ego = node.state[0]
        route.ego.header.stamp = rospy.Time(100)
        message = self.static_map()
        route.on_map(message)
        route.publish_context(None)
        context = route.context_publisher.message
        self.assertEqual(context.map_id, message.map_id)
        self.assertAlmostEqual(context.progress,10.)
        self.assertEqual(context.comparison_goal, message.checkpoints[0])
        self.assertFalse(hasattr(context,'lanes'))
        wire = io.BytesIO()
        context.serialize(wire)
        self.assertLess(len(wire.getvalue()),300)
        node.static_map = None
        node.on_map(message)
        node.route = context
        node.world = WorldModel(objects_valid=True,localization_reset_id=12)
        node.world.header.stamp = rospy.Time(100)
        node.route_status = None
        node.epoch = 12
        node.planner = Planner(node.c)
        node.audit = Output()
        node.report = lambda *args: None
        node.plan(None)
        self.assertIsNotNone(node.selected)
        self.assertEqual(node.selected.target, 'global_route')

        route.config['rddf_geometry_only'] = True
        geometry_message = SimpleNamespace(map_id=message.map_id,reference_sha256=message.reference_sha256,
            lanes=[SimpleNamespace(id=l.id,centerline=l.centerline,route_s=l.route_s) for l in message.lanes])
        route.on_map(geometry_message)
        route.publish_context(None)
        self.assertEqual(route.context.next_checkpoint,0xFFFFFFFF)
        self.assertFalse(route.progress.missed_checkpoint)
        node.c['rddf_geometry_only'] = True
        node.static_map = None
        node.on_map(geometry_message)
        node.route = route.context
        node.route_status = ComponentStatus(state=ComponentStatus.FAULT,reason='old_checkpoint_fault')
        node.plan(None)
        self.assertIsNotNone(node.selected)
        self.assertEqual(node.static_map[3],[])

    def test_geometry_only_reads_no_hd_map_road_metadata(self):
        node = self.node()
        node.c['rddf_geometry_only'] = True
        node.static_map = None
        original = self.static_map()
        # Deliberately omit every HD Map road-attribute field.
        message = SimpleNamespace(map_id='geometry-only',lanes=[SimpleNamespace(
            id=l.id,centerline=l.centerline,route_s=l.route_s) for l in original.lanes])
        node.on_map(message)
        self.assertEqual(node.static_map[3],[])
        self.assertEqual(node.static_map[1]['global_route'].successors,[])
        self.assertAlmostEqual(node.static_map[1]['global_route'].limits[0],58/3.6)

    def test_high_speed_route_distance_reaches_planner_candidates(self):
        from path_planning_pkg.frenet import Lane, Window
        source = Path(__file__).parents[2]/'global_route_manager_pkg/src/route_manager_node.py'
        spec = importlib.util.spec_from_file_location('high_speed_route_producer', source)
        producer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(producer)
        route = producer.RouteManagerNode.__new__(producer.RouteManagerNode)
        route.config = yaml.safe_load((Path(__file__).parents[2]/
            'global_route_manager_pkg/config/route_manager.yaml').read_text())
        route.path_publisher = Output()
        message = self.static_map()
        lane = message.lanes[0]
        lane.centerline.poses = []
        lane.route_s = list(np.arange(0., 1001., 1.))
        for x in lane.route_s:
            pose = PoseStamped()
            pose.pose.position.x = x
            lane.centerline.poses.append(pose)
        policy = dict(high_speed=dict(start_map_xy=[200.,0.], end_map_xy=[700.,0.]))
        with patch.object(producer, 'load_course_speed_policy', return_value=policy):
            route.on_map(message)
        for position, distance in [(199.,100.), (200.,300.), (699.,300.), (700.,100.)]:
            state = route.progress.update((position,0.,0.),0.)
            context = RouteContext(progress=state['progress'], comparison_goal_s=state['comparison_goal_s'])
            wire = io.BytesIO()
            context.serialize(wire)
            context = RouteContext().deserialize(wire.getvalue())
            self.assertAlmostEqual(context.comparison_goal_s-context.progress,distance)
            self.assertAlmostEqual(state['comparison_goal'][0], position+distance)
            stations = np.arange(0.,1001.,.5)
            lanes = {key: Lane(key,np.column_stack((stations,stations*0+y,stations*0)),
                              stations,stations*0-1,[]) for key,y in [('global_route',0.),('side',3.5)]}
            config = yaml.safe_load((Path(__file__).parents[1]/'config/frenet_planner.yaml').read_text())
            candidates = Planner(config).candidates(lanes,[Window('global_route','side',0.,1000.)],
                'global_route',context.progress,context.comparison_goal_s,np.array([position,0.,0.]),0.,150/3.6)
            self.assertEqual(len(candidates)>1,distance==300.)
        # A high-speed zone crossing the route seam retains its interval semantics.
        route.progress.high_speed_interval = (700.,200.)
        for position,distance in [(0.,300.),(199.,300.),(200.,100.),(700.,300.),(950.,300.)]:
            state = route.progress.update((position,0.,0.),0.)
            self.assertAlmostEqual(state['comparison_goal_s']-position,distance)
        route.progress.config['loop_route'] = False
        state = route.progress.update((950.,0.,0.),0.)
        self.assertEqual(state['comparison_goal_s'],1000.)

    def test_progress_without_checkpoints_uses_forward_rddf_station(self):
        from global_route_manager_pkg.progress import RouteProgress
        lanes=[dict(id='global_route',points=[(0.,0.,0.),(200.,0.,0.)],route_s=[0.,200.])]
        p=RouteProgress(lanes,[],0.,dict(initialize_from_current_position=True,
            matching_backward_m=15.,matching_forward_m=120.,comparison_distance_m=100.))
        result=p.update((20.,0.,0.),0.)
        self.assertEqual(result['next_checkpoint'],0xFFFFFFFF)
        self.assertAlmostEqual(result['comparison_goal_s'],120.)
        self.assertFalse(p.missed_checkpoint)
        result=p.update((150.,0.,0.),0.)
        self.assertFalse(p.missed_checkpoint)

    def test_route_completes_within_terminal_tracking_tolerance(self):
        from global_route_manager_pkg.progress import RouteProgress
        lanes=[dict(id='global_route',points=[(0.,0.,0.),(200.,0.,0.)],route_s=[0.,200.])]
        config=dict(initialize_from_current_position=True,matching_backward_m=15.,matching_forward_m=120.,
                    comparison_distance_m=100.,route_completion_tolerance_m=3.)
        result=RouteProgress(lanes,[],0.,config).update((197.1,0.,0.),0.)
        self.assertTrue(result['route_complete'])

    def test_geometry_route_restarts_at_nearest_station_after_reposition(self):
        from global_route_manager_pkg.progress import RouteProgress
        lanes=[dict(id='global_route',points=[(0.,0.,0.),(200.,0.,0.)],route_s=[0.,200.])]
        config=dict(rddf_geometry_only=True,initialize_from_current_position=True,
                    matching_backward_m=15.,matching_forward_m=120.,comparison_distance_m=100.,
                    route_completion_tolerance_m=3.)
        route=RouteProgress(lanes,[],0.,config)
        self.assertTrue(route.update((199.,0.,0.),0.)['route_complete'])
        result=route.update((20.,1.,0.),0.)
        self.assertAlmostEqual(result['progress'],20.)
        self.assertFalse(result['route_complete'])
        self.assertFalse(route.missed_checkpoint)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(100.75))
    def test_plan_remains_active_until_a_new_result(self, _):
        node = self.node()
        self.assertEqual(node.c['input_age_sec'], 0.5)
        node.publish(None)
        self.assertFalse(node.trajectory.message.stop_required)
        with patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(101.01)):
            node.publish(None)
        self.assertFalse(node.trajectory.message.stop_required)

    def test_new_path_and_lower_speed_are_applied_immediately(self):
        node = self.node()
        first = node.selected
        slower = Candidate('slower','global_route',first.xy.copy(),first.route_s.copy(),
                           first.limits.copy(),speed=first.speed.copy()*.5,times=first.times.copy())
        node.offer_selection(slower,rospy.Time.from_sec(100.01))
        self.assertIs(node.selected,slower)
        self.assertEqual(node.selected_stamp,rospy.Time.from_sec(100.01))
        self.assertFalse(node.pending_selection_set)
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(100.02)):
            node.publish(None)
        self.assertFalse(node.trajectory.message.stop_required)
        self.assertLessEqual(max(node.trajectory.message.speed_mps), max(slower.speed))

    def test_input_failure_stops_without_hold(self):
        node = self.node()
        node.report = lambda reason,ready=False,latency=0.: setattr(node,'last_report',(reason,ready))
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(100.01)):
            node.defer_stop('input_unusable')
            node.publish(None)
        self.assertIsNone(node.selected)
        self.assertEqual(node.last_report,('input_unusable',False))
        self.assertTrue(node.trajectory.message.stop_required)

    def test_normal_path_holds_half_second_and_activates_latest_result(self):
        import copy
        node = self.node()
        node.c['minimum_active_path_hold_sec'] = 0.5
        first = node.selected
        first.speed = np.full(4, 2.)
        first.times = np.arange(4.)*.5
        second, latest = copy.deepcopy(first), copy.deepcopy(first)
        node.offer_selection(second, rospy.Time.from_sec(100.1))
        node.offer_selection(latest, rospy.Time.from_sec(100.4))
        with patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(100.499)):
            node.publish(None)
        self.assertIs(node.selected, first)
        self.assertIs(node.pending_selection, latest)
        with patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(100.5)):
            node.publish(None)
        self.assertIs(node.selected, latest)
        self.assertEqual(node.selected_stamp, rospy.Time.from_sec(100.5))
        self.assertFalse(node.pending_selection_set)
        self.assertFalse(node.trajectory.message.stop_required)

    def test_stop_discards_pending_path_during_hold(self):
        import copy
        for stop_candidate in (False, True):
            node = self.node()
            node.c['minimum_active_path_hold_sec'] = 0.5
            node.selected.speed = np.full(4, 2.)
            pending = copy.deepcopy(node.selected)
            node.offer_selection(pending, rospy.Time.from_sec(100.1))
            stop = copy.deepcopy(pending) if stop_candidate else None
            if stop is not None:
                stop.speed[-1] = 0.
            node.offer_selection(stop, rospy.Time.from_sec(100.2))
            self.assertIs(node.selected, stop)
            self.assertFalse(node.pending_selection_set)
            with patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(100.7)):
                node.publish(None)
            self.assertIs(node.selected, stop)

    def test_clock_reversal_clears_held_and_pending_paths(self):
        node = self.node()
        node.pending_selection = node.selected
        node.pending_selection_set = True
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(99)):
            node.publish(None)
        self.assertIsNone(node.selected)
        self.assertFalse(node.pending_selection_set)
        self.assertTrue(node.trajectory.message.stop_required)

    def test_hold_parameter_matches_central_runtime_profile(self):
        root = Path(__file__).parents[2]
        runtime = yaml.safe_load((root/'ros_architecture_pkg/config/messages/frenet_runtime.yaml').read_text())
        config = yaml.safe_load((Path(__file__).parents[1]/'config/frenet_planner.yaml').read_text())
        self.assertEqual(config['minimum_active_path_hold_sec'], 0.5)
        self.assertEqual(config['minimum_active_path_hold_sec'], runtime['minimum_active_path_hold_sec'])
        self.assertEqual(config['world_model_input_age_sec'], runtime['world_model_max_input_age_sec'])

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_development_scene_jitter_tolerance_keeps_other_input_gates(self, _):
        scenarios = [
            # geometry-only, scene age, ego age, scene valid, matching reset, accepted
            (True, .65, .1, True, True, True),
            (True, .7, .1, True, True, True),
            (True, .701, .1, True, True, False),
            (False, .65, .1, True, True, False),
            (True, .65, .51, True, True, False),
            (True, .65, .1, False, True, False),
            (True, .65, .1, True, False, False),
            (True, -.01, .1, True, True, False),
        ]
        for geometry_only, age, ego_age, valid, matching, accepted in scenarios:
            with self.subTest(geometry_only=geometry_only, age=age, ego_age=ego_age,
                              valid=valid, matching=matching):
                node = self.node()
                node.c['rddf_geometry_only'] = geometry_only
                node.static_map = None
                node.on_map(self.static_map())
                node.route = RouteContext(map_id='map-a', current_lane='global_route',
                                          progress=10., comparison_goal_s=20.)
                node.route.header.stamp = rospy.Time(100)
                node.state[0].header.stamp = rospy.Time.from_sec(100-ego_age)
                node.world = WorldModel(objects_valid=valid, localization_reset_id=12 if matching else 11)
                node.world.header.stamp = rospy.Time(100)-rospy.Duration(age)
                original_stamp = node.world.header.stamp
                node.route_status = None
                node.epoch = 12
                node.planner = Planner(node.c)
                node.audit = Output()
                node.report = lambda reason, ready=False, latency=0.: setattr(node, 'last_ready', ready)
                node.plan(None)
                self.assertEqual(node.last_ready, accepted)
                self.assertEqual(node.world.header.stamp, original_stamp)
                if not accepted:
                    self.assertIsNone(node.selected)

    def test_valid_path_leaves_active_stop_immediately(self):
        node = self.node()
        candidate = node.selected
        node.selected = None
        node.selected_stamp = rospy.Time.from_sec(100.0)
        node.offer_selection(candidate,rospy.Time.from_sec(100.2))
        self.assertIs(node.selected,candidate)
        self.assertFalse(node.pending_selection_set)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_incomplete_search_does_not_publish_a_temporary_stop(self, _):
        self.check_search_publication('steering_limit', False, True)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_completed_search_without_a_candidate_still_stops(self, _):
        self.check_search_publication('steering_limit', False, False)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_known_hazards_stop_before_alternative_search_finishes(self, _):
        for reason in ('predicted_cluster_collision', 'insufficient_stopping_distance',
                       'no_collision_free_stop', 'forbidden_boundary'):
            with self.subTest(reason=reason):
                self.check_search_publication(reason, True, True)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(100.1))
    def test_held_geometry_is_rechecked_before_pending_replacement(self, _):
        self.check_search_publication('steering_limit', True, True, hold_active=True)

    def check_search_publication(self, reason, interim_stop, alternative_valid, hold_active=False):
        import copy
        node = self.node()
        node.c['rddf_geometry_only'] = True
        node.static_map = None
        node.on_map(self.static_map())
        node.route = RouteContext(map_id='map-a', current_lane='global_route',
                                  progress=10., comparison_goal_s=20.)
        node.world = WorldModel(objects_valid=True, localization_reset_id=12)
        node.state[0].header.stamp = node.route.header.stamp = node.world.header.stamp = rospy.Time(100)
        node.route_status = None
        node.epoch = 12
        node.planner = Planner(node.c)
        node.audit = Output()
        node.report = lambda *args: None
        if hold_active:
            node.c['minimum_active_path_hold_sec'] = .5
            node.selected.speed = np.full(4, 2.)
            node.selected.times = np.arange(4.)*.5
        fast, alternative = copy.deepcopy(node.selected), copy.deepcopy(node.selected)
        alternative.key = 'alternative'
        fast.feasible, fast.reason = False, reason
        alternative.feasible = alternative_valid
        active_checks = []

        def evaluate(candidate, *args):
            if hold_active and candidate.key == 'committed':
                active_checks.append(candidate)
                candidate.feasible = False
                candidate.reason = 'predicted_cluster_collision'
            if candidate is alternative:
                # Simulate the trajectory timer firing while alternatives are evaluated.
                node.publish(None)
                self.assertEqual(node.trajectory.message.stop_required, interim_stop)
            return candidate

        with patch.object(node.planner, 'candidates', return_value=[fast, alternative]), \
                patch.object(node.planner, 'obstacle_detours', return_value=[]), \
                patch.object(node.planner, 'evaluate', side_effect=evaluate), \
                patch.object(node.planner, 'select', return_value=alternative if alternative_valid else None):
            node.plan(None)
        node.publish(None)
        self.assertEqual(node.trajectory.message.stop_required, not alternative_valid)
        self.assertIs(node.selected, alternative if alternative_valid else None)
        if hold_active:
            self.assertEqual(len(active_checks), 1)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_blocked_commit_evaluates_recovery_and_generates_detours(self, _):
        import copy
        node = self.node()
        node.c['rddf_geometry_only'] = True
        node.static_map = None
        node.on_map(self.static_map())
        node.route = RouteContext(map_id='map-a', current_lane='global_route',
                                  progress=10., comparison_goal_s=20.)
        node.world = WorldModel(objects_valid=True, localization_reset_id=12)
        node.state[0].header.stamp = node.route.header.stamp = node.world.header.stamp = rospy.Time(100)
        node.route_status = None
        node.epoch = 12
        node.planner = Planner(node.c)
        node.audit = Output()
        node.report = lambda *args: None
        held = copy.deepcopy(node.selected)
        held.route_s = np.array([10.,11.,12.,13.])
        held.change_end = 25.
        node.planner.committed = held
        keep, recovery = copy.deepcopy(held), copy.deepcopy(held)
        recovery.key = 'recovery'
        evaluated = []

        def evaluate(candidate, *args):
            evaluated.append(candidate.key)
            candidate.feasible = candidate.key != 'keep'
            candidate.cost = candidate.eta = 5. if candidate.key == 'recovery' else np.inf
            candidate.reason = 'feasible' if candidate.key == 'recovery' else 'stop_wait_prediction_unresolved'
            return candidate

        with patch.object(node.planner, 'candidates', return_value=[keep, recovery]), \
                patch.object(node.planner, 'obstacle_detours', return_value=[]) as detours, \
                patch.object(node.planner, 'evaluate', side_effect=evaluate):
            node.plan(None)
        self.assertEqual(evaluated, ['committed', 'keep', 'recovery'])
        detours.assert_called_once()
        self.assertIs(node.selected, recovery)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_world_model_crossing_clearance_survives_trajectory_serialization(self, _):
        node = self.node()
        node.c['rddf_geometry_only'] = True
        node.static_map = None
        node.on_map(self.static_map())
        node.route = RouteContext(map_id='map-a', current_lane='global_route',
                                  progress=10., comparison_goal_s=20.)
        node.route.header.stamp = node.state[0].header.stamp = rospy.Time(100)
        node.state[1].twist.twist.linear.x = 2.
        world = WorldModel(objects_valid=True, localization_reset_id=12)
        world.header.stamp = rospy.Time(100)
        obj = TrackedObject(velocity_valid=True, source_stamp=rospy.Time(100))
        obj.points = [Point(25.,0.,0.)]
        obj.twist.linear.y = 2.
        world.objects = [obj]
        wire = io.BytesIO()
        world.serialize(wire)
        node.world = WorldModel().deserialize(wire.getvalue())
        node.route_status = None
        node.epoch = 12
        node.planner = Planner(node.c)
        node.audit = Output()
        node.report = lambda *args: None
        node.plan(None)
        node.publish(None)
        message = node.trajectory.message
        wire = io.BytesIO()
        message.serialize(wire)
        decoded = type(message)().deserialize(wire.getvalue())
        self.assertTrue(decoded.valid)
        self.assertFalse(decoded.stop_required)
        self.assertGreater(min(decoded.speed_mps[:10]), 0.)
        self.assertEqual(decoded.reset_id, 12)
        self.assertEqual(decoded.header.frame_id, 'odom')
        self.assertTrue(np.all(np.diff([t.to_sec() for t in decoded.time_from_start]) > 0.))

    def node(self):
        node = module.Node.__new__(module.Node)
        node.c = yaml.safe_load((Path(__file__).parents[1]/'config/frenet_planner.yaml').read_text())
        node.c['rddf_geometry_only'] = False
        node.c['minimum_active_path_hold_sec'] = 0.0  # Unrelated fixtures test immediate selection.
        node.mode_manager = PlannerModeManager(2184.6117233360674, (
            PlannerZone('legacy-frenet-test', 0.0, 2184.6117233360674, FRENET),
        ))
        node.transition_speed = SimpleNamespace(caps=lambda route_s, _mode, _lane:
            np.full(len(route_s), np.inf))
        node.active_planner_mode = FRENET
        node.active_zone = 'legacy-frenet-test'
        node.handoff_candidate = None
        node.handoff_stamp = None
        node.handoff_deadline = None
        node.handoff_reset_id = None
        node.handoff_from_mode = None
        node.handoff_target_mode = None
        node.handoff_started = None
        node.handoff_checked_scene_key = None
        ego, odom = EgoState(), Odometry()
        ego.reset_id = 12
        ego.pose.pose.position.x = 10.
        ego.pose.pose.orientation.w = 1.
        odom.pose.pose.position.x = 100.
        odom.pose.pose.position.y = 200.
        odom.pose.pose.orientation.z = odom.pose.pose.orientation.w = np.sqrt(.5)
        node.state = ego, odom
        node.lock = threading.Lock()
        node.publish_lock = threading.Lock()
        node.last_lane_change_evaluation = -np.inf
        node.pending_selection = None
        node.pending_selection_set = False
        node.pending_selection_stamp = None
        node.trajectory = Output()
        node.path_markers = Output()
        node.selected_stamp = rospy.Time(100)
        node.selected = Candidate('keep', 'global_route', np.array([[10.,0.,0.],[11.,0.,0.],[12.,0.,0.],[13.,0.,0.]]),
                                  np.arange(4.), np.full(4, 2.), speed=np.array([2.,1.,0.,0.]),
                                  times=np.array([0.,2/3,8/3,np.inf]))
        return node

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_future_stop_preserves_frame_epoch_and_monotonic_time(self, _):
        node = self.node()
        node.publish(None)
        message = node.trajectory.message
        wire = io.BytesIO()
        message.serialize(wire)
        message = type(message)().deserialize(wire.getvalue())
        self.assertEqual(message.header.frame_id, 'odom')
        self.assertEqual(message.reset_id, 12)
        self.assertTrue(message.valid)
        self.assertFalse(message.stop_required)
        self.assertEqual(message.speed_mps[-1], 0.)
        self.assertAlmostEqual(message.poses[0].position.x, 100.)
        self.assertAlmostEqual(message.poses[0].position.y, 200.)
        self.assertAlmostEqual(message.poses[-1].position.y, 202.)
        self.assertEqual(len(message.poses), len(message.speed_mps))
        self.assertEqual(len(message.poses), len(message.time_from_start))
        self.assertTrue(np.all(np.diff([x.to_sec() for x in message.time_from_start])>0))

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_no_candidate_stops_at_current_odometry_pose(self, _):
        node = self.node()
        node.selected = None
        node.publish(None)
        message = node.trajectory.message
        self.assertTrue(message.valid and message.stop_required)
        self.assertEqual(message.speed_mps, [0., 0.])
        self.assertEqual(message.poses[0], node.state[1].pose.pose)

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_tracking_error_does_not_deform_published_path(self, _):
        node = self.node()
        node.state[0].pose.pose.position.y = 2.
        node.publish(None)
        message = node.trajectory.message
        # map y=0 remains odom x=102 for EVERY point, including the first.
        # Moving only point zero to the ego pose creates a fictitious sharp turn.
        np.testing.assert_allclose([p.position.x for p in message.poses], 102.)
        self.assertTrue(np.all(np.diff([p.position.y for p in message.poses]) > 0))

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_localization_reset_discards_active_and_pending_paths(self, _):
        node = self.node()
        node.pending_selection = node.selected
        node.pending_selection_set = True
        node.pending_selection_stamp = rospy.Time(100)
        ego, odom = node.state
        import copy
        ego = copy.deepcopy(ego)
        ego.reset_id += 1
        ego.pose.pose.position.x = 500.
        node.on_state(ego, odom)
        node.publish(None)
        self.assertTrue(node.trajectory.message.stop_required)
        self.assertIsNone(node.selected)
        self.assertIsNone(node.pending_selection)

    def test_result_computed_before_reset_cannot_be_activated(self):
        node = self.node()
        candidate = node.selected
        node.offer_selection(candidate, rospy.Time(102), reset_id=11)
        self.assertEqual(node.selected_stamp, rospy.Time(100))

    def test_world_model_box_selects_and_publishes_local_detour(self):
        node = self.node()
        node.c['rddf_geometry_only'] = True
        message = self.static_map()
        lane = message.lanes[0]
        lane.route_s = list(np.arange(0.,151.,.5))
        lane.centerline.poses = []
        lane.speed_limits_mps = [16.]*len(lane.route_s)
        for x in lane.route_s:
            p = PoseStamped()
            p.pose.position.x, p.pose.orientation.w = x, 1.
            lane.centerline.poses.append(p)
        node.static_map = None
        node.on_map(message)
        node.route = RouteContext(map_id='map-a',current_lane='global_route',progress=10.,comparison_goal_s=110.)
        obj = TrackedObject(points=[Point(19., y, 0.) for y in [-.25,.5,1.2]],velocity_valid=True)
        node.world = WorldModel(objects_valid=True,localization_reset_id=12,objects=[obj])
        node.route_status = None
        node.epoch = 12
        node.planner = Planner(node.c)
        node.audit = Output()
        node.report = lambda *args: None
        for sec in [100.,101.]:
            stamp = rospy.Time.from_sec(sec)
            node.state[0].header.stamp = node.route.header.stamp = node.world.header.stamp = obj.source_stamp = stamp
            node.last_lane_change_evaluation = -np.inf
            with patch.object(rospy.Time,'now',return_value=stamp):
                node.plan(None)
                node.publish(None)
        self.assertTrue(node.planner.committed.key.startswith('detour:'))
        stamp = rospy.Time(102)
        node.state[0].header.stamp = node.route.header.stamp = node.world.header.stamp = obj.source_stamp = stamp
        node.last_lane_change_evaluation = -np.inf
        with patch.object(node.planner, 'evaluate', wraps=node.planner.evaluate) as evaluate:
            with patch.object(rospy.Time, 'now', return_value=stamp):
                node.plan(None)
            self.assertEqual(evaluate.call_count, 1)
            self.assertEqual(evaluate.call_args[0][0].key, 'committed')
        with patch.object(rospy.Time,'now',return_value=rospy.Time(102)):
            node.publish(None)
        output = node.trajectory.message
        self.assertFalse(output.stop_required)
        self.assertGreater(len(output.poses),20)
        # Map lateral offset transforms to negative odom x at this 90-degree pose.
        direction = np.sign(float(node.planner.committed.key.split(':')[1]))
        self.assertGreater(max(direction*(100.-p.position.x) for p in output.poses),1.)
        wire = io.BytesIO()
        output.serialize(wire)
        decoded = type(output)().deserialize(wire.getvalue())
        self.assertEqual(len(decoded.speed_mps),len(decoded.poses))
        self.assertTrue(np.all(np.diff([t.to_sec() for t in decoded.time_from_start])>0))

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_remaining_terminal_geometry_has_no_single_point_crash(self, _):
        node = self.node()
        node.state[0].pose.pose.position.x = 13.
        node.publish(None)
        message = node.trajectory.message
        self.assertGreaterEqual(len(message.poses), 2)
        self.assertEqual(message.speed_mps[-1], 0.)
        self.assertTrue(np.all(np.diff([t.to_sec() for t in message.time_from_start]) > 0))

    @patch.object(rospy.Time, 'now', return_value=rospy.Time(100))
    def test_no_finite_forward_interval_publishes_stop(self, _):
        node = self.node()
        node.selected.times = np.array([0., np.inf, np.inf, np.inf])
        node.publish(None)
        self.assertTrue(node.trajectory.message.stop_required)


if __name__ == '__main__':
    unittest.main()
