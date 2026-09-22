import importlib.util
import io
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
from path_planning_pkg.frenet import Candidate, Planner


spec = importlib.util.spec_from_file_location('frenet_node', Path(__file__).parents[1]/'src/frenet_planner_node.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Output:
    def publish(self, message):
        self.message = message


class FrenetOutputTest(unittest.TestCase):
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

    def test_active_path_holds_one_second_and_activates_latest_result(self):
        node = self.node()
        first = node.selected
        second = Candidate('second','global_route',first.xy.copy(),first.route_s.copy(),first.limits.copy())
        latest = Candidate('latest','global_route',first.xy.copy(),first.route_s.copy(),first.limits.copy(),
                           speed=first.speed.copy(),times=first.times.copy())
        node.offer_selection(second,rospy.Time.from_sec(100.4))
        node.offer_selection(latest,rospy.Time.from_sec(100.8))
        self.assertIs(node.selected,first)
        self.assertIs(node.pending_selection,latest)
        self.assertTrue(node.pending_selection_set)
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(101.0)):
            node.publish(None)
        self.assertIs(node.selected,latest)
        self.assertIsNone(node.pending_selection)
        self.assertFalse(node.pending_selection_set)
        self.assertEqual(node.selected_stamp,rospy.Time.from_sec(101.0))

    def test_deferred_stop_keeps_active_path_until_hold_finishes(self):
        node = self.node()
        node.report = lambda reason,ready=False,latency=0.: setattr(node,'last_report',(reason,ready))
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(100.2)):
            node.defer_stop('input_unusable')
        self.assertIsNotNone(node.selected)
        self.assertTrue(node.pending_selection_set)
        self.assertIsNone(node.pending_selection)
        self.assertEqual(node.last_report,('input_unusable; holding_active_path',True))
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(101.19)):
            node.publish(None)
        self.assertFalse(node.trajectory.message.stop_required)
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(101.2)):
            node.publish(None)
        self.assertTrue(node.trajectory.message.stop_required)

    def test_valid_path_leaves_active_stop_immediately(self):
        node = self.node()
        candidate = node.selected
        node.selected = None
        node.selected_stamp = rospy.Time.from_sec(100.0)
        node.offer_selection(candidate,rospy.Time.from_sec(100.2))
        self.assertIs(node.selected,candidate)
        self.assertFalse(node.pending_selection_set)

    def node(self):
        node = module.Node.__new__(module.Node)
        node.c = yaml.safe_load((Path(__file__).parents[1]/'config/frenet_planner.yaml').read_text())
        node.c['rddf_geometry_only'] = False
        ego, odom = EgoState(), Odometry()
        ego.reset_id = 12
        ego.pose.pose.position.x = 10.
        ego.pose.pose.orientation.w = 1.
        odom.pose.pose.position.x = 100.
        odom.pose.pose.position.y = 200.
        odom.pose.pose.orientation.z = odom.pose.pose.orientation.w = np.sqrt(.5)
        node.state = ego, odom
        node.lock = threading.Lock()
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
        with patch.object(rospy.Time,'now',return_value=rospy.Time(102)):
            node.publish(None)
        output = node.trajectory.message
        self.assertFalse(output.stop_required)
        self.assertGreater(len(output.poses),20)
        # Left in map (+y) transforms to negative odom x at this 90-degree pose.
        self.assertLess(min(p.position.x for p in output.poses),99.)
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
