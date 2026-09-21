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
from common_msgs_pkg.msg import EgoState, HdMap, RouteLane, RouteContext, WorldModel, ComponentStatus
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


if __name__ == '__main__':
    unittest.main()
