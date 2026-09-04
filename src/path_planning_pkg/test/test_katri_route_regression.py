#!/usr/bin/env python3

import math
import unittest
from pathlib import Path

from hd_map_pkg.coordinates import CoordinateTransformer
from hd_map_pkg.mgeo_v3 import MGeoV3Dataset
from path_planning_pkg import (
    BoundaryMarking,
    BoxObstacle,
    CorridorMode,
    CorridorPolicyInput,
    HybridAStarConfig,
    HybridAStarPlanner,
    PlanStatus,
    PlanningRequest,
    VehicleGeometry,
)
from path_planning_pkg.mgeo_adapter import LaneChangePair, MGeoPlannerMap
from path_planning_pkg.node_safety import (
    lane_change_reference,
    select_forward_valid_goal,
)


MAP_CONFIG = {
    "conversion": {
        "geometry_simplification_m": 0.0,
        "default_lane_width_m": 3.5,
        "node_deduplication_m": 0.001,
        "boundary_event_merge_tolerance_m": 0.50,
        "boundary_stitch_tolerance_m": 0.50,
        "minimum_lanelet_segment_length_m": 0.50,
        "endpoint_snap_taper_m": 10.0,
    },
    "validation": {
        "max_boundary_to_center_distance_m": 30.0,
        "max_successor_endpoint_gap_m": 5.0,
    },
    "lane_boundary": {
        "stop_line_codes": [530],
        "road_border_codes": [505, 531],
        "centerline_codes": [501],
        "thick_line_codes": [502],
        "ordinary_lane_codes": [503, 504, 506, 515, 525],
        "standalone_marking_codes": [535],
    },
}


class KATRIRoutePlannerRegressionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        package = Path(__file__).resolve().parents[1]
        source = (
            package.parent
            / "hd_map_pkg"
            / "vendor"
            / "verdict_sdk"
            / "map-data"
            / "KATRI"
        )
        dataset = MGeoV3Dataset(source)
        transformer = CoordinateTransformer(
            dataset.local_origin_utm, (302595.0, 4124145.0, 0.0)
        )
        cls.map = MGeoPlannerMap(
            dataset,
            transformer,
            MAP_CONFIG,
            simplification_tolerance_m=0.020,
        )

        vehicle = VehicleGeometry(
            wheelbase_m=3.000,
            length_m=4.635,
            width_m=1.892,
            front_overhang_m=0.845,
            rear_overhang_m=0.790,
            wheel_track_m=1.892,
            minimum_turning_radius_m=5.87,
        )
        maximum_steering = vehicle.maximum_kinematic_steering_rad
        config = HybridAStarConfig(
            xy_resolution_m=1.00,
            yaw_resolution_rad=math.radians(15.0),
            primitive_length_m=3.00,
            collision_check_step_m=0.20,
            steering_candidates_rad=tuple(
                maximum_steering * fraction
                for fraction in (
                    -1.0,
                    -0.8,
                    -0.6,
                    -0.4,
                    -0.2,
                    0.0,
                    0.2,
                    0.4,
                    0.6,
                    0.8,
                    1.0,
                )
            ),
            goal_position_tolerance_m=1.75,
            goal_yaw_tolerance_rad=math.radians(20.0),
            steering_cost_weight=0.20,
            steering_change_cost_weight=0.35,
            boundary_cost_weight=1.50,
            preferred_boundary_clearance_m=0.75,
            bearing_heuristic_weight_m_per_rad=3.0,
            yaw_heuristic_weight_m_per_rad=4.0,
            reference_lateral_heuristic_weight=1.5,
            reference_lookahead_m=7.0,
            maximum_steering_expansions=7,
            max_search_nodes=20000,
            # Keep this map regression deterministic on shared CI hosts. Runtime
            # deadline behavior has a separate focused unit test.
            maximum_planning_time_sec=2.0,
        )
        cls.planner = HybridAStarPlanner(
            vehicle=vehicle,
            config=config,
            boundary_clearance_m=0.295,
            obstacle_clearance_m=0.50,
        )

    @staticmethod
    def _route_policy(build, source_link_id):
        return CorridorPolicyInput(
            mode=(
                CorridorMode.TURN_CONNECTOR
                if build.open_boundary_ids
                else CorridorMode.KEEP_LANE
            ),
            current_lane_id=source_link_id,
            requested_open_boundary_ids=frozenset(build.open_boundary_ids),
            turn_connector_verified=bool(build.topology_verified),
        )

    def test_forward_goal_selection_crosses_three_unsafe_route_seams(self):
        cases = (
            (
                "A2256W000748",
                54.0,
                (
                    "A2256W000751",
                    "A2256W000748",
                    "A2256W000182",
                    "A2256W000329",
                ),
                ("A2256W000748", "A2256W000182", "A2256W000329"),
                "A2256W000182",
            ),
            (
                "A2256W000148",
                8.0,
                (
                    "A2256W000304",
                    "A2256W000148",
                    "A2256W000146",
                    "A2256W000151",
                ),
                ("A2256W000148", "A2256W000146", "A2256W000151"),
                "A2256W000146",
            ),
            (
                "A2256W000146",
                11.0,
                (
                    "A2256W000148",
                    "A2256W000146",
                    "A2256W000151",
                    "A2256W000866",
                ),
                ("A2256W000146", "A2256W000151", "A2256W000866"),
                "A2256W000866",
            ),
        )

        for source_link_id, start_progress, selected, forward, expected_goal in cases:
            with self.subTest(source_link_id=source_link_id):
                build = self.map.build_route_corridor(selected, source_link_id)
                policy = self._route_policy(build, source_link_id)
                effective = self.planner.policy.apply(build.corridor, policy)
                self.assertFalse(effective.decision.fail_closed)

                source_lane = self.map.lane(source_link_id)
                start = source_lane.pose_at(start_progress)
                self.assertTrue(
                    self.planner.checker.check_pose(start, effective).valid
                )
                forward_lanes = tuple(self.map.lane(link_id) for link_id in forward)
                remaining = 18.0
                unsafe_preferred_goal = None
                unsafe_preferred_link = ""
                for lane_index, lane in enumerate(forward_lanes):
                    origin = start_progress if lane_index == 0 else 0.0
                    available = lane.length_m - origin
                    if remaining <= available:
                        unsafe_preferred_goal = lane.pose_at(origin + remaining)
                        unsafe_preferred_link = lane.link_id
                        break
                    remaining -= available
                self.assertIsNotNone(unsafe_preferred_goal)
                self.assertNotEqual(unsafe_preferred_link, expected_goal)
                self.assertFalse(
                    self.planner.checker.check_pose(
                        unsafe_preferred_goal, effective
                    ).valid
                )
                goal, goal_link_id = select_forward_valid_goal(
                    forward_lanes,
                    start_progress_m=start_progress,
                    preferred_distance_m=18.0,
                    minimum_distance_m=6.0,
                    final_end_margin_m=5.0,
                    scan_step_m=0.20,
                    pose_is_valid=lambda pose: self.planner.checker.check_pose(
                        pose, effective
                    ).valid,
                )
                self.assertEqual(goal_link_id, expected_goal)
                self.assertTrue(
                    self.planner.checker.check_pose(goal, effective).valid
                )

                request = PlanningRequest(
                    start,
                    goal,
                    build.corridor,
                    policy,
                    reference_path=build.guidance_points,
                )
                result = self.planner.plan(request)
                self.assertEqual(result.status, PlanStatus.SUCCESS)
                self.assertTrue(result.is_valid)
                self.assertGreaterEqual(
                    result.path[-1].distance_from_start_m,
                    6.0,
                )
                self.assertTrue(
                    self.planner.validate_path(
                        result.path,
                        request.corridor,
                        request.policy,
                    )
                )

    def test_a411_mixed_b038_remains_closed_and_route_pose_is_invalid(self):
        pair = LaneChangePair(
            "A2256W000411",
            "A2256W000409",
            "left",
            "B2256W000038",
            "A2256W000420",
            {},
        )
        lane_change = self.map.build_lane_change_corridor(pair)
        self.assertEqual(lane_change.shared_marking, BoundaryMarking.SOLID)
        denied = self.planner.policy.resolve(
            lane_change.corridor,
            CorridorPolicyInput(
                mode=CorridorMode.HIGHWAY_OVERTAKE,
                current_lane_id=pair.route_link_id,
                requested_open_boundary_ids=frozenset((pair.shared_boundary_id,)),
                overtake_requested=True,
                high_speed_zone=True,
                adjacent_lane_verified=lane_change.topology_verified,
                adjacent_lane_id=pair.adjacent_link_id,
                lead_vehicle_distance_m=None,
                lane_change_latched=True,
            ),
        )
        self.assertTrue(denied.fail_closed)
        self.assertFalse(denied.lane_change_enabled)
        self.assertEqual(denied.opened_boundary_ids, frozenset())
        self.assertIn(pair.shared_boundary_id, denied.hard_boundary_ids)

        route = self.map.build_route_corridor(
            (
                "A2256W000846",
                "A2256W000411",
                "A2256W000420",
            ),
            pair.route_link_id,
        )
        route_policy = self._route_policy(route, pair.route_link_id)
        lane = self.map.lane(pair.route_link_id)
        request = PlanningRequest(
            lane.pose_at(110.0),
            lane.pose_at(128.0),
            route.corridor,
            route_policy,
            reference_path=route.guidance_points,
        )
        result = self.planner.plan(request)
        self.assertEqual(result.status, PlanStatus.INVALID_START)
        self.assertFalse(result.is_valid)
        self.assertEqual(result.path, ())

    def test_enabled_pure_dashed_pairs_generate_a_continuously_safe_crossing(self):
        pairs = (
            LaneChangePair(
                "A2256W000420",
                "A2256W000430",
                "left",
                "B2256W000034",
                "A2256W000153",
                {},
            ),
            LaneChangePair(
                "A2256W000408",
                "A2256W000434",
                "left",
                "B2256W000044",
                "A2256W000153",
                {},
            ),
        )
        for pair in pairs:
            with self.subTest(route_link_id=pair.route_link_id):
                build = self.map.build_lane_change_corridor(pair)
                start = build.current_lane.pose_at(15.0)
                goal = build.target_lane.pose_at(25.0)
                lead_length_m = 4.635
                lead = BoxObstacle(
                    build.current_lane.pose_at(
                        15.0
                        + self.planner.vehicle.longitudinal_max_m
                        + 10.0
                        + 0.5 * lead_length_m
                    ),
                    lead_length_m,
                    1.892,
                    "lead_vehicle",
                )
                policy = CorridorPolicyInput(
                    mode=CorridorMode.HIGHWAY_OVERTAKE,
                    current_lane_id=build.current_lane.link_id,
                    requested_open_boundary_ids=frozenset(
                        build.open_boundary_ids
                    ),
                    overtake_requested=True,
                    high_speed_zone=True,
                    adjacent_lane_verified=build.topology_verified,
                    adjacent_lane_id=build.target_lane.link_id,
                    lead_vehicle_distance_m=10.0,
                )
                request = PlanningRequest(
                    start,
                    goal,
                    build.corridor,
                    policy,
                    obstacles=(lead,),
                    reference_path=lane_change_reference(start, goal),
                )
                result = self.planner.plan(request)
                self.assertEqual(result.status, PlanStatus.SUCCESS)
                self.assertTrue(result.is_valid)
                self.assertTrue(result.diagnostics.policy.lane_change_enabled)
                self.assertTrue(
                    self.planner.validate_path(
                        result.path,
                        request.corridor,
                        request.policy,
                    )
                )


if __name__ == "__main__":
    unittest.main()
