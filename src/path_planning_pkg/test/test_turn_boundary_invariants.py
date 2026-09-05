#!/usr/bin/env python3

import math
import re
import time
import unittest
from pathlib import Path

from hd_map_pkg.coordinates import CoordinateTransformer
from hd_map_pkg.mgeo_v3 import MGeoV3Dataset
from path_planning_pkg import (
    BoundaryMarking,
    BoundarySide,
    CorridorMode,
    CorridorPolicy,
    CorridorPolicyInput,
    FootprintCollisionChecker,
    HybridAStarConfig,
    HybridAStarPlanner,
    PlanStatus,
    PlanningRequest,
    Point2D,
    Pose2D,
    VehicleGeometry,
)
from path_planning_pkg.geometry import edge_matches_polyline, polygon_edges
from path_planning_pkg.global_route_reference import (
    GlobalRouteReference,
    GlobalRouteTrackingConfig,
)
from path_planning_pkg.mgeo_adapter import MGeoPlannerMap


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


class KATRITurnBoundaryInvariantTest(unittest.TestCase):
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
        cls.policy = CorridorPolicy()
        cls.vehicle = VehicleGeometry.ioniq5()
        cls.checker = FootprintCollisionChecker(cls.vehicle, boundary_clearance_m=0.295)

        maximum_steering = cls.vehicle.maximum_kinematic_steering_rad
        cls.runtime_planner = HybridAStarPlanner(
            vehicle=cls.vehicle,
            config=HybridAStarConfig(
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
                reference_lateral_cost_weight=2.0,
                reference_lookahead_m=7.0,
                maximum_steering_expansions=7,
                max_search_nodes=20000,
                maximum_planning_time_sec=0.60,
            ),
            boundary_clearance_m=0.295,
            obstacle_clearance_m=0.50,
        )

        official_path = package.parents[1] / "참고파일들" / "2026_molit_comp_global_path (3).txt"
        official_points = tuple(
            Point2D(*map(float, line.split()[:2]))
            for line in official_path.read_text(encoding="utf-8-sig").splitlines()
            if line.strip()
        )
        cls.global_route_config = GlobalRouteTrackingConfig(
            expected_point_count=4430,
            expected_length_m=2184.612,
            length_tolerance_m=0.10,
            maximum_point_spacing_m=1.00,
            closure_tolerance_m=0.05,
            maximum_index_progress_error_m=1.00,
            maximum_progress_regression_m=0.75,
            projection_backward_points=40,
            projection_forward_points=80,
            matching_tube_radius_m=3.00,
            maximum_projection_progress_error_m=5.00,
            maximum_projection_heading_error_rad=math.radians(100.0),
            turn_detection_heading_change_rad=math.radians(30.0),
            turn_goal_distance_m=12.0,
            heading_score_weight=1.0,
            goal_scan_step_m=0.50,
            expected_xy_sha256=(
                "8df933cdce0d1430db6082ebcc820836ad2999013ca5cc2408b3dd52391abf54"
            ),
        )
        cls.global_route = GlobalRouteReference(
            official_points, cls.global_route_config
        )

        route_config = (
            package.parent
            / "global_route_manager_pkg"
            / "config"
            / "competition_route.yaml"
        ).read_text(encoding="utf-8")
        cls.official_route_link_ids = tuple(
            re.findall(r"- \{id: ([^,]+), start_m:", route_config)
        )
        if len(cls.official_route_link_ids) < 2:
            raise AssertionError("official route link spans were not found")

    @staticmethod
    def _turn_policy(build, current_link_id, requested_ids=None):
        requested = (
            frozenset(build.open_boundary_ids)
            if requested_ids is None
            else frozenset(requested_ids)
        )
        return CorridorPolicyInput(
            mode=(
                CorridorMode.TURN_CONNECTOR
                if build.open_boundary_ids
                else CorridorMode.KEEP_LANE
            ),
            current_lane_id=current_link_id,
            requested_open_boundary_ids=requested,
            turn_connector_verified=bool(build.topology_verified),
        )

    def test_every_official_route_opening_is_a_virtual_two_polygon_seam(self):
        route_ids = self.official_route_link_ids
        for first, second in zip(route_ids, route_ids[1:]):
            if first == second:
                continue
            with self.subTest(first=first, second=second):
                build = self.map.build_route_corridor((first, second), first)
                decision = self.policy.resolve(
                    build.corridor, self._turn_policy(build, first)
                )
                self.assertFalse(decision.fail_closed)
                self.assertEqual(
                    decision.opened_boundary_ids, frozenset(build.open_boundary_ids)
                )

                boundaries = {
                    boundary.boundary_id: boundary
                    for boundary in build.corridor.boundaries
                }
                lateral_ids = {
                    boundary.boundary_id
                    for boundary in build.corridor.boundaries
                    if boundary.side in (BoundarySide.LEFT, BoundarySide.RIGHT)
                }
                self.assertTrue(lateral_ids)
                self.assertTrue(lateral_ids.issubset(decision.hard_boundary_ids))
                self.assertTrue(
                    lateral_ids.isdisjoint(decision.opened_boundary_ids)
                )

                for boundary_id in decision.opened_boundary_ids:
                    opening = boundaries[boundary_id]
                    self.assertTrue(boundary_id.startswith("route:seam:"))
                    self.assertEqual(opening.side, BoundarySide.CONNECTOR_MOUTH)
                    self.assertEqual(opening.marking, BoundaryMarking.VIRTUAL)
                    self.assertEqual(len(opening.lane_ids), 2)

                    matched_lane_ids = {
                        lane.lane_id
                        for lane in build.corridor.lanes
                        if any(
                            edge_matches_polyline(start, finish, opening.points)
                            for start, finish in polygon_edges(lane.vertices)
                        )
                    }
                    self.assertEqual(matched_lane_ids, set(opening.lane_ids))

    def test_katri_left_and_right_turns_keep_lateral_markings_hard(self):
        # MGeo centerline heading changes are approximately +83 degrees on A182
        # (left) and -84 degrees on A318 (right).
        cases = (
            (
                "left",
                ("A2256W000748", "A2256W000182", "A2256W000329"),
                "A2256W000182",
            ),
            (
                "right",
                ("A2256W000315", "A2256W000318", "A2256W000308"),
                "A2256W000318",
            ),
        )
        for turn_name, selected, current in cases:
            with self.subTest(turn=turn_name, current=current):
                build = self.map.build_route_corridor(selected, current)
                policy_input = self._turn_policy(build, current)
                effective = self.policy.apply(build.corridor, policy_input)
                self.assertFalse(effective.decision.fail_closed)

                current_lateral = tuple(
                    boundary
                    for boundary in build.corridor.boundaries
                    if boundary.lane_ids == frozenset((current,))
                    and boundary.side in (BoundarySide.LEFT, BoundarySide.RIGHT)
                )
                self.assertEqual(len(current_lateral), 2)
                for boundary in current_lateral:
                    self.assertIn(
                        boundary.boundary_id,
                        effective.decision.hard_boundary_ids,
                    )
                    self.assertNotIn(
                        boundary.boundary_id,
                        effective.decision.opened_boundary_ids,
                    )

                    segments = tuple(zip(boundary.points, boundary.points[1:]))
                    start, finish = max(
                        segments,
                        key=lambda pair: math.hypot(
                            pair[1].x - pair[0].x, pair[1].y - pair[0].y
                        ),
                    )
                    yaw = math.atan2(finish.y - start.y, finish.x - start.x)
                    midpoint_x = 0.5 * (start.x + finish.x)
                    midpoint_y = 0.5 * (start.y + finish.y)
                    normal_x = -math.sin(yaw)
                    normal_y = math.cos(yaw)
                    inward_sign = -1.0 if boundary.side is BoundarySide.LEFT else 1.0
                    half_track = 0.5 * self.checker.vehicle.wheel_track_m
                    contact_pose = Pose2D(
                        midpoint_x + inward_sign * normal_x * half_track,
                        midpoint_y + inward_sign * normal_y * half_track,
                        yaw,
                    )
                    self.assertFalse(
                        self.checker.check_pose(contact_pose, effective).valid
                    )

                lateral_request = set(build.open_boundary_ids)
                lateral_request.add(current_lateral[0].boundary_id)
                denied = self.policy.resolve(
                    build.corridor,
                    self._turn_policy(build, current, lateral_request),
                )
                self.assertTrue(denied.fail_closed)
                self.assertEqual(denied.opened_boundary_ids, frozenset())

    def test_narrow_official_turn_slices_plan_with_runtime_limits_and_clearance(self):
        # The 18 m normal-driving horizon exhausts the 0.60 s runtime deadline
        # on A182.  Twelve metres is the longest common turn horizon measured
        # with usable headroom on both narrow official-route corners.
        cases = (
            (
                "A182",
                74.0,
                ("A2256W000748", "A2256W000182", "A2256W000329"),
                "A2256W000182",
            ),
            (
                "A146",
                886.0,
                ("A2256W000148", "A2256W000146", "A2256W000151"),
                "A2256W000146",
            ),
        )
        for turn_name, progress_m, selected, current in cases:
            with self.subTest(turn=turn_name):
                build = self.map.build_route_corridor(selected, current)
                policy_input = self._turn_policy(build, current)
                effective = self.policy.apply(build.corridor, policy_input)
                self.assertFalse(effective.decision.fail_closed)

                start = self.global_route.pose_at(progress_m)
                nearest_source_index = min(
                    range(len(self.global_route.source_progress_m)),
                    key=lambda index: abs(
                        self.global_route.source_progress_m[index] - progress_m
                    ),
                )
                route_slice = self.global_route.forward_slice(
                    ego=start,
                    context_progress_m=progress_m,
                    context_route_length_m=self.global_route.length_m,
                    nearest_source_index=nearest_source_index,
                    previous_progress_m=None,
                    preferred_distance_m=18.0,
                    minimum_distance_m=6.0,
                    final_end_margin_m=5.0,
                    pose_is_valid=lambda pose: self.checker.check_pose(
                        pose, effective
                    ).valid,
                    config=self.global_route_config,
                )
                expected_goal = self.global_route.pose_at(progress_m + 12.0)
                self.assertAlmostEqual(route_slice.goal.x, expected_goal.x, places=6)
                self.assertAlmostEqual(route_slice.goal.y, expected_goal.y, places=6)
                request = PlanningRequest(
                    start=start,
                    goal=route_slice.goal,
                    corridor=build.corridor,
                    policy=policy_input,
                    reference_path=route_slice.reference_path,
                )
                runtime_deadline = time.monotonic() + 0.60
                result = self.runtime_planner.plan(
                    request, deadline_monotonic=runtime_deadline
                )
                self.assertEqual(result.status, PlanStatus.SUCCESS)
                self.assertTrue(result.is_valid)
                self.assertTrue(
                    self.runtime_planner.validate_path(
                        result.path,
                        request.corridor,
                        request.policy,
                        deadline_monotonic=runtime_deadline,
                    )
                )
                clearances = tuple(
                    self.runtime_planner.checker.check_pose(point.pose, effective)
                    .minimum_wheel_boundary_clearance_m
                    for point in result.path
                )
                self.assertTrue(clearances)
                self.assertGreaterEqual(min(clearances), 0.295 - 1.0e-9)


if __name__ == "__main__":
    unittest.main()
