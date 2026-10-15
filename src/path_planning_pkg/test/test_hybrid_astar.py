import math
import unittest
from unittest.mock import patch

import numpy as np

from path_planning_pkg.frenet import Lane
from path_planning_pkg.hybrid_astar import (
    HybridAStarConfig,
    HybridAStarPlanner,
    HybridPathPoint,
    HybridPlanResult,
    HybridPlanStatus,
    Pose2D,
    _ReferenceGuide,
)
from path_planning_pkg.hybrid_runtime import build_hybrid_candidate
from path_planning_pkg.hybrid_runtime import hybrid_path_clear


class HybridAStarTest(unittest.TestCase):
    def planner(self):
        return HybridAStarPlanner(HybridAStarConfig(
            maximum_planning_time_sec=2.0,
            max_search_nodes=30000,
            corridor_half_width_m=5.0,
        ))

    @staticmethod
    def full_obstacle_check(planner, pose, points):
        if not len(points):
            return False
        delta = points[:, :2] - np.array([pose.x, pose.y])
        co, si = math.cos(pose.yaw), math.sin(pose.yaw)
        local_x = co * delta[:, 0] + si * delta[:, 1]
        local_y = -si * delta[:, 0] + co * delta[:, 1]
        margin = planner.config.obstacle_margin_m
        return bool(np.any(
            (local_x >= -planner.config.rear_overhang_m - margin) &
            (local_x <= planner.config.front_overhang_m + margin) &
            (np.abs(local_y) <= planner.config.vehicle_width_m / 2.0 + margin)
        ))

    @staticmethod
    def four_corner_corridor_check(planner, pose, guide):
        return all(guide.project(corner)[1] <= planner.config.corridor_half_width_m
                   for corner in planner._corners(pose))

    def test_forward_bicycle_path_reaches_route_goal(self):
        planner = self.planner()
        result = planner.plan(
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(12.0, 0.0, 0.0),
            [(0.0, 0.0), (20.0, 0.0)],
        )
        self.assertEqual(result.status, HybridPlanStatus.SUCCESS)
        self.assertGreater(len(result.path), 2)
        self.assertEqual(result.path[0].pose, Pose2D(0.0, 0.0, 0.0))
        distances = [point.distance_from_start_m for point in result.path]
        self.assertEqual(distances, sorted(distances))
        self.assertTrue(all(second > first for first, second in zip(distances, distances[1:])))

    def test_avoids_measured_obstacle_points_without_occupancy_grid(self):
        planner = self.planner()
        obstacle = [(7.0 + dx, dy) for dx in (-0.5, 0.0, 0.5) for dy in (-0.5, 0.0, 0.5)]
        result = planner.plan(
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(15.0, 0.0, 0.0),
            [(0.0, 0.0), (22.0, 0.0)],
            obstacle,
        )
        self.assertEqual(result.status, HybridPlanStatus.SUCCESS)
        self.assertGreater(result.expanded_nodes, 0)
        self.assertGreater(result.rejected_by_obstacle, 0)
        self.assertTrue(any(abs(point.pose.y) > 1.0 for point in result.path))
        self.assertTrue(planner.path_clear(
            [(point.pose.x, point.pose.y, 0.0) for point in result.path],
            [(0.0, 0.0, 0.0), (22.0, 0.0, 0.0)], obstacle))

    def test_batched_corridor_matches_four_corner_projection(self):
        planner = self.planner()
        theta = np.linspace(0.0, 0.9, 61)
        routes = (
            np.column_stack((np.linspace(-20.0, 20.0, 81), np.zeros(81))),
            np.column_stack((30.0 * np.sin(theta), 30.0 * (1.0 - np.cos(theta)))),
            np.array([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0),
                      (5.0, 10.0), (5.0, 20.0)]),
        )
        rng = np.random.default_rng(1193)
        edge = planner.config.corridor_half_width_m - planner.config.vehicle_width_m / 2.0
        edge_poses = [Pose2D(x, sign * (edge + offset), 0.0)
                      for x in (-20.0, 0.0, 20.0)
                      for sign in (-1.0, 1.0)
                      for offset in (-1.0e-8, 0.0, 1.0e-8)]
        certificate = planner.corridor_fast_accept_radius_m
        edge_poses.extend(Pose2D(0.0, sign * (certificate + offset), yaw)
                          for sign in (-1.0, 1.0)
                          for offset in (-1.0e-8, 0.0, 1.0e-8)
                          for yaw in (0.0, math.pi / 4.0, math.pi / 2.0))
        for route_index, route in enumerate(routes):
            guide = _ReferenceGuide(route)
            low, high = np.min(guide.xy, axis=0) - 7.0, np.max(guide.xy, axis=0) + 7.0
            poses = [Pose2D(float(x), float(y), float(yaw))
                     for (x, y), yaw in zip(rng.uniform(low, high, size=(120, 2)),
                                            rng.uniform(-math.pi, math.pi, size=120))]
            if route_index == 0:
                poses.extend(edge_poses)
            for pose in poses:
                expected = self.four_corner_corridor_check(planner, pose, guide)
                self.assertEqual(planner._inside_corridor(pose, guide), expected,
                                 msg='route=%d pose=%r' % (route_index, pose))

    def test_sorted_obstacle_prefilter_matches_full_footprint(self):
        planner = self.planner()
        rng = np.random.default_rng(3401)
        outcomes = {True: 0, False: 0}
        for _ in range(40):
            pose = Pose2D(float(rng.uniform(-500.0, 500.0)),
                          float(rng.uniform(-500.0, 500.0)),
                          float(rng.uniform(-math.pi, math.pi)))
            far = rng.uniform(-20.0, 20.0, size=(468, 2))
            far = far[np.linalg.norm(far, axis=1) > 7.0] + (pose.x, pose.y)
            dense = rng.uniform(-8.0, 8.0, size=(468, 2)) + (pose.x, pose.y)
            co, si = math.cos(pose.yaw), math.sin(pose.yaw)
            inside = np.array([[pose.x + co * 1.0 - si * 0.2,
                                pose.y + si * 1.0 + co * 0.2]])
            for points in (np.empty((0, 2)), far, dense, np.vstack((far, inside))):
                shuffled = points[rng.permutation(len(points))]
                ordered = shuffled[np.argsort(shuffled[:, 0])]
                expected = self.full_obstacle_check(planner, pose, points)
                outcomes[expected] += 1
                self.assertEqual(planner._hits_obstacle(pose, ordered), expected,
                                 msg='pose=%r point_count=%d' % (pose, len(points)))
        self.assertGreater(outcomes[True], 0)
        self.assertGreater(outcomes[False], 0)

    def test_sorted_obstacle_prefilter_keeps_margin_boundaries(self):
        planner = HybridAStarPlanner(HybridAStarConfig(obstacle_margin_m=0.2))
        config = planner.config
        front = config.front_overhang_m + config.obstacle_margin_m
        rear = -config.rear_overhang_m - config.obstacle_margin_m
        side = config.vehicle_width_m / 2.0 + config.obstacle_margin_m
        pose = Pose2D(30.0, -40.0, -math.atan2(side, front))
        co, si = math.cos(pose.yaw), math.sin(pose.yaw)
        distractors = np.array([[pose.x - 20.0, pose.y], [pose.x + 20.0, pose.y]])
        outcomes = {True: 0, False: 0}
        for axis, edge in ((0, front), (0, rear), (1, side), (1, -side)):
            for offset in (-1.0e-7, 0.0, 1.0e-7):
                local = [0.0, 0.0]
                local[axis] = edge + offset
                x, y = local
                point = [pose.x + co * x - si * y, pose.y + si * x + co * y]
                points = np.vstack((distractors, point))
                expected = self.full_obstacle_check(planner, pose, points)
                outcomes[expected] += 1
                ordered = points[np.argsort(points[:, 0])]
                self.assertEqual(planner._hits_obstacle(pose, ordered), expected,
                                 msg='axis=%d edge=%f offset=%g' % (axis, edge, offset))
        corner = [pose.x + co * front - si * side,
                  pose.y + si * front + co * side]
        points = np.vstack((distractors, corner))
        ordered = points[np.argsort(points[:, 0])]
        self.assertEqual(planner._hits_obstacle(pose, ordered),
                         self.full_obstacle_check(planner, pose, points))
        self.assertGreater(outcomes[True], 0)
        self.assertGreater(outcomes[False], 0)

    def test_plan_and_path_clear_match_full_geometry_checks(self):
        optimized = self.planner()
        reference_planner = self.planner()
        reference = [(0.0, 0.0, 0.0), (22.0, 0.0, 0.0)]
        obstacles = [(7.0 + dx, dy) for dx in (-0.5, 0.0, 0.5)
                     for dy in (-0.5, 0.0, 0.5)]
        obstacles.extend((float(x), 9.0) for x in np.linspace(-5.0, 25.0, 80))
        obstacles = np.asarray(obstacles[::-1], dtype=float)
        start, goal = Pose2D(0.0, 0.0, 0.0), Pose2D(15.0, 0.0, 0.0)

        def original_corridor(pose, guide):
            return self.four_corner_corridor_check(reference_planner, pose, guide)

        def original_obstacles(pose, points):
            return self.full_obstacle_check(reference_planner, pose, points)

        with patch.object(reference_planner, '_inside_corridor', side_effect=original_corridor), \
                patch.object(reference_planner, '_hits_obstacle', side_effect=original_obstacles):
            expected = reference_planner.plan(start, goal, reference, obstacles)
            actual = optimized.plan(start, goal, reference, obstacles)
            self.assertEqual(expected.status, HybridPlanStatus.SUCCESS)
            self.assertEqual(actual.status, expected.status)
            self.assertEqual(actual.path, expected.path)
            self.assertEqual((actual.expanded_nodes, actual.generated_nodes,
                              actual.rejected_by_corridor, actual.rejected_by_obstacle),
                             (expected.expanded_nodes, expected.generated_nodes,
                              expected.rejected_by_corridor, expected.rejected_by_obstacle))

            path = [(point.pose.x, point.pose.y, 0.0) for point in actual.path]
            obstacle_on_path = (actual.path[len(actual.path) // 2].pose.x,
                                actual.path[len(actual.path) // 2].pose.y)
            for points, expected_clear in ((obstacles, True),
                                           (np.vstack((obstacles, obstacle_on_path)), False)):
                self.assertEqual(reference_planner.path_clear(path, reference, points),
                                 expected_clear)
                self.assertEqual(optimized.path_clear(path, reference, points),
                                 expected_clear)

    def test_runtime_adapter_returns_existing_candidate_contract(self):
        route_s = np.linspace(0.0, 40.0, 81)
        lane = Lane(
            "global_route",
            np.column_stack((route_s, np.zeros_like(route_s), np.zeros_like(route_s))),
            route_s,
            np.full_like(route_s, 58.0 / 3.6),
        )
        result = build_hybrid_candidate(
            self.planner(),
            lane,
            Pose2D(0.0, 0.0, 0.0),
            0.0,
            (),
            {"reference_step_m": 0.5, "local_goal_distance_m": 12.0},
            False,
        )
        self.assertIsNotNone(result.candidate)
        self.assertEqual(result.candidate.key, "hybrid_astar")
        self.assertEqual(len(result.candidate.xy), len(result.candidate.route_s))
        self.assertTrue(np.all(np.diff(result.candidate.route_s) >= 0.0))

    def test_runtime_speed_limit_switches_at_source_sample_without_interpolation(self):
        source_s = np.arange(0.0, 21.0, 1.0)
        normal_limit = 58.0 / 3.6
        lane = Lane(
            "global_route",
            np.column_stack((source_s, np.zeros_like(source_s), np.zeros_like(source_s))),
            source_s,
            np.where(source_s < 10.0, -1.0, normal_limit),
        )
        sampled_s = [9.5, 9.75, 10.0, 10.25, 10.5]
        plan = HybridPlanResult(
            HybridPlanStatus.SUCCESS,
            tuple(HybridPathPoint(Pose2D(s, 0.0, 0.0), 0.0, s - sampled_s[0])
                  for s in sampled_s),
            0, 0, 0, 0, 0.0,
        )
        planner = self.planner()
        with patch.object(planner, "plan", return_value=plan):
            result = build_hybrid_candidate(
                planner, lane, Pose2D(9.5, 0.0, 0.0), 9.5, (),
                {"reference_step_m": 0.5, "local_goal_distance_m": 5.0}, False,
            )
        self.assertEqual(result.plan.status, HybridPlanStatus.SUCCESS)
        self.assertIsNotNone(result.candidate)
        np.testing.assert_allclose(result.candidate.route_s, sampled_s)
        np.testing.assert_allclose(result.candidate.limits,
                                   [-1.0, -1.0, normal_limit, normal_limit, normal_limit])

    def test_straight_route_replanning_stays_on_centerline(self):
        planner = self.planner()
        reference = [(float(x), 0.0, 0.0) for x in np.arange(-2.0, 35.5, 0.5)]
        first = planner.plan(Pose2D(0.0, 0.0, 0.0), Pose2D(30.0, 0.0, 0.0), reference)
        repeat = planner.plan(Pose2D(0.0, 0.0, 0.0), Pose2D(30.0, 0.0, 0.0), reference)
        next_plan = planner.plan(Pose2D(3.0, 0.0, 0.0), Pose2D(30.0, 0.0, 0.0), reference)
        for result in (first, repeat, next_plan):
            self.assertEqual(result.status, HybridPlanStatus.SUCCESS)
            self.assertEqual(result.expanded_nodes, 0)
            self.assertLess(max(abs(point.pose.y) for point in result.path), 0.02)
        self.assertEqual(first.path, repeat.path)

    def test_offset_joins_gradually_with_bounded_steering_steps(self):
        planner = self.planner()
        reference = [(float(x), 0.0, 0.0) for x in np.arange(-2.0, 35.5, 0.5)]
        result = planner.plan(Pose2D(0.0, 2.0, 0.0), Pose2D(30.0, 0.0, 0.0), reference)
        self.assertEqual(result.status, HybridPlanStatus.SUCCESS)
        self.assertEqual(result.expanded_nodes, 0)
        self.assertGreater(result.path[10].pose.y, 1.5)  # 3 m after the vehicle
        self.assertLess(abs(result.path[65].pose.y), 0.1)  # after the 18 m join
        for first, second in zip(result.path, result.path[1:]):
            distance = second.distance_from_start_m - first.distance_from_start_m
            allowed = planner.config.maximum_steering_change_rad * distance / planner.config.primitive_length_m
            self.assertLessEqual(abs(second.steering_rad - first.steering_rad), allowed + 0.01501)

    def test_offset_on_moderate_curve_uses_smooth_reference_join(self):
        planner = self.planner()
        station = np.arange(-2.0, 36.0, 0.5)
        reference = np.column_stack((50.0 * np.sin(station / 50.0),
                                     50.0 * (1.0 - np.cos(station / 50.0)),
                                     np.zeros_like(station)))
        goal = Pose2D(50.0 * math.sin(30.0 / 50.0),
                      50.0 * (1.0 - math.cos(30.0 / 50.0)), 30.0 / 50.0)
        result = planner.plan(Pose2D(0.0, 1.0, 0.0), goal, reference)
        self.assertEqual(result.status, HybridPlanStatus.SUCCESS)
        self.assertEqual(result.expanded_nodes, 0)
        self.assertGreater(result.path[10].pose.y, 0.5)
        self.assertLess(math.hypot(result.path[-1].pose.x - goal.x,
                                   result.path[-1].pose.y - goal.y), 0.1)

    def test_retained_path_recheck_uses_current_obstacles_and_markings(self):
        planner = self.planner()
        station = np.arange(0.0, 40.5, 0.5)
        lane = Lane("global_route", np.column_stack((station, station * 0, station * 0)),
                    station, np.full_like(station, 10.0))
        runtime = {"reference_step_m": 0.5, "local_goal_distance_m": 30.0}
        result = build_hybrid_candidate(planner, lane, Pose2D(0.0, 0.0, 0.0),
                                        0.0, (), runtime, False)
        self.assertIsNotNone(result.candidate)
        self.assertTrue(hybrid_path_clear(planner, lane, 0.0, result.candidate,
                                          (), runtime, False))
        self.assertFalse(hybrid_path_clear(planner, lane, 0.0, result.candidate,
                                           [(10.0, 0.0)], runtime, False))
        marking = [np.array([[10.0, -2.0, 0.0], [10.0, 2.0, 0.0]])]
        self.assertFalse(hybrid_path_clear(planner, lane, 0.0, result.candidate,
                                           (), runtime, False, marking))
        overhead = [np.array([[10.0, -2.0, 10.0], [10.0, 2.0, 10.0]])]
        self.assertTrue(hybrid_path_clear(planner, lane, 0.0, result.candidate,
                                          (), runtime, False, overhead))

    def test_no_false_invalid_request_from_sharp_reference_bend(self):
        planner = self.planner()
        route = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (1.0, 2.0),
                 (1.0, 3.0), (3.0, 3.0), (15.0, 3.0)]
        result = planner.plan(Pose2D(0.5, 0.0, 0.0), Pose2D(15.0, 3.0, 0.0), route)
        self.assertNotEqual(result.status, HybridPlanStatus.INVALID_REQUEST)


if __name__ == "__main__":
    unittest.main()
