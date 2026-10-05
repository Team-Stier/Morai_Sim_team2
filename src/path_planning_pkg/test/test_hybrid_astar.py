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

    def test_corridor_batch_matches_individual_corner_projection(self):
        planner = self.planner()
        theta = np.linspace(0.0, 0.9, 61)
        routes = (
            np.column_stack((np.linspace(-20.0, 20.0, 81), np.zeros(81))),
            np.column_stack((30.0 * np.sin(theta), 30.0 * (1.0 - np.cos(theta)))),
            np.array([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0),
                      (5.0, 10.0), (5.0, 20.0)]),
        )
        rng = np.random.default_rng(1193)
        lateral_edge = planner.config.corridor_half_width_m - planner.config.vehicle_width_m / 2.0
        edge_poses = [Pose2D(x, sign * (lateral_edge + delta), 0.0)
                      for x in (-20.0, 0.0, 20.0)
                      for sign in (-1.0, 1.0)
                      for delta in (-1.0e-8, 0.0, 1.0e-8)]

        for route_index, route in enumerate(routes):
            guide = _ReferenceGuide(route)
            low = np.min(guide.xy, axis=0) - 7.0
            high = np.max(guide.xy, axis=0) + 7.0
            random_xy = rng.uniform(low, high, size=(250, 2))
            random_yaw = rng.uniform(-math.pi, math.pi, size=250)
            poses = [Pose2D(float(x), float(y), float(yaw))
                     for (x, y), yaw in zip(random_xy, random_yaw)]
            if route_index == 0:
                poses.extend(edge_poses)
            for pose in poses:
                expected = all(guide.project(corner)[1] <= planner.config.corridor_half_width_m
                               for corner in planner._corners(pose))
                self.assertEqual(planner._inside_corridor(pose, guide), expected,
                                 msg='route=%d pose=%r' % (route_index, pose))

    def test_obstacle_index_matches_full_footprint_at_random_and_boundary_points(self):
        planner = self.planner()
        config = planner.config
        rng = np.random.default_rng(3401)
        outcomes = {True: 0, False: 0}

        def full_footprint_hit(pose, points):
            delta = points[:, :2] - np.array([pose.x, pose.y])
            co, si = math.cos(pose.yaw), math.sin(pose.yaw)
            local_x = co * delta[:, 0] + si * delta[:, 1]
            local_y = -si * delta[:, 0] + co * delta[:, 1]
            margin = config.obstacle_margin_m
            return bool(np.any(
                (local_x >= -config.rear_overhang_m - margin) &
                (local_x <= config.front_overhang_m + margin) &
                (np.abs(local_y) <= config.vehicle_width_m / 2.0 + margin)))

        def check(pose, points):
            sorted_points = points[np.argsort(points[:, 0])]
            expected = full_footprint_hit(pose, points)
            outcomes[expected] += 1
            self.assertEqual(planner._hits_obstacle(pose, sorted_points), expected,
                             msg='pose=%r point_count=%d' % (pose, len(points)))

        for _ in range(300):
            pose = Pose2D(float(rng.uniform(-500.0, 500.0)),
                          float(rng.uniform(-500.0, 500.0)),
                          float(rng.uniform(-math.pi, math.pi)))
            points = rng.uniform(-12.0, 12.0, size=(int(rng.integers(0, 600)), 2))
            points += (pose.x, pose.y)
            check(pose, points)
            far_points = points[np.linalg.norm(points - (pose.x, pose.y), axis=1) >= 7.0]
            check(pose, far_points)
            check(pose, np.empty((0, 2)))
            co, si = math.cos(pose.yaw), math.sin(pose.yaw)
            for longitudinal in (-config.rear_overhang_m - config.obstacle_margin_m,
                                 config.front_overhang_m + config.obstacle_margin_m):
                for lateral in (-config.vehicle_width_m / 2.0 - config.obstacle_margin_m,
                                config.vehicle_width_m / 2.0 + config.obstacle_margin_m):
                    for offset in (-1.0e-8, 0.0, 1.0e-8):
                        local_x = longitudinal + offset
                        point = (pose.x + co * local_x - si * lateral,
                                 pose.y + si * local_x + co * lateral)
                        check(pose, np.vstack((far_points, point)))
        self.assertGreater(outcomes[True], 0)
        self.assertGreater(outcomes[False], 0)

    def test_obstacle_index_matches_full_footprint_with_production_margin(self):
        planner = HybridAStarPlanner(HybridAStarConfig(obstacle_margin_m=0.2))
        config = planner.config
        rng = np.random.default_rng(3402)
        front = config.front_overhang_m + config.obstacle_margin_m
        rear = -config.rear_overhang_m - config.obstacle_margin_m
        side = config.vehicle_width_m / 2.0 + config.obstacle_margin_m
        outcomes = {True: 0, False: 0}

        def check(pose, points):
            delta = points - np.array([pose.x, pose.y])
            co, si = math.cos(pose.yaw), math.sin(pose.yaw)
            local_x = co * delta[:, 0] + si * delta[:, 1]
            local_y = -si * delta[:, 0] + co * delta[:, 1]
            expected = bool(np.any((local_x >= rear) & (local_x <= front) &
                                   (np.abs(local_y) <= side)))
            outcomes[expected] += 1
            shuffled = points[rng.permutation(len(points))]
            sorted_points = shuffled[np.argsort(shuffled[:, 0])]
            self.assertEqual(planner._hits_obstacle(pose, sorted_points), expected,
                             msg='pose=%r margin=0.2 point_count=%d' % (pose, len(points)))

        for _ in range(80):
            pose = Pose2D(float(rng.uniform(-500.0, 500.0)),
                          float(rng.uniform(-500.0, 500.0)),
                          float(rng.uniform(-math.pi, math.pi)))
            co, si = math.cos(pose.yaw), math.sin(pose.yaw)

            def world(local):
                return np.column_stack((pose.x + co * local[:, 0] - si * local[:, 1],
                                        pose.y + si * local[:, 0] + co * local[:, 1]))

            random_local = rng.uniform((-7.0, -4.0), (7.0, 4.0), size=(120, 2))
            check(pose, world(random_local))
            distractors = world(np.column_stack((rng.uniform(-10.0, 10.0, 90),
                                                  np.full(90, side + 5.0))))
            for edge in (front, rear):
                for offset in (-1.0e-7, 0.0, 1.0e-7):
                    check(pose, np.vstack((distractors, world(np.array([[edge + offset, 0.0]])))))
            for edge in (side, -side):
                for offset in (-1.0e-7, 0.0, 1.0e-7):
                    check(pose, np.vstack((distractors, world(np.array([[0.0, edge + offset]])))))

        # This rotated inflated corner approaches the x-prefilter's furthest
        # possible hit, so a too-small search window would miss a real collision.
        yaw = -math.atan2(side, front)
        pose = Pose2D(30.0, -40.0, yaw)
        corner = np.array([[pose.x + math.cos(yaw) * front - math.sin(yaw) * side,
                            pose.y + math.sin(yaw) * front + math.cos(yaw) * side]])
        check(pose, corner)
        self.assertGreater(outcomes[True], 0)
        self.assertGreater(outcomes[False], 0)

    def test_obstacle_index_keeps_detour_search_result(self):
        reference = [(0.0, 0.0), (22.0, 0.0)]
        obstacle = [(7.0 + dx, dy) for dx in (-0.5, 0.0, 0.5)
                    for dy in (-0.5, 0.0, 0.5)]
        obstacle.extend((float(x), 9.0) for x in np.linspace(-5.0, 25.0, 500))
        planner = self.planner()
        old_planner = self.planner()

        def full_footprint_hit(pose, points):
            delta = points[:, :2] - np.array([pose.x, pose.y])
            co, si = math.cos(pose.yaw), math.sin(pose.yaw)
            local_x = co * delta[:, 0] + si * delta[:, 1]
            local_y = -si * delta[:, 0] + co * delta[:, 1]
            margin = old_planner.config.obstacle_margin_m
            return bool(np.any(
                (local_x >= -old_planner.config.rear_overhang_m - margin) &
                (local_x <= old_planner.config.front_overhang_m + margin) &
                (np.abs(local_y) <= old_planner.config.vehicle_width_m / 2.0 + margin)))

        with patch.object(old_planner, '_hits_obstacle', side_effect=full_footprint_hit):
            old = old_planner.plan(Pose2D(0.0, 0.0, 0.0), Pose2D(15.0, 0.0, 0.0),
                                   reference, obstacle)
        new = planner.plan(Pose2D(0.0, 0.0, 0.0), Pose2D(15.0, 0.0, 0.0),
                           reference, obstacle)
        self.assertEqual(new.status, HybridPlanStatus.SUCCESS)
        self.assertEqual(new.path, old.path)
        self.assertEqual((new.expanded_nodes, new.generated_nodes,
                          new.rejected_by_corridor, new.rejected_by_obstacle),
                         (old.expanded_nodes, old.generated_nodes,
                          old.rejected_by_corridor, old.rejected_by_obstacle))

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
