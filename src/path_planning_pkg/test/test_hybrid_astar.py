import math
import unittest

import numpy as np

from path_planning_pkg.frenet import Lane
from path_planning_pkg.hybrid_astar import (
    HybridAStarConfig,
    HybridAStarPlanner,
    HybridPlanStatus,
    Pose2D,
)
from path_planning_pkg.hybrid_runtime import build_hybrid_candidate


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
        self.assertGreater(result.rejected_by_obstacle, 0)
        self.assertTrue(any(abs(point.pose.y) > 1.0 for point in result.path))

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


if __name__ == "__main__":
    unittest.main()
