import math
import unittest

from path_planning_pkg import (
    CircleObstacle,
    CorridorMode,
    CorridorPolicy,
    CorridorPolicyInput,
    DrivingCorridor,
    HybridAStarConfig,
    HybridAStarPlanner,
    PoseValidity,
    PlanningRequest,
    PlanStatus,
    Point2D,
    Pose2D,
)
from path_planning_pkg.geometry import propagate_bicycle


def open_corridor(y_extent=5.0):
    return DrivingCorridor.from_polygon(
        "route-link",
        "route-lane",
        (
            Point2D(-5.0, -y_extent),
            Point2D(30.0, -y_extent),
            Point2D(30.0, y_extent),
            Point2D(-5.0, y_extent),
        ),
    )


class HybridAStarPlannerTest(unittest.TestCase):
    def test_keeps_continuous_pose_but_uses_discrete_key(self):
        planner = HybridAStarPlanner(boundary_clearance_m=0.10)
        first = Pose2D(1.01, 2.01, math.radians(1.0))
        second = Pose2D(1.24, 2.24, math.radians(4.0))
        self.assertNotEqual(first, second)
        self.assertEqual(planner.discrete_key(first), planner.discrete_key(second))

    def test_steering_history_is_part_of_discrete_state(self):
        planner = HybridAStarPlanner(boundary_clearance_m=0.10)
        pose = Pose2D(1.0, 2.0, 0.0)
        neutral_index = planner.config.steering_candidates_rad.index(0.0)
        left_index = 0
        self.assertEqual(
            planner.discrete_key(pose).steering_index,
            neutral_index,
        )
        self.assertNotEqual(
            planner.discrete_key(pose, neutral_index),
            planner.discrete_key(pose, left_index),
        )

    def test_goal_guided_expansion_keeps_safety_escape_controls(self):
        candidates = tuple(math.radians(value) for value in range(-25, 26, 5))
        planner = HybridAStarPlanner(
            config=HybridAStarConfig(
                steering_candidates_rad=candidates,
                maximum_steering_expansions=7,
            ),
            boundary_clearance_m=0.10,
        )
        expansions = planner._steering_expansions(
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(10.0, 2.0, 0.0),
            0.0,
        )
        indices = {index for index, _value in expansions}
        self.assertLessEqual(len(expansions), 7)
        self.assertIn(0, indices)
        self.assertIn(len(candidates) - 1, indices)
        self.assertIn(candidates.index(0.0), indices)

    def test_plans_forward_bicycle_path_and_revalidates_result(self):
        planner = HybridAStarPlanner(boundary_clearance_m=0.10)
        corridor = open_corridor(3.0)
        request = PlanningRequest(
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(12.0, 0.0, 0.0),
            corridor,
        )
        result = planner.plan(request)

        self.assertEqual(result.status, PlanStatus.SUCCESS)
        self.assertTrue(result.is_valid)
        self.assertGreater(len(result.path), 2)
        self.assertEqual(result.path[0].pose, request.start)
        self.assertLessEqual(
            math.hypot(
                result.path[-1].pose.x - request.goal.x,
                result.path[-1].pose.y - request.goal.y,
            ),
            planner.config.goal_position_tolerance_m,
        )
        distances = [point.distance_from_start_m for point in result.path]
        self.assertEqual(distances, sorted(distances))
        self.assertTrue(all(b > a for a, b in zip(distances, distances[1:])))
        self.assertLessEqual(
            max(abs(point.curvature_per_m) for point in result.path),
            1.0 / planner.vehicle.minimum_turning_radius_m + 1.0e-12,
        )
        effective = CorridorPolicy().apply(corridor, CorridorPolicyInput())
        for point in result.path:
            self.assertTrue(planner.checker.check_pose(point.pose, effective).valid)
        self.assertGreater(
            result.diagnostics.minimum_body_boundary_clearance_m,
            planner.checker.boundary_clearance_m,
        )

    def test_avoids_circle_obstacle_with_full_vehicle_rectangle(self):
        config = HybridAStarConfig(collision_check_step_m=0.20, max_search_nodes=20000)
        planner = HybridAStarPlanner(config=config, boundary_clearance_m=0.10)
        obstacle = CircleObstacle(Point2D(7.0, 0.0), 0.60)
        request = PlanningRequest(
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(15.0, 0.0, 0.0),
            open_corridor(),
            obstacles=(obstacle,),
        )
        result = planner.plan(request)

        self.assertEqual(result.status, PlanStatus.SUCCESS)
        self.assertTrue(result.is_valid)
        self.assertGreater(result.diagnostics.rejected_by_obstacle, 0)
        self.assertTrue(any(abs(point.pose.y) > 0.5 for point in result.path))
        effective = CorridorPolicy().apply(request.corridor, request.policy)
        for point in result.path:
            self.assertTrue(
                planner.checker.check_pose(point.pose, effective, request.obstacles).valid
            )

    def test_changed_obstacle_snapshot_revalidates_the_continuous_path(self):
        planner = HybridAStarPlanner(boundary_clearance_m=0.10)
        request = PlanningRequest(
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(12.0, 0.0, 0.0),
            open_corridor(3.0),
            reference_path=(Point2D(0.0, 0.0), Point2D(12.0, 0.0)),
        )
        result = planner.plan(request)
        self.assertTrue(result.is_valid)
        self.assertTrue(
            planner.validate_path(result.path, request.corridor, request.policy)
        )
        self.assertFalse(
            planner.validate_path(
                result.path,
                request.corridor,
                request.policy,
                (CircleObstacle(Point2D(7.0, 0.0), 0.5),),
            )
        )

    def test_reference_path_rejects_non_finite_or_single_point_input(self):
        corridor = open_corridor()
        with self.assertRaises(ValueError):
            PlanningRequest(
                Pose2D(0.0, 0.0, 0.0),
                Pose2D(10.0, 0.0, 0.0),
                corridor,
                reference_path=(Point2D(0.0, 0.0),),
            )
        with self.assertRaises(ValueError):
            PlanningRequest(
                Pose2D(0.0, 0.0, 0.0),
                Pose2D(10.0, 0.0, 0.0),
                corridor,
                reference_path=(Point2D(0.0, 0.0), Point2D(math.nan, 1.0)),
            )

    def test_invalid_start_and_search_limit_return_no_path(self):
        corridor = open_corridor(3.0)
        planner = HybridAStarPlanner(boundary_clearance_m=0.10)
        touching_start = Pose2D(0.0, 3.0 - planner.vehicle.half_width_m, 0.0)
        invalid = planner.plan(
            PlanningRequest(touching_start, Pose2D(10.0, 0.0, 0.0), corridor)
        )
        self.assertEqual(invalid.status, PlanStatus.INVALID_START)
        self.assertFalse(invalid.is_valid)
        self.assertEqual(invalid.path, ())

        limited_config = HybridAStarConfig(max_search_nodes=1)
        limited_planner = HybridAStarPlanner(
            config=limited_config, boundary_clearance_m=0.10
        )
        limited = limited_planner.plan(
            PlanningRequest(
                Pose2D(0.0, 0.0, 0.0), Pose2D(10.0, 0.0, 0.0), corridor
            )
        )
        self.assertEqual(limited.status, PlanStatus.SEARCH_LIMIT)
        self.assertFalse(limited.is_valid)
        self.assertEqual(limited.path, ())
        self.assertEqual(limited.diagnostics.expanded_nodes, 1)

        timed_planner = HybridAStarPlanner(
            config=HybridAStarConfig(maximum_planning_time_sec=1.0e-12),
            boundary_clearance_m=0.10,
        )
        timed = timed_planner.plan(
            PlanningRequest(
                Pose2D(0.0, 0.0, 0.0), Pose2D(10.0, 0.0, 0.0), corridor
            )
        )
        self.assertEqual(timed.status, PlanStatus.TIME_LIMIT)
        self.assertFalse(timed.is_valid)
        self.assertEqual(timed.path, ())

    def test_denied_wall_opening_cannot_return_a_fallback_success(self):
        planner = HybridAStarPlanner(boundary_clearance_m=0.10)
        request = PlanningRequest(
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(10.0, 0.0, 0.0),
            open_corridor(3.0),
            CorridorPolicyInput(
                mode=CorridorMode.HIGHWAY_OVERTAKE,
                current_lane_id="route-lane",
                overtake_requested=True,
                high_speed_zone=True,
            ),
        )
        result = planner.plan(request)
        self.assertEqual(result.status, PlanStatus.POLICY_BLOCKED)
        self.assertFalse(result.is_valid)
        self.assertEqual(result.path, ())

    def test_rejects_steering_beyond_minimum_turning_radius(self):
        too_large = HybridAStarConfig(
            steering_candidates_rad=(0.0, math.radians(30.0))
        )
        with self.assertRaises(ValueError):
            HybridAStarPlanner(config=too_large)

    def test_collision_evaluation_budget_must_be_a_positive_integer(self):
        for invalid in (True, 0, -1, 1.5):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    HybridAStarConfig(
                        maximum_collision_evaluations_per_primitive=invalid
                    )

    def test_primitive_evaluation_cap_is_not_a_global_time_limit(self):
        planner = HybridAStarPlanner(boundary_clearance_m=0.10)
        capped = PoseValidity(
            False,
            "collision_check_evaluation_limit",
            -math.inf,
            -math.inf,
            -math.inf,
        )
        expired = PoseValidity(
            False,
            "collision_check_time_limit",
            -math.inf,
            -math.inf,
            -math.inf,
        )
        self.assertEqual(planner._rejection_kind((capped,)), "evaluation_limit")
        self.assertEqual(planner._rejection_kind((expired,)), "time_limit")

    def test_capped_primitive_does_not_hide_a_safe_steering_alternative(self):
        vehicle = HybridAStarPlanner().vehicle
        steering = vehicle.maximum_kinematic_steering_rad
        config = HybridAStarConfig(
            primitive_length_m=1.0,
            collision_check_step_m=0.20,
            steering_candidates_rad=(0.0, steering),
            maximum_steering_expansions=2,
            goal_position_tolerance_m=0.05,
            goal_yaw_tolerance_rad=0.05,
            max_search_nodes=100,
            maximum_planning_time_sec=2.0,
            maximum_collision_evaluations_per_primitive=50,
        )
        planner = HybridAStarPlanner(
            vehicle=vehicle,
            config=config,
            boundary_clearance_m=0.10,
            obstacle_clearance_m=0.10,
        )
        start = Pose2D(0.0, 0.0, 0.0)
        goal = propagate_bicycle(
            start, steering, 1.0, vehicle.wheelbase_m
        )
        obstacle = CircleObstacle(Point2D(1.3, -1.25), 0.20)
        result = planner.plan(
            PlanningRequest(
                start,
                goal,
                open_corridor(10.0),
                obstacles=(obstacle,),
            )
        )
        self.assertEqual(result.status, PlanStatus.SUCCESS)
        self.assertTrue(result.is_valid)


if __name__ == "__main__":
    unittest.main()
