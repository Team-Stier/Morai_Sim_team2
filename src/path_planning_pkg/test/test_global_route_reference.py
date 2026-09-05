import math
import unittest
from pathlib import Path

from path_planning_pkg import Point2D, Pose2D
from path_planning_pkg.global_route_reference import (
    GlobalRouteReference,
    GlobalRouteTrackingConfig,
)


def rectangular_route_with_duplicate():
    points = [Point2D(float(x), 0.0) for x in range(11)]
    points += [Point2D(10.0, float(y)) for y in range(1, 6)]
    points += [Point2D(float(x), 5.0) for x in range(9, -1, -1)]
    points += [Point2D(0.0, float(y)) for y in range(4, -1, -1)]
    points.insert(6, points[5])
    return tuple(points)


def tracking_config(points):
    length = sum(
        math.hypot(second.x - first.x, second.y - first.y)
        for first, second in zip(points, points[1:])
    )
    return GlobalRouteTrackingConfig(
        expected_point_count=len(points),
        expected_length_m=length,
        length_tolerance_m=1.0e-6,
        maximum_point_spacing_m=1.1,
        closure_tolerance_m=1.0e-6,
        maximum_index_progress_error_m=0.6,
        maximum_progress_regression_m=0.75,
        projection_backward_points=4,
        projection_forward_points=8,
        matching_tube_radius_m=1.25,
        maximum_projection_progress_error_m=2.0,
        maximum_projection_heading_error_rad=math.radians(100.0),
        turn_detection_heading_change_rad=math.pi,
        turn_goal_distance_m=3.0,
        heading_score_weight=1.0,
        goal_scan_step_m=0.5,
    )


def competition_config():
    return GlobalRouteTrackingConfig(
        expected_point_count=4430,
        expected_length_m=2184.612,
        length_tolerance_m=0.10,
        maximum_point_spacing_m=1.0,
        closure_tolerance_m=0.05,
        maximum_index_progress_error_m=1.0,
        maximum_progress_regression_m=0.75,
        projection_backward_points=40,
        projection_forward_points=80,
        matching_tube_radius_m=3.0,
        maximum_projection_progress_error_m=5.0,
        maximum_projection_heading_error_rad=math.radians(100.0),
        turn_detection_heading_change_rad=math.radians(30.0),
        turn_goal_distance_m=12.0,
        heading_score_weight=1.0,
        goal_scan_step_m=0.5,
        expected_xy_sha256=(
            "8df933cdce0d1430db6082ebcc820836ad2999013ca5cc2408b3dd52391abf54"
        ),
    )


def competition_points():
    repository = Path(__file__).resolve().parents[3]
    source = repository / "참고파일들" / "2026_molit_comp_global_path (3).txt"
    return tuple(
        Point2D(*map(float, line.split()[:2]))
        for line in source.read_text(encoding="utf-8-sig").splitlines()
        if line.strip()
    )


class GlobalRouteReferenceTest(unittest.TestCase):
    def setUp(self):
        self.points = rectangular_route_with_duplicate()
        self.config = tracking_config(self.points)
        self.route = GlobalRouteReference(self.points, self.config)

    def test_duplicate_source_point_is_preserved_but_slice_is_distinct(self):
        self.assertEqual(len(self.route.raw_points), len(self.points))
        self.assertEqual(
            self.route.source_progress_m[5], self.route.source_progress_m[6]
        )
        result = self.route.forward_slice(
            ego=Pose2D(4.2, 0.35, 0.04),
            context_progress_m=4.25,
            context_route_length_m=self.route.length_m,
            nearest_source_index=4,
            previous_progress_m=None,
            preferred_distance_m=4.0,
            minimum_distance_m=2.0,
            final_end_margin_m=1.0,
            pose_is_valid=lambda _pose: True,
            config=self.config,
        )
        self.assertAlmostEqual(result.accepted_progress_m, 4.25)
        self.assertAlmostEqual(result.reference_path[0].x, 4.25)
        self.assertAlmostEqual(result.goal.x, 8.25)
        self.assertTrue(
            all(first != second for first, second in zip(
                result.reference_path, result.reference_path[1:]
            ))
        )

    def test_checked_in_competition_path_matches_locked_geometry(self):
        points = competition_points()
        config = competition_config()
        route = GlobalRouteReference(points, config)
        self.assertEqual(len(route.raw_points), 4430)
        self.assertEqual(route.duplicate_count, 38)
        self.assertAlmostEqual(route.length_m, 2184.6117233360674, places=6)

        altered = list(points)
        altered[100] = Point2D(altered[100].x + 0.001, altered[100].y)
        with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
            GlobalRouteReference(altered, config)

    def test_real_left_and_right_turn_slices_remain_local_and_ordered(self):
        route = GlobalRouteReference(competition_points(), competition_config())
        for source_index, expected_sign in ((160, 1.0), (260, -1.0)):
            progress = route.source_progress_m[source_index]
            ego = route.pose_at(progress)
            result = route.forward_slice(
                ego=ego,
                context_progress_m=progress,
                context_route_length_m=route.length_m,
                nearest_source_index=source_index,
                previous_progress_m=None,
                preferred_distance_m=18.0,
                minimum_distance_m=6.0,
                final_end_margin_m=5.0,
                # This test isolates immutable artifact slicing. Map footprint
                # validity and the adapter fallback are tested separately.
                pose_is_valid=lambda _pose: True,
                config=competition_config(),
            )
            self.assertAlmostEqual(result.accepted_progress_m, progress)
            self.assertGreater(len(result.reference_path), 20)
            self.assertLessEqual(len(result.reference_path), 40)
            yaw_change = math.atan2(
                math.sin(result.goal.yaw - ego.yaw),
                math.cos(result.goal.yaw - ego.yaw),
            )
            self.assertGreater(expected_sign * yaw_change, 0.25)

    def test_turn_horizon_is_geometry_derived_not_changed_by_ego_yaw_noise(self):
        route = GlobalRouteReference(competition_points(), competition_config())
        progress = 886.0
        route_pose = route.pose_at(progress)
        source_index = min(
            range(len(route.source_progress_m)),
            key=lambda index: abs(route.source_progress_m[index] - progress),
        )
        expected_goal = route.pose_at(progress + 12.0)
        for yaw_noise_deg in (-5.0, 0.0, 5.0):
            ego = Pose2D(
                route_pose.x,
                route_pose.y,
                route_pose.yaw + math.radians(yaw_noise_deg),
            )
            result = route.forward_slice(
                ego=ego,
                context_progress_m=progress,
                context_route_length_m=route.length_m,
                nearest_source_index=source_index,
                previous_progress_m=None,
                preferred_distance_m=18.0,
                minimum_distance_m=6.0,
                final_end_margin_m=5.0,
                pose_is_valid=lambda _pose: True,
                config=competition_config(),
            )
            self.assertAlmostEqual(result.goal.x, expected_goal.x, places=6)
            self.assertAlmostEqual(result.goal.y, expected_goal.y, places=6)

    def test_closed_route_slice_wraps_without_scanning_beyond_local_horizon(self):
        route = GlobalRouteReference(competition_points(), competition_config())
        start_progress = route.length_m - 4.0
        source_index = min(
            range(len(route.source_progress_m)),
            key=lambda index: abs(route.source_progress_m[index] - start_progress),
        )
        ego = route.pose_at(start_progress)
        result = route.forward_slice(
            ego=ego,
            context_progress_m=start_progress,
            context_route_length_m=route.length_m,
            nearest_source_index=source_index,
            previous_progress_m=start_progress - 1.0,
            preferred_distance_m=8.0,
            minimum_distance_m=6.0,
            final_end_margin_m=5.0,
            pose_is_valid=lambda _pose: True,
            config=competition_config(),
        )
        expected_goal = route.pose_at(4.0)
        self.assertAlmostEqual(result.goal.x, expected_goal.x, places=6)
        self.assertAlmostEqual(result.goal.y, expected_goal.y, places=6)
        self.assertGreater(len(result.reference_path), 10)
        self.assertLessEqual(len(result.reference_path), 20)
        self.assertTrue(
            all(first != second for first, second in zip(
                result.reference_path, result.reference_path[1:]
            ))
        )


    def test_small_noisy_backstep_is_clamped_and_large_regression_rejected(self):
        result = self.route.forward_slice(
            ego=Pose2D(5.1, 0.7, 0.0),
            context_progress_m=5.1,
            context_route_length_m=self.route.length_m,
            nearest_source_index=6,
            previous_progress_m=5.5,
            preferred_distance_m=3.0,
            minimum_distance_m=2.0,
            final_end_margin_m=1.0,
            pose_is_valid=lambda _pose: True,
            config=self.config,
        )
        self.assertEqual(result.accepted_progress_m, 5.5)

        with self.assertRaisesRegex(ValueError, "regressed"):
            self.route.forward_slice(
                ego=Pose2D(3.0, 0.0, 0.0),
                context_progress_m=3.0,
                context_route_length_m=self.route.length_m,
                nearest_source_index=3,
                previous_progress_m=5.5,
                preferred_distance_m=3.0,
                minimum_distance_m=2.0,
                final_end_margin_m=1.0,
                pose_is_valid=lambda _pose: True,
                config=self.config,
            )

    def test_large_forward_advance_recovers_when_context_and_pose_agree(self):
        source_index = 20
        progress = self.route.source_progress_m[source_index]
        ego = self.route.pose_at(progress)
        result = self.route.forward_slice(
            ego=ego,
            context_progress_m=progress,
            context_route_length_m=self.route.length_m,
            nearest_source_index=source_index,
            previous_progress_m=2.0,
            preferred_distance_m=3.0,
            minimum_distance_m=2.0,
            final_end_margin_m=1.0,
            pose_is_valid=lambda _pose: True,
            config=self.config,
        )
        self.assertEqual(result.accepted_progress_m, progress)

    def test_matching_tube_rejects_a_wrong_or_excessively_noisy_pose(self):
        with self.assertRaisesRegex(ValueError, "matching tube"):
            self.route.forward_slice(
                ego=Pose2D(4.0, 2.0, 0.0),
                context_progress_m=4.0,
                context_route_length_m=self.route.length_m,
                nearest_source_index=4,
                previous_progress_m=None,
                preferred_distance_m=3.0,
                minimum_distance_m=2.0,
                final_end_margin_m=1.0,
                pose_is_valid=lambda _pose: True,
                config=self.config,
            )

    def test_goal_scan_stays_on_official_polyline(self):
        result = self.route.forward_slice(
            ego=Pose2D(4.0, 0.0, 0.0),
            context_progress_m=4.0,
            context_route_length_m=self.route.length_m,
            nearest_source_index=4,
            previous_progress_m=None,
            preferred_distance_m=4.0,
            minimum_distance_m=2.0,
            final_end_margin_m=1.0,
            pose_is_valid=lambda pose: pose.x <= 7.5,
            config=self.config,
        )
        self.assertAlmostEqual(result.goal.x, 7.5)
        self.assertAlmostEqual(result.goal.y, 0.0)

    def test_goal_scan_never_exceeds_configured_local_horizon(self):
        tested = []

        with self.assertRaisesRegex(ValueError, "no safe goal"):
            self.route.forward_slice(
                ego=Pose2D(4.0, 0.0, 0.0),
                context_progress_m=4.0,
                context_route_length_m=self.route.length_m,
                nearest_source_index=4,
                previous_progress_m=None,
                preferred_distance_m=4.0,
                minimum_distance_m=2.0,
                final_end_margin_m=1.0,
                pose_is_valid=lambda pose: tested.append(pose) or False,
                config=self.config,
            )
        self.assertTrue(tested)
        self.assertLessEqual(max(pose.x for pose in tested), 8.0)


if __name__ == "__main__":
    unittest.main()
