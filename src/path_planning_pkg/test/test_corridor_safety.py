import dataclasses
import math
import time
import unittest

from path_planning_pkg import (
    BoundaryMarking,
    BoundarySegment,
    BoundarySide,
    BoxObstacle,
    CircleObstacle,
    CorridorMode,
    CorridorPolicy,
    CorridorPolicyInput,
    DrivingCorridor,
    FootprintCollisionChecker,
    LanePolygon,
    Point2D,
    Pose2D,
    VehicleGeometry,
)


def rectangular_corridor(y_min=-3.0, y_max=3.0):
    return DrivingCorridor.from_polygon(
        "link",
        "lane",
        (
            Point2D(-5.0, y_min),
            Point2D(20.0, y_min),
            Point2D(20.0, y_max),
            Point2D(-5.0, y_max),
        ),
    )


def side_by_side_corridor(marking=BoundaryMarking.DASHED, directional=False):
    lower = LanePolygon(
        "lower",
        (
            Point2D(-5.0, -3.0),
            Point2D(20.0, -3.0),
            Point2D(20.0, 0.0),
            Point2D(-5.0, 0.0),
        ),
    )
    upper = LanePolygon(
        "upper",
        (
            Point2D(-5.0, 0.0),
            Point2D(20.0, 0.0),
            Point2D(20.0, 3.0),
            Point2D(-5.0, 3.0),
        ),
    )
    shared = BoundarySegment(
        "shared",
        (Point2D(-5.0, 0.0), Point2D(20.0, 0.0)),
        BoundarySide.SHARED,
        marking,
        frozenset(("lower", "upper")),
        directional,
    )
    return DrivingCorridor("link", ("link",), (lower, upper), (shared,))


def qualified_overtake(**overrides):
    values = dict(
        mode=CorridorMode.HIGHWAY_OVERTAKE,
        current_lane_id="lower",
        requested_open_boundary_ids=frozenset(("shared",)),
        overtake_requested=True,
        high_speed_zone=True,
        adjacent_lane_verified=True,
        adjacent_lane_id="upper",
        lead_vehicle_distance_m=10.0,
    )
    values.update(overrides)
    return CorridorPolicyInput(**values)


class VehicleGeometryTest(unittest.TestCase):
    def test_ioniq5_uses_rear_axle_pose_and_asymmetric_overhangs(self):
        vehicle = VehicleGeometry.ioniq5()
        self.assertAlmostEqual(vehicle.wheelbase_m, 3.000)
        self.assertAlmostEqual(vehicle.length_m, 4.635)
        self.assertAlmostEqual(vehicle.width_m, 1.892)
        self.assertAlmostEqual(vehicle.longitudinal_min_m, -0.790)
        self.assertAlmostEqual(vehicle.longitudinal_max_m, 3.845)

        corners = vehicle.body_corners(Pose2D(10.0, 2.0, 0.0))
        self.assertAlmostEqual(min(point.x for point in corners), 9.210)
        self.assertAlmostEqual(max(point.x for point in corners), 13.845)
        self.assertAlmostEqual(min(point.y for point in corners), 2.0 - 0.946)
        self.assertAlmostEqual(max(point.y for point in corners), 2.0 + 0.946)
        self.assertAlmostEqual(
            math.tan(vehicle.maximum_kinematic_steering_rad) / vehicle.wheelbase_m,
            1.0 / 5.87,
        )


class StrictCorridorSafetyTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = VehicleGeometry.ioniq5()
        self.checker = FootprintCollisionChecker(self.vehicle, 0.10)
        self.policy = CorridorPolicy()

    def test_boundary_contact_is_invalid_and_positive_clearance_is_inclusive(self):
        effective = self.policy.apply(rectangular_corridor(), CorridorPolicyInput())
        touching_y = 3.0 - self.vehicle.half_width_m
        exact_clearance_y = touching_y - 0.10

        self.assertFalse(
            self.checker.check_pose(Pose2D(0.0, touching_y, 0.0), effective).valid
        )
        self.assertTrue(
            self.checker.check_pose(
                Pose2D(0.0, exact_clearance_y, 0.0), effective
            ).valid
        )
        self.assertTrue(
            self.checker.check_pose(
                Pose2D(0.0, exact_clearance_y - 1.0e-4, 0.0), effective
            ).valid
        )

    def test_body_overhang_may_cross_paint_when_every_wheel_clears(self):
        short_corridor = DrivingCorridor.from_polygon(
            "link",
            "lane",
            (
                Point2D(-5.0, -3.0),
                Point2D(3.5, -3.0),
                Point2D(3.5, 3.0),
                Point2D(-5.0, 3.0),
            ),
        )
        effective = self.policy.apply(short_corridor, CorridorPolicyInput())
        pose = Pose2D(0.0, 0.0, 0.0)
        self.assertGreater(self.vehicle.longitudinal_max_m, 3.5)
        self.assertLess(self.vehicle.wheelbase_m, 3.5)
        result = self.checker.check_pose(pose, effective)
        self.assertTrue(result.valid)
        self.assertEqual(result.minimum_body_boundary_clearance_m, 0.0)
        self.assertGreater(result.minimum_wheel_boundary_clearance_m, 0.10)

    def test_keep_lane_makes_dashed_shared_line_a_wall(self):
        corridor = side_by_side_corridor()
        effective = self.policy.apply(
            corridor, CorridorPolicyInput(current_lane_id="lower")
        )
        decision = effective.decision
        self.assertFalse(decision.lane_change_enabled)
        self.assertEqual(decision.opened_boundary_ids, frozenset())
        self.assertIn("shared", decision.hard_boundary_ids)
        self.assertFalse(
            self.checker.check_pose(Pose2D(0.0, 0.0, 0.0), effective).valid
        )

    def test_highway_overtake_opens_only_after_every_gate(self):
        corridor = side_by_side_corridor()
        valid_input = qualified_overtake()
        valid = self.policy.apply(corridor, valid_input)
        self.assertTrue(valid.decision.lane_change_enabled)
        self.assertEqual(valid.decision.opened_boundary_ids, frozenset(("shared",)))
        self.assertTrue(
            self.checker.check_pose(Pose2D(0.0, 0.0, 0.0), valid).valid
        )

        denied_variants = (
            dataclasses.replace(valid_input, overtake_requested=False),
            dataclasses.replace(valid_input, high_speed_zone=False),
            dataclasses.replace(valid_input, adjacent_lane_verified=False),
            dataclasses.replace(valid_input, adjacent_lane_id=""),
            dataclasses.replace(valid_input, lead_vehicle_distance_m=None),
            dataclasses.replace(valid_input, lead_vehicle_distance_m=-0.01),
            dataclasses.replace(valid_input, lead_vehicle_distance_m=10.0001),
            dataclasses.replace(valid_input, requested_open_boundary_ids=frozenset()),
            dataclasses.replace(
                valid_input, requested_open_boundary_ids=frozenset(("missing",))
            ),
        )
        for denied in denied_variants:
            with self.subTest(denied=denied):
                decision = self.policy.resolve(corridor, denied)
                self.assertTrue(decision.fail_closed)
                self.assertFalse(decision.lane_change_enabled)
                self.assertEqual(decision.opened_boundary_ids, frozenset())

    def test_latched_crossing_waives_only_the_fresh_lead_gap(self):
        corridor = side_by_side_corridor()
        latched = qualified_overtake(
            lead_vehicle_distance_m=None,
            lane_change_latched=True,
        )
        decision = self.policy.resolve(corridor, latched)
        self.assertFalse(decision.fail_closed)
        self.assertTrue(decision.lane_change_enabled)
        self.assertEqual(decision.opened_boundary_ids, frozenset(("shared",)))

        no_latch = dataclasses.replace(
            latched,
            lane_change_latched=False,
            lead_vehicle_distance_m=10.0001,
        )
        denied = self.policy.resolve(corridor, no_latch)
        self.assertTrue(denied.fail_closed)
        self.assertEqual(denied.opened_boundary_ids, frozenset())

        still_required = (
            dataclasses.replace(latched, overtake_requested=False),
            dataclasses.replace(latched, high_speed_zone=False),
            dataclasses.replace(latched, adjacent_lane_verified=False),
            dataclasses.replace(latched, current_lane_id=""),
            dataclasses.replace(latched, adjacent_lane_id=""),
            dataclasses.replace(latched, requested_open_boundary_ids=frozenset()),
        )
        for missing_gate in still_required:
            with self.subTest(missing_gate=missing_gate):
                denied = self.policy.resolve(corridor, missing_gate)
                self.assertTrue(denied.fail_closed)
                self.assertEqual(denied.opened_boundary_ids, frozenset())

    def test_latch_cannot_open_solid_unknown_or_outer_boundary(self):
        latched = qualified_overtake(
            lead_vehicle_distance_m=None,
            lane_change_latched=True,
        )
        for marking in (BoundaryMarking.SOLID, BoundaryMarking.UNKNOWN):
            with self.subTest(marking=marking):
                corridor = side_by_side_corridor(marking)
                decision = self.policy.resolve(corridor, latched)
                self.assertTrue(decision.fail_closed)
                self.assertEqual(decision.opened_boundary_ids, frozenset())

        corridor = side_by_side_corridor()
        shared = corridor.boundaries[0]
        outer_role = BoundarySegment(
            shared.boundary_id,
            shared.points,
            BoundarySide.LEFT,
            BoundaryMarking.DASHED,
            shared.lane_ids,
        )
        corridor_with_outer_role = DrivingCorridor(
            corridor.source_link_id,
            corridor.route_link_ids,
            corridor.lanes,
            (outer_role,),
        )
        denied_outer = self.policy.resolve(corridor_with_outer_role, latched)
        self.assertTrue(denied_outer.fail_closed)
        self.assertEqual(denied_outer.opened_boundary_ids, frozenset())

    def test_solid_and_mixed_lines_never_open(self):
        solid = side_by_side_corridor(BoundaryMarking.SOLID, directional=False)
        denied = self.policy.resolve(solid, qualified_overtake())
        self.assertTrue(denied.fail_closed)
        self.assertEqual(denied.opened_boundary_ids, frozenset())

        still_solid = side_by_side_corridor(BoundaryMarking.SOLID, directional=True)
        still_denied = self.policy.resolve(still_solid, qualified_overtake())
        self.assertTrue(still_denied.fail_closed)

        mixed_without_permission = side_by_side_corridor(
            BoundaryMarking.SOLID_DASHED, directional=False
        )
        mixed_denied = self.policy.resolve(mixed_without_permission, qualified_overtake())
        self.assertTrue(mixed_denied.fail_closed)

        directionally_permitted_mixed = side_by_side_corridor(
            BoundaryMarking.SOLID_DASHED, directional=True
        )
        still_mixed = self.policy.resolve(
            directionally_permitted_mixed, qualified_overtake()
        )
        self.assertTrue(still_mixed.fail_closed)
        self.assertEqual(still_mixed.opened_boundary_ids, frozenset())

    def test_overtake_keeps_outer_walls(self):
        corridor = side_by_side_corridor()
        effective = self.policy.apply(corridor, qualified_overtake())
        touching_outer_y = 3.0 - self.vehicle.half_width_m
        self.assertFalse(
            self.checker.check_pose(
                Pose2D(0.0, touching_outer_y, 0.0), effective
            ).valid
        )
        exact_outer_clearance_y = 3.0 - self.vehicle.half_width_m - 0.10
        self.assertTrue(
            self.checker.check_pose(
                Pose2D(0.0, exact_outer_clearance_y, 0.0), effective
            ).valid
        )

    def test_turn_connector_opens_only_verified_virtual_connector_mouth(self):
        incoming = LanePolygon(
            "incoming",
            (
                Point2D(-5.0, -3.0),
                Point2D(5.0, -3.0),
                Point2D(5.0, 3.0),
                Point2D(-5.0, 3.0),
            ),
        )
        outgoing = LanePolygon(
            "outgoing",
            (
                Point2D(5.0, -3.0),
                Point2D(20.0, -3.0),
                Point2D(20.0, 3.0),
                Point2D(5.0, 3.0),
            ),
        )
        downstream = BoundarySegment(
            "mouth",
            (Point2D(5.0, -3.0), Point2D(5.0, 3.0)),
            BoundarySide.CONNECTOR_MOUTH,
            BoundaryMarking.VIRTUAL,
            frozenset(("incoming", "outgoing")),
        )
        corridor = DrivingCorridor(
            "turn-link", ("turn-link",), (incoming, outgoing), (downstream,)
        )
        unverified = CorridorPolicyInput(
            mode=CorridorMode.TURN_CONNECTOR,
            current_lane_id="incoming",
            requested_open_boundary_ids=frozenset(("mouth",)),
        )
        self.assertTrue(self.policy.resolve(corridor, unverified).fail_closed)

        verified = dataclasses.replace(unverified, turn_connector_verified=True)
        effective = self.policy.apply(corridor, verified)
        self.assertFalse(effective.decision.fail_closed)
        self.assertEqual(
            effective.decision.opened_boundary_ids, frozenset(("mouth",))
        )
        self.assertTrue(
            self.checker.check_pose(Pose2D(5.0, 0.0, 0.0), effective).valid
        )

        for invalid_mouth in (
            dataclasses.replace(downstream, side=BoundarySide.DOWNSTREAM_END),
            dataclasses.replace(downstream, marking=BoundaryMarking.DASHED),
            dataclasses.replace(downstream, lane_ids=frozenset(("incoming",))),
        ):
            with self.subTest(invalid_mouth=invalid_mouth):
                invalid_corridor = DrivingCorridor(
                    "turn-link",
                    ("turn-link",),
                    (incoming, outgoing),
                    (invalid_mouth,),
                )
                denied = self.policy.resolve(invalid_corridor, verified)
                self.assertTrue(denied.fail_closed)
                self.assertEqual(denied.opened_boundary_ids, frozenset())

    def test_circle_contact_with_complete_body_is_invalid(self):
        effective = self.policy.apply(
            rectangular_corridor(-5.0, 5.0), CorridorPolicyInput()
        )
        front_x = self.vehicle.longitudinal_max_m
        tangent = CircleObstacle(Point2D(front_x + 1.0, 0.0), 1.0)
        separated = CircleObstacle(Point2D(front_x + 1.001, 0.0), 1.0)
        self.assertFalse(
            self.checker.check_pose(Pose2D(0.0, 0.0, 0.0), effective, (tangent,)).valid
        )
        self.assertTrue(
            self.checker.check_pose(
                Pose2D(0.0, 0.0, 0.0), effective, (separated,)
            ).valid
        )

    def test_oriented_box_uses_width_instead_of_a_circumscribed_circle(self):
        effective = self.policy.apply(
            rectangular_corridor(-5.0, 5.0), CorridorPolicyInput()
        )
        separated = BoxObstacle(Pose2D(0.0, 3.0, 0.0), 4.0, 1.0)
        self.assertTrue(
            self.checker.check_pose(
                Pose2D(0.0, 0.0, 0.0), effective, (separated,)
            ).valid
        )
        touching = BoxObstacle(
            Pose2D(0.0, self.vehicle.half_width_m + 0.5, 0.0),
            4.0,
            1.0,
        )
        result = self.checker.check_pose(
            Pose2D(0.0, 0.0, 0.0), effective, (touching,)
        )
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "obstacle_clearance")

    def test_bicycle_primitive_adaptively_certifies_between_fixed_samples(self):
        effective = self.policy.apply(rectangular_corridor(), CorridorPolicyInput())
        checker = FootprintCollisionChecker(self.vehicle, 0.20)
        start = Pose2D(
            0.0,
            3.0 - self.vehicle.half_width_m - 0.25,
            0.0,
        )
        valid, fixed_samples, evaluated = checker.check_bicycle_primitive(
            start,
            0.0,
            0.10,
            self.vehicle.wheelbase_m,
            0.10,
            effective,
        )
        self.assertTrue(valid)
        self.assertEqual(len(fixed_samples), 1)
        # Start + fixed endpoint would be two evaluations.  A third means the
        # ambiguous interval was bisected and certified rather than assumed safe.
        self.assertGreater(len(evaluated), 2)

    def test_evaluation_cap_allows_exactly_that_many_pose_checks(self):
        effective = self.policy.apply(rectangular_corridor(), CorridorPolicyInput())
        valid, fixed_samples, evaluated = self.checker.check_bicycle_primitive(
            Pose2D(0.0, 0.0, 0.0),
            0.0,
            0.10,
            self.vehicle.wheelbase_m,
            0.10,
            effective,
            maximum_evaluated_poses=2,
        )

        self.assertTrue(valid)
        self.assertEqual(len(fixed_samples), 1)
        self.assertEqual(len(evaluated), 2)
        self.assertTrue(all(result.valid for result in evaluated))

    def test_adversarial_continuous_check_stops_at_evaluation_budget(self):
        checker = FootprintCollisionChecker(self.vehicle, 0.20)
        delta_m = 1.1e-9
        corridor_half_width = self.vehicle.half_width_m + 0.20 + delta_m
        effective = self.policy.apply(
            rectangular_corridor(-corridor_half_width, corridor_half_width),
            CorridorPolicyInput(),
        )
        start = Pose2D(0.0, 0.0, 0.0)
        self.assertTrue(checker.check_pose(start, effective).valid)

        began = time.monotonic()
        valid, _samples, evaluated = checker.check_bicycle_primitive(
            start,
            0.0,
            0.10,
            self.vehicle.wheelbase_m,
            0.10,
            effective,
            maximum_evaluated_poses=8,
        )
        elapsed = time.monotonic() - began

        self.assertFalse(valid)
        self.assertLess(elapsed, 0.25)
        self.assertEqual(
            evaluated[-1].reason, "collision_check_evaluation_limit"
        )

    def test_expired_continuous_check_deadline_fails_closed(self):
        effective = self.policy.apply(rectangular_corridor(), CorridorPolicyInput())
        valid, samples, evaluated = self.checker.check_bicycle_primitive(
            Pose2D(0.0, 0.0, 0.0),
            0.0,
            0.10,
            self.vehicle.wheelbase_m,
            0.10,
            effective,
            deadline_monotonic=time.monotonic() - 1.0,
        )
        self.assertFalse(valid)
        self.assertEqual(samples, ())
        self.assertEqual(evaluated[-1].reason, "collision_check_time_limit")


if __name__ == "__main__":
    unittest.main()
