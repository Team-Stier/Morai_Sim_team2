import math
import unittest
from types import SimpleNamespace

from path_planning_pkg import Point2D, Pose2D
from path_planning_pkg.node_safety import (
    aligned_path_index,
    lane_change_reference,
    lead_fault_reason,
    lead_reason,
    nearest_lane_id,
    odometry_payload_reason,
    observation_reason,
    select_forward_valid_goal,
    same_route_snapshot,
)


class Stamp(object):
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


def vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def lead(stamp=10.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map", stamp=Stamp(stamp)),
        valid=True,
        lane_link_id="A",
        pose=SimpleNamespace(
            position=vector(1.0, 2.0, 3.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=SimpleNamespace(linear=vector(), angular=vector()),
        length_m=4.0,
        width_m=2.0,
        longitudinal_distance_m=10.0,
        confidence=0.9,
    )


def odometry():
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=vector(1.0, 2.0, 3.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=[0.0] * 36,
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(linear=vector(), angular=vector()),
            covariance=[0.0] * 36,
        ),
    )


class NodeSafetyTest(unittest.TestCase):
    def test_any_future_timestamp_is_rejected(self):
        message = lead(stamp=10.000001)
        self.assertEqual(
            observation_reason(message, 10.0, 0.3, "map"),
            "future_timestamp",
        )

    def test_lead_validation_covers_orientation_and_twist(self):
        message = lead()
        message.pose.orientation.w = 0.0
        self.assertEqual(
            lead_reason(message, 10.1, 0.3, "map", "A", 0.5),
            "invalid_orientation",
        )
        message = lead()
        message.twist.angular.z = math.nan
        self.assertEqual(
            lead_reason(message, 10.1, 0.3, "map", "A", 0.5),
            "non_finite_lead",
        )
        message = lead()
        message.valid = False
        message.pose.position.x = math.nan
        self.assertEqual(
            lead_reason(message, 10.1, 0.3, "map", "A", 0.5),
            "non_finite_lead",
        )

    def test_lead_input_requires_an_explicit_fresh_no_lead_heartbeat(self):
        self.assertEqual(
            lead_fault_reason(None, 10.1, 0.3, "map", "A", 0.5),
            "missing_input",
        )

        no_lead = lead()
        no_lead.valid = False
        self.assertEqual(
            lead_fault_reason(no_lead, 10.1, 0.3, "map", "A", 0.5),
            "",
        )

        stale_no_lead = lead(stamp=9.0)
        stale_no_lead.valid = False
        self.assertEqual(
            lead_fault_reason(stale_no_lead, 10.1, 0.3, "map", "A", 0.5),
            "stale_input",
        )

        wrong_lane = lead()
        wrong_lane.lane_link_id = "B"
        self.assertEqual(
            lead_fault_reason(wrong_lane, 10.1, 0.3, "map", "A", 0.5),
            "lead_lane_mismatch",
        )

    def test_odometry_validation_covers_twist_and_both_covariances(self):
        message = odometry()
        self.assertEqual(odometry_payload_reason(message), "")
        message.twist.twist.angular.y = math.nan
        self.assertEqual(
            odometry_payload_reason(message), "non_finite_ego_twist"
        )
        message = odometry()
        message.pose.covariance[17] = math.inf
        self.assertEqual(
            odometry_payload_reason(message), "invalid_pose_covariance"
        )
        message = odometry()
        message.twist.covariance = [0.0] * 35
        self.assertEqual(
            odometry_payload_reason(message), "invalid_twist_covariance"
        )

    def test_latest_ego_must_track_a_certified_path_with_useful_horizon(self):
        points = tuple(
            SimpleNamespace(
                pose=Pose2D(float(index), 0.0, 0.0),
                distance_from_start_m=float(index),
            )
            for index in range(11)
        )
        self.assertEqual(
            aligned_path_index(points, Pose2D(3.1, 0.1, 0.02), 0.25, 0.1, 5.0),
            3,
        )
        self.assertIsNone(
            aligned_path_index(points, Pose2D(3.0, 0.4, 0.0), 0.25, 0.1, 5.0)
        )
        self.assertIsNone(
            aligned_path_index(points, Pose2D(9.0, 0.0, 0.0), 0.25, 0.1, 5.0)
        )

    def test_forward_goal_scans_past_an_invalid_link_seam(self):
        class Lane(object):
            def __init__(self, identifier, start, length):
                self.link_id = identifier
                self.start = start
                self.length_m = length

            def pose_at(self, progress):
                return Pose2D(self.start + progress, 0.0, 0.0)

        lanes = (Lane("A", 0.0, 20.0), Lane("B", 20.0, 20.0))
        pose, lane_id = select_forward_valid_goal(
            lanes,
            start_progress_m=0.0,
            preferred_distance_m=18.0,
            minimum_distance_m=6.0,
            final_end_margin_m=5.0,
            scan_step_m=0.5,
            pose_is_valid=lambda candidate: not 17.5 <= candidate.x <= 21.0,
        )
        self.assertEqual(lane_id, "B")
        self.assertGreater(pose.x, 21.0)

    def test_route_snapshot_change_is_detected(self):
        first = SimpleNamespace(
            valid=True,
            current_link_id="A",
            horizon_link_ids=("A", "B"),
            speed_limit_exempt_zone=True,
        )
        same = SimpleNamespace(**vars(first))
        changed = SimpleNamespace(**vars(first))
        changed.horizon_link_ids = ("A", "C")
        self.assertTrue(same_route_snapshot(first, same))
        self.assertFalse(same_route_snapshot(first, changed))

    def test_lane_change_guide_has_exact_pose_endpoints(self):
        start = Pose2D(0.0, 0.0, 0.0)
        goal = Pose2D(10.0, 3.5, 0.0)
        points = lane_change_reference(start, goal)
        self.assertEqual(points[0], Point2D(start.x, start.y))
        self.assertEqual(points[-1], Point2D(goal.x, goal.y))
        self.assertGreater(len(points), 3)

    def test_nearest_lane_id_changes_with_geometry(self):
        class Lane(object):
            def __init__(self, identifier, y_value):
                self.link_id = identifier
                self.y_value = y_value

            def nearest_progress(self, point):
                return point.x, abs(point.y - self.y_value)

        lanes = (Lane("lower", 0.0), Lane("upper", 3.5))
        self.assertEqual(nearest_lane_id(Point2D(1.0, 0.2), lanes), "lower")
        self.assertEqual(nearest_lane_id(Point2D(1.0, 3.2), lanes), "upper")


if __name__ == "__main__":
    unittest.main()
