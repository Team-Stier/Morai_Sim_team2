#!/usr/bin/env python3

import math
import unittest

from runtime_evaluation_pkg.planning_visualization import (
    IONIQ5_BODY_CENTER_X_M,
    IONIQ5_HEIGHT_M,
    PoseSample,
    Quaternion,
    TraceHistory,
    marker_center,
    remaining_marker_lifetime_sec,
    validate_pose_sample,
)


def sample(**overrides):
    values = {
        "stamp_sec": 9.9,
        "frame_id": "map",
        "child_frame_id": "base_link",
        "position": (10.0, 20.0, 1.0),
        "orientation": Quaternion(0.0, 0.0, 0.0, 1.0),
        "linear_velocity": (1.0, 0.0, 0.0),
        "angular_velocity": (0.0, 0.0, 0.1),
        "pose_covariance": (0.0,) * 36,
        "twist_covariance": (0.0,) * 36,
    }
    values.update(overrides)
    return PoseSample(**values)


class ValidationTest(unittest.TestCase):
    def test_accepts_finite_fresh_contract_frames(self):
        result = validate_pose_sample(sample(), now_sec=10.0)
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.age_sec, 0.1)

    def test_rejects_wrong_frames(self):
        self.assertEqual(
            validate_pose_sample(sample(frame_id="odom"), 10.0).reason,
            "wrong_parent_frame",
        )
        self.assertEqual(
            validate_pose_sample(sample(child_frame_id="ego"), 10.0).reason,
            "wrong_child_frame",
        )

    def test_rejects_stale_and_future_timestamps(self):
        self.assertEqual(
            validate_pose_sample(sample(stamp_sec=9.79), 10.0).reason,
            "stale_timestamp",
        )
        self.assertEqual(
            validate_pose_sample(sample(stamp_sec=10.01), 10.0).reason,
            "future_timestamp",
        )

    def test_rejects_non_finite_payload_and_bad_quaternion(self):
        self.assertEqual(
            validate_pose_sample(sample(position=(math.nan, 0.0, 0.0)), 10.0).reason,
            "non_finite_position",
        )
        self.assertEqual(
            validate_pose_sample(
                sample(orientation=Quaternion(0.0, 0.0, 0.0, 0.0)), 10.0
            ).reason,
            "non_unit_orientation",
        )
        covariance = [0.0] * 36
        covariance[4] = math.inf
        self.assertEqual(
            validate_pose_sample(sample(pose_covariance=covariance), 10.0).reason,
            "invalid_pose_covariance",
        )


class GeometryAndHistoryTest(unittest.TestCase):
    def test_marker_lifetime_is_anchored_to_sample_timestamp(self):
        self.assertAlmostEqual(
            remaining_marker_lifetime_sec(9.90, 10.00, timeout_sec=0.25),
            0.15,
        )
        self.assertEqual(
            remaining_marker_lifetime_sec(9.75, 10.00, timeout_sec=0.25),
            0.0,
        )
        self.assertEqual(
            remaining_marker_lifetime_sec(10.01, 10.00, timeout_sec=0.25),
            0.0,
        )
        self.assertEqual(
            remaining_marker_lifetime_sec(math.nan, 10.00, timeout_sec=0.25),
            0.0,
        )

    def test_body_center_uses_rear_axle_offset_in_vehicle_frame(self):
        yaw_90 = Quaternion(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
        center = marker_center(sample(position=(1.0, 2.0, 3.0), orientation=yaw_90))
        self.assertAlmostEqual(center[0], 1.0, places=6)
        self.assertAlmostEqual(center[1], 2.0 + IONIQ5_BODY_CENTER_X_M, places=6)
        self.assertAlmostEqual(center[2], 3.0 + IONIQ5_HEIGHT_M * 0.5, places=6)

    def test_history_is_bounded_and_strictly_time_ordered(self):
        history = TraceHistory(max_samples=2)
        self.assertTrue(history.append(sample(stamp_sec=1.0)))
        self.assertFalse(history.append(sample(stamp_sec=1.0)))
        self.assertTrue(history.append(sample(stamp_sec=2.0)))
        self.assertTrue(history.append(sample(stamp_sec=3.0)))
        self.assertEqual([pose.stamp_sec for pose in history.snapshot()], [2.0, 3.0])


if __name__ == "__main__":
    unittest.main()
