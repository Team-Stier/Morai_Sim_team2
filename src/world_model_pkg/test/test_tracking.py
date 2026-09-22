import sys
import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))
from world_model_pkg.tracking import Detection, MultiObjectTracker, TrackerConfig


def detection(stamp_ns, x=10.0, y=5.0, local_id=0):
    return Detection(
        center=(x, y, 0.5),
        size=(2.0, 1.0, 1.0),
        source_stamp_ns=stamp_ns,
        source_frame_id="lidar_link",
        source_local_id=local_id,
        timestamp_provenance="ingress_fallback",
        calibration_id="development-lidar",
        calibration_verified=False,
    )


class TrackerTest(unittest.TestCase):
    def setUp(self):
        self.tracker = MultiObjectTracker(TrackerConfig(
            association_distance_m=2.0,
            maximum_coast_sec=0.5,
            minimum_confirmation_hits=2,
            minimum_velocity_hits=3,
        ))

    def test_static_map_detection_keeps_id_and_becomes_confirmed(self):
        first = self.tracker.update([detection(10_000_000_000)], 10_000_000_000, 1)
        second = self.tracker.update([detection(10_100_000_000, x=10.05)], 10_100_000_000, 1)
        self.assertEqual(first[0].track_id, second[0].track_id)
        self.assertEqual(first[0].state, self.tracker.TENTATIVE)
        self.assertEqual(second[0].state, self.tracker.CONFIRMED)
        self.assertLess(abs(second[0].center[0] - 10.05), 0.05)

    def test_scan_local_id_is_not_persistent_identity(self):
        first = self.tracker.update([detection(10_000_000_000, local_id=4)], 10_000_000_000, 1)
        second = self.tracker.update([detection(10_100_000_000, local_id=99)], 10_100_000_000, 1)
        self.assertEqual(first[0].track_id, second[0].track_id)
        self.assertEqual(second[0].source_local_id, 99)

    def test_unmatched_track_coasts_then_expires(self):
        first = self.tracker.update([detection(10_000_000_000)], 10_000_000_000, 1)
        coast = self.tracker.update([], 10_300_000_000, 1)
        expired = self.tracker.update([], 10_600_000_000, 1)
        self.assertEqual(coast[0].track_id, first[0].track_id)
        self.assertEqual(coast[0].state, self.tracker.COASTING)
        self.assertEqual(expired, ())

    def test_velocity_is_estimated_after_three_hits(self):
        self.tracker.update([detection(10_000_000_000, x=0.0)], 10_000_000_000, 1)
        self.tracker.update([detection(10_100_000_000, x=0.1)], 10_100_000_000, 1)
        third = self.tracker.update([detection(10_200_000_000, x=0.2)], 10_200_000_000, 1)
        self.assertTrue(third[0].velocity_valid)
        self.assertAlmostEqual(third[0].velocity[0], 1.0, places=6)

    def test_localization_reset_flushes_without_reusing_id(self):
        first = self.tracker.update([detection(10_000_000_000)], 10_000_000_000, 1)
        second = self.tracker.update([detection(10_100_000_000)], 10_100_000_000, 2)
        self.assertNotEqual(first[0].track_id, second[0].track_id)
        self.assertEqual(second[0].state, self.tracker.TENTATIVE)

    def test_regressing_stamp_fails_and_clears_tracks(self):
        self.tracker.update([detection(10_000_000_000)], 10_000_000_000, 1)
        with self.assertRaises(ValueError):
            self.tracker.update([detection(9_000_000_000)], 9_000_000_000, 1)
        self.assertEqual(self.tracker.views(10_000_000_000), ())


if __name__ == "__main__":
    unittest.main()
