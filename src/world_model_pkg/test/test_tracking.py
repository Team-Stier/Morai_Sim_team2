import sys
import unittest
import numpy as np
from dataclasses import replace
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))
from world_model_pkg.tracking import Detection, MultiObjectTracker, TrackerConfig
from world_model_pkg.point_motion import translation


def detection(stamp_ns, x=10.0, y=5.0, local_id=0):
    return Detection(
        center=(x, y, 0.5),
        points=((x, y, 0.5), (x+0.1, y, 0.5)),
        source_stamp_ns=stamp_ns,
        source_frame_id="lidar_link",
        source_local_id=local_id,
        timestamp_provenance="ingress_fallback",
        calibration_id="development-lidar",
        calibration_verified=False,
    )


class TrackerTest(unittest.TestCase):
    def test_partial_static_surface_does_not_follow_centroid(self):
        points = np.column_stack((np.zeros(800), np.linspace(0,20,800), np.ones(800)))
        shift = translation(points[:600], points[80:680], max_points=160, iterations=8)
        self.assertLess(np.linalg.norm(shift), 0.1)
        self.assertGreater(np.linalg.norm(points[80:680].mean(0)-points[:600].mean(0)), 1.9)

    def test_compact_fast_object_translation_and_points_preserved(self):
        rng = np.random.default_rng(2)
        cloud = np.column_stack((rng.uniform([-2,-1],[2,1],(96,2)), np.ones(96)))
        tracker = MultiObjectTracker(TrackerConfig(maximum_speed_mps=60))
        for i in range(4):
            points = tuple(map(tuple, cloud+[5*i,0,0]))
            stamp = 10_000_000_000+i*100_000_000
            d = replace(detection(stamp), center=tuple(np.mean(points,axis=0)), points=points)
            result = tracker.update([d],stamp,1)[0]
        self.assertAlmostEqual(result.velocity[0],50.0)
        self.assertTrue(result.velocity_valid)
        self.assertEqual(result.points,points)
        self.assertEqual(result.source_stamp_ns,stamp)

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
        self.assertEqual(second[0].points, detection(10_100_000_000).points)

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

    def test_fast_rear_vehicle_keeps_identity_at_ten_hz(self):
        tracker = MultiObjectTracker(TrackerConfig(maximum_speed_mps=60.0))
        tracks = [tracker.update([detection(10_000_000_000+i*100_000_000, x=-30.+5.*i)],
                                 10_000_000_000+i*100_000_000, 1)[0] for i in range(6)]
        self.assertEqual(len({t.track_id for t in tracks}), 1)
        self.assertTrue(tracks[-1].velocity_valid)
        self.assertAlmostEqual(tracks[-1].velocity[0], 50.0)

    def test_regressing_stamp_fails_and_clears_tracks(self):
        self.tracker.update([detection(10_000_000_000)], 10_000_000_000, 1)
        with self.assertRaises(ValueError):
            self.tracker.update([detection(9_000_000_000)], 9_000_000_000, 1)
        self.assertEqual(self.tracker.views(10_000_000_000), ())


if __name__ == "__main__":
    unittest.main()
