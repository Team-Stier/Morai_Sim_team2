import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from common_msgs_pkg.validation import (validate_component, validate_ego,
    validate_localization, validate_pair, validate_freshness)
from fixtures import component, ego, localization, stamp


class ValidationTest(unittest.TestCase):
    def test_valid_fixtures(self):
        validate_component(component())
        validate_pair(ego(), localization())

    def test_gps_blackout_can_preserve_local_estimation(self):
        value = localization()
        self.assertFalse(value.gps_fix_valid)
        self.assertFalse(value.stop_required)
        validate_localization(value)

    def test_unknown_state_is_not_ready(self):
        for state in (0, 1, 4, 5, 255):
            value = component()
            value.state = state
            with self.assertRaises(ValueError):
                validate_component(value)

    def test_missing_age_must_be_explicit(self):
        value = component()
        value.ready = False
        value.data_stamp = stamp(0)
        with self.assertRaises(ValueError):
            validate_component(value)
        value.data_age_sec = -1.
        validate_component(value)

    def test_frame_rename_is_not_coordinate_transform(self):
        value = ego()
        value.header.frame_id = 'odom'
        with self.assertRaises(ValueError):
            validate_ego(value)

    def test_unavailable_motion_is_not_a_zero_observation(self):
        value = ego()
        value.twist.twist.linear.y = float('nan')
        validate_ego(value)  # unavailable axis ignored
        value.twist_valid[1] = True
        with self.assertRaises(ValueError):
            validate_ego(value)

    def test_invalid_quaternion(self):
        value = ego()
        value.pose.pose.orientation.w = 0.
        with self.assertRaises(ValueError):
            validate_ego(value)

    def test_covariance_must_be_psd(self):
        for entry in (-1., float('nan')):
            value = ego()
            value.pose.covariance[0] = entry
            with self.assertRaises(ValueError):
                validate_ego(value)
        value = ego()
        value.pose.covariance[1] = value.pose.covariance[6] = 2.
        with self.assertRaises(ValueError):
            validate_ego(value)

    def test_reset_and_sample_mismatch(self):
        for change in ('reset', 'stamp'):
            value = localization()
            if change == 'reset':
                value.reset_id += 1
            else:
                value.ego_state_stamp = stamp(9)
            with self.assertRaises(ValueError):
                validate_pair(ego(), value)

    def test_valid_map_requires_xy_yaw_and_matching_uncertainty(self):
        value = ego()
        value.pose_valid[1] = False
        with self.assertRaises(ValueError):
            validate_pair(value, localization())
        status = localization()
        status.map_position_stddev_m = 0.
        with self.assertRaises(ValueError):
            validate_pair(ego(), status)

    def test_lost_local_motion_requires_stop(self):
        value = localization()
        value.mode = 5
        value.map_pose_valid = value.local_odometry_valid = False
        with self.assertRaises(ValueError):
            validate_localization(value)
        value.stop_required = True
        validate_localization(value)

    def test_no_gps_cannot_claim_tracking(self):
        value = localization()
        value.mode = 2
        with self.assertRaises(ValueError):
            validate_localization(value)

    def test_relocalizing_requires_stop_and_invalid_pose(self):
        value = localization()
        value.mode = 4
        value.stop_required = True
        value.map_pose_valid = value.local_odometry_valid = False
        validate_localization(value)
        for field in ('map_pose_valid', 'local_odometry_valid', 'stop_required'):
            original = getattr(value, field)
            setattr(value, field, not original)
            with self.assertRaises(ValueError):
                validate_localization(value)
            setattr(value, field, original)

    def test_freshness_zero_future_stale_regression(self):
        validate_freshness(stamp(9), 10., 1.)
        for value in (0, 11, 8):
            with self.assertRaises(ValueError):
                validate_freshness(stamp(value), 10., 1.)
        with self.assertRaises(ValueError):
            validate_freshness(stamp(9), 10., 1., previous_sec=10.)


if __name__ == '__main__':
    unittest.main()
