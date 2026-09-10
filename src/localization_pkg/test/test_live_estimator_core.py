#!/usr/bin/env python3
import math
import unittest
import numpy as np
from localization_pkg.live_estimator import GpsImuEstimator, GpsObservation, ImuObservation, rotation


def imu(t, q=(0., 0., 0., 1.), acceleration=None):
    return ImuObservation(t, q, rotation(q).T.dot([0., 0., 9.80665]) if acceleration is None else acceleration, [0., 0., 0.])


class CoreTest(unittest.TestCase):
    def test_rotated_arm_and_stationary_gravity(self):
        q = np.array([.12, -.18, .3, .9]); q /= np.linalg.norm(q)
        core = GpsImuEstimator()
        core.process_imu(imu(1., q))
        expected = np.array([20., -30., 4.])
        self.assertTrue(core.process_gps(GpsObservation(1., expected+rotation(q).dot([0., 0., 1.3]))))
        for t in np.arange(1.02, 2., .02):
            self.assertTrue(core.process_imu(imu(t, q)))
        state = core.snapshot()
        np.testing.assert_allclose(state['map_position'], expected, atol=1e-9)
        np.testing.assert_allclose(state['local_position'], 0, atol=1e-9)
        self.assertAlmostEqual(state['map_position_stddev'], math.sqrt(core.P[0, 0]+core.P[1, 1]))
        for key in ('map_pose_covariance', 'local_pose_covariance', 'twist_covariance'):
            self.assertGreaterEqual(np.linalg.eigvalsh(state[key]).min(), -1e-10)

    def test_gps_correction_does_not_jump_local_pose_and_blackout_recovers(self):
        core = GpsImuEstimator()
        core.process_imu(imu(1.))
        core.process_gps(GpsObservation(1., [0., 0., 1.3]))
        for t in np.arange(1.02, 4., .02):
            core.process_imu(imu(t))
        before = core.local_position.copy()
        self.assertTrue(core.process_gps(GpsObservation(core.stamp, [.5, -.3, 1.3])))
        np.testing.assert_allclose(core.local_position, before)
        self.assertGreater(core.state[0], 0.)
        np.testing.assert_allclose(core.snapshot()['map_odom_translation']+core.local_position, core.state[:3])

    def test_rejections_and_reset(self):
        core = GpsImuEstimator()
        core.process_imu(imu(1.))
        core.process_gps(GpsObservation(1., [0., 0., 1.3]))
        core.process_imu(imu(1.1))
        for t in (0., 1., 1.1, float('nan')):
            self.assertFalse(core.process_imu(imu(t)))
        self.assertFalse(core.process_imu(imu(1.2, acceleration=[float('nan'), 0., 9.8])))
        self.assertFalse(core.process_gps(GpsObservation(1.05, [0., 0., 1.3])))
        self.assertFalse(core.process_gps(GpsObservation(1.1, [0., 0., 1.3], np.diag([-1., 0., 0.]))))
        core.reset()
        self.assertIsNone(core.snapshot())
        self.assertIsNone(core.last_gps_stamp)


class GpsInnovationResetTest(unittest.TestCase):
    def setUp(self):
        self.core = GpsImuEstimator()
        self.core.process_imu(imu(1.))
        self.core.process_gps(GpsObservation(1., [0., 0., 1.3]))
        self.t = 1.

    def advance(self, seconds=.2, acceleration=None):
        steps = round(seconds/.02)
        for _ in range(steps):
            self.t += .02
            self.core.process_imu(imu(self.t, acceleration=acceleration))

    def gps(self, x=100., y=0., noise=None):
        return self.core.process_gps(GpsObservation(self.t, [x, y, 1.3], noise))

    def test_first_exceeding_fix_resets_during_motion_and_preserves_odom(self):
        self.core.state[3] = 7.
        self.t += .02
        sample = imu(self.t, acceleration=[5., 0., 9.80665])
        sample.angular_velocity_radps = [0., 0., 1.]
        self.core.process_imu(sample)
        old_local = self.core.local_position.copy()
        old_uncertainty = self.core.local_position_stddev.copy()
        self.assertTrue(self.gps(3.))  # One fix, less than the old 15 m jump requirement.
        self.assertTrue(self.core.gps_reinitialized)
        np.testing.assert_allclose(self.core.state, [3., 0., 0., 0., 0., 0.], atol=1e-8)
        np.testing.assert_allclose(self.core.local_position, old_local)
        np.testing.assert_allclose(self.core.local_position_stddev, old_uncertainty)
        self.assertEqual(self.core.last_gps_stamp, self.t)
        self.assertIn('GPS innovation reset', self.core.gps_diagnostic)
        self.assertGreater(np.linalg.eigvalsh(self.core.P).min(), 0.)

    def test_normal_correction_does_not_reset(self):
        self.advance(.02)
        self.assertTrue(self.gps(.1))
        self.assertFalse(self.core.gps_reinitialized)
        self.assertGreater(self.core.state[0], 0.)
        self.assertLess(self.core.state[0], .1)

    def test_first_fix_after_long_blackout_can_reset(self):
        self.advance(20.)
        self.assertTrue(self.gps(1000.))
        self.assertTrue(self.core.gps_reinitialized)
        self.assertEqual(self.core.state[0], 1000.)
        self.assertEqual(self.core.last_gps_stamp, self.t)

    def test_every_exceeding_fix_is_accepted_without_cooldown(self):
        for x in (100., 0., 110.):
            self.advance(.02)
            self.assertTrue(self.gps(x))
            self.assertTrue(self.core.gps_reinitialized)
            self.assertEqual(self.core.state[0], x)
        self.advance(.02)
        self.assertTrue(self.gps(110.))
        self.assertFalse(self.core.gps_reinitialized)

    def test_threshold_boundary_uses_strictly_greater(self):
        # Identity innovations make residual [3, 4, 0] exactly chi2=25.
        for dx, expected_reset in ((0., False), (.001, True)):
            self.setUp()
            self.core.gps_translation[:] = 0.
            self.core.config.gps_position_stddev_floor_m = .5
            self.advance(.02)
            self.core.P[:3, :3] = np.eye(3)*.75
            position = np.array([3.+dx, 4., 0.])
            self.assertTrue(self.core.process_gps(GpsObservation(self.t, position)))
            self.assertEqual(self.core.gps_reinitialized, expected_reset)
            np.testing.assert_allclose(self.core.state[:3], position if expected_reset else .75*position)


if __name__ == '__main__':
    unittest.main()
