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


class RelocationTest(unittest.TestCase):
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

    def test_consistent_jump_reanchors_map_resets_velocity_preserves_odom(self):
        self.core.state[3] = .5
        for index in range(5):
            self.advance()
            old_local = self.core.local_position.copy()
            accepted = self.gps()
            self.assertEqual(accepted, index == 4)
            self.assertEqual(self.core.gps_reinitialized, index == 4)
            if index < 4:
                self.assertTrue(self.core.relocation.pending)
                self.assertEqual(self.core.last_gps_stamp, 1.)
        np.testing.assert_allclose(self.core.state, [100., 0., 0., 0., 0., 0.], atol=1e-8)
        np.testing.assert_allclose(self.core.local_position, old_local)
        self.assertAlmostEqual(self.core.last_gps_stamp, self.t)
        self.assertFalse(self.core.relocation.pending)
        self.assertGreater(np.linalg.eigvalsh(self.core.P).min(), 0.)

    def test_single_outlier_and_inconsistent_cluster_do_not_reset(self):
        self.advance(); self.gps()
        self.advance(); self.gps(0.)
        self.assertFalse(self.core.relocation.pending)
        for x in [100., 110., 100., 120., 100., 110.]:
            self.advance(); self.gps(x)
            self.assertFalse(self.core.gps_reinitialized)

    def test_blackout_return_is_not_a_discontinuity_event(self):
        self.advance(2.)
        for _ in range(8):
            self.advance(); self.gps()
            self.assertFalse(self.core.gps_reinitialized)
            self.assertFalse(self.core.relocation.pending)

    def test_gps_and_imu_gaps_break_confirmation(self):
        self.advance(); self.gps()
        self.assertTrue(self.core.relocation.pending)
        self.advance(1.)
        self.assertFalse(self.core.relocation.pending)
        self.gps()
        self.assertFalse(self.core.gps_reinitialized)
        self.setUp()
        self.advance(); self.gps()
        self.assertTrue(self.core.relocation.pending)
        self.t += .4
        self.assertFalse(self.core.process_imu(imu(self.t)))
        self.assertFalse(self.core.relocation.pending)

    def test_duplicate_nan_and_uncertain_gps_break_confirmation(self):
        for mode in ['duplicate', 'nan', 'uncertain']:
            self.setUp()
            self.advance(); self.gps()
            self.assertTrue(self.core.relocation.pending)
            if mode == 'duplicate': self.gps()
            elif mode == 'nan':
                self.advance(); self.gps(float('nan'))
            else:
                self.advance(); self.gps(noise=np.eye(3)*100.)
            self.assertFalse(self.core.relocation.pending)
            self.assertFalse(self.core.gps_reinitialized)

    def test_motion_and_disable_prevent_stationary_reinitialization(self):
        self.advance(); self.gps()
        self.advance(acceleration=[5., 0., 9.80665]); self.gps()
        self.assertFalse(self.core.relocation.pending)
        self.setUp()
        self.core.relocation.config.enabled = False
        for _ in range(8):
            self.advance(); self.gps()
            self.assertFalse(self.core.gps_reinitialized)

    def test_rate_alone_cannot_confirm_and_reset_clears_evidence(self):
        for _ in range(6):
            self.advance(.02); self.gps()
            self.assertFalse(self.core.gps_reinitialized)
        self.assertTrue(self.core.relocation.pending)
        self.core.reset()
        self.assertFalse(self.core.relocation.pending)
        self.assertIsNone(self.core.relocation.previous)


if __name__ == '__main__':
    unittest.main()
