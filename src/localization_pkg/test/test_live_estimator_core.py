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


if __name__ == '__main__':
    unittest.main()
