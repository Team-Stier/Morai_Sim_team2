#!/usr/bin/env python3
"""Motion-model covariance: no new sensor, no pseudo zero lateral speed."""
import unittest
from pathlib import Path

import numpy as np
import yaml
from localization_pkg.live_estimator import (
    EstimatorConfig, GpsImuEstimator, GpsObservation, ImuObservation)


def propagate(density, dt=.02, acceleration=2.):
    core = GpsImuEstimator(EstimatorConfig(
        motion_model_spectral_density_m2ps3=density))
    def imu(t):
        return ImuObservation(t, [0., 0., 0., 1.],
                              [acceleration, 0., 9.80665], [0., 0., 0.])
    core.process_imu(imu(1.))
    core.process_gps(GpsObservation(1., [0., 0., 1.3]))
    core.state[3:6] = [10., .6, 0.]  # Deliberate nonzero lateral velocity.
    for i in range(1, round(2./dt)+1):
        assert core.process_imu(imu(1.+i*dt)), core.last_rejection
    return core


class MotionUncertaintyTest(unittest.TestCase):
    def test_exact_continuous_covariance_and_rate_invariance(self):
        expected = np.kron([[8./3., 2.], [2., 2.]], np.eye(3))
        for dt in (.01, .02, .05, .1):
            low, high = propagate(1.e-12, dt), propagate(1., dt)
            np.testing.assert_allclose(high.P[:6, :6]-low.P[:6, :6],
                                       expected, rtol=1e-8, atol=1e-8)
            np.testing.assert_allclose(high.state, low.state, atol=1e-10)
            self.assertGreater(np.linalg.eigvalsh(high.P).min(), 0.)

    def test_quiet_motion_preserves_calibration_model(self):
        for acceleration in (0., .05, .2):
            low = propagate(1.e-12, acceleration=acceleration)
            high = propagate(1., acceleration=acceleration)
            np.testing.assert_allclose(high.P, low.P, atol=1e-10)

    def test_partial_motion_blends_continuously(self):
        low = propagate(1.e-12, acceleration=.6)
        high = propagate(1., acceleration=.6)
        expected = .5*np.kron([[8./3., 2.], [2., 2.]], np.eye(3))
        np.testing.assert_allclose(high.P[:6, :6]-low.P[:6, :6], expected, atol=1e-8)

    def test_blackout_preserves_lateral_motion_and_reports_more_uncertainty(self):
        low, high = propagate(1.e-12), propagate(1.)
        self.assertAlmostEqual(high.state[1], 1.2)
        self.assertAlmostEqual(high.state[4], .6)
        self.assertAlmostEqual(high.state[0], 24.)
        self.assertIsNone(high.last_stationary_update)
        self.assertEqual(high.last_gps_stamp, 1.)
        self.assertGreater(high.snapshot()['map_position_stddev'],
                           low.snapshot()['map_position_stddev'])
        self.assertGreater(high.snapshot()['local_position_stddev'],
                           low.snapshot()['local_position_stddev'])

    def test_gps_correction_never_jumps_local_pose(self):
        core = propagate(1.)
        local = core.local_position.copy()
        stamp = core.stamp
        self.assertTrue(core.process_gps(GpsObservation(stamp, [24.5, 1.4, 1.3])))
        np.testing.assert_allclose(core.local_position, local)
        self.assertEqual(core.stamp, stamp)
        self.assertFalse(core.gps_reinitialized)

    def test_yaml_and_runtime_defaults_match(self):
        root = Path(__file__).resolve().parents[1]
        values = yaml.safe_load((root/'config/localization.yaml').read_text())
        config = EstimatorConfig()
        for key in vars(config):
            if key.startswith('motion_model_'):
                self.assertEqual(values[key], getattr(config, key))

    def test_independent_acceleration_model_mismatch_before_blackout(self):
        # Analytic scenario, NOT the MORAI recording: 4 m/s cruise, 3 s at
        # 2 m/s^2, then 3 s blackout. Only the pre-blackout IMU has 20% scale
        # error. The filter must not interpret transient model error as bias.
        errors = []
        for density in (1.e-12, 1.):
            core = GpsImuEstimator(EstimatorConfig(
                motion_model_spectral_density_m2ps3=density))
            position, velocity = 0., 4.
            for i in range(1051):
                elapsed = i*.02
                acceleration = 2. if 15. < elapsed <= 18. else 0.
                position += velocity*.02 + .5*acceleration*.02**2
                velocity += acceleration*.02
                measured = acceleration*.8 if elapsed <= 18. else acceleration
                core.process_imu(ImuObservation(1.+elapsed, [0., 0., 0., 1.],
                    [measured, 0., 9.80665], [0., 0., 0.]))
                if i % 10 == 0 and elapsed <= 18.:
                    self.assertTrue(core.process_gps(GpsObservation(
                        1.+elapsed, [position, 0., 1.3])))
            errors.append(abs(core.state[0]-position))
        self.assertLess(errors[1], .7)
        self.assertLess(errors[1], errors[0]*.4)

    def test_invalid_model_config(self):
        for key in ('motion_model_spectral_density_m2ps3',
                    'motion_model_quiet_acceleration_mps2',
                    'motion_model_full_acceleration_mps2'):
            for value in (0., -1., float('nan'), float('inf')):
                with self.assertRaises(ValueError):
                    EstimatorConfig(**{key: value})
        with self.assertRaises(ValueError):
            EstimatorConfig(motion_model_full_acceleration_mps2=.1)


if __name__ == '__main__':
    unittest.main()
