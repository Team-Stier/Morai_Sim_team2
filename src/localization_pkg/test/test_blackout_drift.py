#!/usr/bin/env python3
"""Deterministic physical trajectories, without ROS or simulator truth inputs."""
import unittest

import numpy as np

from localization_pkg.live_estimator import (
    EstimatorConfig, GpsImuEstimator, GpsObservation, ImuObservation, rotation)


BIAS = np.array([0.04, -0.025, 0.015])


def sample(t, q=(0., 0., 0., 1.), acceleration=(0., 0., 0.), bias=BIAS):
    force = rotation(q).T.dot(np.array(acceleration)+[0., 0., 9.80665])+bias
    return ImuObservation(t, q, force, [0., 0., 0.])


def drive(core, duration, trajectory, gps_until, q=(0., 0., 0., 1.)):
    """50 Hz IMU, 5 Hz GPS, timestamps all in one measurement clock."""
    snapshots = {}
    for i in range(round(duration*50)+1):
        elapsed, stamp = i/50., 1.+i/50.
        position, acceleration = trajectory(elapsed)
        observation = sample(stamp, q, acceleration)
        accepted = core.process_imu(observation)
        if i and not accepted:
            raise AssertionError(core.last_rejection)
        if i % 10 == 0 and elapsed <= gps_until:
            fix = GpsObservation(stamp, np.array(position)+rotation(q).dot([0., 0., 1.3]))
            if not core.process_gps(fix):
                raise AssertionError(core.last_rejection)
        if i % 50 == 0:
            snapshots[int(elapsed)] = core.snapshot()
    return snapshots


def stationary(_):
    return [0., 0., 0.], [0., 0., 0.]


def tunnel_stop(t):
    # Cruise 4 m/s until 20 s, smoothly brake over 2 s, then remain stopped.
    if t <= 20.:
        return [4.*t, 0., 0.], [0., 0., 0.]
    if t < 22.:
        u = (t-20.)/2.
        return [80.+8.*(u-u**3+0.5*u**4), 0., 0.], [12.*(u*u-u), 0., 0.]
    return [84., 0., 0.], [0., 0., 0.]


class BlackoutDriftTest(unittest.TestCase):
    def test_stationary_bias_then_fifteen_second_blackout(self):
        core = GpsImuEstimator()
        records = drive(core, 35., stationary, 20.)
        self.assertIsNotNone(core.last_stationary_update)
        self.assertLessEqual(core.last_stationary_update, 21.)
        np.testing.assert_allclose(core.state[6:], BIAS, atol=.003)
        self.assertLess(np.linalg.norm(records[35]['map_position']-records[20]['map_position']), .25)
        self.assertLess(np.linalg.norm(records[35]['local_position']-records[20]['local_position']), .25)
        self.assertGreater(records[35]['map_position_stddev'], records[20]['map_position_stddev'])
        self.assertGreater(np.linalg.eigvalsh(core.P).min(), 0.)

    def test_driving_into_blackout_then_stopping(self):
        core = GpsImuEstimator()
        records = drive(core, 35., tunnel_stop, 20.)
        self.assertIsNone(core.last_stationary_update, 'Quiet cruising is not a stop')
        self.assertLess(np.linalg.norm(core.state[:3]-[84., 0., 0.]), .35)
        self.assertLess(np.linalg.norm(records[35]['map_position']-records[22]['map_position']), .25)
        self.assertLess(np.linalg.norm(core.state[3:6]), .04)

    def test_quiet_cruising_never_receives_zero_velocity(self):
        core = GpsImuEstimator()
        drive(core, 35., lambda t: ([4*t, 0., 0.], [0., 0., 0.]), 20.)
        self.assertIsNone(core.last_stationary_update)
        self.assertAlmostEqual(core.state[0], 140., delta=.35)
        self.assertAlmostEqual(core.state[3], 4., delta=.04)

    def test_tilted_stationary_sensor_estimates_body_bias(self):
        q = np.array([.12, -.18, .3, .9])
        q /= np.linalg.norm(q)
        core = GpsImuEstimator()
        records = drive(core, 35., stationary, 20., q)
        np.testing.assert_allclose(core.state[6:], BIAS, atol=.003)
        self.assertLess(np.linalg.norm(records[35]['map_position']-records[20]['map_position']), .25)

    def test_single_fix_and_quiet_imu_do_not_invent_stationarity(self):
        core = GpsImuEstimator()
        records = drive(core, 15., stationary, 0.)
        self.assertIsNone(core.last_stationary_update)
        np.testing.assert_allclose(core.state[6:], 0.)
        self.assertGreater(records[15]['map_position_stddev'], records[0]['map_position_stddev'])
        self.assertGreater(np.linalg.norm(core.state[:3]), 1.)

    def test_blackout_does_not_repeat_zero_velocity_and_acceleration_is_preserved(self):
        core = GpsImuEstimator()
        drive(core, 20., stationary, 20.)
        last_update = core.last_stationary_update
        start = core.state[:3].copy()
        # Acceleration starts after the last GPS. Never learn it as a bias or pin pose.
        for i in range(1, 101):
            self.assertTrue(core.process_imu(sample(21.+i/50., acceleration=[1., 0., 0.])))
        self.assertEqual(core.last_stationary_update, last_update)
        self.assertAlmostEqual(core.state[3], 1.99, delta=.03)
        self.assertAlmostEqual(core.state[0]-start[0], 1.98, delta=.04)
        np.testing.assert_allclose(core.state[6:], BIAS, atol=.003)

    def test_relocation_and_reset_discard_bias_and_stationary_evidence(self):
        core = GpsImuEstimator()
        drive(core, 10., stationary, 10.)
        old_local = core.local_position.copy()
        self.assertTrue(core.process_imu(sample(11.02)))
        old_local = core.local_position.copy()
        self.assertTrue(core.process_gps(GpsObservation(11.02, [100., 0., 1.3])))
        self.assertTrue(core.gps_reinitialized)
        np.testing.assert_allclose(core.state[6:], 0.)
        np.testing.assert_allclose(core.local_position, old_local)
        self.assertIsNone(core.last_stationary_update)
        self.assertFalse(core.stationary_gps)
        core.reset()
        self.assertIsNone(core.snapshot())
        self.assertIsNone(core.quiet_since)

    def test_uncertain_gps_and_out_and_back_motion_do_not_confirm_stop(self):
        for uncertain in (False, True):
            core = GpsImuEstimator()
            for i in range(201):
                stamp = 1.+i/50.
                core.process_imu(sample(stamp, bias=np.zeros(3)))
                if i % 10 == 0:
                    # Same endpoints with a large intermediate excursion must fail.
                    x = 0. if uncertain else .3*np.sin(i*np.pi/100.)
                    noise = np.eye(3)*4. if uncertain else None
                    core.process_gps(GpsObservation(stamp, [x, 0., 1.3], noise))
            self.assertIsNone(core.last_stationary_update)

    def test_invalid_parameters(self):
        for value in (0., -1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                EstimatorConfig(initial_accelerometer_bias_stddev_mps2=value)


if __name__ == '__main__':
    unittest.main()
