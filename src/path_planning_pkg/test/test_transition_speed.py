from pathlib import Path
import unittest

import numpy as np
import yaml

from hd_map_pkg.course_speed import CourseSpeedZones, load_course_speed_policy
from hd_map_pkg.geometry import cumulative_lengths
from hd_map_pkg.lane_rddf import read_route
from path_planning_pkg.frenet import Candidate, Lane, Planner, sample_limits
from path_planning_pkg.planner_mode_manager import FRENET, HYBRID_ASTAR
from path_planning_pkg.transition_speed import TransitionSpeedPolicy


ROOT = Path(__file__).resolve().parents[1]


class TransitionSpeedPolicyTest(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT / 'config/frenet_planner.yaml').read_text())
        self.config['planner_mode'] = yaml.safe_load(
            (ROOT / 'config/planner_mode.yaml').read_text())['planner_mode']
        self.policy = TransitionSpeedPolicy(self.config)
        self.b1, self.b2, self.b3 = (
            self.config['planner_mode']['zones'][index]['end_s'] for index in (0, 1, 3))
        stations = np.array([0., self.b1 - 1., self.b1, self.b2 - 1.,
                             self.b2, 1118.7417511690987, self.b3 - 1.,
                             self.b3, self.config['planner_mode']['route_length_m']])
        limits = np.array([58. / 3.6] * 5 + [-1., -1., 58. / 3.6, 58. / 3.6])
        xyz = np.column_stack([stations, np.zeros_like(stations), np.zeros_like(stations)])
        self.lane = Lane('global_route', xyz, stations, limits)

    def test_all_mode_boundaries_are_thirty_and_continuous(self):
        transition = 30. / 3.6
        for boundary, mode in ((self.b1, FRENET),
                               (self.b2, HYBRID_ASTAR),
                               (self.b3, FRENET)):
            stations = np.array([boundary - 1.e-5, boundary, boundary + 1.e-5])
            caps = self.policy.caps(stations, mode, self.lane)
            np.testing.assert_allclose(caps, transition, atol=1.e-5)

    def test_candidate_tail_uses_adjacent_mode_and_hybrid_base_is_twenty_five(self):
        hybrid = 25. / 3.6
        entry = self.policy.caps([self.b1 - 10., self.b1, self.b1 + 20.,
                                  self.b1 + 100.], FRENET, self.lane)
        self.assertAlmostEqual(entry[1], 30. / 3.6)
        self.assertAlmostEqual(entry[2], hybrid)
        self.assertAlmostEqual(entry[3], hybrid)
        exit_caps = self.policy.caps([self.b2 - 40., self.b2 - 10., self.b2,
                                      self.b2 + 100.], HYBRID_ASTAR, self.lane)
        self.assertAlmostEqual(exit_caps[0], hybrid)
        self.assertGreater(exit_caps[1], hybrid)
        self.assertAlmostEqual(exit_caps[2], 30. / 3.6)
        self.assertTrue(np.isinf(exit_caps[3]))
        self.assertTrue(np.isinf(self.policy.caps([100.], FRENET, self.lane)[0]))

    def test_z2_and_z5_interior_profiles_use_twenty_five_cap(self):
        hybrid = 25. / 3.6
        for start in (self.b1 + 80., self.b3 + 80.):
            with self.subTest(start=start):
                stations = np.linspace(start, start + 20., 41)
                xyz = np.column_stack([stations, np.zeros_like(stations),
                                       np.zeros_like(stations)])
                candidate = Candidate('hybrid_astar', 'global_route', xyz, stations,
                                      np.full(len(stations), 58. / 3.6))
                candidate.speed_cap_mps = self.policy.caps(stations, HYBRID_ASTAR,
                                                            self.lane)
                _, _, _, speeds, _ = Planner(self.config).profile(
                    candidate, 20. / 3.6)
                np.testing.assert_allclose(candidate.speed_cap_mps, hybrid)
                self.assertTrue(np.all(speeds <= hybrid + 1.e-9))

    def test_linear_ramps_fit_acceleration_and_begin_early_for_high_speed(self):
        high = 150. / 3.6
        boundary_speed = 30. / 3.6
        distance = max(
            self.config['mode_transition_min_distance_m'],
            high * (high - boundary_speed) / self.config['deceleration_mps2'] +
            self.config['mode_transition_reaction_sec'] * high,
        )
        start = self.b3 - distance
        self.assertLess(start, 1118.7417511690987)
        samples = np.array([start, start + 10., start + 20.,
                            self.b3 - 20., self.b3 - 10., self.b3])
        caps = self.policy.caps(samples, FRENET, self.lane)
        self.assertAlmostEqual(caps[0], high)
        self.assertAlmostEqual(caps[-1], boundary_speed)
        expected_slope = (boundary_speed - high) / distance
        np.testing.assert_allclose(np.diff(caps) / np.diff(samples),
                                   expected_slope, atol=1.e-12)
        self.assertLessEqual(high * abs(expected_slope),
                             self.config['deceleration_mps2'])

    def test_profile_combines_transition_and_obstacle_caps(self):
        stations = np.arange(0., 101.)
        xyz = np.column_stack([stations, np.zeros_like(stations), np.zeros_like(stations)])
        candidate = Candidate('keep', 'global_route', xyz, stations.copy(),
                              np.full(len(stations), 58. / 3.6))
        candidate.speed_cap_mps = np.full(len(stations), 30. / 3.6)
        obstacle_cap = np.full(len(stations), 7.)
        _, _, _, speeds, _ = Planner(self.config).profile(candidate, 0., cap=obstacle_cap)
        self.assertAlmostEqual(speeds[50], 7.)
        self.assertTrue(np.all(speeds <= obstacle_cap + 1.e-9))
        candidate.speed_cap_mps = np.ones(3)
        with self.assertRaises(ValueError):
            Planner(self.config).profile(candidate, 0.)

    def test_official_high_speed_exit_limit_has_no_interpolated_zero_notch(self):
        source = ROOT.parents[1] / '참고파일들/2026_molit_comp_global_path (3).txt'
        raw = read_route(source)
        zones = CourseSpeedZones(raw, load_course_speed_policy())
        retained = [index for index, point in enumerate(raw)
                    if index == 0 or point != raw[index - 1]]
        route = [raw[index] for index in retained]
        station = cumulative_lengths(route)
        end = retained.index(zones.end)
        self.assertAlmostEqual(station[end], self.b3)
        lane_s = np.asarray(station[end - 1:end + 2])
        lane = Lane('global_route', np.asarray(route[end - 1:end + 2]), lane_s,
                    np.array([-1., 58. / 3.6, 58. / 3.6]))
        query = np.array([lane_s[0], (lane_s[0] + lane_s[1]) / 2.,
                          lane_s[1], lane_s[1] + 0.01])
        np.testing.assert_array_equal(sample_limits(lane, query),
                                      [-1., -1., 58. / 3.6, 58. / 3.6])


if __name__ == '__main__':
    unittest.main()
