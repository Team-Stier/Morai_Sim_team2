import copy
import math
from pathlib import Path
import unittest
import numpy as np
import yaml
from path_planning_pkg.frenet import Planner, Lane, Window, Obstacle, footprint_hit, quintic, geometry


class FrenetTest(unittest.TestCase):
    def setUp(self):
        self.c = yaml.safe_load((Path(__file__).parents[1]/'config/frenet_planner.yaml').read_text())
        self.p = Planner(self.c)
        s = np.arange(0., 121., .5)
        def lane(key, y):
            return Lane(key, np.column_stack((s, s*0+y, s*0)), s, s*0+58/3.6, [])
        self.lanes = {'global_route':lane('global_route', 0), 'side':lane('side', 3.5)}
        self.windows = [Window('global_route', 'side', 0, 100), Window('side', 'global_route', 30, 100)]

    def candidates(self, objects=(), boundaries=(), speed=2.):
        candidates = self.p.candidates(self.lanes, self.windows, 'global_route', 0, 80., np.array([0.,0.,0.]), 0., speed)
        return [self.p.evaluate(c, speed, objects, boundaries, 80.) for c in candidates]

    def test_quintic_boundary_conditions(self):
        q = np.array([0., 1e-5, 19.99999, 20.])
        d = quintic(1., .2, 3.5, 20., q)
        self.assertAlmostEqual(d[0], 1.)
        self.assertAlmostEqual(d[-1], 3.5)
        self.assertAlmostEqual((d[1]-d[0])/1e-5, .2, places=4)
        self.assertAlmostEqual((d[-1]-d[-2])/1e-5, 0., places=4)

    def test_clear_road_keeps_lane(self):
        candidates = self.candidates()
        self.assertEqual(self.p.select(candidates, 1., 0.).key, 'keep')
        self.assertTrue(math.isfinite(candidates[0].cost))

    def test_static_obstacle_selects_legal_alternative_after_confirmation(self):
        obstacle = Obstacle(np.array([[25.,-.5,0.],[25.,0.,0.],[25.,.5,0.]]), np.zeros(2))
        candidates = self.candidates([obstacle])
        self.assertTrue(candidates[0].feasible)
        self.assertTrue(math.isinf(candidates[0].cost))
        alternatives = [x for x in candidates[1:] if x.feasible and math.isfinite(x.cost)]
        self.assertTrue(alternatives)
        self.assertEqual(self.p.select(candidates, 1., 0.).key, 'keep')
        self.assertNotEqual(self.p.select(candidates, 1.61, 0.).key, 'keep')

    def test_solid_boundary_blocks_change(self):
        boundary = np.array([[0.,1.75,0.],[120.,1.75,0.]])
        candidates = self.candidates(boundaries=[boundary])
        self.assertFalse(any(x.feasible for x in candidates[1:]))
        self.assertTrue(candidates[0].feasible)

    def test_short_dashed_window_yields_no_change(self):
        self.windows = [Window('global_route','side',0.,8.)]
        self.assertEqual(len(self.candidates()), 1)

    def test_actual_points_do_not_fill_empty_box_interior(self):
        points = np.array([[0.,-2.,0.],[0.,2.,0.]])
        self.assertFalse(footprint_hit(points, np.zeros(3), 0., self.c))
        self.assertTrue(footprint_hit(np.array([[3.8,0.,0.]]), np.zeros(3), 0., self.c))

    def test_both_lanes_blocked_keep_stop(self):
        obs = [Obstacle(np.array([[25.,y,0.]]),np.zeros(2)) for y in (0.,3.5)]
        candidates = self.candidates(obs)
        choice = self.p.select(candidates,1.,0.)
        self.assertEqual(choice.key,'keep')
        self.assertTrue(np.any(choice.speed == 0))

    def test_fast_rear_vehicle_rejects_change(self):
        obs = Obstacle(np.array([[-25.,3.5,0.]]),np.array([20.,0.]))
        candidates = self.candidates([obs])
        self.assertTrue(any(x.reason == 'predicted_cluster_collision' for x in candidates[1:]))

    def test_score_formula(self):
        for candidate in self.candidates():
            if math.isfinite(candidate.cost):
                self.assertAlmostEqual(candidate.cost, candidate.eta+.75*candidate.changes+.25*candidate.comfort)

    def test_comfort_uses_sample_mean_of_normalized_acceleration_and_steering_rate(self):
        candidate = self.candidates()[0]
        goal = int(np.searchsorted(candidate.route_s, 80.))
        dt = np.diff(candidate.times[:goal+1])
        acceleration = np.diff(candidate.speed[:goal+1])/dt
        steering = np.arctan(3.*geometry(candidate.xy)[2][:goal+1])
        expected = np.mean((acceleration/2.)**2+(np.diff(steering)/dt/1.5)**2)
        self.assertAlmostEqual(candidate.comfort, expected)

    def test_slow_leader_increases_eta(self):
        clear=self.candidates()[0]
        following=self.candidates([Obstacle(np.array([[25.,0.,0.]]),np.array([1.,0.]))])[0]
        self.assertGreater(following.eta,clear.eta)
        self.assertEqual(following.reason,'following')

    def test_crossing_vehicle_has_finite_wait_candidate(self):
        keep=self.candidates([Obstacle(np.array([[15.,-5.,0.]]),np.array([0.,1.]))])[0]
        self.assertTrue(keep.feasible)
        self.assertTrue(math.isfinite(keep.cost))
        self.assertGreater(keep.wait,0.)
        self.assertEqual(keep.reason,'waiting_for_crossing')

    def test_high_speed_exit_is_braked_before_normal_zone(self):
        self.c['test_speed_cap_kph']=0.
        self.p=Planner(self.c)
        candidate=self.p.candidates(self.lanes,[], 'global_route',0.,80.,np.zeros(3),0.,20.)[0]
        candidate.limits[:80]=-1.
        result=self.p.profile(candidate,20.)
        self.assertIsNotNone(result)
        self.assertLessEqual(result[3][80],56/3.6)
        self.assertLess(result[3][79],20.)

    def test_commit_is_retained_after_cost_reversal(self):
        candidates=self.candidates()
        chosen=next(x for x in candidates[1:] if x.feasible)
        self.p.committed=chosen
        retained=copy.deepcopy(chosen);retained.key='committed';retained.cost=1000.
        self.assertIs(self.p.select(candidates+[retained],4.,0.),retained)

    def test_future_stop_time_is_not_immediate_stop(self):
        c = self.candidates([Obstacle(np.array([[25.,0.,0.]]),np.zeros(2))])[0]
        self.assertGreater(c.speed[0],0)
        self.assertTrue(np.any(c.speed==0))


if __name__ == '__main__':
    unittest.main()
