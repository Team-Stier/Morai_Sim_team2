import copy
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
import yaml
from path_planning_pkg.frenet import Planner, Lane, Window, Obstacle, ObstacleGrid, Candidate, footprint_hit, footprint_hits, quintic, geometry, candidate_geometry, geometry_windows


# Frozen scalar oracle for collision batching regression checks.
def scalar_collision(self, xy, theta, times, objects, start_delay=0.):
    c = self.c
    # Both spatial and temporal interpolation: short clusters cannot fall
    # between coarse trajectory samples, including a fast rear vehicle.
    obstacle_grid = objects if isinstance(objects,ObstacleGrid) else ObstacleGrid(objects,c)
    travelled = 0.
    for i in range(len(xy)-1):
        travelled += np.linalg.norm(xy[i+1,:2]-xy[i,:2])
        if travelled > c['collision_precision_distance_m']:
            break
        if not np.isfinite(times[i+1]) or times[i]+start_delay > c['prediction_horizon_sec']:
            break
        duration = times[i+1]-times[i]
        inspected_duration = min(duration, c['prediction_horizon_sec']-times[i]-start_delay)
        end_fraction = inspected_duration/duration if duration > 0 else 1.
        count = max(1, int(math.ceil(end_fraction*np.linalg.norm(xy[i+1, :2]-xy[i, :2])/c['collision_step_m'])),
                    int(math.ceil(inspected_duration/c['collision_time_step_sec'])))
        for u in np.linspace(0, end_fraction, count+1):
            t = times[i]+u*(times[i+1]-times[i])+start_delay
            pos = xy[i]*(1-u)+xy[i+1]*u
            angle = theta[i]*(1-u)+theta[i+1]*u
            for obj in obstacle_grid.near(pos):
                points = obj.points.copy()
                if obj.velocity_valid:
                    points[:, :2] += (t+obj.age)*obj.velocity[:2]
                if footprint_hit(points, pos, angle, c):
                    return i
    return None


class FrenetTest(unittest.TestCase):
    def setUp(self):
        self.c = yaml.safe_load((Path(__file__).parents[1]/'config/frenet_planner.yaml').read_text())
        self.c['test_speed_cap_kph'] = 10.0  # Low-speed scenario fixtures.
        self.c['rddf_geometry_only'] = False  # Original map-rule scenarios.
        self.c['loop_route'] = False
        self.c['gain_confirmation_sec'] = 0.05  # Explicit delayed-selection fixtures.
        self.p = Planner(self.c)
        s = np.arange(0., 121., .5)
        def lane(key, y):
            return Lane(key, np.column_stack((s, s*0+y, s*0)), s, s*0+58/3.6, [])
        self.lanes = {'global_route':lane('global_route', 0), 'side':lane('side', 3.5)}
        self.windows = [Window('global_route', 'side', 0, 100), Window('side', 'global_route', 30, 100)]

    def candidates(self, objects=(), boundaries=(), speed=2.):
        candidates = self.p.candidates(self.lanes, self.windows, 'global_route', 0, 80., np.array([0.,0.,0.]), 0., speed)
        return [self.p.evaluate(c, speed, objects, boundaries, 80.) for c in candidates]

    def test_geometry_only_neighbors_require_near_parallel_rddfs(self):
        windows = geometry_windows(self.lanes,self.c)
        self.assertEqual({(w.source,w.target) for w in windows},
                         {('global_route','side'),('side','global_route')})
        self.lanes['side'].xy[:,1] = 10.
        self.assertEqual(geometry_windows(self.lanes,self.c),[])
        self.lanes['side'].xy[:,1] = 3.5
        self.lanes['side'].xy = self.lanes['side'].xy[::-1]
        self.assertEqual(geometry_windows(self.lanes,self.c),[])

    def test_geometry_only_does_not_require_return_to_checkpoint_lane(self):
        self.c['rddf_geometry_only'] = True
        self.p = Planner(self.c)
        candidates = self.candidates()
        self.assertTrue(any(c.changes == 1 and c.feasible and c.target == 'side' for c in candidates))

    def test_short_rddf_returns_before_endpoint_on_curved_reference(self):
        self.c['rddf_geometry_only'] = True
        s = np.arange(0.,151.,.5)
        angle = s/70.
        ref = np.column_stack((70*np.sin(angle),70*(1-np.cos(angle)),s*0))
        normal = np.column_stack((-np.sin(angle),np.cos(angle),s*0))
        lanes = {'global_route':Lane('global_route',ref,s,s*0+16.)}
        mask = s <= 80.
        lanes['side'] = Lane('side',(ref+normal*3.5)[mask],s[mask],s[mask]*0+16.)
        ego = lanes['side'].xy[80]
        candidate = self.p.candidates(lanes, [], 'side', 40., 120., ego,40/70.,5.)[0]
        after = candidate.route_s >= 80.
        expected = np.column_stack([np.interp(candidate.route_s[after],s,ref[:,i]) for i in range(3)])
        np.testing.assert_allclose(candidate.xy[after],expected,atol=1e-8)
        np.testing.assert_allclose(candidate.xy[0],ego)
        self.assertEqual(candidate.target,'global_route')
        self.assertEqual(candidate.changes,1)
        self.assertLess(candidate.change_end,80.)
        self.assertLess(np.max(np.abs(np.arctan(3*geometry(candidate.xy)[2]))),self.c['max_steering_rad'])

    def test_route_progress_delay_does_not_create_reverse_first_segment(self):
        candidate = self.p.candidates(self.lanes,self.windows,'global_route',10.,80.,
                                     np.array([12.,0.,0.]),0.,10.)[0]
        self.assertAlmostEqual(candidate.route_s[0],12.)
        self.assertTrue(np.all(np.diff(candidate.xy[:,0])>0.))
        # A roadside object must not be hit by a spurious 180-degree turn.
        obj = Obstacle(np.array([[12.,2.5,0.]]),np.zeros(2),False)
        evaluated = self.p.evaluate(candidate,10.,[obj],[],80.)
        self.assertTrue(evaluated.feasible)
        self.assertTrue(math.isfinite(evaluated.eta))

    def test_quintic_boundary_conditions(self):
        q = np.array([0., 1e-5, 19.99999, 20.])
        d = quintic(1., .2, 3.5, 20., q)
        self.assertAlmostEqual(d[0], 1.)
        self.assertAlmostEqual(d[-1], 3.5)
        self.assertAlmostEqual((d[1]-d[0])/1e-5, .2, places=4)
        self.assertAlmostEqual((d[-1]-d[-2])/1e-5, 0., places=4)

    def test_batched_footprints_match_scalar_for_rotations_chunks_and_edges(self):
        rng = np.random.RandomState(20260922)
        points = rng.uniform(-30.,30.,(1100,3))
        poses = rng.uniform(-25.,25.,(75,3))
        headings = rng.uniform(-math.pi,math.pi,75)
        for margin in (0., .2):
            config = dict(self.c, object_margin_m=margin)
            expected = [footprint_hit(points,p,h,config) for p,h in zip(poses,headings)]
            np.testing.assert_array_equal(footprint_hits(points,poses,headings,config),expected)
        for point in ([self.c['front_overhang_m'],0.,0.],
                      [-self.c['rear_overhang_m'],0.,0.],
                      [0.,self.c['vehicle_width_m']/2,0.],
                      [self.c['front_overhang_m']+1e-7,0.,0.]):
            points = np.array([point])
            self.assertEqual(footprint_hits(points,np.zeros((1,3)),np.zeros(1),self.c)[0],
                             footprint_hit(points,np.zeros(3),0.,self.c))
        self.assertFalse(footprint_hits(np.empty((0,3)),poses,headings,self.c).any())

    def test_batched_collision_matches_scalar_static_and_moving_objects(self):
        rng = np.random.RandomState(20260923)
        for scenario in range(40):
            with self.subTest(scenario=scenario):
                x = np.arange(0.,20.,.5)
                xy = np.column_stack((x, np.sin(x/8.)*(scenario%3), x*0))
                s,theta,_ = geometry(xy)
                times = s/(1.+scenario%9)
                if scenario%7 == 0:
                    times[20:] = math.inf
                objects = [Obstacle(rng.uniform([-5.,-5.,0.],[25.,5.,1.],(2,3)),
                    rng.uniform(-4.,4.,2), scenario%4 != 0, .15) for _ in range(3)]
                # Include a crossing cluster and a fast rear vehicle.
                objects += [Obstacle(np.array([[7.,4.,0.]]), np.array([0.,-2.]),True),
                            Obstacle(np.array([[-4.,0.,0.]]), np.array([12.,0.]),True)]
                if scenario%5 == 0:
                    objects = [Obstacle(np.array([[100.,100.,0.]]), np.zeros(2))]
                delay = (scenario%4)*1.5
                grid = ObstacleGrid(objects,self.c)
                expected = scalar_collision(self.p,xy,theta,times,grid,delay)
                self.assertEqual(self.p.collision(xy,theta,times,grid,delay),expected)
        self.assertIsNone(self.p.collision(xy,theta,np.full(len(xy),math.inf),[]))
        self.assertIsNone(self.p.collision(xy,theta,times,[],start_delay=9.))

    def test_clear_road_keeps_lane(self):
        candidates = self.candidates()
        self.assertEqual(self.p.select(candidates, 1., 0.).key, 'keep')
        self.assertTrue(math.isfinite(candidates[0].cost))

    def test_static_box_detour_without_adjacent_rddf_window(self):
        base = self.p.candidates(self.lanes, [], 'global_route', 0., 80.,
                                 np.array([0.,0.,0.]), 0., 0.)[0]
        box = Obstacle(np.array([[9.,-.25,0.],[9.,.5,0.],[9.,1.2,0.]]), np.zeros(2), True)
        self.p.evaluate(base, 0., [box], [], 80.)
        self.assertFalse(math.isfinite(base.cost))
        detours = self.p.obstacle_detours(base, [box])
        evaluated = [self.p.evaluate(x, 0., [box], [], 80.) for x in detours]
        clear = [x for x in evaluated if x.feasible and math.isfinite(x.cost)]
        self.assertTrue(clear)
        for candidate in clear:
            s, theta, curvature = geometry(candidate.xy)
            np.testing.assert_allclose(candidate.xy[[0,-1]], base.xy[[0,-1]])
            self.assertIsNone(self.p.collision(candidate.xy, theta, candidate.times, [box]))
            self.assertLessEqual(np.max(np.abs(np.arctan(3*curvature))), self.c['max_steering_rad'])
            self.assertLessEqual(np.max(candidate.speed[candidate.route_s <= candidate.change_end]),
                                 self.c['local_detour_speed_kph']/3.6+1e-8)
        candidates = [base]+evaluated
        self.assertIs(self.p.select(candidates, 1., 0.), base)
        chosen = self.p.select(candidates, 2., 0.)
        self.assertTrue(chosen.key.startswith('detour:'))
        self.assertIs(self.p.committed, chosen)

    def test_detours_choose_either_side_when_opposite_side_is_blocked(self):
        for blocked_side in (-1., 1.):
            with self.subTest(blocked_side=blocked_side):
                planner = Planner(self.c)
                base = planner.candidates(self.lanes, [], 'global_route', 0., 80.,
                                          np.zeros(3), 0., 0.)[0]
                box = Obstacle(np.array([[9., y, 0.] for y in np.linspace(-.5,.5,11)]),
                               np.zeros(2), True)
                wall = Obstacle(np.array([[x, blocked_side*y, 0.]
                    for x in np.arange(1.,25.,.5) for y in np.arange(1.,5.,.25)]),
                    np.zeros(2), True)
                objects = [box,wall]
                planner.evaluate(base,0.,objects,[],80.)
                detours = planner.obstacle_detours(base,objects)
                offsets = {float(c.key.split(':')[1]) for c in detours}
                self.assertEqual(offsets, {-3.5,-2.5,-1.5,1.5,2.5,3.5})
                evaluated = [planner.evaluate(c,0.,objects,[],80.) for c in detours]
                planner.select([base]+evaluated,1.,0.)
                chosen = planner.select([base]+evaluated,2.,0.)
                self.assertIsNotNone(chosen)
                self.assertTrue(math.isfinite(chosen.cost))
                self.assertLess(float(chosen.key.split(':')[1])*blocked_side,0.)

    def test_local_detour_never_bypasses_a_fully_blocked_corridor(self):
        base = self.p.candidates(self.lanes, [], 'global_route', 0., 80.,
                                 np.array([0.,0.,0.]), 0., 0.)[0]
        wall = Obstacle(np.array([[9., y, 0.] for y in np.arange(-7.,7.,.2)]), np.zeros(2), False)
        evaluated = [self.p.evaluate(x, 0., [wall], [], 80.)
                     for x in self.p.obstacle_detours(base, [wall])]
        self.assertTrue(evaluated)
        self.assertFalse(any(x.feasible and math.isfinite(x.cost) for x in evaluated))

    def test_local_detours_only_for_static_obstruction(self):
        base = self.p.candidates(self.lanes, [], 'global_route', 0., 80.,
                                 np.array([0.,0.,0.]), 0., 0.)[0]
        moving = Obstacle(np.array([[9.,0.,0.]]), np.array([3.,0.]), True)
        self.assertEqual(self.p.obstacle_detours(base, [moving]), [])
        self.assertEqual(self.p.obstacle_detours(base, []), [])

    def test_loop_candidate_continues_across_identical_endpoints(self):
        self.c['loop_route']=True
        angles=np.linspace(0.,2*math.pi,1001)
        xyz=np.column_stack((50*np.cos(angles),50*np.sin(angles),angles*0))
        s=geometry(xyz)[0]
        lane=Lane('global_route',xyz,s,np.full(len(s),58/3.6),[])
        progress=s[-1]-2.
        ego=np.array([np.interp(progress,s,xyz[:,i]) for i in range(3)])
        result=Planner(self.c).candidates({'global_route':lane},[], 'global_route',progress,
            progress+100.,ego,math.pi/2-2/50,2.)[0]
        self.assertGreater(result.route_s[-1],s[-1]+90.)
        self.assertGreater(result.limits[-1],0.)
        self.assertLess(np.max(np.linalg.norm(np.diff(result.xy[:,:2],axis=0),axis=1)),1.)

    def test_static_obstacle_selects_legal_alternative_after_confirmation(self):
        obstacle = Obstacle(np.array([[25.,-.5,0.],[25.,0.,0.],[25.,.5,0.]]), np.zeros(2))
        candidates = self.candidates([obstacle])
        self.assertTrue(candidates[0].feasible)
        self.assertTrue(math.isinf(candidates[0].cost))
        alternatives = [x for x in candidates[1:] if x.feasible and math.isfinite(x.cost)]
        self.assertTrue(alternatives)
        self.assertEqual(self.p.select(candidates, 1., 0.).key, 'keep')
        self.assertEqual(self.p.select(candidates, 1.04, 0.).key, 'keep')
        self.assertNotEqual(self.p.select(candidates, 1.06, 0.).key, 'keep')

    def test_zero_confirmation_selects_checked_detour_without_an_extra_stop_tick(self):
        self.c['gain_confirmation_sec'] = 0.0
        obstacle = Obstacle(np.array([[25.,-.5,0.],[25.,0.,0.],[25.,.5,0.]]), np.zeros(2))
        candidates = self.candidates([obstacle])
        candidates[0].feasible = False
        chosen = self.p.select(candidates, 1., 0.)
        self.assertIsNotNone(chosen)
        self.assertIsNot(chosen, candidates[0])
        self.assertTrue(chosen.feasible)
        self.assertTrue(math.isfinite(chosen.cost))
        _, theta, _ = geometry(chosen.xy)
        self.assertIsNone(self.p.collision(chosen.xy, theta, chosen.times, [obstacle]))

    def test_zero_confirmation_does_not_select_when_every_path_is_blocked(self):
        self.c['gain_confirmation_sec'] = 0.0
        obstacle = Obstacle(np.array([[0.,0.,0.]]), np.zeros(2))
        candidates = self.candidates([obstacle])
        self.assertFalse(any(c.feasible for c in candidates))
        self.assertIsNone(self.p.select(candidates, 1., 0.))

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

    def test_obstacle_grid_returns_only_nearby_measured_clusters(self):
        near=Obstacle(np.array([[5.,0.,0.]]),np.zeros(2))
        far=Obstacle(np.array([[80.,20.,0.]]),np.zeros(2))
        grid=ObstacleGrid([near,far],self.c)
        self.assertEqual(grid.near(np.array([5.,0.,0.])),[near])
        self.assertEqual(grid.near(np.array([80.,20.,0.])),[far])

    def test_collision_precision_horizon_ignores_far_geometry_for_collision_only(self):
        candidate=self.p.candidates(self.lanes,[], 'global_route',0.,80.,np.zeros(3),0.,2.)[0]
        s,theta,_=candidate_geometry(candidate)
        profile=self.p.profile(candidate,2.)
        far=Obstacle(np.array([[60.,0.,0.]]),np.zeros(2))
        self.assertIsNone(self.p.collision(candidate.xy,theta,profile[4],ObstacleGrid([far],self.c)))
        near=Obstacle(np.array([[20.,0.,0.]]),np.zeros(2))
        self.assertIsNotNone(self.p.collision(candidate.xy,theta,profile[4],ObstacleGrid([near],self.c)))

    def test_candidate_geometry_is_cached(self):
        candidate=Candidate('x','x',self.lanes['global_route'].xy,self.lanes['global_route'].s,
                            self.lanes['global_route'].limits)
        first=candidate_geometry(candidate)
        self.assertIs(first,candidate_geometry(candidate))

    def test_near_standstill_collision_sampling_ends_at_prediction_horizon(self):
        obstacle = Obstacle(np.array([[4.5,0.,0.]]),np.zeros(2))
        with patch('path_planning_pkg.frenet.footprint_hits', wraps=footprint_hits) as check:
            self.assertIsNone(self.p.collision(np.array([[0.,0.,0.],[1.,0.,0.]]),np.zeros(2),
                             np.array([0.,1e8]),[obstacle]))
        positions = np.concatenate([call[0][1] for call in check.call_args_list])
        self.assertLessEqual(len(positions),162)
        self.assertLessEqual(positions[-1][0],8e-8)

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

    def test_speed_above_immediate_curve_cap_brakes_without_rejecting_path(self):
        candidate=self.p.candidates(self.lanes,[], 'global_route',0.,80.,np.zeros(3),0.,12.)[0]
        result=self.p.profile(candidate,12.)
        self.assertIsNotNone(result)
        self.assertEqual(result[3][0],12.)
        self.assertGreater(result[3][1],0.)
        self.assertLess(result[3][1],result[3][0])

    def test_keep_path_is_not_rejected_by_local_rddf_curvature_noise(self):
        candidate=self.p.candidates(self.lanes,[], 'global_route',0.,80.,np.zeros(3),0.,2.)[0]
        candidate.xy[20,1]=2.
        evaluated=self.p.evaluate(candidate,2.,[],[],80.)
        self.assertTrue(evaluated.feasible)
        self.assertNotIn(evaluated.reason,('steering_limit','steering_rate_limit'))

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
