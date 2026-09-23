import unittest
import numpy as np
from localization_pkg.wall_matching import WallMatcher
from localization_pkg.live_estimator import GpsImuEstimator, GpsObservation, ImuObservation


LINES = [[[0,-6,0],[100,-6,0]], [[0,6,0],[100,6,0]]]
MOUNT = np.array([2.,0.,1.5])


def scan(sides=(-6.,6.)):
    return np.array([[x,y,z] for y in sides for x in np.linspace(-25,25,121)
                     for z in (2.,2.5,3.)])-MOUNT


def imu(stamp):
    return ImuObservation(stamp,[0,0,0,1],[0,0,9.80665],[0,0,0])


class WallMatchingTest(unittest.TestCase):
    def setUp(self):
        self.matcher = WallMatcher(LINES, 0.4)
        self.core = GpsImuEstimator()
        self.core.process_gps(GpsObservation(1.,[50,.8,1.3]),imu(1.))
        self.core.process_imu(imu(1.))

    def test_corrects_lateral_without_observing_tangent_or_jumping_odom(self):
        old = self.core.state.copy(); P = self.core.P.copy(); local=self.core.local_position.copy()
        self.assertTrue(self.core.process_wall(1.,scan(),imu(1.),self.matcher,MOUNT))
        self.assertLess(self.core.state[1],old[1]);self.assertGreater(self.core.state[1],0)
        self.assertAlmostEqual(self.core.state[0],old[0])
        self.assertAlmostEqual(self.core.P[0,0],P[0,0])
        self.assertLess(self.core.P[1,1],P[1,1])
        np.testing.assert_array_equal(self.core.local_position,local)
        self.assertEqual(self.core.last_gps_stamp,1.)

    def test_repeated_wall_aid_reduces_lateral_drift_and_preserves_tangent_uncertainty(self):
        for k in range(1,50):
            t=1+k*.02
            self.core.process_imu(imu(t))
            if k % 5 == 0:
                self.assertTrue(self.core.process_wall(t,scan(),imu(t),self.matcher,MOUNT))
        self.assertLess(abs(self.core.state[1]),0.25)
        self.assertGreater(self.core.P[0,0],1.)
        self.assertLess(self.core.P[1,1],self.core.P[0,0])

    def test_single_wall_short_obstacle_wrong_width_and_bad_heading_rejected(self):
        cloud=scan();angle=.2;rot=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        bad=[scan((-6.,)), np.array([[x,y,2.5] for x in (0,1,2) for y in (-6,6)]),
             scan((-4.,4.)), (cloud+MOUNT)@rot.T-MOUNT]
        for points in bad:
            with self.subTest(size=len(points)), self.assertRaises(ValueError):
                self.matcher.match(points,[0,0,0,1],[50,0,0],MOUNT)

    def test_no_gps_initialization_no_wall_bootstrap_and_no_delayed_updates(self):
        core=GpsImuEstimator()
        self.assertFalse(core.process_wall(1.,scan(),imu(1.),self.matcher,MOUNT))
        self.core.process_imu(imu(1.02));old=self.core.state.copy()
        self.assertFalse(self.core.process_wall(1.,scan(),imu(1.),self.matcher,MOUNT))
        np.testing.assert_array_equal(self.core.state,old)

    def test_nan_points_do_not_poison_match_and_wrong_map_geometry_rejected(self):
        cloud=np.vstack((scan(),[np.nan,1,1]))
        result=self.matcher.match(cloud,[0,0,0,1],[50,.8,0],MOUNT)
        self.assertAlmostEqual(abs(result['residual']),.8)
        with self.assertRaises(ValueError):WallMatcher([[[0,0,0],[0,0,0]],LINES[1]],.4)

    def test_direction_is_unobservable_even_with_cross_covariance(self):
        # A correlated prior must not turn a normal measurement into forward certainty.
        self.core.P[0,1]=self.core.P[1,0]=.02
        P=self.core.P.copy();x=self.core.state[0]
        self.assertTrue(self.core.process_wall(1.,scan(),imu(1.),self.matcher,MOUNT))
        self.assertAlmostEqual(self.core.P[0,0],P[0,0]);self.assertEqual(self.core.state[0],x)
        self.assertGreaterEqual(np.linalg.eigvalsh(self.core.P).min(),-1e-9)
