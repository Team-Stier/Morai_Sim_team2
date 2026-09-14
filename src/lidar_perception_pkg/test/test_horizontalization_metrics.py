import math
from pathlib import Path
import unittest
import numpy as np
import yaml
from lidar_perception_pkg.horizontalization_metrics import paired_metrics, distribution


class MetricsTest(unittest.TestCase):
    def setUp(self):
        self.config=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/horizontalization_metrics.yaml').read_text())
        x,y=np.meshgrid(np.linspace(3,18,40),np.linspace(-4,4,25))
        self.level=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-1.8)))
        angle=.06;c,s=math.cos(angle),math.sin(angle)
        self.rotation=np.array([[c,0,s],[0,1,0],[-s,0,c]])

    def test_known_flat_plane_recovers_but_plane_residual_does_not_improve(self):
        self.level[:,2]+=np.random.RandomState(2).normal(0,.004,len(self.level))
        raw=self.level@self.rotation
        metrics,before,after=paired_metrics(raw,self.rotation,self.config)
        self.assertGreater(metrics['before_tilt_deg'],3.)
        self.assertLess(metrics['after_tilt_deg'],.03)
        self.assertLess(metrics['after_horizontal_height_rmse_m'],.006)
        self.assertAlmostEqual(metrics['before_orthogonal_plane_rmse_m'],metrics['after_orthogonal_plane_rmse_m'],places=12)
        self.assertLess(metrics['range_preservation_rmse_m'],1e-12)
        self.assertLess(metrics['round_trip_rmse_m'],1e-12)
        np.testing.assert_allclose(after,before@self.rotation.T)

    def test_real_slope_is_reported_as_residual_and_not_forced_to_zero(self):
        self.level[:,2]+=.02*(self.level[:,0]-10)
        metrics,_,_=paired_metrics(self.level@self.rotation,self.rotation,self.config)
        self.assertAlmostEqual(metrics['after_tilt_deg'],math.degrees(math.atan(.02)),places=6)
        self.assertNotIn('accuracy',metrics)

    def test_raw_patch_selection_is_identical_for_different_corrections(self):
        raw=self.level@self.rotation
        _,first,_=paired_metrics(raw,self.rotation,self.config)
        _,second,_=paired_metrics(raw,np.eye(3),self.config)
        np.testing.assert_array_equal(first,second)

    def test_unsupported_geometry_and_non_rotation_are_rejected(self):
        for points in (np.empty((0,3)),np.ones((200,3))*np.nan,self.level[:10]):
            with self.assertRaises(ValueError):paired_metrics(points,np.eye(3),self.config)
        with self.assertRaises(ValueError):paired_metrics(self.level,np.diag([1,1,2]),self.config)
        line=np.column_stack((np.linspace(3,18,300),np.zeros(300),np.full(300,-1.8)))
        with self.assertRaises(ValueError):paired_metrics(line,np.eye(3),self.config)

    def test_empty_summary_is_not_a_perfect_score(self):
        self.assertIsNone(distribution([]))
        self.assertIsNone(distribution([float('nan')]))


if __name__=='__main__':unittest.main()
