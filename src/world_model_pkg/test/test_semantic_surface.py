import unittest
import numpy as np
from world_model_pkg.semantic_surface import surface_mask

CLASSES = ['background', 'person', 'animal', 'box', 'bollard', 'barrier']
CONFIG = {'enabled': True, 'minimum_support_points': 3, 'depth_gap_m': .45,
          'class_extents_m': {'person': [1.2, 2.5, 1.]}}
K = np.array([[10.,0.,50.],[0.,10.,50.],[0.,0.,1.]])


def image():
    scores = np.zeros((100,100,6), np.float32)
    scores[:,:,0] = 1
    scores[40:60,40:60,0] = 0
    scores[40:60,40:60,1] = 1
    return scores


class SurfaceTest(unittest.TestCase):
    def test_person_front_kept_and_collinear_wall_removed(self):
        xyz = np.array([[0,0,5],[.1,0,5],[.2,0,5],
                        [0,0,20],[.4,0,20],[.8,0,20]], float)
        np.testing.assert_array_equal(surface_mask(xyz,image(),np.eye(4),K,CLASSES,CONFIG),
                                      [True,True,True,False,False,False])

    def test_touching_depth_chain_is_still_limited_to_object_size(self):
        xyz = np.array([[0,0,z] for z in np.arange(5.,10.,.2)])
        selected = surface_mask(xyz,image(),np.eye(4),K,CLASSES,CONFIG)
        self.assertTrue(selected[:5].all())
        self.assertLessEqual(xyz[selected,2].max()-xyz[selected,2].min(),1.)
        self.assertFalse(selected[-1])

    def test_independent_image_people_at_different_depths_are_kept(self):
        scores = image(); scores[40:60,80:90,0] = 0; scores[40:60,80:90,1] = 1
        xyz = np.array([[0,0,5],[.1,0,5],[.2,0,5],
                        [34,0,10],[34.1,0,10],[34.2,0,10]],float)
        self.assertTrue(surface_mask(xyz,scores,np.eye(4),K,CLASSES,CONFIG).all())

    def test_sparse_front_layer_does_not_fall_back_to_dense_rear(self):
        xyz = np.array([[0,0,5],[0,0,20],[.1,0,20],[.2,0,20]],float)
        self.assertFalse(surface_mask(xyz,image(),np.eye(4),K,CLASSES,CONFIG).any())

    def test_lateral_and_vertical_outliers_are_excluded(self):
        xyz = np.array([[0,0,5],[.1,0,5],[.2,0,5],[-4,0,5],[4,0,5],[0,4,5]],float)
        selected = surface_mask(xyz,image(),np.eye(4),K,CLASSES,CONFIG)
        np.testing.assert_array_equal(selected,[True,True,True,False,False,False])

    def test_invalid_configuration_and_empty_cloud(self):
        self.assertFalse(surface_mask(np.empty((0,3)),image(),np.eye(4),K,CLASSES,CONFIG).any())
        with self.assertRaises(ValueError):
            surface_mask(np.ones((3,3)),image(),np.eye(4),K,CLASSES,dict(CONFIG,depth_gap_m=0))
