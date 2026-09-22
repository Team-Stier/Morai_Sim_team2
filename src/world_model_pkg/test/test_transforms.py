import math
import unittest
from world_model_pkg.transforms import transform_points

class TransformPointsTest(unittest.TestCase):
    def test_rotation_preserves_each_point_and_order(self):
        half = math.sqrt(0.5)
        result = transform_points(((5,2,.5), (6,4,1)), (10,20,1.5), (0,0,half,half))
        for actual, expected in zip(result, ((8,25,2), (6,26,2.5))):
            for a, b in zip(actual, expected): self.assertAlmostEqual(a,b)
    def test_rejects_invalid_geometry(self):
        for points, q in (((), (0,0,0,1)), (((float('nan'),0,0),), (0,0,0,1)), (((0,0,0),), (0,0,0,0))):
            with self.assertRaises(ValueError): transform_points(points, (0,0,0), q)
