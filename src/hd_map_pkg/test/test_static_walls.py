import unittest
from types import SimpleNamespace
from hd_map_pkg.coordinates import CoordinateTransformer
from hd_map_pkg.static_walls import build_static_walls


class StaticWallsTest(unittest.TestCase):
    def setUp(self):
        self.points = [[-2795, 1300, 28], [-2790, 1300, 28.5]]
        self.dataset = SimpleNamespace(data={'object_set': [dict(idx='wall1', name='wall', points=self.points)]})
        self.transform = CoordinateTransformer([305390, 4122845, 0], [302595, 4124145, 0])
        self.config = {'static_walls': {'source_object_ids': ['wall1']}}

    def test_source_xyz_preserved_without_fabricated_height(self):
        wall = build_static_walls(self.dataset, self.transform, self.config)[0]
        self.assertEqual(wall['p'], [[0, 0, 28], [5, 0, 28.5]])
        self.assertIsNone(wall['height_m'])
        self.assertFalse(wall['physical_alignment_verified'])
        self.assertEqual(self.points[0], [-2795, 1300, 28])

    def test_missing_wrong_type_duplicate_and_nan_rejected(self):
        for records, ids in [([], ['wall1']),
                             ([dict(idx='wall1', name='guardrail', points=self.points)], ['wall1']),
                             (self.dataset.data['object_set'], ['wall1', 'wall1']),
                             ([dict(idx='wall1', name='wall', points=[[float('nan'),0,0], [1,1,1]])], ['wall1'])]:
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                build_static_walls(SimpleNamespace(data={'object_set': records}), self.transform,
                                   {'static_walls': {'source_object_ids': ids}})
