import copy
import math
import unittest
from types import SimpleNamespace

from hd_map_pkg.static_obstacles import build_static_obstacles


class StaticObstaclesTest(unittest.TestCase):
    def setUp(self):
        self.route = [[0., 0., 28.], [100., 0., 28.]]
        self.transform = SimpleNamespace(mgeo_to_sim=lambda p: [p[0]+10, p[1], p[2]])
        self.cfg = {'static_obstacles': dict(kinds=['wall', 'guardrail', 'light_pole', 'sign_pole',
            'concrete_barrier'], route_distance_m=15., maximum_height_difference_m=1.5,
            sample_spacing_m=.25, padding_m=.4)}

    def build(self, rows):
        return build_static_obstacles(SimpleNamespace(data={'object_set': rows}),
                                      self.transform, self.route, self.cfg)

    def test_long_sparse_wall_is_sampled_and_cropped_without_endpoint_loss(self):
        rows = [dict(idx='wall', name='wall', points=[[-100., 3., 28.], [200., 3., 28.]])]
        original = copy.deepcopy(rows)
        result = self.build(rows)
        self.assertEqual(rows, original)
        self.assertEqual(result[0]['id'], 'wall')
        points = result[0]['points']
        self.assertTrue(any(abs(p[0]-50.) < .01 for p in points))
        self.assertLess(max(p[0] for p in points), 116.)
        self.assertGreater(min(p[0] for p in points), -16.)
        self.assertTrue(all(p[2] == 28. for p in points))

    def test_only_fixed_near_route_same_level_source_objects(self):
        rows = [dict(idx=name, name=kind, points=[point]) for name, kind, point in [
            ('pole', 'light_pole', [20., 2., 28.]), ('sign', 'sign_pole', [21., 3., 28.]),
            ('far', 'guardrail', [20., 20., 28.]), ('bridge', 'wall', [20., 0., 35.]),
            ('unknown', 'other_facilities', [20., 0., 28.]),
            ('npc', 'vehicle', [20., 0., 28.])]]
        self.assertEqual({row['id'] for row in self.build(rows)}, {'pole', 'sign'})
        points = self.build(rows)[0]['points']
        self.assertAlmostEqual(min(p[0] for p in points), 29.6)
        self.assertAlmostEqual(max(p[1] for p in points), 2.4)

    def test_invalid_geometry_and_settings_fail(self):
        for value in [math.nan, -1., 0.]:
            self.cfg['static_obstacles']['sample_spacing_m'] = value
            with self.assertRaises(ValueError):
                self.build([])
        self.setUp()
        with self.assertRaises(ValueError):
            self.build([dict(idx='bad', name='wall', points=[[math.nan, 0., 28.]])])
