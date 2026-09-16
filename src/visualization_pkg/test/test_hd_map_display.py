import unittest
from types import SimpleNamespace
from hd_map_pkg.display_geometry import display_layers
from visualization_pkg.hd_map_display import map_markers


class MapDisplayTest(unittest.TestCase):
    def setUp(self):
        self.dataset = SimpleNamespace(global_info={
            'global_coordinate_system': '+proj=utm +zone=52 +datum=WGS84 +units=m',
            'local_origin_in_global': [305390, 4122845, 0]},
            lane_boundaries={'a': {'points': [[-2795, 1300, 28], [-2790, 1300, 28]]}},
            links={})
        self.projection = {'epsg': 32652, 'map_frame': 'map', 'origin_utm_m': [302595, 4124145, 0]}

    def test_same_origin_as_localization_and_display_only_flattening(self):
        layers = display_layers(self.dataset, self.projection)
        self.assertEqual(layers['lane_boundaries'][0][0], [0, 0, 28])
        self.assertEqual(layers['lane_boundaries'][0][-1], [5, 0, 28])
        marker = map_markers(layers, -0.1, 0.1).markers[0]
        self.assertEqual(marker.header.frame_id, self.projection['map_frame'])
        self.assertEqual([(p.x, p.y, p.z) for p in marker.points], [(0, 0, -0.1), (5, 0, -0.1)])
        self.assertEqual(self.dataset.lane_boundaries['a']['points'][0][2], 28)

    def test_wrong_projection_and_nonfinite_geometry_rejected(self):
        self.projection['epsg'] = 32651
        with self.assertRaises(ValueError):
            display_layers(self.dataset, self.projection)
        self.projection['epsg'] = 32652
        self.dataset.lane_boundaries['a']['points'][0][0] = float('nan')
        with self.assertRaises(ValueError):
            display_layers(self.dataset, self.projection)
