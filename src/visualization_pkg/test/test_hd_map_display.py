import unittest
import copy
import io
import re
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from hd_map_pkg.display_geometry import display_layers
from visualization_pkg.hd_map_display import map_markers, checkpoint_markers, load_map_markers, stop_line_labels


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


class LaneRddfDisplayTest(unittest.TestCase):
    def test_unlimited_route_and_lane_rddf_have_same_distinct_color(self):
        lines = [[[0, 0, 28], [1, 0, 28]]]
        markers = map_markers(dict(global_route=lines, global_route_unlimited=lines,
                                  lane_rddf_unlimited=lines), -0.1, 0.1).markers
        self.assertEqual(markers[1].color, markers[2].color)
        self.assertNotEqual(markers[0].color, markers[1].color)
        self.assertGreater(markers[1].color.r, markers[1].color.g)
        self.assertEqual(markers[-2].text, 'NO LIMIT | cruise 150 km/h')
        self.assertEqual(markers[-1].text, 'MAX 58 km/h')

    def test_layers_are_separate_lines_and_use_map_without_changing_height(self):
        layers = {'global_route': [[[0,0,28],[1,0,28]]],
                  'lane_rddf': [[[0,3,28],[1,3,28]], [[5,3,28],[6,3,28]]],
                  'lane_change_windows': [[[0,0,28],[0,3,28]]]}
        markers = map_markers(layers, -0.1, 0.1).markers
        self.assertEqual([m.ns for m in markers], list(layers))
        self.assertEqual(len(markers[1].points), 4)  # No artificial connector between runs.
        self.assertGreater(markers[1].color.b, markers[1].color.r)
        self.assertGreater(markers[2].color.r, markers[2].color.b)
        self.assertTrue(all(m.header.frame_id == 'map' for m in markers))
        self.assertEqual(layers['lane_rddf'][0][0][2], 28)


class CheckpointDisplayTest(unittest.TestCase):
    def test_points_and_labels_survive_ros_serialization_without_changing_source(self):
        points = [[-96., -365., 28.5], [70., -366., 28.3], [-132., -428., 28.5]]
        original = copy.deepcopy(points)
        markers = checkpoint_markers(points, -.1, 1.5, 1.5)
        wire = io.BytesIO()
        markers.serialize(wire)
        markers = MarkerArray().deserialize(wire.getvalue()).markers
        self.assertEqual(markers[0].type, Marker.SPHERE_LIST)
        self.assertEqual([(p.x, p.y) for p in markers[0].points], [(p[0], p[1]) for p in points])
        self.assertTrue(all(abs(p.z-.65) < 1e-10 for p in markers[0].points))
        self.assertEqual([m.text for m in markers[1:]], ['CP 1', 'CP 2', 'START / END'])
        self.assertTrue(all(m.header.frame_id == 'map' and m.header.stamp.to_sec() == 0 for m in markers))
        self.assertEqual(len({(m.ns, m.id) for m in markers}), len(markers))
        self.assertEqual(points, original)

    @patch('visualization_pkg.hd_map_display.load_route_display_layers',
           return_value=({}, {'counts': {}, 'bounds': []}))
    @patch('visualization_pkg.hd_map_display.rospy.get_param', side_effect=lambda key, default: default)
    def test_map_loader_displays_central_checkpoints_matching_readme(self, _params, _layers):
        root = Path(__file__).resolve().parents[3]
        central = yaml.safe_load((root/'src/ros_architecture_pkg/config/map/checkpoints.yaml').read_text())
        section = (root/'README.md').read_text().split('## 6. 체크포인트')[1].split('## 7.')[0]
        rows = re.findall(r'^\| (START / END|\d+) \| ([^|]+) \| ([^|]+) \| ([^|]+) \|', section, re.M)
        table = {label: [float(x), float(y), float(z)] for label, x, y, z in rows}
        self.assertEqual(central['points'], [table[str(i)] for i in range(1, 15)]+[table['START / END']])
        markers = load_map_markers().markers
        dots = next(m for m in markers if m.ns == 'checkpoints')
        self.assertEqual([(p.x, p.y) for p in dots.points], [(p[0], p[1]) for p in central['points']])
        self.assertEqual(len(dots.points), 15)
        labels = [m.text for m in markers if m.ns == 'checkpoint_labels']
        self.assertEqual(labels, ['CP %d' % i for i in range(1, 15)]+['START / END'])

    def test_invalid_coordinates_and_sizes_are_rejected(self):
        for points, diameter, height in (([[float('nan'), 0., 0.]], 1.5, 1.5),
                                          ([[0., 0.]], 1.5, 1.5),
                                          ([[0., 0., 0.]], 0., 1.5),
                                          ([[0., 0., 0.]], 1.5, float('inf'))):
            with self.assertRaises(ValueError):
                checkpoint_markers(points, -.1, diameter, height)
