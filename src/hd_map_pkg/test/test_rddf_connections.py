import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from hd_map_pkg.geometry import cumulative_lengths, distance_2d
from hd_map_pkg.lane_rddf import SegmentIndex
from hd_map_pkg.rddf_connections import connect_course_lanes, smooth_connection


class CourseConnectionTest(unittest.TestCase):
    def setUp(self):
        self.route = [[0., 0., 28.], [200., 0., 28.]]
        self.lines = {'reference': self.route, 'a': [[0., 7., 28.], [100., 7., 28.]],
                      'b': [[100., 7., 28.], [200., 0., 28.]]}
        self.index = SegmentIndex({'route': self.route})
        self.dataset = SimpleNamespace(successors={'a': ['b']})
        self.settings = dict(sample_spacing_m=.5, course_corridor_m=15., route_match_m=.8,
                             successor_gap_m=.5, allowed_link_ids=['a', 'b'],
                             course_connections=dict(entry_blend_m=50., exit_blend_m=40.,
                                 start_reference_link='reference', end_reference_link='reference',
                                 chains=[dict(id='connected', link_ids=['a', 'b'])]))
        lane = dict(id='a_0', link_id='a', points=self.lines['a'], route_s=[0., 100.],
                    successors=['b_0'], source_indices=[0, 1])
        lane_b = dict(id='b_0', link_id='b', points=self.lines['b'], route_s=[100., 200.],
                      successors=['global_route'], source_indices=[0, 1])
        self.result = dict(lanes=[lane, lane_b], crossings=[], counts={}, graph=dict(
            longitudinal_connections=[], lane_changes=[dict(
                source_lane='global_route', target_lane='a_0',
                source_s_start=60., source_s_end=90., target_s_start=60., target_s_end=90.,
                boundary_id='dashed', side='left')]))

    def build(self):
        return connect_course_lanes(copy.deepcopy(self.result), self.dataset,
                                    self.lines, self.route, self.index, self.settings)

    def test_endpoints_tangents_spacing_and_original_source(self):
        original = copy.deepcopy(self.lines)
        lane = self.build()['lanes'][0]
        points = lane['points']
        self.assertEqual(points[0], self.route[0])
        self.assertEqual(points[-1], self.route[-1])
        self.assertTrue(all(b > a for a, b in zip(lane['route_s'], lane['route_s'][1:])))
        self.assertLessEqual(max(distance_2d(a, b) for a, b in zip(points, points[1:])), .5001)
        for a, b in [(points[0], points[1]), (points[-2], points[-1])]:
            self.assertLess(abs(math.atan2(b[1]-a[1], b[0]-a[0])), .001)
        self.assertEqual(self.lines, original)
        self.assertEqual(lane['successors'], ['global_route'])

    def test_graph_and_legal_windows_follow_new_lane_arclength(self):
        result = self.build()
        edges = result['graph']['longitudinal_connections']
        self.assertEqual([(e['source_lane'], e['target_lane']) for e in edges],
                         [('global_route', 'connected'), ('connected', 'global_route')])
        window = result['graph']['lane_changes'][0]
        self.assertEqual(window['target_lane'], 'connected')
        self.assertGreater(window['target_s_start'], 60.)
        self.assertAlmostEqual(window['target_s_end']-window['target_s_start'], 30.)
        self.result['graph']['lane_changes'][0]['target_s_start'] = 30.
        self.assertEqual(self.build()['graph']['lane_changes'], [])

    def test_disconnected_excluded_duplicate_and_short_chains_rejected(self):
        self.dataset.successors = {}
        with self.assertRaises(ValueError):
            self.build()
        self.dataset.successors = {'a': ['b']}
        self.settings['allowed_link_ids'] = ['a']
        with self.assertRaises(ValueError):
            self.build()
        self.settings['allowed_link_ids'] = ['a', 'b']
        self.settings['course_connections']['chains'][0]['link_ids'] = ['a', 'a']
        with self.assertRaises(ValueError):
            self.build()
        with self.assertRaises(ValueError):
            smooth_connection(self.route, self.lines['a'], self.index, .5, 80., 45., 15.)


if __name__ == '__main__':
    unittest.main()
