import copy
import pathlib
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src'))
from hd_map_pkg.lane_rddf import build_lane_rddf, read_route, SegmentIndex


class LaneRddfTest(unittest.TestCase):
    def setUp(self):
        self.config = dict(sample_spacing_m=0.5, course_corridor_m=15,
                           minimum_heading_dot=0.8, maximum_lane_width_m=5, maximum_height_difference_m=1.5,
                           boundary_end_clearance_m=3, successor_gap_m=0.5,
                           route_match_m=0.8, route_exclusion_m=0.8, minimum_run_m=2)
        self.route = [[0,0,28], [40,0,28]]
        self.transform = SimpleNamespace(mgeo_to_sim=lambda p: list(p))
        self.data = SimpleNamespace(links={
            'a': dict(points=self.route, lane_mark_left=['b'], can_move_left_lane=True,
                      left_lane_change_dst_link_idx='c'),
            'c': dict(points=[[0,3.5,28],[40,3.5,28]], lane_mark_right=['b'],
                      can_move_right_lane=True, right_lane_change_dst_link_idx='a')},
            lane_boundaries={'b': dict(points=[[0,1.75,28],[40,1.75,28]],
                                      lane_type=[503],lane_shape=['broken'],lane_color=['white'])},
            successors={})

    def build(self):
        return build_lane_rddf(self.data, self.transform, self.route, self.config)

    def test_reachable_lane_excludes_route_preserves_z_and_clearance(self):
        result = self.build()
        self.assertEqual({p['link_id'] for p in result['lanes']}, {'c'})
        self.assertTrue(result['crossings'])
        for crossing in result['crossings']:
            self.assertGreaterEqual(crossing['points'][0][0], 3)
            self.assertLessEqual(crossing['points'][0][0], 37)
        self.assertTrue(all(p[2] == 28 for lane in result['lanes'] for p in lane['points']))

    def test_prohibited_unknown_and_compound_markings_fail_closed(self):
        original = copy.deepcopy(self.data)
        for field, values in [('lane_shape', [['solid'],['solid broken'],['none']]),
                              ('lane_color', [['yellow'], ['blue']]),
                              ('lane_type', [[501], [505], [999]]),
                              ('pass_restr', ['prohibited'])]:
            for value in values:
                self.data = copy.deepcopy(original)
                self.data.lane_boundaries['b'][field] = value
                with self.subTest(field=field,value=value):
                    self.assertFalse(self.build()['lanes'])

    def test_wrong_way_or_missing_return_not_exported(self):
        self.data.links['c']['points'].reverse()
        self.assertFalse(self.build()['lanes'])
        self.data.links['c']['points'].reverse()
        self.data.links['c']['can_move_right_lane'] = False
        self.assertFalse(self.build()['lanes'])

    def test_required_intersection_exclusion_removes_lane_and_crossings(self):
        self.config['excluded_link_ids'] = ['c']
        result = self.build()
        self.assertFalse(result['lanes'])
        self.assertFalse(result['crossings'])
        self.config['excluded_link_ids'] = ['missing']
        with self.assertRaises(ValueError):
            self.build()

    def test_overpass_is_not_a_route_seed_or_alternative(self):
        for p in self.data.links['c']['points']:
            p[2] += 5
        self.assertFalse(self.build()['lanes'])

    def test_disconnected_parallel_lane_not_included(self):
        self.data.links['d'] = dict(points=[[0,7,28],[40,7,28]])
        self.assertEqual({p['link_id'] for p in self.build()['lanes']}, {'c'})

    def test_fragment_solid_transition_not_crossed(self):
        self.data.lane_boundaries['b']['points'][-1][0] = 20
        self.data.lane_boundaries['solid'] = dict(points=[[20,1.75,28],[40,1.75,28]],
                                                lane_type=[503],lane_shape=['solid'],lane_color=['white'])
        self.data.links['a']['lane_mark_left'].append('solid')
        self.data.links['c']['lane_mark_right'].append('solid')
        result = self.build()
        self.assertTrue(result['crossings'])
        self.assertTrue(all(c['points'][0][0] <= 17 for c in result['crossings']))

    def test_bad_route_and_configuration_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory)/'route.txt'
            path.write_text('0 0 0\nnan 0 0\n')
            with self.assertRaises(ValueError):
                read_route(path)
        self.config['sample_spacing_m'] = 0
        with self.assertRaises(ValueError):
            self.build()

    def test_segment_index_projection_not_nearest_vertex(self):
        hit = SegmentIndex({'a': [[0,0,0],[100,0,0]]}).nearest([50,1,0], 2)
        self.assertEqual(hit[:2], (1,50))


if __name__ == '__main__':
    unittest.main()
