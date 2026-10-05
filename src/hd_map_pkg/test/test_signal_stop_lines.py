import copy
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from hd_map_pkg.stop_lines import build_signal_stop_lines, match_stop_lines


class SignalStopLineTest(unittest.TestCase):
    def setUp(self):
        def line(end, y=0., reverse=False):
            points = [[float(x), y, 28.] for x in range(0, end+1, 5)]
            return dict(points=points[::-1] if reverse else points)

        def bar(x, code=530):
            return dict(idx='bar-%g' % x, points=[[x, -1., 28.1], [x, 5., 28.2]], lane_type=[code])

        self.links = dict(incoming=line(100), parallel=line(100, 3.5),
                          opposing=line(100, reverse=True), pedestrian=line(60))
        associations = dict(car_a=['incoming', 'parallel'], car_b=['incoming'],
                            opposite=['opposing'], pedestrian=['pedestrian'])
        self.dataset = SimpleNamespace(links=self.links,
            lane_boundaries=dict(stop=bar(90.), reverse_stop=bar(10.),
                                 pedestrian_stop=bar(55.), other=bar(180.), stripe=bar(95., 503)),
            traffic_lights={key: dict(type='pedestrian' if key == 'pedestrian' else 'car')
                            for key in associations}, traffic_light_link_ids=lambda: associations)
        self.transform = SimpleNamespace(mgeo_to_sim=lambda p: list(p))
        self.route = [[0., 0., 28.], [100., 0., 28.]]
        self.config = dict(conversion=dict(stop_line_search_radius_m=40.,
            stop_line_intersection_tolerance_m=.75, geometry_simplification_m=.02),
            lane_boundary=dict(stop_line_codes=[530]), lane_rddf=dict(
                allowed_link_ids=['parallel'], excluded_link_ids=[],
                route_match_m=.8, minimum_heading_dot=.8))

    def test_course_signals_share_original_stop_bar_without_opposing_or_pedestrian(self):
        original = copy.deepcopy(self.dataset.lane_boundaries)
        rows = build_signal_stop_lines(self.dataset, self.transform, self.route, self.config)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['id'], 'stop')
        self.assertEqual(rows[0]['points'], original['stop']['points'])
        self.assertEqual(rows[0]['approach_link_ids'], ['incoming', 'parallel'])
        self.assertEqual(rows[0]['signal_ids'], ['car_a', 'car_b'])
        self.assertEqual(self.dataset.lane_boundaries, original)

    def test_excluded_approach_and_unmatched_stop_line_are_not_fabricated(self):
        self.config['lane_rddf']['excluded_link_ids'] = ['incoming', 'parallel']
        self.assertEqual(build_signal_stop_lines(self.dataset, self.transform, self.route, self.config), [])
        self.dataset.lane_boundaries = {'other': self.dataset.lane_boundaries['other']}
        self.config['lane_rddf']['excluded_link_ids'] = []
        self.assertEqual(build_signal_stop_lines(self.dataset, self.transform, self.route, self.config), [])

    def test_matching_cache_and_downstream_radius_keep_exporter_behavior(self):
        bars = {'stop': self.dataset.lane_boundaries['stop']['points'],
                'far': [[30., -1., 28.], [30., 1., 28.]]}
        cache = {}
        first = match_stop_lines(['incoming'], self.links, bars, 40., .75, cache)
        self.assertEqual(first[0][0], 'stop')
        self.assertEqual(first, match_stop_lines(['incoming'], self.links, bars, 40., .75, cache))
        self.assertEqual(match_stop_lines(['incoming'], self.links, {'far': bars['far']}, 40., .75, {}), [])


if __name__ == '__main__':
    unittest.main()
