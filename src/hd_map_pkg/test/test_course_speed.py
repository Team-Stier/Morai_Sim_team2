import hashlib
from pathlib import Path
import unittest

from hd_map_pkg.course_speed import CourseSpeedZones, load_course_speed_policy
from hd_map_pkg.lane_rddf import read_route


class CourseSpeedTest(unittest.TestCase):
    def test_fixed_policy_matches_original_route_and_boundary_coordinates(self):
        root = Path(__file__).resolve().parents[3]
        source = root/'참고파일들/2026_molit_comp_global_path (3).txt'
        policy = load_course_speed_policy()
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), policy['reference']['sha256'])
        zones = CourseSpeedZones(read_route(source), policy)
        self.assertEqual((zones.start, zones.end), (2275, 3529))
        self.assertEqual(policy['normal_limit_kph'], 58)
        self.assertEqual(policy['high_speed']['cruise_kph'], 150)
        self.assertIsNone(policy['high_speed']['limit_kph'])
        for i in (zones.start-1, zones.end, 0, len(zones.route)-1):
            self.assertEqual(zones.limit_kph(i), 58)
        for i in (zones.start, zones.end-1):
            self.assertIsNone(zones.limit_kph(i))

    def test_wraparound_and_split_preserve_all_edges_and_heights(self):
        route = [[i, 0, 28] for i in range(10)]
        policy = {'normal_limit_kph': 50, 'high_speed': {'start_map_xy': [8, 0], 'end_map_xy': [2, 0]}}
        zones = CourseSpeedZones(route, policy)
        self.assertEqual([i for i in range(10) if zones.unlimited(i)], [0, 1, 8, 9])
        sections = zones.split_line(route, list(range(10)))
        self.assertEqual([s['speed_limit_kph'] for s in sections], [None, 50, None])
        self.assertEqual(sum(len(s['p'])-1 for s in sections), 9)
        self.assertTrue(all(p[2] == 28 for s in sections for p in s['p']))
        result = zones.annotate_lanes({'lanes': [{'points': [[i, 3.5, 28] for i in range(10)]}]})
        self.assertEqual(result['lanes'][0]['speed_limit_kph'], [None, None]+[50]*6+[None, None])
