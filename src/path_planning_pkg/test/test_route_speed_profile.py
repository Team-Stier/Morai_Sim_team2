import math
from pathlib import Path
import unittest

from hd_map_pkg.course_speed import CourseSpeedZones, load_course_speed_policy
from hd_map_pkg.lane_rddf import read_route
from path_planning_pkg.route_speed_profile import build_speed_profile


class RouteSpeedProfileTest(unittest.TestCase):
    def setUp(self):
        raw = read_route(Path(__file__).resolve().parents[3]/'참고파일들/2026_molit_comp_global_path (3).txt')
        self.points = [p for i,p in enumerate(raw) if i == 0 or p != raw[i-1]][:-1]
        self.zones = CourseSpeedZones(self.points, load_course_speed_policy())

    def test_100_cruise_exists_only_inside_high_speed_section(self):
        speeds = build_speed_profile(self.points, self.zones, 2, 2, 2, 15)
        self.assertAlmostEqual(max(speeds)*3.6, 100)
        for i, speed in enumerate(speeds):
            self.assertGreater(speed, 0)
            if not self.zones.unlimited(i):
                self.assertLessEqual(speed*3.6, 48.000001)
        self.assertLessEqual(speeds[self.zones.end-1]*3.6, 48.000001)
        self.assertLess(min(speeds)*3.6, 30)  # Curve demand reduces cruise.

    def test_cyclic_deceleration_envelope_reaches_50_zone_before_entry(self):
        speeds = build_speed_profile(self.points, self.zones, 2, 2, 2, 0)
        count = len(speeds)
        for i,p in enumerate(self.points):
            j = (i+1) % count
            ds = math.hypot(self.points[j][0]-p[0], self.points[j][1]-p[1])
            self.assertLessEqual(speeds[i]**2, speeds[j]**2+4*ds+1e-8)
        self.assertLessEqual(speeds[self.zones.end]*3.6, 48)
