import math
from pathlib import Path
import unittest

import yaml
from global_route_manager_pkg.progress import RouteProgress


class ComparisonDistanceTest(unittest.TestCase):
    def test_checkpoint_goal_is_preserved_even_inside_high_speed_zone(self):
        lanes = [dict(id='global_route', points=[(0.,0.,0.),(1000.,0.,0.)], route_s=[0.,1000.])]
        config = dict(initialize_from_current_position=True, matching_backward_m=15.,
                      matching_forward_m=120., comparison_distance_m=100.,
                      high_speed_comparison_distance_m=300.)
        route = RouteProgress(lanes, [(600.,0.,0.)], 3., config)
        route.high_speed_interval = (200.,700.)
        self.assertEqual(route.update((250.,0.,0.),0.)['comparison_goal_s'],600.)

    def test_regular_distance_without_high_speed_interval(self):
        lanes = [dict(id='global_route', points=[(0.,0.,0.),(1000.,0.,0.)], route_s=[0.,1000.])]
        config = dict(rddf_geometry_only=True, matching_backward_m=15., matching_forward_m=120.,
                      comparison_distance_m=100., high_speed_comparison_distance_m=300.)
        route = RouteProgress(lanes, [], 0., config)
        self.assertEqual(route.update((250.,0.,0.),0.)['comparison_goal_s'],350.)


class AmendedCheckpointProgressTest(unittest.TestCase):
    def test_old_cp11_does_not_pass_new_cp11_and_new_position_advances_to_cp12(self):
        root = Path(__file__).resolve().parents[3]
        central = yaml.safe_load((root / 'src/ros_architecture_pkg/config/map/checkpoints.yaml').read_text())
        raw = [tuple(map(float, line.split())) for line in
               (root / '참고파일들/2026_molit_comp_global_path (3).txt').read_text().splitlines()
               if line.strip()]
        points = [p for i, p in enumerate(raw) if i == 0 or p != raw[i-1]]
        lanes = [dict(id='global_route', points=points)]
        config = dict(initialize_from_current_position=True, matching_backward_m=15.,
                      matching_forward_m=120., comparison_distance_m=100.)
        route = RouteProgress(lanes, central['points'], central['radius_m'], config)
        old = (75.414203703, -217.855053961, 28.382283641)
        new = central['points'][10]
        yaw = math.atan2(new[1]-old[1], new[0]-old[0])
        before = route.update(old, yaw)
        self.assertEqual(before['next_checkpoint'], 10)  # Public indices are zero based.
        self.assertEqual(before['comparison_goal'], new)
        self.assertNotIn(10, route.passed_checkpoints)
        after = route.update(new, yaw)
        self.assertEqual(after['next_checkpoint'], 11)
        self.assertEqual(after['comparison_goal'], central['points'][11])
        self.assertFalse(route.missed_checkpoint)
