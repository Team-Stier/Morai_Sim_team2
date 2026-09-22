import unittest
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
