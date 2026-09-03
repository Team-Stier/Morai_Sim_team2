#!/usr/bin/env python3

import unittest

from hd_map_pkg.lanelet2_export import Lanelet2Exporter


class EndpointSmoothingTest(unittest.TestCase):
    def test_endpoint_snap_is_tapered_across_short_bound(self):
        exporter = object.__new__(Lanelet2Exporter)
        exporter.config = {"conversion": {"endpoint_snap_taper_m": 10.0}}
        points = [
            [0.0, 0.0, 0.0],
            [2.5, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [7.5, 0.0, 0.0],
            [10.0, 0.0, 0.0],
        ]

        adjusted = exporter._apply_endpoint_anchors(
            points, [0.0, 1.0, 0.0], [10.0, -1.0, 0.0])

        self.assertEqual(adjusted[0], [0.0, 1.0, 0.0])
        self.assertEqual(adjusted[-1], [10.0, -1.0, 0.0])
        self.assertAlmostEqual(adjusted[2][1], 0.0)
        self.assertGreater(adjusted[1][1], 0.0)
        self.assertLess(adjusted[3][1], 0.0)
        self.assertEqual(points[0], [0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
