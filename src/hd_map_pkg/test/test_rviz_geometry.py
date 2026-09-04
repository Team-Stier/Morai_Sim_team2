#!/usr/bin/env python3

import unittest

from hd_map_pkg.rviz_geometry import boundary_layer


class BoundaryLayerTest(unittest.TestCase):
    def test_dashed_is_not_presented_as_solid(self):
        self.assertEqual(
            boundary_layer({"type": "line_thin", "subtype": "dashed"}),
            "dashed",
        )

    def test_mixed_line_is_distinct(self):
        self.assertEqual(
            boundary_layer({"type": "line_thin", "subtype": "solid_dashed"}),
            "mixed",
        )

    def test_road_border_has_priority(self):
        self.assertEqual(
            boundary_layer({"type": "road_border", "subtype": "solid"}),
            "road_border",
        )


if __name__ == "__main__":
    unittest.main()
