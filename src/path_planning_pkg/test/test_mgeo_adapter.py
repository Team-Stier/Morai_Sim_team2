#!/usr/bin/env python3

import unittest
from pathlib import Path

from hd_map_pkg.coordinates import CoordinateTransformer
from hd_map_pkg.mgeo_v3 import MGeoV3Dataset
from path_planning_pkg import (
    BoundaryMarking,
    CorridorMode,
    CorridorPolicy,
    CorridorPolicyInput,
)
from path_planning_pkg.mgeo_adapter import (
    LaneChangePair,
    MGeoPlannerMap,
    parse_lane_change_pairs,
)


MAP_CONFIG = {
    "conversion": {
        "geometry_simplification_m": 0.0,
        "default_lane_width_m": 3.5,
        "node_deduplication_m": 0.001,
        "boundary_event_merge_tolerance_m": 0.50,
        "boundary_stitch_tolerance_m": 0.50,
        "minimum_lanelet_segment_length_m": 0.50,
        "endpoint_snap_taper_m": 10.0,
    },
    "validation": {
        "max_boundary_to_center_distance_m": 30.0,
        "max_successor_endpoint_gap_m": 5.0,
    },
    "lane_boundary": {
        "stop_line_codes": [530],
        "road_border_codes": [505, 531],
        "centerline_codes": [501],
        "thick_line_codes": [502],
        "ordinary_lane_codes": [503, 504, 506, 515, 525],
        "standalone_marking_codes": [535],
    },
}


class KATRIAdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        package = Path(__file__).resolve().parents[1]
        source = package.parent / "hd_map_pkg" / "vendor" / "verdict_sdk" / "map-data" / "KATRI"
        dataset = MGeoV3Dataset(source)
        transformer = CoordinateTransformer(
            dataset.local_origin_utm, (302595.0, 4124145.0, 0.0)
        )
        cls.map = MGeoPlannerMap(dataset, transformer, MAP_CONFIG)

    @staticmethod
    def pair(current, adjacent, boundary):
        return LaneChangePair(current, adjacent, "left", boundary, "A2256W000153", {})

    def test_route_turn_opens_only_verified_longitudinal_seams(self):
        build = self.map.build_route_corridor(
            ("A2256W000315", "A2256W000318", "A2256W000308"),
            "A2256W000318",
        )
        decision = CorridorPolicy().resolve(
            build.corridor,
            CorridorPolicyInput(
                mode=CorridorMode.TURN_CONNECTOR,
                requested_open_boundary_ids=frozenset(build.open_boundary_ids),
                turn_connector_verified=True,
            ),
        )
        self.assertTrue(decision.opened_boundary_ids)
        self.assertTrue(
            all("route:seam:" in value for value in decision.opened_boundary_ids)
        )

    def test_pure_dashed_highway_pairs_are_map_verified(self):
        for pair in (
            self.pair("A2256W000420", "A2256W000430", "B2256W000034"),
            self.pair("A2256W000408", "A2256W000434", "B2256W000044"),
        ):
            build = self.map.build_lane_change_corridor(pair)
            self.assertTrue(build.topology_verified)
            self.assertEqual(build.shared_marking, BoundaryMarking.DASHED)

    def test_mixed_solid_boundary_is_never_pure_dashed(self):
        build = self.map.build_lane_change_corridor(
            self.pair("A2256W000411", "A2256W000409", "B2256W000038")
        )
        self.assertEqual(build.shared_marking, BoundaryMarking.SOLID)

    def test_synthetic_shared_boundary_fails_closed(self):
        build = self.map.build_lane_change_corridor(
            self.pair("A2256W000445", "A2256W000422", "")
        )
        self.assertFalse(build.topology_verified)
        self.assertEqual(build.shared_marking, BoundaryMarking.VIRTUAL)

    def test_branching_successor_does_not_collapse_route_lane_width(self):
        lane = self.map.lane("A2256W000220")
        self.assertGreater(
            min(
                ((left.x - right.x) ** 2 + (left.y - right.y) ** 2) ** 0.5
                for left, right in (
                    (lane.left[0], lane.right[0]),
                    (lane.left[-1], lane.right[-1]),
                )
            ),
            3.0,
        )

    def test_lane_change_config_requires_explicit_boolean_opt_in(self):
        disabled = {
            "A": {
                "adjacent_link_id": "B",
                "direction": "left",
                "merge_route_link_id": "C",
            }
        }
        self.assertEqual(parse_lane_change_pairs(disabled), {})
        disabled["A"]["enabled"] = "false"
        with self.assertRaises(TypeError):
            parse_lane_change_pairs(disabled)
        disabled["A"]["enabled"] = True
        self.assertIn("A", parse_lane_change_pairs(disabled))


if __name__ == "__main__":
    unittest.main()
