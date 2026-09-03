#!/usr/bin/env python3
"""Regression tests for chainage-based Lanelet splitting."""

import json
import pathlib
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


PACKAGE_SOURCE = pathlib.Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SOURCE))

from hd_map_pkg.coordinates import CoordinateTransformer  # noqa: E402
from hd_map_pkg.lanelet2_export import Lanelet2Exporter  # noqa: E402
from hd_map_pkg.mgeo_v3 import MGeoV3Dataset  # noqa: E402
from test_mgeo_pipeline import _fixture_payloads  # noqa: E402


def _tags(element):
    return {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}


class PiecewiseBoundaryTest(unittest.TestCase):
    def test_attribute_change_splits_lanelet_without_collapsing_semantics(self):
        payloads = _fixture_payloads()
        link = dict(payloads["link_set.json"][0])
        link.update({
            "to_node_idx": "N2",
            "points": [[0.0, 0.0, 0.0], [5.0, 0.0, 0.0],
                       [10.0, 0.0, 0.0], [15.0, 0.0, 0.0],
                       [20.0, 0.0, 0.0]],
            "lane_mark_left": ["B-DASHED", "B-DASHED-TAIL", "B-SOLID"],
            "lane_mark_right": ["B-CENTER-1", "B-CENTER-2"],
        })
        payloads["link_set.json"] = [link]
        payloads["traffic_light_set.json"] = []
        payloads["intersection_controller_set.json"] = []
        with tempfile.TemporaryDirectory(prefix="hd-map-piecewise-") as directory:
            root = pathlib.Path(directory)
            for filename, payload in payloads.items():
                (root / filename).write_text(
                    json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
            dataset = MGeoV3Dataset(root)
            transformer = CoordinateTransformer(
                dataset.local_origin_utm, (302595.0, 4124145.0, 0.0))
            config = {
                "coordinates": {"simulator_scene": "test.scene",
                                "simulator_scene_origin_utm": [302595, 4124145, 0]},
                "conversion": {"geometry_simplification_m": 0.0,
                               "node_deduplication_m": 0.001,
                               "default_lane_width_m": 4.0},
                "validation": {"max_successor_endpoint_gap_m": 5.0},
                "lane_boundary": {"ordinary_lane_codes": [503],
                                  "centerline_codes": [501],
                                  "stop_line_codes": [530]},
            }
            osm = root / "piecewise.osm"
            exporter = Lanelet2Exporter(dataset, transformer, config)
            exporter.export(osm)

            xml = ET.parse(str(osm)).getroot()
            ways = {int(value.attrib["id"]): value for value in xml.findall("way")}
            lanelets = sorted(
                (value for value in xml.findall("relation")
                 if _tags(value).get("type") == "lanelet"),
                key=lambda value: int(_tags(value)["mgeo:segment_index"]))
            self.assertEqual(len(lanelets), 2)
            self.assertEqual([_tags(value)["mgeo:segment_count"]
                              for value in lanelets], ["2", "2"])

            left_ways = []
            for lanelet in lanelets:
                left_id = next(
                    int(member.attrib["ref"])
                    for member in lanelet.findall("member")
                    if member.attrib.get("role") == "left")
                left_ways.append(ways[left_id])
            self.assertEqual([_tags(value)["subtype"] for value in left_ways],
                             ["dashed", "solid"])
            self.assertEqual(
                _tags(left_ways[0])["mgeo:source_boundaries"],
                "B-DASHED,B-DASHED-TAIL")
            self.assertEqual(
                _tags(left_ways[1])["mgeo:source_boundaries"], "B-SOLID")
            first_nodes = [int(value.attrib["ref"])
                           for value in left_ways[0].findall("nd")]
            second_nodes = [int(value.attrib["ref"])
                            for value in left_ways[1].findall("nd")]
            self.assertEqual(first_nodes[-1], second_nodes[0])


if __name__ == "__main__":
    unittest.main()
