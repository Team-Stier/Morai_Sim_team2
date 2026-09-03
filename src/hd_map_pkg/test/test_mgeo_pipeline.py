#!/usr/bin/env python3
"""End-to-end tests for the dependency-free MGeo 3.0 conversion pipeline."""

import copy
import json
import pathlib
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from collections import defaultdict


PACKAGE_SOURCE = pathlib.Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SOURCE))

from hd_map_pkg.coordinates import CoordinateTransformer  # noqa: E402
from hd_map_pkg.lanelet2_export import Lanelet2Exporter  # noqa: E402
from hd_map_pkg.mgeo_v3 import DATASET_FILES, MGeoV3Dataset  # noqa: E402
from hd_map_pkg.validation import validate_osm, validate_source  # noqa: E402


def _boundary(identifier, start_node, end_node, points, lane_type, shape, color):
    return {
        "idx": identifier,
        "from_node_idx": start_node,
        "to_node_idx": end_node,
        "points": points,
        "lane_type": [lane_type],
        "lane_shape": [shape],
        "lane_color": [color],
    }


def _link(identifier, start_node, end_node, points, left, right, speed, turn):
    return {
        "idx": identifier,
        "from_node_idx": start_node,
        "to_node_idx": end_node,
        "points": points,
        "lane_mark_left": list(left) if isinstance(left, (list, tuple)) else [left],
        "lane_mark_right": list(right) if isinstance(right, (list, tuple)) else [right],
        "max_speed": speed,
        "related_signal": turn,
        "road_id": "ROAD-1",
        "ego_lane": 1,
        "width_start": 4.0,
        "width_end": 4.0,
        "left_lane_change_dst_link_idx": None,
        "right_lane_change_dst_link_idx": None,
        "can_move_left_lane": False,
        "can_move_right_lane": False,
    }


def _fixture_payloads():
    payloads = {filename: [] for filename in DATASET_FILES}
    payloads["global_info.json"] = {
        "maj_ver": 3,
        "min_ver": 0,
        "global_coordinate_system": (
            "+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs"
        ),
        "local_origin_in_global": [305390.0, 4122845.0, 0.0],
        "mgeo_file_hash": {},
    }
    payloads["node_set.json"] = [
        {"idx": "N0", "point": [0.0, 0.0, 0.0]},
        {"idx": "N1", "point": [10.0, 0.0, 0.0]},
        {"idx": "N2", "point": [20.0, 0.0, 0.0]},
    ]
    payloads["lane_node_set.json"] = [
        {"idx": "LEFT-0", "point": [0.0, 2.0, 0.0]},
        {"idx": "LEFT-MID", "point": [5.0, 2.0, 0.0]},
        {"idx": "LEFT-1", "point": [10.0, 2.0, 0.0]},
        {"idx": "LEFT-2", "point": [20.0, 2.0, 0.0]},
        {"idx": "RIGHT-0", "point": [0.0, -2.0, 0.0]},
        {"idx": "RIGHT-1", "point": [10.0, -2.0, 0.0]},
        {"idx": "RIGHT-2", "point": [20.0, -2.0, 0.0]},
        {"idx": "STOP-LEFT", "point": [10.0, 2.0, 0.0]},
        {"idx": "STOP-RIGHT", "point": [10.0, -2.0, 0.0]},
    ]
    payloads["lane_boundary_set.json"] = [
        _boundary(
            "B-DASHED",
            "LEFT-0",
            "LEFT-MID",
            [[0.0, 2.0, 0.0], [5.0, 2.0, 0.0]],
            503,
            "broken",
            "white",
        ),
        _boundary(
            "B-DASHED-TAIL",
            "LEFT-MID",
            "LEFT-1",
            [[5.0, 2.0, 0.0], [10.0, 2.0, 0.0]],
            503,
            "broken",
            "white",
        ),
        _boundary(
            "B-SOLID",
            "LEFT-1",
            "LEFT-2",
            [[10.0, 2.0, 0.0], [20.0, 2.0, 0.0]],
            503,
            "solid",
            "white",
        ),
        _boundary(
            "B-CENTER-1",
            "RIGHT-0",
            "RIGHT-1",
            [[0.0, -2.0, 0.0], [10.0, -2.0, 0.0]],
            501,
            "solid",
            "yellow",
        ),
        _boundary(
            "B-CENTER-2",
            "RIGHT-1",
            "RIGHT-2",
            [[10.0, -2.0, 0.0], [20.0, -2.0, 0.0]],
            501,
            "solid",
            "yellow",
        ),
        _boundary(
            "B-STOP",
            "STOP-LEFT",
            "STOP-RIGHT",
            [[10.0, 2.0, 0.0], [10.0, -2.0, 0.0]],
            530,
            "solid",
            "white",
        ),
    ]
    payloads["link_set.json"] = [
        _link(
            "L1",
            "N0",
            "N1",
            [[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
            ["B-DASHED", "B-DASHED-TAIL"],
            "B-CENTER-1",
            30,
            "straight",
        ),
        _link(
            "L2",
            "N1",
            "N2",
            [[10.0, 0.0, 0.0], [15.0, 0.0, 0.0], [20.0, 0.0, 0.0]],
            "B-SOLID",
            "B-CENTER-2",
            50,
            "left_unprotected",
        ),
    ]
    payloads["road_set.json"] = [
        {"idx": "ROAD-1", "links": ["L1", "L2"], "ref_lines": ["L1"]}
    ]
    payloads["traffic_light_set.json"] = [
        {
            "idx": "TL1",
            "link_id_list": ["L2"],
            "point": [20.0, 0.0, 5.0],
            "heading": 0.0,
            "width": 1.2,
            "height": 0.4,
            "type": "car",
            "sub_type": [[5, 0], [4, 0]],
            "orientation": "horizontal",
        }
    ]
    payloads["intersection_controller_set.json"] = [
        {"idx": "CTRL1", "TL": [["TL1"]]}
    ]
    payloads["singlecrosswalk_set.json"] = [
        {
            "idx": "CW-PED",
            "points": [[3.0, -2.0, 0.0], [4.0, -2.0, 0.0],
                       [4.0, 2.0, 0.0], [3.0, 2.0, 0.0]],
            "sign_type": "5321",
            "link_id_list": ["L1"],
        },
        {
            "idx": "CW-RAISED",
            "points": [[12.0, -2.0, 0.0], [13.0, -2.0, 0.0],
                       [13.0, 2.0, 0.0], [12.0, 2.0, 0.0]],
            "sign_type": "533",
            "link_id_list": ["L2"],
        },
        {
            "idx": "CW-BICYCLE",
            "points": [[16.0, -2.0, 0.0], [17.0, -2.0, 0.0],
                       [17.0, 2.0, 0.0], [16.0, 2.0, 0.0]],
            "sign_type": "534",
            "link_id_list": ["L2"],
        },
    ]
    payloads["surface_marking_set.json"] = [
        {
            "idx": "SM-STRAIGHT",
            "points": [[1.0, -0.5, 0.0], [2.0, -0.5, 0.0],
                       [2.0, 0.5, 0.0], [1.0, 0.5, 0.0]],
            "sub_type": "5371",
            "link_id_list": ["L1"],
        },
        {
            "idx": "SM-SPEED-BUMP",
            "points": [[14.0, -2.0, 0.0], [15.0, -2.0, 0.0],
                       [15.0, 2.0, 0.0], [14.0, 2.0, 0.0]],
            "sub_type": "speedbump",
            "link_id_list": ["L2"],
        },
    ]
    return payloads


def _reference_fixture_payloads(use_signal_alias=False):
    payloads = _fixture_payloads()
    signal_id = "TL1_0" if use_signal_alias else "TL1"
    payloads["node_set.json"][2]["traffic_light_id"] = signal_id
    payloads["traffic_light_set.json"][0]["ref_crosswalk_id"] = "CW1"
    if use_signal_alias:
        alias = copy.deepcopy(payloads["traffic_light_set.json"][0])
        alias["idx"] = signal_id
        payloads["traffic_light_set.json"].append(alias)
    payloads["singlecrosswalk_set.json"] = [
        {
            "idx": "SCW1",
            "points": [[18.0, -2.0, 0.0], [20.0, -2.0, 0.0],
                       [20.0, 2.0, 0.0], [18.0, 2.0, 0.0]],
            "sign_type": "5321",
            "link_id_list": ["L2"],
        }
    ]
    payloads["crosswalk_set.json"] = [
        {
            "idx": "CW1",
            "single_crosswalk_list": ["SCW1"],
            "ref_traffic_light_list": [signal_id],
        }
    ]
    payloads["synced_traffic_light_set.json"] = [
        {
            "idx": "SYNC1",
            "intersection_controller_id": "CTRL1",
            "link_id_list": ["L2"],
            "signal_id_list": [signal_id],
        }
    ]
    payloads["intersection_controller_set.json"] = [
        {"idx": "CTRL1", "TL": [[signal_id]]}
    ]
    payloads["intersection_controller_data.json"] = [
        {
            "idx": "CTRL1",
            "synced_light": [[signal_id]],
            "phase": [{"id": 0, "duration": 10, "state": {signal_id: []}}],
        }
    ]
    payloads["junction_set.json"] = [
        {"idx": "J1", "road_id_list": ["ROAD-1"]}
    ]
    return payloads


def _write_fixture(root, payloads=None):
    for filename, payload in (payloads or _fixture_payloads()).items():
        (root / filename).write_text(
            json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8",
        )


def _tags(element):
    return {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}


class MGeoPipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._temporary = tempfile.TemporaryDirectory(prefix="hd-map-pipeline-")
        cls.root = pathlib.Path(cls._temporary.name)
        cls.source = cls.root / "mgeo"
        cls.source.mkdir()
        _write_fixture(cls.source)

        cls.dataset = MGeoV3Dataset(cls.source)
        cls.transformer = CoordinateTransformer(
            cls.dataset.local_origin_utm,
            (302595.0, 4124145.0, 0.0),
        )
        cls.config = {
            "coordinates": {
                "simulator_scene": "unit-test.scene",
                "simulator_scene_origin_utm": [302595.0, 4124145.0, 0.0],
            },
            "conversion": {
                "geometry_simplification_m": 0.0,
                "node_deduplication_m": 0.001,
                "default_lane_width_m": 4.0,
                "traffic_light_bar_width_m": 0.6,
                "stop_line_search_radius_m": 5.0,
                "osm_generator": "hd_map_pkg/test",
            },
            "lane_boundary": {
                "ordinary_lane_codes": [503],
                "centerline_codes": [501],
                "road_border_codes": [531],
                "stop_line_codes": [530],
            },
        }
        cls.osm_path = cls.root / "fixture.osm"
        cls.routing_path = cls.root / "routing.json"
        cls.exporter = Lanelet2Exporter(cls.dataset, cls.transformer, cls.config)
        cls.exporter.export(cls.osm_path, cls.routing_path)

        cls.xml_root = ET.parse(str(cls.osm_path)).getroot()
        cls.nodes = {
            int(element.attrib["id"]): element
            for element in cls.xml_root.findall("node")
        }
        cls.ways = {
            int(element.attrib["id"]): element
            for element in cls.xml_root.findall("way")
        }
        cls.relations = {
            int(element.attrib["id"]): element
            for element in cls.xml_root.findall("relation")
        }
        cls.boundary_ways = {
            _tags(element).get("mgeo:id"): element
            for element in cls.ways.values()
            if _tags(element).get("mgeo:boundary_category")
        }
        cls.lanelets = defaultdict(list)
        for element in cls.relations.values():
            tags = _tags(element)
            if tags.get("type") == "lanelet":
                cls.lanelets[tags.get("mgeo:id")].append(element)
        for values in cls.lanelets.values():
            values.sort(key=lambda element: int(
                _tags(element).get("mgeo:segment_index", 0)))

    @classmethod
    def tearDownClass(cls):
        cls._temporary.cleanup()

    def test_mgeo_importer_builds_successor_and_signal_indexes(self):
        self.assertEqual(self.dataset.counts()["links"], 2)
        self.assertEqual(self.dataset.reference_errors(), [])
        self.assertEqual(self.dataset.successors["L1"], ["L2"])
        self.assertEqual(self.dataset.predecessors["L2"], ["L1"])
        # Direct MGeo signal links are connector links.  Their incoming
        # predecessor is the controlled approach lanelet.
        self.assertEqual(self.dataset.traffic_light_link_ids(), {"TL1": ["L1"]})

    def test_reference_errors_cover_exporter_foreign_keys(self):
        def errors_after(mutate):
            with tempfile.TemporaryDirectory(prefix="hd-map-reference-") as directory:
                root = pathlib.Path(directory)
                payloads = _reference_fixture_payloads()
                mutate(payloads)
                _write_fixture(root, payloads)
                return MGeoV3Dataset(root).reference_errors()

        cases = (
            (
                "node N2: missing traffic_light TL-MISSING",
                lambda data: data["node_set.json"][2].__setitem__(
                    "traffic_light_id", "TL-MISSING"),
            ),
            (
                "traffic light TL1: missing link L-MISSING",
                lambda data: data["traffic_light_set.json"][0].__setitem__(
                    "link_id_list", ["L-MISSING"]),
            ),
            (
                "traffic light TL1: missing ref_crosswalk CW-MISSING",
                lambda data: data["traffic_light_set.json"][0].__setitem__(
                    "ref_crosswalk_id", "CW-MISSING"),
            ),
            (
                "single crosswalk SCW1: missing link L-MISSING",
                lambda data: data["singlecrosswalk_set.json"][0].__setitem__(
                    "link_id_list", ["L-MISSING"]),
            ),
            (
                "synced traffic light SYNC1: missing link L-MISSING",
                lambda data: data["synced_traffic_light_set.json"][0].__setitem__(
                    "link_id_list", ["L-MISSING"]),
            ),
            (
                "synced traffic light SYNC1: missing traffic_light TL-MISSING",
                lambda data: data["synced_traffic_light_set.json"][0].__setitem__(
                    "signal_id_list", ["TL-MISSING"]),
            ),
            (
                "synced traffic light SYNC1: missing controller CTRL-MISSING",
                lambda data: data["synced_traffic_light_set.json"][0].__setitem__(
                    "intersection_controller_id", "CTRL-MISSING"),
            ),
            (
                "controller CTRL1: missing traffic_light TL-MISSING",
                lambda data: data["intersection_controller_set.json"][0].__setitem__(
                    "TL", [["TL-MISSING"]]),
            ),
            (
                "controller data CTRL-MISSING: missing controller CTRL-MISSING",
                lambda data: data["intersection_controller_data.json"][0].__setitem__(
                    "idx", "CTRL-MISSING"),
            ),
            (
                "controller data CTRL1: missing synced traffic_light TL-MISSING",
                lambda data: data["intersection_controller_data.json"][0].__setitem__(
                    "synced_light", [["TL-MISSING"]]),
            ),
            (
                "controller data CTRL1: phase 0 missing traffic_light TL-MISSING",
                lambda data: data["intersection_controller_data.json"][0]["phase"][0]
                .__setitem__("state", {"TL-MISSING": []}),
            ),
            (
                "junction J1: missing road ROAD-MISSING",
                lambda data: data["junction_set.json"][0].__setitem__(
                    "road_id_list", ["ROAD-MISSING"]),
            ),
            (
                "road ROAD-1: missing link L-MISSING",
                lambda data: data["road_set.json"][0].__setitem__(
                    "links", ["L-MISSING"]),
            ),
            (
                "road ROAD-1: missing ref_line L-MISSING",
                lambda data: data["road_set.json"][0].__setitem__(
                    "ref_lines", ["L-MISSING"]),
            ),
            (
                "link L1: missing road ROAD-MISSING",
                lambda data: data["link_set.json"][0].__setitem__(
                    "road_id", "ROAD-MISSING"),
            ),
        )
        for expected, mutate in cases:
            with self.subTest(expected=expected):
                self.assertEqual(errors_after(mutate), [expected])

    def test_reference_errors_resolve_verified_suffix_aliases_first(self):
        with tempfile.TemporaryDirectory(prefix="hd-map-reference-alias-") as directory:
            root = pathlib.Path(directory)
            payloads = _reference_fixture_payloads(use_signal_alias=True)
            _write_fixture(root, payloads)
            dataset = MGeoV3Dataset(root)

        self.assertEqual(dataset.canonical_id("TL1_0"), "TL1")
        self.assertEqual(dataset.nodes["N2"]["traffic_light_id"], "TL1")
        self.assertEqual(dataset.controller_data["CTRL1"]["phase"][0]["state"],
                         {"TL1": []})
        self.assertEqual(dataset.reference_errors(), [])

    def test_boundary_line_semantics_are_exported(self):
        dashed = _tags(self.boundary_ways["B-DASHED"])
        self.assertEqual(dashed["type"], "line_thin")
        self.assertEqual(dashed["subtype"], "dashed")
        self.assertEqual(dashed["color"], "white")
        self.assertEqual(dashed["lane_change"], "yes")

        solid = _tags(self.boundary_ways["B-SOLID"])
        self.assertEqual(solid["type"], "line_thin")
        self.assertEqual(solid["subtype"], "solid")
        self.assertEqual(solid["lane_change"], "no")

        centerline = _tags(self.boundary_ways["B-CENTER-1"])
        self.assertEqual(centerline["type"], "line_thick")
        self.assertEqual(centerline["subtype"], "solid")
        self.assertEqual(centerline["color"], "yellow")
        self.assertEqual(centerline["mgeo:boundary_category"], "centerline")

        stop_line = _tags(self.boundary_ways["B-STOP"])
        self.assertEqual(stop_line["type"], "stop_line")
        self.assertEqual(stop_line["subtype"], "solid")
        self.assertEqual(stop_line["lane_change"], "no")

    def test_surface_markings_have_one_area_and_linked_lanelet_segment(self):
        expected = {
            "SM-STRAIGHT": ("arrow", "L1"),
            "SM-SPEED-BUMP": ("speed_bump", "L2"),
        }
        for marking_id, (subtype, link_id) in expected.items():
            areas = [
                relation for relation in self.relations.values()
                if _tags(relation).get("type") == "multipolygon"
                and _tags(relation).get("mgeo:id") == marking_id
            ]
            self.assertEqual(len(areas), 1)
            area = areas[0]
            area_tags = _tags(area)
            self.assertEqual(area_tags["area"], "yes")
            self.assertEqual(area_tags["subtype"], subtype)
            self.assertEqual(area_tags["mgeo:link_ids"], link_id)
            outer_ref = next(
                int(member.attrib["ref"])
                for member in area.findall("member")
                if member.attrib["type"] == "way"
                and member.attrib["role"] == "outer"
            )
            outer_tags = _tags(self.ways[outer_ref])
            # Area-relation outer members must remain Lanelet2 LineStrings.
            # area=yes would make official IO classify the way as Polygon.
            self.assertNotIn("area", outer_tags)
            self.assertEqual(outer_tags["type"], "road_marking")
            self.assertEqual(outer_tags["subtype"], subtype)
            self.assertEqual(outer_tags["mgeo:id"], marking_id)
            associated_segments = [
                lanelet for lanelet in self.lanelets[link_id]
                if marking_id in _tags(lanelet).get(
                    "mgeo:surface_markings", "").split(",")
            ]
            self.assertTrue(associated_segments)

    def test_crosswalk_subtype_and_participant_semantics(self):
        expected = {
            "CW-PED": ("crosswalk", "pedestrian", False),
            "CW-RAISED": ("crosswalk", "pedestrian", True),
            "CW-BICYCLE": ("bicycle_crossing", "bicycle", False),
        }
        for crossing_id, (subtype, participant, raised) in expected.items():
            areas = [
                relation for relation in self.relations.values()
                if _tags(relation).get("type") == "multipolygon"
                and _tags(relation).get("mgeo:id") == crossing_id
            ]
            self.assertEqual(len(areas), 1)
            tags = _tags(areas[0])
            self.assertEqual(tags["subtype"], subtype)
            self.assertEqual(tags["one_way"], "no")
            self.assertEqual(tags["participant:{}".format(participant)], "yes")
            excluded = "bicycle" if participant == "pedestrian" else "pedestrian"
            self.assertNotEqual(tags.get("participant:{}".format(excluded)), "yes")
            self.assertEqual(tags.get("mgeo:raised") == "yes", raised)

    def test_blank_crossing_and_surface_link_references_are_warnings(self):
        with tempfile.TemporaryDirectory(prefix="hd-map-blank-link-") as directory:
            root = pathlib.Path(directory)
            payloads = _fixture_payloads()
            payloads["singlecrosswalk_set.json"][0]["link_id_list"].append("")
            payloads["surface_marking_set.json"][0]["link_id_list"].append("   ")
            _write_fixture(root, payloads)
            dataset = MGeoV3Dataset(root)
            checks = {check["name"]: check for check in validate_source(dataset)}

        warning = checks["mgeo_blank_link_references"]
        self.assertEqual(warning["status"], "warning")
        self.assertEqual(warning["metrics"]["blank_references"], 2)
        self.assertEqual(warning["metrics"]["affected_records"], 2)
        self.assertTrue(any("CW-PED" in sample for sample in warning["samples"]))
        self.assertTrue(any("SM-STRAIGHT" in sample for sample in warning["samples"]))

    def test_degenerate_source_boundary_is_omitted_with_a_warning(self):
        with tempfile.TemporaryDirectory(prefix="hd-map-degenerate-boundary-") as directory:
            root = pathlib.Path(directory)
            source = root / "mgeo"
            source.mkdir()
            payloads = _fixture_payloads()
            payloads["lane_node_set.json"].extend([
                {"idx": "DEGENERATE-START", "point": [30.0, 30.0, 0.0]},
                {"idx": "DEGENERATE-END", "point": [30.0, 30.0, 0.0]},
            ])
            payloads["lane_boundary_set.json"].append(_boundary(
                "B-DEGENERATE",
                "DEGENERATE-START",
                "DEGENERATE-END",
                [[30.0, 30.0, 0.0], [30.0, 30.0, 0.0]],
                503,
                "solid",
                "white",
            ))
            _write_fixture(source, payloads)
            dataset = MGeoV3Dataset(source)
            transformer = CoordinateTransformer(
                dataset.local_origin_utm, (302595.0, 4124145.0, 0.0)
            )
            osm_path = root / "fixture.osm"
            routing_path = root / "routing.json"
            Lanelet2Exporter(dataset, transformer, self.config).export(
                osm_path, routing_path
            )
            checks = {check["name"]: check for check in validate_osm(
                osm_path, dataset, routing_path, self.config)}

        self.assertEqual(checks["boundary_attribute_coverage"]["status"], "pass")
        omission = checks["degenerate_source_boundaries"]
        self.assertEqual(omission["status"], "warning")
        self.assertEqual(omission["metrics"]["degenerate_source_boundaries"], 1)
        self.assertEqual(omission["samples"], ["B-DEGENERATE"])

    def test_lanelet_members_speed_and_turn_direction(self):
        expected_sides = {
            "L1": ({"B-DASHED", "B-DASHED-TAIL"},
                   {"B-CENTER-1"}, "30 km/h", "straight"),
            "L2": ({"B-SOLID"}, {"B-CENTER-2"}, "50 km/h", "left"),
        }
        for link_id, (left_ids, right_ids, speed, turn) in expected_sides.items():
            self.assertTrue(self.lanelets[link_id])
            observed_left_ids = set()
            observed_right_ids = set()
            for lanelet in self.lanelets[link_id]:
                tags = _tags(lanelet)
                self.assertEqual(tags["speed_limit"], speed)
                self.assertEqual(tags["turn_direction"], turn)

                members = {
                    member.attrib["role"]: int(member.attrib["ref"])
                    for member in lanelet.findall("member")
                    if member.attrib["role"] in ("left", "right", "centerline")
                }
                self.assertEqual(set(members), {"left", "right", "centerline"})
                left_tags = _tags(self.ways[members["left"]])
                right_tags = _tags(self.ways[members["right"]])
                segment_left_ids = set(
                    filter(None, left_tags["mgeo:source_boundaries"].split(",")))
                segment_right_ids = set(
                    filter(None, right_tags["mgeo:source_boundaries"].split(",")))
                self.assertTrue(segment_left_ids <= left_ids)
                self.assertTrue(segment_right_ids <= right_ids)
                observed_left_ids.update(segment_left_ids)
                observed_right_ids.update(segment_right_ids)
                center_tags = _tags(self.ways[members["centerline"]])
                self.assertEqual(center_tags["type"], "virtual")
                self.assertEqual(center_tags["subtype"], "centerline")
                self.assertEqual(center_tags["mgeo:id"], link_id)
            self.assertEqual(observed_left_ids, left_ids)
            self.assertEqual(observed_right_ids, right_ids)

    def test_lanelet_segment_metadata_covers_each_source_link(self):
        source_lengths = {"L1": 10.0, "L2": 10.0}
        # Adjacent source fragments with identical semantics are stitched into
        # one bound; only an actual attribute change requires a new lanelet.
        self.assertEqual(len(self.lanelets["L1"]), 1)
        for link_id, lanelets in self.lanelets.items():
            tags = [_tags(lanelet) for lanelet in lanelets]
            self.assertEqual(
                [int(value["mgeo:segment_index"]) for value in tags],
                list(range(len(tags))),
            )
            self.assertTrue(all(
                int(value["mgeo:segment_count"]) == len(tags) for value in tags
            ))
            self.assertEqual(
                [value["mgeo:segment_id"] for value in tags],
                ["{}#{}".format(link_id, index)
                 for index in range(len(tags))],
            )
            intervals = [
                (float(value["mgeo:start_chainage_m"]),
                 float(value["mgeo:end_chainage_m"]))
                for value in tags
            ]
            self.assertAlmostEqual(intervals[0][0], 0.0, places=3)
            for previous, following in zip(intervals, intervals[1:]):
                self.assertAlmostEqual(previous[1], following[0], places=3)
            self.assertAlmostEqual(
                intervals[-1][1], source_lengths[link_id], places=3
            )

    def test_successor_is_preserved_and_has_shared_boundary_endpoints(self):
        routing = json.loads(self.routing_path.read_text(encoding="utf-8"))
        self.assertEqual(routing["links"]["L1"]["successors"], ["L2"])
        self.assertEqual(routing["links"]["L2"]["predecessors"], ["L1"])
        self.assertEqual(_tags(self.lanelets["L1"][-1])["mgeo:successors"], "L2")

        def node_refs(way):
            return [int(node.attrib["ref"]) for node in way.findall("nd")]

        def lanelet_side(link_id, role, segment_index):
            ref = next(
                int(member.attrib["ref"])
                for member in self.lanelets[link_id][segment_index].findall("member")
                if member.attrib["type"] == "way"
                and member.attrib["role"] == role
            )
            return self.ways[ref]

        self.assertEqual(
            node_refs(lanelet_side("L1", "left", -1))[-1],
            node_refs(lanelet_side("L2", "left", 0))[0],
        )
        self.assertEqual(
            node_refs(lanelet_side("L1", "right", -1))[-1],
            node_refs(lanelet_side("L2", "right", 0))[0],
        )

    def test_traffic_light_regulation_is_attached_to_its_lanelet(self):
        signal_way = next(
            way
            for way in self.ways.values()
            if _tags(way).get("type") == "traffic_light"
            and _tags(way).get("mgeo:id") == "TL1"
        )
        regulation = next(
            relation
            for relation in self.relations.values()
            if _tags(relation).get("type") == "regulatory_element"
            and _tags(relation).get("subtype") == "traffic_light"
            and _tags(relation).get("mgeo:id") == "TL1"
        )
        regulation_id = int(regulation.attrib["id"])
        role_refs = {
            member.attrib["role"]: int(member.attrib["ref"])
            for member in regulation.findall("member")
        }
        self.assertEqual(role_refs["refers"], int(signal_way.attrib["id"]))
        self.assertEqual(
            role_refs["ref_line"], int(self.boundary_ways["B-STOP"].attrib["id"])
        )

        l2_regulations = {
            int(member.attrib["ref"])
            for lanelet in self.lanelets["L2"]
            for member in lanelet.findall("member")
            if member.attrib["type"] == "relation"
            and member.attrib["role"] == "regulatory_element"
        }
        l1_regulations = {
            int(member.attrib["ref"])
            for lanelet in self.lanelets["L1"]
            for member in lanelet.findall("member")
            if member.attrib["type"] == "relation"
            and member.attrib["role"] == "regulatory_element"
        }
        self.assertIn(regulation_id, l1_regulations)
        self.assertNotIn(regulation_id, l2_regulations)

    def test_validator_rejects_missing_surface_link_and_wrong_crossing_participant(self):
        tree = ET.parse(str(self.osm_path))
        root = tree.getroot()
        for relation in root.findall("relation"):
            tags = _tags(relation)
            if tags.get("type") == "lanelet" and tags.get("mgeo:id") == "L1":
                for tag in list(relation.findall("tag")):
                    if tag.attrib["k"] == "mgeo:surface_markings":
                        values = [value for value in tag.attrib["v"].split(",")
                                  if value != "SM-STRAIGHT"]
                        if values:
                            tag.attrib["v"] = ",".join(values)
                        else:
                            relation.remove(tag)
                    if (tags.get("mgeo:segment_index") == "0" and
                            tag.attrib["k"] == "mgeo:end_chainage_m"):
                        tag.attrib["v"] = "9.000"
            if tags.get("type") == "multipolygon" and tags.get(
                    "mgeo:id") == "CW-BICYCLE":
                for tag in list(relation.findall("tag")):
                    if tag.attrib["k"] == "participant:bicycle":
                        relation.remove(tag)
                ET.SubElement(
                    relation, "tag", {"k": "participant:pedestrian", "v": "yes"}
                )

        broken_path = self.root / "invalid-semantics.osm"
        tree.write(str(broken_path), encoding="utf-8", xml_declaration=True)
        checks = {check["name"]: check for check in validate_osm(
            broken_path, self.dataset, self.routing_path, self.config)}
        self.assertEqual(checks["lanelet_semantics"]["status"], "fail")
        self.assertEqual(checks["surface_marking_coverage"]["status"], "fail")
        self.assertEqual(checks["crosswalk_semantics"]["status"], "fail")

    def test_osm_xml_references_are_complete_and_globally_unique(self):
        primitive_ids = (
            list(self.nodes) + list(self.ways) + list(self.relations)
        )
        self.assertTrue(all(identifier > 0 for identifier in primitive_ids))
        self.assertEqual(len(primitive_ids), len(set(primitive_ids)))

        for way in self.ways.values():
            refs = [int(node.attrib["ref"]) for node in way.findall("nd")]
            self.assertGreaterEqual(len(refs), 2)
            self.assertTrue(all(ref in self.nodes for ref in refs))

        containers = {
            "node": self.nodes,
            "way": self.ways,
            "relation": self.relations,
        }
        for relation in self.relations.values():
            for member in relation.findall("member"):
                self.assertIn(member.attrib["type"], containers)
                self.assertIn(
                    int(member.attrib["ref"]), containers[member.attrib["type"]]
                )

        checks = validate_osm(
            self.osm_path, self.dataset, self.routing_path, self.config
        )
        failures = [check for check in checks if check["status"] == "fail"]
        warnings = [check for check in checks if check["status"] == "warning"]
        self.assertEqual(failures, [])
        self.assertEqual(warnings, [])


if __name__ == "__main__":
    unittest.main()
