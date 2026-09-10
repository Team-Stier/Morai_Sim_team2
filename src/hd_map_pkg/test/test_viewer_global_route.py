#!/usr/bin/env python3

import pathlib
import tempfile
import unittest

from hd_map_pkg.viewer import build_viewer_data, load_global_route, write_viewer


class _IdentityTransformer(object):
    @staticmethod
    def mgeo_to_sim(point):
        return list(point)


class _Dataset(object):
    lane_boundaries = {}
    surface_markings = {}
    single_crosswalks = {}
    traffic_lights = {}
    junctions = {}
    links_by_road = {}
    predecessors = {"L1": []}
    successors = {"L1": []}
    links = {
        "L1": {
            "points": [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]],
            "max_speed": 30,
            "related_signal": "straight",
        }
    }

    @staticmethod
    def traffic_light_link_ids():
        return {}


class ViewerGlobalRouteTest(unittest.TestCase):
    def test_route_loader_preserves_sim_local_points_and_duplicates(self):
        with tempfile.TemporaryDirectory(prefix="hd-map-route-") as directory:
            path = pathlib.Path(directory) / "route with spaces.txt"
            path.write_text(
                "10.12349 20.98761 3\n"
                "10.12349 20.98761 3\n\n"
                "30 40\n"
                "10.12349 20.98761 3",
                encoding="utf-8",
            )
            route = load_global_route(path)

        self.assertEqual(route["point_count"], 4)
        self.assertEqual(route["p"], [
            [10.123, 20.988],
            [10.123, 20.988],
            [30.0, 40.0],
            [10.123, 20.988],
        ])
        self.assertTrue(route["closed"])
        self.assertEqual(route["coordinate_frame"],
                         "MORAI SIM local ENU (metres)")
        self.assertEqual(route["source"], "route with spaces.txt")

    def test_viewer_includes_route_in_bounds_count_and_html_layer(self):
        with tempfile.TemporaryDirectory(prefix="hd-map-viewer-") as directory:
            root = pathlib.Path(directory)
            route_path = root / "route.txt"
            route_path.write_text("-5 -6 0\n8 9 0\n", encoding="utf-8")
            data = build_viewer_data(
                _Dataset(), _IdentityTransformer(), {
                    "conversion": {"viewer_simplification_m": 0.2},
                    "source": {"status": "test", "commit": "deadbeef"},
                    "coordinates": {"simulator_scene": "test.scene"},
                    "lane_boundary": {},
                }, reference_path=route_path)
            output = write_viewer(data, root / "preview.html")
            html = output.read_text(encoding="utf-8")

        self.assertEqual(data["metadata"]["counts"]["global_route_points"], 2)
        self.assertEqual(data["metadata"]["bounds"], {
            "min_x": -5.0, "min_y": -6.0, "max_x": 8.0, "max_y": 9.0,
        })
        self.assertIn('data-layer="globalRoute"', html)
        self.assertIn("전역경로 TXT", html)
        self.assertIn("#39ff88", html)
        self.assertIn('"point_count":2', html)
        self.assertIn("consider('global_route'", html)
        self.assertLess(html.index("consider('global_route'"),
                        html.index("consider('lane/link'"))

    def test_viewer_discards_map_features_outside_route_extent(self):
        class CroppedDataset(_Dataset):
            links = {
                "inside": {"points": [[0, 0, 0], [5, 5, 0]]},
                "outside": {"points": [[100, 100, 0], [110, 110, 0]]},
                "crossing": {"points": [[-20, 5, 0], [20, 5, 0]]},
            }
            predecessors = {}
            successors = {}
            traffic_lights = {
                "inside-signal": {"point": [2, 2, 0]},
                "outside-signal": {"point": [102, 102, 0]},
            }

        with tempfile.TemporaryDirectory(prefix="hd-map-crop-") as directory:
            route_path = pathlib.Path(directory) / "route.txt"
            route_path.write_text("0 0 0\n10 10 0\n", encoding="utf-8")
            data = build_viewer_data(
                CroppedDataset(), _IdentityTransformer(), {
                    "conversion": {
                        "viewer_simplification_m": 0.2,
                        "viewer_route_crop_margin_m": 0.0,
                    },
                    "source": {}, "coordinates": {}, "lane_boundary": {},
                }, reference_path=route_path)

        self.assertEqual([item["id"] for item in data["centerlines"]],
                         ["crossing", "inside"])
        self.assertEqual([item["id"] for item in data["signals"]],
                         ["inside-signal"])
        self.assertTrue(data["metadata"]["route_crop_applied"])
        self.assertEqual(data["metadata"]["bounds"], {
            "min_x": 0.0, "min_y": 0.0, "max_x": 10.0, "max_y": 10.0,
        })

    def test_viewer_applies_configured_route_crop_margin(self):
        with tempfile.TemporaryDirectory(prefix="hd-map-margin-") as directory:
            route_path = pathlib.Path(directory) / "route.txt"
            route_path.write_text("0 0 0\n10 20 0\n", encoding="utf-8")
            data = build_viewer_data(
                _Dataset(), _IdentityTransformer(), {
                    "conversion": {
                        "viewer_simplification_m": 0.2,
                        "viewer_route_crop_margin_m": 30.0,
                    },
                    "source": {}, "coordinates": {}, "lane_boundary": {},
                }, reference_path=route_path)

        self.assertEqual(data["metadata"]["bounds"], {
            "min_x": -30.0, "min_y": -30.0,
            "max_x": 40.0, "max_y": 50.0,
        })

    def test_viewer_expands_crop_to_configured_boundary_anchors(self):
        class AnchoredDataset(_Dataset):
            lane_boundaries = {
                "anchor": {
                    "idx": "anchor", "points": [[-20, 30, 0], [40, 50, 0]],
                    "lane_type": [505], "lane_shape": ["solid"],
                    "lane_color": ["white"],
                },
                "outside": {
                    "idx": "outside", "points": [[100, 100, 0], [110, 110, 0]],
                    "lane_type": [505], "lane_shape": ["solid"],
                    "lane_color": ["white"],
                },
            }

        with tempfile.TemporaryDirectory(prefix="hd-map-anchor-") as directory:
            route_path = pathlib.Path(directory) / "route.txt"
            route_path.write_text("0 0 0\n10 10 0\n", encoding="utf-8")
            data = build_viewer_data(
                AnchoredDataset(), _IdentityTransformer(), {
                    "conversion": {
                        "viewer_simplification_m": 0.2,
                        "viewer_route_crop_margin_m": 5.0,
                        "viewer_crop_anchor_boundary_ids": ["anchor"],
                    },
                    "source": {}, "coordinates": {},
                    "lane_boundary": {"road_border_codes": [505]},
                }, reference_path=route_path)

        self.assertEqual(data["metadata"]["bounds"], {
            "min_x": -20.0, "min_y": -5.0,
            "max_x": 40.0, "max_y": 50.0,
        })
        self.assertEqual(data["metadata"]["crop_anchor_boundary_ids"],
                         ["anchor"])
        self.assertEqual([item["id"] for item in data["boundaries"]],
                         ["anchor"])

    def test_viewer_classifies_tunnel_lane_control_signals_separately(self):
        class SignalDataset(_Dataset):
            traffic_lights = {
                "LCS01": {
                    "point": [2, 2, 0], "type": "car", "type_def": "mgeo",
                    "sub_type": [[5, 2], [5, 0]], "dynamic": True,
                },
                "C1": {
                    "point": [3, 3, 0], "type": "car",
                    "type_def": "ngii_model2",
                },
            }

            @staticmethod
            def traffic_light_link_ids():
                return {"LCS01": [], "C1": ["L1"]}

        with tempfile.TemporaryDirectory(prefix="hd-map-lcs-") as directory:
            root = pathlib.Path(directory)
            route_path = root / "route.txt"
            route_path.write_text("0 0 0\n10 10 0\n", encoding="utf-8")
            data = build_viewer_data(
                SignalDataset(), _IdentityTransformer(), {
                    "conversion": {"viewer_simplification_m": 0.2},
                    "source": {}, "coordinates": {}, "lane_boundary": {},
                }, reference_path=route_path)
            html = write_viewer(data, root / "preview.html").read_text(
                encoding="utf-8")

        signals = {item["id"]: item for item in data["signals"]}
        self.assertEqual(signals["LCS01"]["category"],
                         "tunnel_lane_control")
        self.assertEqual(signals["C1"]["category"], "vehicle")
        self.assertEqual(signals["LCS01"]["source_sub_type"],
                         [[5, 2], [5, 0]])
        self.assertEqual(
            data["metadata"]["counts"]["tunnel_lane_control_signals"], 1)
        self.assertIn('data-layer="laneControlSignals"', html)
        self.assertIn("터널 차로제어신호(LCS)", html)
        self.assertIn("function laneControlSignal", html)


if __name__ == "__main__":
    unittest.main()
