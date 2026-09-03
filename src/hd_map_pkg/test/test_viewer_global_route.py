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


if __name__ == "__main__":
    unittest.main()
