#!/usr/bin/env python3

import hashlib
import json
import math
import pathlib
import re
import tempfile
import unittest

from global_route_manager_pkg.reference_route import (
    LinkSpan,
    RouteFormatError,
    RouteMatcher,
    RouteTopology,
    gate_observation_stamp,
    load_reference_route,
    odometry_payload_invalid_reason,
    observation_invalid_reason,
)


EXPECTED_COMPETITION_SPANS = (
    ("A2256W000751", 0.0, 0.0),
    ("A2256W000748", 0.5, 72.6),
    ("A2256W000182", 73.1, 95.0),
    ("A2256W000329", 95.5, 160.0),
    ("A2256W000236", 160.5, 186.3),
    ("A2256W000219", 186.8, 208.5),
    ("A2256W000215", 209.0, 237.4),
    ("A2256W000213", 237.9, 315.4),
    ("A2256W000222", 315.9, 346.7),
    ("A2256W000220", 347.2, 365.7),
    ("A2256W000202", 365.8, 427.8),
    ("A2256W000728", 427.9, 436.4),
    ("A2256W000599", 436.8, 462.9),
    ("A2256W000207", 463.4, 470.9),
    ("A2256W000205", 471.1, 482.8),
    ("A2256W000315", 483.3, 550.6),
    ("A2256W000318", 550.6, 580.9),
    ("A2256W000308", 581.4, 609.9),
    ("A2256W000083", 609.9, 635.1),
    ("A2256W000304", 635.6, 858.5),
    ("A2256W000148", 859.0, 885.1),
    ("A2256W000146", 885.6, 903.1),
    ("A2256W000151", 903.1, 914.8),
    ("A2256W000866", 915.3, 930.3),
    ("A2256W000054", 930.6, 984.6),
    ("A2256W000846", 984.7, 1118.7),
    ("A2256W000411", 1118.7, 1285.7),
    ("A2256W000420", 1286.2, 1368.7),
    ("A2256W000408", 1368.7, 1518.2),
    ("A2256W000445", 1518.7, 1593.6),
    ("A2256W000153", 1593.6, 1741.6),
    ("A2256W000451", 1741.7, 1781.6),
    ("A2256W000446", 1782.1, 1859.6),
    ("A2256W000448", 1860.1, 1877.6),
    ("A2256W000126", 1877.7, 1996.7),
    ("A2256W000128", 1997.2, 2109.7),
    ("A2256W000333", 2109.8, 2121.0),
    ("A2256W000154", 2121.0, 2132.2),
    ("A2256W000332", 2132.7, 2165.8),
    ("A2256W000751", 2166.3, 2184.612),
)

LINK_SPAN_PATTERN = re.compile(
    r"^\s*-\s*\{id:\s*([^,]+),\s*start_m:\s*([^,]+),"
    r"\s*end_m:\s*([^}]+)\}\s*$",
    re.MULTILINE,
)


def _competition_config_text():
    package_root = pathlib.Path(__file__).resolve().parents[1]
    return (package_root / "config" / "competition_route.yaml").read_text(
        encoding="utf-8")


def _competition_spans(config_text):
    return tuple(
        (link_id.strip(), float(start_m), float(end_m))
        for link_id, start_m, end_m in LINK_SPAN_PATTERN.findall(config_text)
    )


class ReferenceRouteTest(unittest.TestCase):
    def _write_route(self, text):
        directory = tempfile.TemporaryDirectory(prefix="global-route-")
        path = pathlib.Path(directory.name) / "route with spaces.txt"
        path.write_text(text, encoding="utf-8")
        self.addCleanup(directory.cleanup)
        return path

    def test_loader_preserves_source_indices_and_filters_only_matching_geometry(self):
        path = self._write_route(
            "0 0 3\n"
            "1 0 3\n"
            "1 0 4\n"
            "2 0 3\n"
            "0 0 3\n")
        expected_sha = hashlib.sha256(path.read_bytes()).hexdigest()
        route = load_reference_route(
            path,
            expected_point_count=5,
            expected_sha256=expected_sha,
            expected_length_m=4.0,
        )

        self.assertEqual(route.source_point_count, 5)
        self.assertEqual(route.duplicate_count, 1)
        self.assertEqual([point.source_index for point in route.points], [0, 1, 3, 4])
        self.assertAlmostEqual(route.length_m, 4.0)
        self.assertTrue(route.closed)
        self.assertEqual(len(route.raw_headings()), 5)
        self.assertTrue(all(math.isfinite(yaw) for yaw in route.raw_headings()))

    def test_loader_rejects_nonfinite_and_integrity_mismatch(self):
        nonfinite = self._write_route("0 0\nnan 1\n")
        with self.assertRaises(RouteFormatError):
            load_reference_route(nonfinite)

        valid = self._write_route("0 0\n1 0\n")
        with self.assertRaises(RouteFormatError):
            load_reference_route(valid, expected_sha256="0" * 64)
        with self.assertRaises(RouteFormatError):
            load_reference_route(valid, expected_point_count=3)

    def test_matcher_initializes_closed_route_at_zero_and_is_monotonic(self):
        route = load_reference_route(self._write_route(
            "0 0\n10 0\n10 10\n0 10\n0 0\n"))
        matcher = RouteMatcher(
            route,
            max_lateral_distance_m=2.0,
            max_heading_error_rad=math.radians(60.0),
        )

        start = matcher.match(0.0, 0.0, yaw_rad=0.0)
        forward = matcher.match(6.0, 0.2, yaw_rad=0.0)
        noisy_backstep = matcher.match(5.5, -0.1, yaw_rad=0.0)

        self.assertTrue(start.valid)
        self.assertAlmostEqual(start.progress_m, 0.0)
        self.assertTrue(forward.valid)
        self.assertGreater(forward.progress_m, start.progress_m)
        self.assertEqual(noisy_backstep.progress_m, forward.progress_m)

        before_invalid = matcher.last_progress_m
        invalid = matcher.match(100.0, 100.0, yaw_rad=0.0)
        self.assertFalse(invalid.valid)
        self.assertEqual(invalid.reason, "off_route")
        self.assertEqual(matcher.last_progress_m, before_invalid)

    def test_matcher_rejects_wrong_direction(self):
        route = load_reference_route(self._write_route("0 0\n10 0\n"))
        matcher = RouteMatcher(
            route,
            max_heading_error_rad=math.radians(45.0),
        )
        result = matcher.match(2.0, 0.0, yaw_rad=math.pi)
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "heading_mismatch")

    def test_official_route_integrity_and_duplicate_accounting(self):
        repository = pathlib.Path(__file__).resolve().parents[3]
        path = repository / "참고파일들" / "2026_molit_comp_global_path (3).txt"
        if not path.is_file():
            self.skipTest("immutable competition route is unavailable")
        route = load_reference_route(
            path,
            expected_point_count=4430,
            expected_sha256=(
                "50658991e607d9339d76e4cd6cb169dfc733ea53b93de2c3e222460bb497cc05"),
            expected_length_m=2184.612,
            length_tolerance_m=0.1,
        )
        self.assertTrue(route.closed)
        self.assertEqual(route.duplicate_count, 38)
        self.assertAlmostEqual(route.length_m, 2184.612, places=2)

        matcher = RouteMatcher(route)
        headings = route.raw_headings()
        for index in range(0, route.source_point_count, 5):
            point = route.raw_points[index]
            result = matcher.match(point.x_m, point.y_m, headings[index])
            self.assertTrue(result.valid, "official route index {}".format(index))
        final_point = route.raw_points[-1]
        final = matcher.match(
            final_point.x_m, final_point.y_m, headings[-1])
        self.assertTrue(final.valid)
        self.assertAlmostEqual(final.progress_m, route.length_m)


class RouteTopologyTest(unittest.TestCase):
    def setUp(self):
        self.topology = RouteTopology(
            spans=(
                LinkSpan("PRE", 0.0, 9.5),
                LinkSpan("FAST_START", 10.0, 19.5),
                LinkSpan("FAST_END", 20.0, 29.5),
                LinkSpan("POST", 30.0, 40.0),
            ),
            route_length_m=40.0,
            high_speed_start_link_id="FAST_START",
            high_speed_end_link_id="FAST_END",
            continuity_tolerance_m=0.5,
        )

    def _match(self, progress_m, valid=True):
        from global_route_manager_pkg.reference_route import MatchResult
        return MatchResult(
            valid=valid,
            reason="ok" if valid else "stale_odometry",
            progress_m=progress_m,
            lateral_distance_m=0.2,
            segment_index=0,
            source_index=7,
            projected_x_m=0.0,
            projected_y_m=0.0,
        )

    def test_context_uses_nearest_span_in_rounded_gap(self):
        before_midpoint = self.topology.context_for_match(self._match(9.7))
        after_midpoint = self.topology.context_for_match(self._match(9.8))
        self.assertEqual(before_midpoint.current_link_id, "PRE")
        self.assertEqual(after_midpoint.current_link_id, "FAST_START")

    def test_high_speed_interval_is_inclusive_and_horizon_is_ordered(self):
        start = self.topology.context_for_match(self._match(10.0), 3)
        end = self.topology.context_for_match(self._match(29.5), 3)
        post = self.topology.context_for_match(self._match(30.0), 3)
        self.assertTrue(start.speed_limit_exempt_zone)
        self.assertTrue(end.speed_limit_exempt_zone)
        self.assertFalse(post.speed_limit_exempt_zone)
        self.assertEqual(
            start.horizon_link_ids,
            ("FAST_START", "FAST_END", "POST"),
        )

    def test_shared_endpoint_advances_to_successor(self):
        topology = RouteTopology(
            spans=(
                LinkSpan("PRE", 0.0, 10.0),
                LinkSpan("FAST_START", 10.0, 20.0),
                LinkSpan("FAST_END", 20.0, 30.0),
            ),
            route_length_m=30.0,
            high_speed_start_link_id="FAST_START",
            high_speed_end_link_id="FAST_END",
        )
        state = topology.context_for_match(self._match(10.0))
        self.assertEqual(state.current_link_id, "FAST_START")
        self.assertTrue(state.speed_limit_exempt_zone)

    def test_invalid_match_clears_route_authority(self):
        state = self.topology.context_for_match(self._match(15.0, valid=False))
        self.assertFalse(state.valid)
        self.assertEqual(state.current_link_id, "")
        self.assertEqual(state.horizon_link_ids, tuple())
        self.assertFalse(state.speed_limit_exempt_zone)

    def test_topology_rejects_incomplete_or_out_of_bounds_coverage(self):
        with self.assertRaises(ValueError):
            RouteTopology(
                spans=(
                    LinkSpan("FAST_START", 0.0, 10.0),
                    LinkSpan("FAST_END", 10.0, 38.0),
                ),
                route_length_m=40.0,
                high_speed_start_link_id="FAST_START",
                high_speed_end_link_id="FAST_END",
                continuity_tolerance_m=0.5,
            )
        with self.assertRaises(ValueError):
            RouteTopology(
                spans=(
                    LinkSpan("FAST_START", -0.1, 10.0),
                    LinkSpan("FAST_END", 10.0, 40.0),
                ),
                route_length_m=40.0,
                high_speed_start_link_id="FAST_START",
                high_speed_end_link_id="FAST_END",
            )

    def test_observation_metadata_is_fail_closed(self):
        self.assertEqual(
            observation_invalid_reason(
                9.9, 10.0, 0.25, "map", "map", "base_link", "base_link"),
            "")
        self.assertEqual(
            observation_invalid_reason(
                9.0, 10.0, 0.25, "map", "map", "base_link", "base_link"),
            "stale_odometry")
        self.assertEqual(
            observation_invalid_reason(
                9.9, 10.0, 0.25, "odom", "map", "base_link", "base_link"),
            "frame_mismatch")
        self.assertEqual(
            observation_invalid_reason(
                9.9, 10.0, 0.25, "map", "map", "sensor", "base_link"),
            "child_frame_mismatch")
        self.assertEqual(
            observation_invalid_reason(
                10.000001,
                10.0,
                0.25,
                "map",
                "map",
                "base_link",
                "base_link",
            ),
            "future_timestamp")

    def test_odometry_payload_checks_every_numeric_group(self):
        valid = {
            "position_xyz": (1.0, 2.0, 3.0),
            "orientation_xyzw": (0.0, 0.0, 0.0, 1.0),
            "linear_velocity_xyz": (4.0, 0.0, 0.0),
            "angular_velocity_xyz": (0.0, 0.0, 0.1),
            "pose_covariance": (0.0,) * 36,
            "twist_covariance": (-1.0,) + (0.0,) * 35,
        }
        self.assertEqual(odometry_payload_invalid_reason(**valid), "")

        cases = (
            ("position_xyz", (1.0, 2.0, math.nan), "non_finite_pose"),
            (
                "orientation_xyzw",
                (0.0, 0.0, math.inf, 1.0),
                "non_finite_orientation",
            ),
            (
                "linear_velocity_xyz",
                (math.nan, 0.0, 0.0),
                "non_finite_twist",
            ),
            (
                "angular_velocity_xyz",
                (0.0, math.inf, 0.0),
                "non_finite_twist",
            ),
            (
                "pose_covariance",
                (0.0,) * 35 + (math.nan,),
                "non_finite_covariance",
            ),
            (
                "twist_covariance",
                (0.0,) * 35 + (math.inf,),
                "non_finite_covariance",
            ),
        )
        for field, replacement, expected_reason in cases:
            payload = dict(valid)
            payload[field] = replacement
            with self.subTest(field=field):
                self.assertEqual(
                    odometry_payload_invalid_reason(**payload),
                    expected_reason,
                )

        zero_quaternion = dict(valid)
        zero_quaternion["orientation_xyzw"] = (0.0, 0.0, 0.0, 0.0)
        self.assertEqual(
            odometry_payload_invalid_reason(**zero_quaternion),
            "invalid_orientation",
        )
        malformed_covariance = dict(valid)
        malformed_covariance["pose_covariance"] = (0.0,) * 35
        self.assertEqual(
            odometry_payload_invalid_reason(**malformed_covariance),
            "malformed_odometry",
        )

    def test_invalid_future_stamp_does_not_poison_accepted_watermark(self):
        reason, watermark = gate_observation_stamp(
            10.0, 1000.0, "future_timestamp")
        self.assertEqual(reason, "future_timestamp")
        self.assertEqual(watermark, 10.0)

        reason, watermark = gate_observation_stamp(watermark, 10.1, "")
        self.assertEqual(reason, "")
        self.assertEqual(watermark, 10.1)

        reason, unchanged = gate_observation_stamp(watermark, 10.0, "")
        self.assertEqual(reason, "out_of_order_odometry")
        self.assertEqual(unchanged, watermark)


class OfficialCompetitionTopologyTest(unittest.TestCase):
    def test_exact_spans_coverage_and_high_speed_bounds_are_locked(self):
        config_text = _competition_config_text()
        spans = _competition_spans(config_text)
        self.assertEqual(len(spans), 40)
        self.assertEqual(spans, EXPECTED_COMPETITION_SPANS)
        self.assertEqual(spans[0][1], 0.0)
        self.assertEqual(spans[-1][2], 2184.612)
        for previous, current in zip(spans, spans[1:]):
            self.assertLessEqual(abs(current[1] - previous[2]), 1.0)

        start_match = re.search(
            r"^\s*start_link_id:\s*(\S+)\s*$", config_text, re.MULTILINE)
        end_match = re.search(
            r"^\s*end_link_id:\s*(\S+)\s*$", config_text, re.MULTILINE)
        self.assertIsNotNone(start_match)
        self.assertIsNotNone(end_match)
        self.assertEqual(start_match.group(1), "A2256W000411")
        self.assertEqual(end_match.group(1), "A2256W000153")
        self.assertEqual(spans[26], ("A2256W000411", 1118.7, 1285.7))
        self.assertEqual(spans[30], ("A2256W000153", 1593.6, 1741.6))

        repository = pathlib.Path(__file__).resolve().parents[3]
        route_path = (
            repository / "참고파일들" / "2026_molit_comp_global_path (3).txt")
        route_length_m = 2184.612
        if route_path.is_file():
            route_length_m = load_reference_route(route_path).length_m
        topology = RouteTopology(
            spans=tuple(LinkSpan(*span) for span in spans),
            route_length_m=route_length_m,
            high_speed_start_link_id=start_match.group(1),
            high_speed_end_link_id=end_match.group(1),
            continuity_tolerance_m=1.0,
        )
        self.assertAlmostEqual(topology.high_speed_start_m, 1118.7)
        self.assertAlmostEqual(topology.high_speed_end_m, 1741.6)

    def test_every_configured_span_link_exists_in_pinned_mgeo(self):
        repository = pathlib.Path(__file__).resolve().parents[3]
        link_set_path = (
            repository
            / "src"
            / "hd_map_pkg"
            / "vendor"
            / "verdict_sdk"
            / "map-data"
            / "KATRI"
            / "link_set.json"
        )
        if not link_set_path.is_file():
            self.skipTest("pinned KATRI MGeo link_set.json is unavailable")
        records = json.loads(link_set_path.read_text(encoding="utf-8"))
        self.assertIsInstance(records, list)
        map_link_ids = {
            str(record["idx"])
            for record in records
            if isinstance(record, dict) and "idx" in record
        }
        configured_ids = {
            link_id for link_id, _start_m, _end_m
            in _competition_spans(_competition_config_text())
        }
        self.assertFalse(configured_ids.difference(map_link_ids))


if __name__ == "__main__":
    unittest.main()
