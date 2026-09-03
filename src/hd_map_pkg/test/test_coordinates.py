#!/usr/bin/env python3
"""Unit tests for the dependency-free HD-map coordinate transforms."""

import math
import pathlib
import sys
import unittest


PACKAGE_SOURCE = pathlib.Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SOURCE))

from hd_map_pkg.coordinates import (  # noqa: E402
    CoordinateTransformer,
    mgeo_local_to_sim_local,
    mgeo_local_to_utm,
    utm52n_to_wgs84,
    utm_to_wgs84,
)


class CoordinateTransformerTest(unittest.TestCase):
    def setUp(self):
        self.transformer = CoordinateTransformer(
            mgeo_origin_utm=(305390.0, 4122845.0, 0.0),
            simulator_origin_utm=(302595.0, 4124145.0, 0.0),
        )

    def test_exporter_facing_api(self):
        self.assertEqual(
            self.transformer.mgeo_to_utm((100.25, -20.5, 28.0)),
            (305490.25, 4122824.5, 28.0),
        )
        self.assertEqual(
            self.transformer.mgeo_to_sim((100.25, -20.5, 28.0)),
            (2895.25, -1320.5, 28.0),
        )

        latitude, longitude = self.transformer.utm_to_wgs84(
            (305390.0, 4122845.0, 123.0)
        )
        self.assertAlmostEqual(latitude, 37.231827104419, places=8)
        self.assertAlmostEqual(longitude, 126.806249031091, places=8)

    def test_configuration_is_exposed_as_immutable_tuples(self):
        self.assertEqual(
            self.transformer.mgeo_origin_utm,
            (305390.0, 4122845.0, 0.0),
        )
        self.assertEqual(
            self.transformer.simulator_origin_utm,
            (302595.0, 4124145.0, 0.0),
        )
        self.assertEqual(self.transformer.utm_zone, 52)
        self.assertTrue(self.transformer.northern)

    def test_invalid_constructor_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            CoordinateTransformer(
                (305390.0, 4122845.0),
                (302595.0, 4124145.0),
                utm_zone=61,
            )


class LocalCoordinateTransformTest(unittest.TestCase):
    def test_mgeo_local_to_utm_adds_global_origin(self):
        self.assertEqual(
            mgeo_local_to_utm(
                (-2926.689797551, 871.668977062, 28.543960282),
                (305390.0, 4122845.0, 0.0),
            ),
            (302463.310202449, 4123716.668977062, 28.543960282),
        )

    def test_two_dimensional_values_have_zero_height(self):
        self.assertEqual(
            mgeo_local_to_utm((10, -20), (305390, 4122845)),
            (305400.0, 4122825.0, 0.0),
        )

    def test_mgeo_local_to_competition_scene_local(self):
        # KATRI MGeo origin minus competition scene origin is (+2795, -1300).
        self.assertEqual(
            mgeo_local_to_sim_local(
                (100.25, -20.5, 28.0),
                (305390.0, 4122845.0, 0.0),
                (302595.0, 4124145.0, 0.0),
            ),
            (2895.25, -1320.5, 28.0),
        )

    def test_z_origins_are_respected(self):
        self.assertEqual(
            mgeo_local_to_sim_local(
                (0.0, 0.0, 3.5),
                (305390.0, 4122845.0, 10.0),
                (302595.0, 4124145.0, 7.0),
            ),
            (2795.0, -1300.0, 6.5),
        )


class UtmInverseTest(unittest.TestCase):
    def test_katri_origin_matches_independent_epsg_projection(self):
        # Expected values were generated independently with EPSG:32652->4326.
        latitude, longitude = utm52n_to_wgs84(305390.0, 4122845.0)
        self.assertAlmostEqual(latitude, 37.231827104419, places=8)
        self.assertAlmostEqual(longitude, 126.806249031091, places=8)

    def test_scene_origin_matches_independent_epsg_projection(self):
        latitude, longitude = utm_to_wgs84(
            302595.0, 4124145.0, zone_number=52
        )
        self.assertAlmostEqual(latitude, 37.242948840936, places=8)
        self.assertAlmostEqual(longitude, 126.774419400919, places=8)

    def test_zone_central_meridian_at_equator(self):
        latitude, longitude = utm_to_wgs84(500000.0, 0.0, zone_number=52)
        self.assertAlmostEqual(latitude, 0.0, places=12)
        self.assertAlmostEqual(longitude, 129.0, places=12)

    def test_southern_false_northing_is_removed(self):
        latitude, longitude = utm_to_wgs84(
            500000.0,
            10000000.0,
            zone_number=52,
            northern_hemisphere=False,
        )
        self.assertAlmostEqual(latitude, 0.0, places=12)
        self.assertAlmostEqual(longitude, 129.0, places=12)

    def test_repeated_conversion_is_bitwise_deterministic(self):
        first = utm52n_to_wgs84(305390.125, 4122845.875)
        for _ in range(20):
            self.assertEqual(utm52n_to_wgs84(305390.125, 4122845.875), first)


class InvalidInputTest(unittest.TestCase):
    def test_invalid_point_dimension_is_rejected(self):
        with self.assertRaises(ValueError):
            mgeo_local_to_utm((1.0,), (305390.0, 4122845.0, 0.0))

    def test_non_finite_point_is_rejected(self):
        with self.assertRaises(ValueError):
            mgeo_local_to_utm(
                (math.nan, 0.0, 0.0), (305390.0, 4122845.0, 0.0)
            )

    def test_invalid_zone_is_rejected(self):
        with self.assertRaises(ValueError):
            utm_to_wgs84(500000.0, 0.0, zone_number=0)

    def test_boolean_zone_is_rejected(self):
        with self.assertRaises(TypeError):
            utm_to_wgs84(500000.0, 0.0, zone_number=True)

    def test_out_of_range_utm_coordinate_is_rejected(self):
        with self.assertRaises(ValueError):
            utm_to_wgs84(99999.0, 4122845.0, zone_number=52)


if __name__ == "__main__":
    unittest.main()
