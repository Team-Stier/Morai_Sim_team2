"""Deterministic coordinate transforms used by the HD-map conversion tools.

MGeo geometry is stored as local ENU coordinates relative to the projected
origin in ``global_info.json``.  MORAI scene coordinates use the same ENU axis
convention but can have a different projected origin.  This module deliberately
uses only Python's standard library so conversion does not depend on a local
PROJ installation.
"""

import math
from typing import Sequence, Tuple


Point3 = Tuple[float, float, float]
GeodeticPoint = Tuple[float, float]

WGS84_SEMI_MAJOR_AXIS_M = 6378137.0
WGS84_INVERSE_FLATTENING = 298.257223563
UTM_SCALE_FACTOR = 0.9996
UTM_FALSE_EASTING_M = 500000.0
UTM_FALSE_NORTHING_M = 10000000.0


def _as_xyz(values: Sequence[float], name: str) -> Point3:
    """Return a finite XYZ tuple, accepting a two-dimensional value as Z=0."""

    if isinstance(values, (str, bytes)):
        raise TypeError("{} must be a numeric sequence".format(name))

    try:
        size = len(values)
    except TypeError:
        raise TypeError("{} must be a numeric sequence".format(name))

    if size not in (2, 3):
        raise ValueError("{} must contain two or three coordinates".format(name))

    try:
        x = float(values[0])
        y = float(values[1])
        z = float(values[2]) if size == 3 else 0.0
    except (TypeError, ValueError):
        raise TypeError("{} must contain only numeric coordinates".format(name))

    if not all(math.isfinite(value) for value in (x, y, z)):
        raise ValueError("{} coordinates must be finite".format(name))

    return x, y, z


def _validate_utm_configuration(
    zone_number: int, northern_hemisphere: bool
) -> None:
    if isinstance(zone_number, bool) or not isinstance(zone_number, int):
        raise TypeError("zone_number must be an integer")
    if not 1 <= zone_number <= 60:
        raise ValueError("zone_number must be in the range 1..60")
    if not isinstance(northern_hemisphere, bool):
        raise TypeError("northern_hemisphere must be a bool")


def mgeo_local_to_utm(
    local_point: Sequence[float], global_origin_utm: Sequence[float]
) -> Point3:
    """Convert an MGeo-local ENU point to projected UTM coordinates in metres.

    ``global_origin_utm`` is the ``origin`` value from MGeo ``global_info.json``.
    The caller is responsible for ensuring that the origin belongs to the UTM
    zone used for subsequent geodetic conversion (UTM zone 52N for KATRI).
    """

    local_x, local_y, local_z = _as_xyz(local_point, "local_point")
    origin_e, origin_n, origin_z = _as_xyz(global_origin_utm, "global_origin_utm")
    return (
        origin_e + local_x,
        origin_n + local_y,
        origin_z + local_z,
    )


def mgeo_local_to_sim_local(
    local_point: Sequence[float],
    mgeo_origin_utm: Sequence[float],
    scene_origin_utm: Sequence[float],
) -> Point3:
    """Convert MGeo-local ENU coordinates to MORAI scene-local ENU coordinates.

    Both origins must use the same projected CRS.  For the KATRI candidate and
    the competition sample scene that CRS is WGS84 / UTM zone 52N.
    """

    utm_e, utm_n, utm_z = mgeo_local_to_utm(local_point, mgeo_origin_utm)
    scene_e, scene_n, scene_z = _as_xyz(scene_origin_utm, "scene_origin_utm")
    return utm_e - scene_e, utm_n - scene_n, utm_z - scene_z


def utm_to_wgs84(
    easting_m: float,
    northing_m: float,
    zone_number: int = 52,
    northern_hemisphere: bool = True,
) -> GeodeticPoint:
    """Convert a WGS84 UTM coordinate to ``(latitude, longitude)`` in degrees.

    The implementation follows the standard inverse Transverse Mercator series
    for WGS84 and is independent of third-party projection libraries.  Inputs
    are restricted to the conventional UTM easting/northing ranges so malformed
    map metadata fails early instead of producing plausible-looking positions.
    """

    _validate_utm_configuration(zone_number, northern_hemisphere)

    try:
        easting = float(easting_m)
        northing = float(northing_m)
    except (TypeError, ValueError):
        raise TypeError("easting_m and northing_m must be numeric")

    if not math.isfinite(easting) or not math.isfinite(northing):
        raise ValueError("easting_m and northing_m must be finite")
    if not 100000.0 <= easting <= 1000000.0:
        raise ValueError("easting_m is outside the conventional UTM range")
    if not 0.0 <= northing <= UTM_FALSE_NORTHING_M:
        raise ValueError("northing_m is outside the conventional UTM range")

    flattening = 1.0 / WGS84_INVERSE_FLATTENING
    eccentricity_sq = flattening * (2.0 - flattening)
    second_eccentricity_sq = eccentricity_sq / (1.0 - eccentricity_sq)

    x = easting - UTM_FALSE_EASTING_M
    y = northing
    if not northern_hemisphere:
        y -= UTM_FALSE_NORTHING_M

    meridional_arc = y / UTM_SCALE_FACTOR
    footprint_denominator = WGS84_SEMI_MAJOR_AXIS_M * (
        1.0
        - eccentricity_sq / 4.0
        - 3.0 * eccentricity_sq**2 / 64.0
        - 5.0 * eccentricity_sq**3 / 256.0
    )
    mu = meridional_arc / footprint_denominator

    sqrt_one_minus_eccentricity = math.sqrt(1.0 - eccentricity_sq)
    e1 = (1.0 - sqrt_one_minus_eccentricity) / (
        1.0 + sqrt_one_minus_eccentricity
    )
    j1 = 3.0 * e1 / 2.0 - 27.0 * e1**3 / 32.0
    j2 = 21.0 * e1**2 / 16.0 - 55.0 * e1**4 / 32.0
    j3 = 151.0 * e1**3 / 96.0
    j4 = 1097.0 * e1**4 / 512.0
    footprint_latitude = (
        mu
        + j1 * math.sin(2.0 * mu)
        + j2 * math.sin(4.0 * mu)
        + j3 * math.sin(6.0 * mu)
        + j4 * math.sin(8.0 * mu)
    )

    sin_footprint = math.sin(footprint_latitude)
    cos_footprint = math.cos(footprint_latitude)
    tan_footprint = math.tan(footprint_latitude)
    one_minus_e_sin_sq = 1.0 - eccentricity_sq * sin_footprint**2
    radius_prime_vertical = WGS84_SEMI_MAJOR_AXIS_M / math.sqrt(
        one_minus_e_sin_sq
    )
    radius_meridian = (
        WGS84_SEMI_MAJOR_AXIS_M
        * (1.0 - eccentricity_sq)
        / one_minus_e_sin_sq**1.5
    )
    tangent_sq = tan_footprint**2
    curvature = second_eccentricity_sq * cos_footprint**2
    normalized_easting = x / (radius_prime_vertical * UTM_SCALE_FACTOR)

    latitude = footprint_latitude - (
        radius_prime_vertical * tan_footprint / radius_meridian
    ) * (
        normalized_easting**2 / 2.0
        - (
            5.0
            + 3.0 * tangent_sq
            + 10.0 * curvature
            - 4.0 * curvature**2
            - 9.0 * second_eccentricity_sq
        )
        * normalized_easting**4
        / 24.0
        + (
            61.0
            + 90.0 * tangent_sq
            + 298.0 * curvature
            + 45.0 * tangent_sq**2
            - 252.0 * second_eccentricity_sq
            - 3.0 * curvature**2
        )
        * normalized_easting**6
        / 720.0
    )

    central_meridian_deg = zone_number * 6.0 - 183.0
    longitude_delta = (
        normalized_easting
        - (1.0 + 2.0 * tangent_sq + curvature)
        * normalized_easting**3
        / 6.0
        + (
            5.0
            - 2.0 * curvature
            + 28.0 * tangent_sq
            - 3.0 * curvature**2
            + 8.0 * second_eccentricity_sq
            + 24.0 * tangent_sq**2
        )
        * normalized_easting**5
        / 120.0
    ) / cos_footprint

    latitude_deg = math.degrees(latitude)
    longitude_deg = central_meridian_deg + math.degrees(longitude_delta)
    longitude_deg = (longitude_deg + 180.0) % 360.0 - 180.0
    return latitude_deg, longitude_deg


def utm52n_to_wgs84(easting_m: float, northing_m: float) -> GeodeticPoint:
    """Convert a WGS84 / UTM zone 52N point to latitude and longitude."""

    return utm_to_wgs84(
        easting_m,
        northing_m,
        zone_number=52,
        northern_hemisphere=True,
    )


class CoordinateTransformer:
    """Coordinate-transform facade used by the MGeo importer and exporter."""

    __slots__ = (
        "_mgeo_origin_utm",
        "_simulator_origin_utm",
        "_utm_zone",
        "_northern",
    )

    def __init__(
        self,
        mgeo_origin_utm: Sequence[float],
        simulator_origin_utm: Sequence[float],
        utm_zone: int = 52,
        northern: bool = True,
    ) -> None:
        _validate_utm_configuration(utm_zone, northern)
        self._mgeo_origin_utm = _as_xyz(
            mgeo_origin_utm, "mgeo_origin_utm"
        )
        self._simulator_origin_utm = _as_xyz(
            simulator_origin_utm, "simulator_origin_utm"
        )
        self._utm_zone = utm_zone
        self._northern = northern

    @property
    def mgeo_origin_utm(self) -> Point3:
        return self._mgeo_origin_utm

    @property
    def simulator_origin_utm(self) -> Point3:
        return self._simulator_origin_utm

    @property
    def utm_zone(self) -> int:
        return self._utm_zone

    @property
    def northern(self) -> bool:
        return self._northern

    def mgeo_to_utm(self, point: Sequence[float]) -> Point3:
        """Convert an MGeo-local point to UTM XYZ in metres."""

        return mgeo_local_to_utm(point, self._mgeo_origin_utm)

    def mgeo_to_sim(self, point: Sequence[float]) -> Point3:
        """Convert an MGeo-local point to simulator-local ENU XYZ in metres."""

        return mgeo_local_to_sim_local(
            point,
            self._mgeo_origin_utm,
            self._simulator_origin_utm,
        )

    def utm_to_wgs84(self, point: Sequence[float]) -> GeodeticPoint:
        """Convert a two- or three-dimensional UTM point to latitude/longitude."""

        easting, northing, _ = _as_xyz(point, "point")
        return utm_to_wgs84(
            easting,
            northing,
            zone_number=self._utm_zone,
            northern_hemisphere=self._northern,
        )
