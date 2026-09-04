# -*- coding: utf-8 -*-
"""GPS NMEA 파서 단위 테스트. ROS 없이 python3 -m unittest로 실행 가능."""

import os
import sys
import unittest

# 빌드 전에도 실행되도록 패키지 src 경로 추가.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from morai_interface_pkg.protocol import nmea_gps  # noqa: E402


class ChecksumTest(unittest.TestCase):
    def test_known_checksum(self):
        # 널리 알려진 GGA 예제 문장의 checksum은 0x47.
        payload = ("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,"
                   "46.9,M,,")
        self.assertEqual(nmea_gps.nmea_checksum(payload), 0x47)

    def test_format_and_roundtrip_checksum(self):
        sentence = nmea_gps.format_sentence("GPGGA,1,2,3")
        self.assertTrue(sentence.startswith("$GPGGA,1,2,3*"))
        # 만든 문장은 checksum 검증을 통과해야 한다.
        nmea_gps.parse_nmea_sentence(  # 예외가 없으면 통과(미지원 아님)
            nmea_gps.build_gga(37.0, 126.0, 10.0))


class GgaParseTest(unittest.TestCase):
    def test_gga_basic_ne(self):
        fix = nmea_gps.parse_nmea_sentence(
            nmea_gps.build_gga(37.3874583, 126.7751283, 28.5, quality=1))
        self.assertEqual(fix.sentence_type, "GGA")
        self.assertAlmostEqual(fix.latitude, 37.3874583, places=4)
        self.assertAlmostEqual(fix.longitude, 126.7751283, places=4)
        self.assertAlmostEqual(fix.altitude, 28.5, places=2)
        self.assertEqual(fix.status, nmea_gps.STATUS_FIX)
        self.assertTrue(fix.has_position)

    def test_gga_southern_western_hemisphere_sign(self):
        fix = nmea_gps.parse_nmea_sentence(
            nmea_gps.build_gga(-33.8688, -151.2093, 5.0))
        self.assertLess(fix.latitude, 0.0)
        self.assertLess(fix.longitude, 0.0)
        self.assertAlmostEqual(fix.latitude, -33.8688, places=3)
        self.assertAlmostEqual(fix.longitude, -151.2093, places=3)

    def test_gga_quality_zero_is_no_fix(self):
        fix = nmea_gps.parse_nmea_sentence(
            nmea_gps.build_gga(37.0, 126.0, 10.0, quality=0))
        self.assertEqual(fix.status, nmea_gps.STATUS_NO_FIX)

    def test_gga_dgps_quality_is_gbas(self):
        fix = nmea_gps.parse_nmea_sentence(
            nmea_gps.build_gga(37.0, 126.0, 10.0, quality=2))
        self.assertEqual(fix.status, nmea_gps.STATUS_GBAS_FIX)


class RmcParseTest(unittest.TestCase):
    def test_rmc_active(self):
        fix = nmea_gps.parse_nmea_sentence(
            nmea_gps.build_rmc(37.3874583, 126.7751283, status="A"))
        self.assertEqual(fix.sentence_type, "RMC")
        self.assertAlmostEqual(fix.latitude, 37.3874583, places=4)
        self.assertIsNone(fix.altitude)  # RMC에는 고도가 없다
        self.assertEqual(fix.status, nmea_gps.STATUS_FIX)

    def test_rmc_void_is_no_fix(self):
        fix = nmea_gps.parse_nmea_sentence(
            nmea_gps.build_rmc(37.0, 126.0, status="V"))
        self.assertEqual(fix.status, nmea_gps.STATUS_NO_FIX)


class MalformedTest(unittest.TestCase):
    def test_no_dollar(self):
        with self.assertRaises(nmea_gps.NmeaParseError):
            nmea_gps.parse_nmea_sentence("GPGGA,1,2,3*00")

    def test_bad_checksum(self):
        good = nmea_gps.build_gga(37.0, 126.0, 10.0)
        bad = good[:-2] + "00"  # checksum 두 자리를 00으로 훼손
        with self.assertRaises(nmea_gps.NmeaParseError):
            nmea_gps.parse_nmea_sentence(bad)

    def test_unsupported_sentence(self):
        with self.assertRaises(nmea_gps.NmeaParseError):
            nmea_gps.parse_nmea_sentence(nmea_gps.format_sentence("GPGSV,1,2,3"))

    def test_missing_checksum_when_required(self):
        with self.assertRaises(nmea_gps.NmeaParseError):
            nmea_gps.parse_nmea_sentence("$GPGGA,1,2,3", require_checksum=True)

    def test_garbage_does_not_crash(self):
        for junk in ("", "$", "$*", "\x00\x00", "$GPGGA,,,,,,,,,,,,,"):
            with self.assertRaises(nmea_gps.NmeaParseError):
                nmea_gps.parse_nmea_sentence(junk)

    def test_empty_position_gga_is_no_fix_not_crash(self):
        # 위치 필드가 비어도 예외 없이 no-fix로 처리되어야 한다.
        sentence = nmea_gps.format_sentence("GPGGA,123519,,,,,0,00,,,M,,M,,")
        fix = nmea_gps.parse_nmea_sentence(sentence)
        self.assertFalse(fix.has_position)
        self.assertEqual(fix.status, nmea_gps.STATUS_NO_FIX)


if __name__ == "__main__":
    unittest.main()

