# -*- coding: utf-8 -*-
"""
UDP 수신기 + NMEA 파서 loopback 테스트(ROS 불필요).
가짜 송신기가 localhost로 보낸 NMEA를 UdpReceiver로 받아 파서에 통과시킨다.
"""

import os
import socket
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from morai_udp_bridge.protocol import nmea_gps          # noqa: E402
from morai_udp_bridge.udp_receiver import UdpReceiver   # noqa: E402


class UdpLoopbackTest(unittest.TestCase):
    def setUp(self):
        # 임시 포트를 하나 잡아 재사용(하드코딩 회피).
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        self.port = probe.getsockname()[1]
        probe.close()
        self.receiver = UdpReceiver("127.0.0.1", self.port,
                                    buffer_bytes=2048, timeout_sec=1.0)
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.receiver.close()
        self.sender.close()

    def _send(self, text):
        self.sender.sendto(text.encode("ascii"), ("127.0.0.1", self.port))

    def test_receive_and_parse_gga(self):
        self._send(nmea_gps.build_gga(37.3874583, 126.7751283, 28.5))
        received = self.receiver.receive()
        self.assertIsNotNone(received, "datagram을 수신하지 못했다")
        data, _ = received
        fix = nmea_gps.parse_nmea_sentence(data.decode("ascii"))
        self.assertAlmostEqual(fix.latitude, 37.3874583, places=4)
        self.assertAlmostEqual(fix.altitude, 28.5, places=2)
        self.assertEqual(fix.status, nmea_gps.STATUS_FIX)

    def test_receive_and_parse_rmc(self):
        self._send(nmea_gps.build_rmc(37.3874583, 126.7751283))
        data, _ = self.receiver.receive()
        fix = nmea_gps.parse_nmea_sentence(data.decode("ascii"))
        self.assertEqual(fix.sentence_type, "RMC")
        self.assertAlmostEqual(fix.longitude, 126.7751283, places=4)

    def test_timeout_returns_none(self):
        # 아무것도 보내지 않으면 timeout으로 None을 반환해야 한다.
        self.assertIsNone(self.receiver.receive())


if __name__ == "__main__":
    unittest.main()
