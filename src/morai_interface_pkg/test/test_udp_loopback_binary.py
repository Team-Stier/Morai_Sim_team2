# -*- coding: utf-8 -*-
"""
IMU / EgoVehicleStatus 바이너리 패킷에 대한 UDP 수신기+파서 loopback 테스트.
ROS 불필요. 가짜 송신(소켓)→UdpReceiver→파서 경로를 검증한다.
"""

import os
import socket
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from morai_interface_pkg.protocol import imu_packet          # noqa: E402
from morai_interface_pkg.protocol import ego_status_packet   # noqa: E402
from morai_interface_pkg.udp_receiver import UdpReceiver     # noqa: E402


class BinaryUdpLoopbackTest(unittest.TestCase):
    def setUp(self):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        self.port = probe.getsockname()[1]
        probe.close()
        self.receiver = UdpReceiver("127.0.0.1", self.port, timeout_sec=1.0)
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.receiver.close()
        self.sender.close()

    def _send(self, payload):
        self.sender.sendto(payload, ("127.0.0.1", self.port))

    def test_imu_loopback(self):
        self._send(imu_packet.build_imu_packet(
            sec=1, nsec=2, ori=(1.0, 0.0, 0.0, 0.0), ang_vel=(0.0, 0.0, 0.3)))
        data, _ = self.receiver.receive()
        reading = imu_packet.parse_imu_packet(data)
        self.assertAlmostEqual(reading.ang_vel_z, 0.3, places=6)

    def test_ego_status_loopback(self):
        self._send(ego_status_packet.build_ego_status_packet(vel=(2.5, 0.0, 0.0)))
        data, _ = self.receiver.receive()
        reading = ego_status_packet.parse_ego_status_packet(data)
        self.assertAlmostEqual(reading.vel_x, 2.5, places=4)

    def test_corrupt_short_packet_raises_not_crash(self):
        self._send(b"\x01\x02\x03")
        data, _ = self.receiver.receive()
        with self.assertRaises(imu_packet.ImuParseError):
            imu_packet.parse_imu_packet(data)


if __name__ == "__main__":
    unittest.main()

