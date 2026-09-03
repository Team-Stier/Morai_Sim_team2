# -*- coding: utf-8 -*-
"""EgoVehicleStatus 바이너리 패킷 파서 단위 테스트. ROS 없이 실행 가능."""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from morai_udp_bridge.protocol import ego_status_packet  # noqa: E402


class EgoStatusPacketTest(unittest.TestCase):
    def test_packet_size_constant(self):
        packet = ego_status_packet.build_ego_status_packet()
        self.assertEqual(len(packet), ego_status_packet.EGO_PACKET_SIZE)
        self.assertEqual(len(packet), 229)

    def test_roundtrip_fields(self):
        packet = ego_status_packet.build_ego_status_packet(
            sec=7, nsec=8, signed_vel=5.5,
            vel=(3.0, 0.5, 0.0), ang_vel=(0.0, 0.0, 0.2),
            pos=(10.0, 20.0, 1.0), yaw=1.23)
        reading = ego_status_packet.parse_ego_status_packet(packet)
        self.assertEqual(reading.sec, 7)
        self.assertEqual(reading.nsec, 8)
        self.assertAlmostEqual(reading.signed_vel, 5.5, places=4)
        self.assertAlmostEqual(reading.vel_x, 3.0, places=4)
        self.assertAlmostEqual(reading.vel_y, 0.5, places=4)
        self.assertAlmostEqual(reading.ang_vel_z, 0.2, places=4)
        self.assertAlmostEqual(reading.pos_x, 10.0, places=4)
        self.assertAlmostEqual(reading.yaw, 1.23, places=4)

    def test_offsets_match_definition(self):
        # 정의(off 101 = vel_x float, off 37 = signed_vel float) 위치 직접 확인.
        packet = bytearray(ego_status_packet.build_ego_status_packet())
        struct.pack_into("<f", packet, 101, 4.25)   # vel_x
        struct.pack_into("<f", packet, 37, 9.75)     # signed_vel
        reading = ego_status_packet.parse_ego_status_packet(bytes(packet))
        self.assertAlmostEqual(reading.vel_x, 4.25, places=4)
        self.assertAlmostEqual(reading.signed_vel, 9.75, places=4)

    def test_short_packet_raises(self):
        with self.assertRaises(ego_status_packet.EgoStatusParseError):
            ego_status_packet.parse_ego_status_packet(b"\x00" * 100)

    def test_wrong_length_raises_exact(self):
        # 정확 길이 검증: 초과 길이도 거부한다.
        with self.assertRaises(ego_status_packet.EgoStatusParseError):
            ego_status_packet.parse_ego_status_packet(
                b"\x00" * (ego_status_packet.EGO_PACKET_SIZE + 1))

    def test_none_raises(self):
        with self.assertRaises(ego_status_packet.EgoStatusParseError):
            ego_status_packet.parse_ego_status_packet(None)


if __name__ == "__main__":
    unittest.main()
