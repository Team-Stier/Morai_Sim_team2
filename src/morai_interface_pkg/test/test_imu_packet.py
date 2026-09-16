# -*- coding: utf-8 -*-
"""IMU 바이너리 패킷 파서 단위 테스트. ROS 없이 실행 가능."""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from morai_udp_bridge.protocol import imu_packet  # noqa: E402


class ImuPacketTest(unittest.TestCase):
    def test_packet_size_constant(self):
        packet = imu_packet.build_imu_packet()
        self.assertEqual(len(packet), imu_packet.IMU_PACKET_SIZE)
        self.assertEqual(len(packet), 115)

    def test_roundtrip_fields(self):
        packet = imu_packet.build_imu_packet(
            sec=12, nsec=345, ori=(0.7071, 0.0, 0.0, 0.7071),
            ang_vel=(0.1, 0.2, 0.3), lin_acc=(1.0, 2.0, 9.81))
        reading = imu_packet.parse_imu_packet(packet)
        self.assertEqual(reading.sec, 12)
        self.assertEqual(reading.nsec, 345)
        self.assertAlmostEqual(reading.ori_w, 0.7071, places=6)
        self.assertAlmostEqual(reading.ori_z, 0.7071, places=6)
        self.assertAlmostEqual(reading.ang_vel_x, 0.1, places=6)
        self.assertAlmostEqual(reading.ang_vel_z, 0.3, places=6)
        self.assertAlmostEqual(reading.lin_acc_z, 9.81, places=6)

    def test_orientation_order_is_wxyz(self):
        # W,X,Y,Z 순서 확인: 서로 다른 값으로 자리 검증.
        reading = imu_packet.parse_imu_packet(
            imu_packet.build_imu_packet(ori=(1.0, 2.0, 3.0, 4.0)))
        self.assertEqual(
            (reading.ori_w, reading.ori_x, reading.ori_y, reading.ori_z),
            (1.0, 2.0, 3.0, 4.0))

    def test_little_endian_offset_matches_definition(self):
        # 정의(off 33 = ori_w double)와 실제 언패킹 위치가 일치하는지 직접 확인.
        packet = bytearray(imu_packet.build_imu_packet())
        struct.pack_into("<d", packet, 33, 0.5)  # ori_w
        reading = imu_packet.parse_imu_packet(bytes(packet))
        self.assertAlmostEqual(reading.ori_w, 0.5, places=9)

    def test_short_packet_raises(self):
        with self.assertRaises(imu_packet.ImuParseError):
            imu_packet.parse_imu_packet(b"\x00" * 50)

    def test_wrong_length_raises_exact(self):
        # 정확 길이 검증: 초과 길이도 거부한다.
        with self.assertRaises(imu_packet.ImuParseError):
            imu_packet.parse_imu_packet(b"\x00" * (imu_packet.IMU_PACKET_SIZE + 1))

    def test_none_raises(self):
        with self.assertRaises(imu_packet.ImuParseError):
            imu_packet.parse_imu_packet(None)


if __name__ == "__main__":
    unittest.main()
