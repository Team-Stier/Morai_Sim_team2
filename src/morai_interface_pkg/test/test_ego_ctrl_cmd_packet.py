# -*- coding: utf-8 -*-
"""EgoCtrlCmd packer 단위 테스트. ROS 없이 실행 가능."""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from morai_interface_pkg.protocol import ego_ctrl_cmd_packet as pkt  # noqa: E402


class EgoCtrlCmdPackTest(unittest.TestCase):
    def test_size_header_tail_datalength(self):
        packet = pkt.build_ego_ctrl_cmd_packet(
            cmd_type=1, velocity=0.0, acceleration=0.0, accel=0.3,
            brake=0.0, steer=0.1, ctrl_mode=0, gear=0)
        self.assertEqual(len(packet), pkt.EGO_CTRL_CMD_SIZE)
        self.assertEqual(len(packet), 55)
        self.assertEqual(packet[0:14], b"#MoraiCtrlCmd$")
        self.assertEqual(packet[-2:], b"\r\n")
        # data_lenght(int, off14) == 23
        self.assertEqual(struct.unpack_from("<i", packet, 14)[0], 23)
        # aux_data(off18) == 0,0,0
        self.assertEqual(struct.unpack_from("<3i", packet, 18), (0, 0, 0))

    def test_field_offsets_little_endian(self):
        packet = pkt.build_ego_ctrl_cmd_packet(
            cmd_type=1, velocity=2.0, acceleration=3.0, accel=0.4,
            brake=0.5, steer=-0.25, ctrl_mode=2, gear=4)
        self.assertEqual(struct.unpack_from("<b", packet, 30)[0], 2)   # ctrl_mode
        self.assertEqual(struct.unpack_from("<b", packet, 31)[0], 4)   # gear
        self.assertEqual(struct.unpack_from("<b", packet, 32)[0], 1)   # cmd_type
        self.assertAlmostEqual(struct.unpack_from("<f", packet, 33)[0], 2.0, places=5)
        self.assertAlmostEqual(struct.unpack_from("<f", packet, 37)[0], 3.0, places=5)
        self.assertAlmostEqual(struct.unpack_from("<f", packet, 41)[0], 0.4, places=5)
        self.assertAlmostEqual(struct.unpack_from("<f", packet, 45)[0], 0.5, places=5)
        self.assertAlmostEqual(struct.unpack_from("<f", packet, 49)[0], -0.25, places=5)

    def test_pack_parse_roundtrip(self):
        packet = pkt.build_ego_ctrl_cmd_packet(
            cmd_type=1, velocity=5.0, acceleration=1.5, accel=0.2,
            brake=0.1, steer=0.05, ctrl_mode=1, gear=2)
        fields = pkt.parse_ego_ctrl_cmd_packet(packet)
        self.assertEqual(fields.cmd_type, 1)
        self.assertEqual(fields.ctrl_mode, 1)
        self.assertEqual(fields.gear, 2)
        self.assertAlmostEqual(fields.accel, 0.2, places=5)
        self.assertAlmostEqual(fields.brake, 0.1, places=5)
        self.assertAlmostEqual(fields.steer, 0.05, places=5)

    def test_default_ctrl_mode_and_gear_are_automode_drive(self):
        # 기본값(공식 예제): ctrl_mode=2(AutoMode, off30), gear=4(Drive, off31).
        packet = pkt.build_ego_ctrl_cmd_packet(
            cmd_type=1, velocity=0.0, acceleration=0.0, accel=0.0,
            brake=0.0, steer=0.0)
        self.assertEqual(struct.unpack_from("<b", packet, 30)[0], 2)  # ctrl_mode
        self.assertEqual(struct.unpack_from("<b", packet, 31)[0], 4)  # gear
        fields = pkt.parse_ego_ctrl_cmd_packet(packet)
        self.assertEqual(fields.ctrl_mode, 2)
        self.assertEqual(fields.gear, 4)

    def test_ctrl_mode_gear_override_values(self):
        # YAML override 상당: 다른 ctrl_mode/gear 값이 정확한 offset에 반영되는지.
        packet = pkt.build_ego_ctrl_cmd_packet(
            cmd_type=1, velocity=0.0, acceleration=0.0, accel=0.0,
            brake=0.0, steer=0.0, ctrl_mode=1, gear=1)
        self.assertEqual(struct.unpack_from("<b", packet, 30)[0], 1)  # Keyboard
        self.assertEqual(struct.unpack_from("<b", packet, 31)[0], 1)  # Park

    def test_cmd_type_out_of_int8_raises(self):
        with self.assertRaises(pkt.EgoCtrlCmdPackError):
            pkt.build_ego_ctrl_cmd_packet(
                cmd_type=200, velocity=0, acceleration=0, accel=0,
                brake=0, steer=0)

    def test_parse_bad_length_raises(self):
        with self.assertRaises(pkt.EgoCtrlCmdPackError):
            pkt.parse_ego_ctrl_cmd_packet(b"\x00" * 10)

    def test_parse_bad_header_raises(self):
        packet = bytearray(pkt.build_ego_ctrl_cmd_packet(
            cmd_type=1, velocity=0, acceleration=0, accel=0, brake=0, steer=0))
        packet[0:1] = b"X"
        with self.assertRaises(pkt.EgoCtrlCmdPackError):
            pkt.parse_ego_ctrl_cmd_packet(bytes(packet))

    def test_parse_bad_data_length_raises(self):
        # data_lenght(off14)는 공식 정의로 23이 확인됨 -> 검증.
        packet = bytearray(pkt.build_ego_ctrl_cmd_packet(
            cmd_type=1, velocity=0, acceleration=0, accel=0, brake=0, steer=0))
        struct.pack_into("<i", packet, 14, 99)
        with self.assertRaises(pkt.EgoCtrlCmdPackError):
            pkt.parse_ego_ctrl_cmd_packet(bytes(packet))

    def test_parse_bad_aux_raises(self):
        # aux_data(off18)는 공식 정의로 (0,0,0)이 확인됨 -> 검증.
        packet = bytearray(pkt.build_ego_ctrl_cmd_packet(
            cmd_type=1, velocity=0, acceleration=0, accel=0, brake=0, steer=0))
        struct.pack_into("<i", packet, 18, 7)
        with self.assertRaises(pkt.EgoCtrlCmdPackError):
            pkt.parse_ego_ctrl_cmd_packet(bytes(packet))


if __name__ == "__main__":
    unittest.main()

