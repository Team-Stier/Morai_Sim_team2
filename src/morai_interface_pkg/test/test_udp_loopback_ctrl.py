# -*- coding: utf-8 -*-
"""
Ctrl Cmd 송신 경로 loopback 테스트(ROS 불필요).
UdpSender로 EgoCtrlCmd 패킷을 보내 UdpReceiver로 받고 파싱해 값이 일치하는지 확인한다.
"""

import os
import socket
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from morai_interface_pkg.protocol import ego_ctrl_cmd_packet as pkt   # noqa: E402
from morai_interface_pkg.udp_receiver import UdpReceiver              # noqa: E402
from morai_interface_pkg.udp_sender import UdpSender                 # noqa: E402


class CtrlCmdUdpLoopbackTest(unittest.TestCase):
    def setUp(self):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        self.port = probe.getsockname()[1]
        probe.close()
        self.receiver = UdpReceiver("127.0.0.1", self.port, timeout_sec=1.0)
        self.sender = UdpSender("127.0.0.1", self.port)

    def tearDown(self):
        self.receiver.close()
        self.sender.close()

    def test_send_receive_parse(self):
        packet = pkt.build_ego_ctrl_cmd_packet(
            cmd_type=1, velocity=4.0, acceleration=0.0, accel=0.6,
            brake=0.0, steer=-0.2, ctrl_mode=1, gear=4)
        sent = self.sender.send(packet)
        self.assertEqual(sent, pkt.EGO_CTRL_CMD_SIZE)

        received = self.receiver.receive()
        self.assertIsNotNone(received, "datagram 수신 실패")
        data, _ = received
        fields = pkt.parse_ego_ctrl_cmd_packet(data)
        self.assertEqual(fields.cmd_type, 1)
        self.assertEqual(fields.gear, 4)
        self.assertAlmostEqual(fields.accel, 0.6, places=5)
        self.assertAlmostEqual(fields.steer, -0.2, places=5)

    def test_default_ctrl_mode_gear_over_udp(self):
        # 기본값(AutoMode=2, Drive=4)이 UDP 왕복 후에도 정확한지.
        packet = pkt.build_ego_ctrl_cmd_packet(
            cmd_type=1, velocity=0.0, acceleration=0.0, accel=0.1,
            brake=0.0, steer=0.0)
        self.sender.send(packet)
        data, _ = self.receiver.receive()
        fields = pkt.parse_ego_ctrl_cmd_packet(data)
        self.assertEqual(fields.ctrl_mode, 2)
        self.assertEqual(fields.gear, 4)


if __name__ == "__main__":
    unittest.main()

