# -*- coding: utf-8 -*-
"""
Camera 청크에 대한 UDP 수신기+재조립기 loopback 테스트(ROS 불필요).
가짜 송신(소켓)으로 여러 MOR 청크를 보내 UdpReceiver로 받고 프레임을 완성한다.
"""

import os
import socket
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from morai_interface_pkg.protocol import camera_packet          # noqa: E402
from morai_interface_pkg.udp_receiver import UdpReceiver        # noqa: E402


class CameraUdpLoopbackTest(unittest.TestCase):
    def setUp(self):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        self.port = probe.getsockname()[1]
        probe.close()
        self.receiver = UdpReceiver("127.0.0.1", self.port,
                                    buffer_bytes=65536, timeout_sec=1.0)
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.assembler = camera_packet.JpegFrameAssembler()

    def tearDown(self):
        self.receiver.close()
        self.sender.close()

    def test_multi_chunk_frame_over_udp(self):
        jpeg = b"\xff\xd8" + (bytes(range(256)) * 30) + b"\xff\xd9"  # ~7.7KB
        pieces = [jpeg[i:i + 2000] for i in range(0, len(jpeg), 2000)]
        for index, piece in enumerate(pieces):
            packet = camera_packet.build_camera_packet(
                jpeg_chunk=piece, index=index, size=len(jpeg),
                is_end=(index == len(pieces) - 1))
            self.sender.sendto(packet, ("127.0.0.1", self.port))

        frame = None
        for _ in range(len(pieces)):
            received = self.receiver.receive()
            self.assertIsNotNone(received, "청크 datagram 수신 실패")
            data, _ = received
            _, chunk = camera_packet.parse_camera_packet(data)
            result = self.assembler.add_chunk(chunk, 0.0)
            if result is not None:
                frame, _, _ = result
        self.assertEqual(frame, jpeg)
        self.assertEqual(self.assembler.completed_frames, 1)


if __name__ == "__main__":
    unittest.main()

