# -*- coding: utf-8 -*-
"""Camera 청크 파서 + JPEG 재조립기 단위 테스트. ROS 없이 실행 가능."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from morai_udp_bridge.protocol import camera_packet  # noqa: E402


def make_jpeg(payload):
    return b"\xff\xd8" + payload + b"\xff\xd9"


def split_to_packets(jpeg, chunk_bytes, sec=0, nsec=0):
    """jpeg를 chunk_bytes 크기로 나눠 MOR 패킷 리스트로 만든다(마지막에 EI)."""
    pieces = [jpeg[i:i + chunk_bytes] for i in range(0, len(jpeg), chunk_bytes)]
    packets = []
    for index, piece in enumerate(pieces):
        packets.append(camera_packet.build_camera_packet(
            jpeg_chunk=piece, sec=sec, nsec=nsec, index=index,
            size=len(jpeg), is_end=(index == len(pieces) - 1)))
    return packets


class CameraParseTest(unittest.TestCase):
    def test_parse_mor_chunk(self):
        packet = camera_packet.build_camera_packet(
            jpeg_chunk=b"ABCD", sec=3, nsec=4, index=2, is_end=True)
        kind, chunk = camera_packet.parse_camera_packet(packet)
        self.assertEqual(kind, "MOR")
        self.assertEqual(chunk.sec, 3)
        self.assertEqual(chunk.index, 2)
        self.assertEqual(chunk.jpeg_data, b"ABCD")
        self.assertTrue(chunk.is_end)

    def test_box_is_ignored_kind(self):
        packet = camera_packet.build_camera_packet(header=camera_packet.HEADER_BOX)
        kind, chunk = camera_packet.parse_camera_packet(packet)
        self.assertEqual(kind, "BOX")
        self.assertIsNone(chunk)

    def test_bad_header_raises(self):
        with self.assertRaises(camera_packet.CameraParseError):
            camera_packet.parse_camera_packet(b"XYZ" + bytes(30))

    def test_too_short_raises(self):
        with self.assertRaises(camera_packet.CameraParseError):
            camera_packet.parse_camera_packet(b"MO")


class ReassemblyTest(unittest.TestCase):
    def setUp(self):
        self.assembler = camera_packet.JpegFrameAssembler(
            max_frame_bytes=1000000, frame_timeout_sec=1.0, check_chunk_index=True)

    def _feed(self, packets, now=0.0):
        result = None
        for packet in packets:
            _, chunk = camera_packet.parse_camera_packet(packet)
            result = self.assembler.add_chunk(chunk, now)
        return result

    def test_single_chunk_frame(self):
        jpeg = make_jpeg(b"hello")
        result = self._feed(split_to_packets(jpeg, 1000))
        self.assertIsNotNone(result)
        frame, _, _ = result
        self.assertEqual(frame, jpeg)
        self.assertEqual(self.assembler.completed_frames, 1)

    def test_multi_chunk_reassembly(self):
        jpeg = make_jpeg(bytes(range(256)) * 20)  # ~5KB
        frame, _, _ = self._feed(split_to_packets(jpeg, 1000))
        self.assertEqual(frame, jpeg)

    def test_trailing_padding_trimmed(self):
        # 마지막 청크에 EOI 뒤로 padding(0x00)이 붙어도 잘라내야 한다.
        jpeg = make_jpeg(b"payload")
        p1 = camera_packet.build_camera_packet(jpeg_chunk=jpeg + b"\x00\x00\x00",
                                               index=0, is_end=True)
        _, chunk = camera_packet.parse_camera_packet(p1)
        frame, _, _ = self.assembler.add_chunk(chunk, 0.0)
        self.assertEqual(frame, jpeg)  # padding 제거됨

    def test_packet_timestamp_propagated(self):
        jpeg = make_jpeg(b"ts")
        result = self._feed(split_to_packets(jpeg, 1000, sec=11, nsec=22))
        _, sec, nsec = result
        self.assertEqual((sec, nsec), (11, 22))

    def test_out_of_order_discards_frame(self):
        jpeg = make_jpeg(bytes(300))
        packets = split_to_packets(jpeg, 100)  # 여러 청크
        # index 1 청크를 건너뛰어 순서 오류 유발
        broken = [packets[0]] + packets[2:]
        result = self._feed(broken)
        self.assertGreaterEqual(self.assembler.discarded_frames, 1)
        self.assertEqual(self.assembler.last_discard_reason, "out_of_order")
        # 그래도 노드는 계속: 이어서 정상 프레임은 완성되어야 한다.
        self.assembler_reset_ok()

    def test_missing_first_chunk_discards_frame(self):
        jpeg = make_jpeg(bytes(300))
        packets = split_to_packets(jpeg, 100)
        result = self._feed(packets[1:])
        self.assertIsNone(result)
        self.assertEqual(self.assembler.last_discard_reason,
                         "missing_first_chunk")

    def test_new_zero_index_resynchronizes_after_missing_end(self):
        jpeg = make_jpeg(bytes(300))
        packets = split_to_packets(jpeg, 100)
        self._feed(packets[:-1])
        next_jpeg = make_jpeg(b"next")
        result = self._feed(split_to_packets(next_jpeg, 1000))
        self.assertEqual(result[0], next_jpeg)
        self.assertGreaterEqual(self.assembler.discarded_frames, 1)

    def test_invalid_jpeg_markers_are_discarded(self):
        packet = camera_packet.build_camera_packet(
            jpeg_chunk=b"not-a-jpeg", index=0, is_end=True)
        result = self._feed([packet])
        self.assertIsNone(result)
        self.assertEqual(self.assembler.last_discard_reason,
                         "invalid_jpeg_markers")

    def assembler_reset_ok(self):
        jpeg = make_jpeg(b"again")
        frame, _, _ = self._feed(split_to_packets(jpeg, 1000))
        self.assertEqual(frame, jpeg)

    def test_default_check_chunk_index_is_false(self):
        # 라이브 config는 true지만 라이브러리 기본값은 기존 호출자 호환을 위해 off.
        self.assertFalse(camera_packet.JpegFrameAssembler().check_chunk_index)

    def test_ei_only_default_completes_on_gap_no_discard(self):
        # 기본(off): index 불연속이어도 EI로 프레임을 종료하고 폐기하지 않는다.
        assembler = camera_packet.JpegFrameAssembler()  # check off
        jpeg = make_jpeg(bytes(300))
        packets = split_to_packets(jpeg, 100)
        broken = [packets[0]] + packets[2:]  # 중간 청크 누락
        for packet in broken:
            _, chunk = camera_packet.parse_camera_packet(packet)
            assembler.add_chunk(chunk, 0.0)
        self.assertEqual(assembler.discarded_frames, 0)
        self.assertEqual(assembler.completed_frames, 1)

    def test_oversize_discards_frame(self):
        small = camera_packet.JpegFrameAssembler(max_frame_bytes=10,
                                                 frame_timeout_sec=1.0)
        jpeg = make_jpeg(bytes(100))
        result = None
        for packet in split_to_packets(jpeg, 5):
            _, chunk = camera_packet.parse_camera_packet(packet)
            result = small.add_chunk(chunk, 0.0)
        self.assertGreaterEqual(small.discarded_frames, 1)
        self.assertEqual(small.last_discard_reason, "oversize")

    def test_timeout_discards_stale_frame(self):
        jpeg = make_jpeg(bytes(300))
        packets = split_to_packets(jpeg, 100)
        _, c0 = camera_packet.parse_camera_packet(packets[0])
        self.assembler.add_chunk(c0, now=0.0)      # 프레임 시작
        _, c1 = camera_packet.parse_camera_packet(packets[1])
        self.assembler.add_chunk(c1, now=5.0)      # timeout(>1.0s) -> 폐기 후 새 시작
        self.assertGreaterEqual(self.assembler.discarded_frames, 1)
        self.assertEqual(self.assembler.last_discard_reason, "timeout")


if __name__ == "__main__":
    unittest.main()
