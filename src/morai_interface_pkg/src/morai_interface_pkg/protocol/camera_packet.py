# -*- coding: utf-8 -*-
"""
camera_packet.py
- 역할: MORAI Camera UDP 청크를 파싱하고, 여러 MOR 청크를 하나의 JPEG 프레임으로
        재조립한다. ROS에 의존하지 않으므로 단위 테스트가 가능하다.
- 근거: MORAI-NetworkModule lib/define/Camera.py.
    * 패킷 = header(char[3]) + data(byte[64997]), 총 65000바이트.
    * header 'MOR' = 이미지 청크, 'BOX' = bounding box(이번 단계에서는 무시).
    * MOR data(=IMAGE) 레이아웃: int sec, int nsec, int index, int size,
      byte jpeg_data[64979], char tail[2].
    * 프레임 종료 조건(공식 예제): tail == 'EI' 이면 마지막 청크.
- 재조립은 각 MOR 청크의 jpeg_data를 이어 붙이고, 완료 시 JPEG EOI(0xFFD9)까지
  잘라 완성된 JPEG bytes를 만든다(padding 제거). MORAI의 'size' 필드 의미는
  자료로 확정되지 않아 사용하지 않고 참고용으로만 노출한다.
"""

import struct

HEADER_LEN = 3
HEADER_MOR = b"MOR"
HEADER_BOX = b"BOX"
TAIL_END = b"EI"                 # 마지막 청크 표시
_META_OFFSET = HEADER_LEN        # sec,nsec,index,size 시작 위치(=3)
_META_STRUCT = "<4i"             # int x4 (little-endian)
_META_SIZE = 16
_JPEG_START = _META_OFFSET + _META_SIZE   # =19
_TAIL_LEN = 2
_JPEG_EOI = b"\xff\xd9"


class CameraParseError(Exception):
    """Camera 패킷을 해석할 수 없을 때 발생한다."""
    pass


class ImageChunk(object):
    """한 개의 MOR(이미지) 청크."""

    __slots__ = ("sec", "nsec", "index", "size", "jpeg_data", "is_end")

    def __init__(self, sec, nsec, index, size, jpeg_data, is_end):
        self.sec = sec
        self.nsec = nsec
        self.index = index
        self.size = size          # MORAI 필드(의미 미확정, 참고용)
        self.jpeg_data = jpeg_data
        self.is_end = is_end


# 함수이름: parse_camera_packet
# 기능: Camera UDP 패킷을 (kind, ImageChunk) 로 파싱한다.
# 인자: data - 수신 bytes
# 반환값: ('MOR', ImageChunk) | ('BOX', None)
# 예외: CameraParseError (길이 부족, 알 수 없는 header 등)
def parse_camera_packet(data):
    if data is None or len(data) < HEADER_LEN:
        raise CameraParseError("패킷 길이 부족")
    header = bytes(data[0:HEADER_LEN])
    if header == HEADER_BOX:
        return ("BOX", None)
    if header != HEADER_MOR:
        raise CameraParseError("알 수 없는 header: %r" % header)
    if len(data) < _JPEG_START + _TAIL_LEN:
        raise CameraParseError("MOR 패킷 길이 부족: %d" % len(data))
    sec, nsec, index, size = struct.unpack_from(_META_STRUCT, data, _META_OFFSET)
    jpeg_data = bytes(data[_JPEG_START:-_TAIL_LEN])
    tail = bytes(data[-_TAIL_LEN:])
    is_end = tail == TAIL_END
    return ("MOR", ImageChunk(sec, nsec, index, size, jpeg_data, is_end))


# 함수이름: _trim_to_jpeg_eoi
# 기능: JPEG EOI(0xFFD9) 마지막 위치까지 잘라 padding을 제거한다.
def _trim_to_jpeg_eoi(data):
    marker = data.rfind(_JPEG_EOI)
    if marker != -1:
        return data[:marker + len(_JPEG_EOI)]
    return data


class JpegFrameAssembler(object):
    """MOR 청크들을 이어 붙여 완성된 JPEG 프레임을 만든다.

    유실·순서 오류·크기 초과·timeout이 발생하면 해당 프레임만 폐기하고
    다음 청크부터 새 프레임을 시작한다(카운터로 관찰 가능).
    """

    def __init__(self, max_frame_bytes=2000000, frame_timeout_sec=0.5,
                 check_chunk_index=False):
        self.max_frame_bytes = int(max_frame_bytes)
        self.frame_timeout_sec = float(frame_timeout_sec)
        self.check_chunk_index = bool(check_chunk_index)

        self.received_chunks = 0
        self.completed_frames = 0
        self.discarded_frames = 0
        self.last_discard_reason = None

        self._buffer = bytearray()
        self._started = False
        self._poisoned = False           # 이상 발생 프레임: EI에서 폐기 예정
        self._poison_reason = None
        self._frame_start_time = 0.0
        self._last_index = 0
        self._frame_sec = 0
        self._frame_nsec = 0

    def _reset(self):
        self._buffer = bytearray()
        self._started = False
        self._poisoned = False
        self._poison_reason = None

    def _begin(self, chunk, now):
        self._buffer = bytearray(chunk.jpeg_data)
        self._started = True
        self._poisoned = len(chunk.jpeg_data) > self.max_frame_bytes
        self._poison_reason = "oversize" if self._poisoned else None
        self._frame_start_time = now
        self._last_index = chunk.index
        self._frame_sec = chunk.sec
        self._frame_nsec = chunk.nsec

    def _poison(self, reason):
        self._poisoned = True
        self._poison_reason = reason

    def _discard_current(self, reason):
        if self._started:
            self.discarded_frames += 1
            self.last_discard_reason = reason
        self._reset()

    # 함수이름: add_chunk
    # 기능: MOR 청크 하나를 재조립에 반영한다. 프레임이 완성되면
    #       (jpeg_bytes, sec, nsec)를 반환하고, 아니면 None.
    #       유실/순서오류/크기초과가 발생한 프레임은 EI 시점에 통째로 폐기한다
    #       (잘린 프레임을 발행하지 않는다). timeout은 새 청크 도착 시 판정한다.
    # 인자: chunk - ImageChunk, now - 단조 시각(초, 프레임 timeout 판정용)
    def add_chunk(self, chunk, now):
        self.received_chunks += 1

        if self._started and (now - self._frame_start_time) > self.frame_timeout_sec:
            self._discard_current("timeout")

        if not self._started:
            self._begin(chunk, now)
        else:
            if self.check_chunk_index and chunk.index != self._last_index + 1:
                self._poison("out_of_order")
            self._last_index = chunk.index
            if len(self._buffer) + len(chunk.jpeg_data) > self.max_frame_bytes:
                self._poison("oversize")  # 메모리 보호: 더 이상 누적하지 않음
            elif not self._poisoned:
                self._buffer += chunk.jpeg_data

        if self._started and chunk.is_end:
            if self._poisoned:
                reason = self._poison_reason
                self._discard_current(reason)
                return None
            frame = _trim_to_jpeg_eoi(bytes(self._buffer))
            sec, nsec = self._frame_sec, self._frame_nsec
            self.completed_frames += 1
            self._reset()
            return frame, sec, nsec
        return None


# 함수이름: build_camera_packet
# 기능: 테스트/모의 송신용 MOR(또는 BOX) 카메라 패킷을 만든다.
# 인자: jpeg_chunk - 이 청크의 jpeg bytes, is_end - 마지막 청크 여부
def build_camera_packet(jpeg_chunk=b"", sec=0, nsec=0, index=0, size=0,
                        is_end=False, header=HEADER_MOR):
    if header == HEADER_BOX:
        return HEADER_BOX + bytes(64997)
    tail = TAIL_END if is_end else b"MI"
    meta = struct.pack(_META_STRUCT, sec, nsec, index, size)
    return HEADER_MOR + meta + bytes(jpeg_chunk) + tail

