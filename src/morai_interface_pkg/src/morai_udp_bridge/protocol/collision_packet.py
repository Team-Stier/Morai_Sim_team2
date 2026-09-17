# -*- coding: utf-8 -*-
"""MORAI 24.R2.0 CollisionData UDP packet parser."""

import struct

COLLISION_PACKET_SIZE = 181
COLLISION_SLOT_COUNT = 5
COLLISION_SLOT_SIZE = 28

_OFF_DATA_LENGTH = 15
_OFF_SEC = 31
_OFF_NSEC = 35
_OFF_DATA = 39


class CollisionPacketParseError(Exception):
    pass


class CollisionSlot(object):
    __slots__ = (
        "obj_type", "obj_id",
        "pose_x", "pose_y", "pose_z",
        "global_offset_x", "global_offset_y", "global_offset_z",
    )

    def __init__(self, obj_type, obj_id, pose, global_offset):
        self.obj_type = obj_type
        self.obj_id = obj_id
        self.pose_x, self.pose_y, self.pose_z = pose
        self.global_offset_x, self.global_offset_y, self.global_offset_z = global_offset

    @property
    def has_payload(self):
        values = (
            self.obj_type, self.obj_id,
            self.pose_x, self.pose_y, self.pose_z,
            self.global_offset_x, self.global_offset_y, self.global_offset_z,
        )
        return any(value != 0 for value in values)


class CollisionPacket(object):
    __slots__ = ("data_length", "sec", "nsec", "slots")

    def __init__(self, data_length, sec, nsec, slots):
        self.data_length = data_length
        self.sec = sec
        self.nsec = nsec
        self.slots = list(slots)

    @property
    def active_slots(self):
        # MORAI 예제는 고정 5칸의 별도 active-count를 정의하지 않는다.
        # non-zero slot 판정은 live packet 확인 전 개발용 probe 정책이다.
        return [slot for slot in self.slots if slot.has_payload]


def parse_collision_packet(data):
    if data is None or len(data) != COLLISION_PACKET_SIZE:
        raise CollisionPacketParseError(
            "CollisionData 패킷 길이 오류: %d != %d"
            % (0 if data is None else len(data), COLLISION_PACKET_SIZE)
        )
    try:
        data_length = struct.unpack_from("<i", data, _OFF_DATA_LENGTH)[0]
        sec = struct.unpack_from("<i", data, _OFF_SEC)[0]
        nsec = struct.unpack_from("<i", data, _OFF_NSEC)[0]
        slots = []
        for index in range(COLLISION_SLOT_COUNT):
            offset = _OFF_DATA + index * COLLISION_SLOT_SIZE
            obj_type, obj_id = struct.unpack_from("<hh", data, offset)
            pose = struct.unpack_from("<3f", data, offset + 4)
            global_offset = struct.unpack_from("<3f", data, offset + 16)
            slots.append(CollisionSlot(obj_type, obj_id, pose, global_offset))
    except struct.error as error:
        raise CollisionPacketParseError(
            "CollisionData struct 언패킹 실패: %s" % error
        )
    if nsec < 0 or nsec >= 1000000000:
        raise CollisionPacketParseError("CollisionData nsec 범위 오류: %d" % nsec)
    return CollisionPacket(data_length, sec, nsec, slots)


def build_collision_packet(sec=0, nsec=0, slots=None, data_length=0):
    """테스트용 181-byte CollisionData packet을 만든다."""
    packet = bytearray(COLLISION_PACKET_SIZE)
    struct.pack_into("<i", packet, _OFF_DATA_LENGTH, int(data_length))
    struct.pack_into("<i", packet, _OFF_SEC, int(sec))
    struct.pack_into("<i", packet, _OFF_NSEC, int(nsec))
    slots = list(slots or [])
    for index, values in enumerate(slots[:COLLISION_SLOT_COUNT]):
        offset = _OFF_DATA + index * COLLISION_SLOT_SIZE
        obj_type, obj_id, pose, global_offset = values
        struct.pack_into("<hh", packet, offset, int(obj_type), int(obj_id))
        struct.pack_into("<3f", packet, offset + 4, *pose)
        struct.pack_into("<3f", packet, offset + 16, *global_offset)
    return bytes(packet)
