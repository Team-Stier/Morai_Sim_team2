# -*- coding: utf-8 -*-
"""MORAI 24.R2.0 EgoCtrlCmd serializer for competition mode."""

import math
import struct

EGO_CTRL_CMD_PACKET_SIZE = 55
EGO_CTRL_CMD_HEADER = b"#MoraiCtrlCmd$"
EGO_CTRL_CMD_DATA_LENGTH = 23
EGO_CTRL_CMD_TAIL = b"\r\n"
COMPETITION_CTRL_MODE = 2
COMPETITION_CMD_TYPE = 1
_FORMAT = "<14si3ibbbfffff2s"


class EgoCtrlCmdPacketError(ValueError):
    pass


def _finite_in_range(name, value, minimum, maximum):
    value = float(value)
    if not math.isfinite(value) or value < minimum or value > maximum:
        raise EgoCtrlCmdPacketError(
            "%s 범위 오류: %r (허용 %.3f..%.3f)"
            % (name, value, minimum, maximum)
        )
    return value


def serialize_ego_ctrl_cmd(gear, accel, brake, steer_normalized):
    gear = int(gear)
    if gear < 0 or gear > 5:
        raise EgoCtrlCmdPacketError("gear 범위 오류: %r (허용 0..5)" % gear)
    accel = _finite_in_range("accel", accel, 0.0, 1.0)
    brake = _finite_in_range("brake", brake, 0.0, 1.0)
    steer_normalized = _finite_in_range(
        "steer_normalized", steer_normalized, -1.0, 1.0
    )
    packet = struct.pack(
        _FORMAT,
        EGO_CTRL_CMD_HEADER,
        EGO_CTRL_CMD_DATA_LENGTH,
        0, 0, 0,
        COMPETITION_CTRL_MODE,
        gear,
        COMPETITION_CMD_TYPE,
        0.0,
        0.0,
        accel,
        brake,
        steer_normalized,
        EGO_CTRL_CMD_TAIL,
    )
    if len(packet) != EGO_CTRL_CMD_PACKET_SIZE:
        raise EgoCtrlCmdPacketError(
            "EgoCtrlCmd 직렬화 길이 오류: %d" % len(packet)
        )
    return packet


def parse_ego_ctrl_cmd(packet):
    """테스트에서 wire field를 확인하기 위한 역직렬화 helper."""
    if packet is None or len(packet) != EGO_CTRL_CMD_PACKET_SIZE:
        raise EgoCtrlCmdPacketError(
            "EgoCtrlCmd 패킷 길이 오류: %d != %d"
            % (0 if packet is None else len(packet), EGO_CTRL_CMD_PACKET_SIZE)
        )
    unpacked = struct.unpack(_FORMAT, packet)
    return {
        "header": unpacked[0],
        "data_length": unpacked[1],
        "aux_data": unpacked[2:5],
        "ctrl_mode": unpacked[5],
        "gear": unpacked[6],
        "cmd_type": unpacked[7],
        "velocity": unpacked[8],
        "acceleration": unpacked[9],
        "accel": unpacked[10],
        "brake": unpacked[11],
        "steer_normalized": unpacked[12],
        "tail": unpacked[13],
    }
