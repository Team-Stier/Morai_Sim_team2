# -*- coding: utf-8 -*-
"""
ego_ctrl_cmd_packet.py
- 역할: ROS 제어 명령을 MORAI EgoCtrlCmd UDP 바이너리로 packing한다(그 역도 파싱).
        ROS에 의존하지 않으므로 단위 테스트가 가능하다.
- 근거: MORAI-NetworkModule lib/define/EgoCtrlCmd.py (ctypes, _pack_=1, little-endian).
        오프셋은 ctypes로 실측 확인했다.

패킷 레이아웃(총 55바이트, little-endian, 패딩 없음):
  off  0  char[14] header       '#MoraiCtrlCmd$'
  off 14  int      data_lenght  23 (고정)
  off 18  int[3]   aux_data     (0,0,0)
  off 30  int8     ctrl_mode
  off 31  int8     gear
  off 32  int8     cmd_type
  off 33  float    velocity
  off 37  float    acceleration
  off 41  float    accel
  off 45  float    brake
  off 49  float    steer
  off 53  char[2]  tail         '\r\n'
"""

import struct

HEADER = b"#MoraiCtrlCmd$"      # 14바이트
DATA_LENGTH = 23                # 정의 고정값
TAIL = b"\r\n"                  # 2바이트
EGO_CTRL_CMD_SIZE = 55

# little-endian 명시. 14s + i + 3i + b + b + b + 5f + 2s = 55바이트.
_STRUCT = struct.Struct("<14s i 3i b b b 5f 2s")

_INT8_MIN, _INT8_MAX = -128, 127


class EgoCtrlCmdPackError(Exception):
    """EgoCtrlCmd packing/parsing이 불가능할 때 발생한다."""
    pass


def _as_int8(name, value):
    value = int(value)
    if not (_INT8_MIN <= value <= _INT8_MAX):
        raise EgoCtrlCmdPackError("%s int8 범위 초과: %d" % (name, value))
    return value


# 함수이름: build_ego_ctrl_cmd_packet
# 기능: 제어 값들을 55바이트 EgoCtrlCmd UDP 패킷으로 packing한다.
# 인자: cmd_type, velocity, acceleration, accel, brake, steer, ctrl_mode, gear
#       ctrl_mode/gear 기본값은 MORAI 공식 예제 기준 AutoMode(2)/Drive(4).
# 반환값: 55바이트 bytes
# 예외: EgoCtrlCmdPackError (int8 범위 초과, struct 오류)
def build_ego_ctrl_cmd_packet(cmd_type, velocity, acceleration, accel, brake,
                              steer, ctrl_mode=2, gear=4):
    ctrl_mode = _as_int8("ctrl_mode", ctrl_mode)
    gear = _as_int8("gear", gear)
    cmd_type = _as_int8("cmd_type", cmd_type)
    try:
        return _STRUCT.pack(
            HEADER, DATA_LENGTH, 0, 0, 0,
            ctrl_mode, gear, cmd_type,
            float(velocity), float(acceleration), float(accel),
            float(brake), float(steer), TAIL)
    except struct.error as error:
        raise EgoCtrlCmdPackError("struct packing 실패: %s" % error)


class EgoCtrlCmdFields(object):
    """파싱된 EgoCtrlCmd 값(loopback 테스트/검증용)."""

    __slots__ = ("ctrl_mode", "gear", "cmd_type", "velocity",
                 "acceleration", "accel", "brake", "steer")

    def __init__(self, ctrl_mode, gear, cmd_type, velocity, acceleration,
                 accel, brake, steer):
        self.ctrl_mode = ctrl_mode
        self.gear = gear
        self.cmd_type = cmd_type
        self.velocity = velocity
        self.acceleration = acceleration
        self.accel = accel
        self.brake = brake
        self.steer = steer


# 함수이름: parse_ego_ctrl_cmd_packet
# 기능: 55바이트 EgoCtrlCmd 패킷을 필드로 파싱한다.
#       EgoCtrlCmd는 송신 struct라 header/data_lenght(23)/aux_data(0,0,0)/tail(\r\n)
#       값이 공식 정의로 확인되므로 모두 검증한다.
# 예외: EgoCtrlCmdPackError
def parse_ego_ctrl_cmd_packet(data):
    if data is None or len(data) != EGO_CTRL_CMD_SIZE:
        raise EgoCtrlCmdPackError(
            "EgoCtrlCmd 길이 오류: %d != %d"
            % (0 if data is None else len(data), EGO_CTRL_CMD_SIZE))
    (header, data_length, aux0, aux1, aux2, ctrl_mode, gear, cmd_type,
     velocity, acceleration, accel, brake, steer, tail) = _STRUCT.unpack(data)
    if header != HEADER:
        raise EgoCtrlCmdPackError("header 불일치: %r" % header)
    if data_length != DATA_LENGTH:
        raise EgoCtrlCmdPackError("data_lenght 불일치: %d != %d"
                                  % (data_length, DATA_LENGTH))
    if (aux0, aux1, aux2) != (0, 0, 0):
        raise EgoCtrlCmdPackError("aux_data 불일치: %r" % ((aux0, aux1, aux2),))
    if tail != TAIL:
        raise EgoCtrlCmdPackError("tail 불일치: %r" % tail)
    return EgoCtrlCmdFields(ctrl_mode, gear, cmd_type, velocity,
                            acceleration, accel, brake, steer)

