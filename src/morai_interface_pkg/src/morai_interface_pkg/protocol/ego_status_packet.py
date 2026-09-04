# -*- coding: utf-8 -*-
"""
ego_status_packet.py
- 역할: MORAI EgoVehicleStatus UDP 바이너리 패킷을 파싱해 raw 값 구조체로 변환한다.
        ROS에 의존하지 않으므로 단위 테스트가 가능하다.
- 근거: MORAI-NetworkModule lib/define/EgoVehicleStatus.py (ctypes, _pack_=1, LE).
        오프셋은 ctypes.offset으로 실측 확인했다.

이 파서는 Vehicle Twist 변환에 필요한 필드와 로그 비교에 유용한 필드만 뽑는다.
속도 필드의 좌표계·단위는 자료로 확정되지 않았으므로 노드에서
source/scale 파라미터로 처리한다(여기서는 raw 값만 반환).

관련 오프셋(총 229바이트, little-endian):
  off 27  int    sec        (packet timestamp)
  off 31  int    nsec
  off 37  float  signed_vel (부호 있는 종방향 속도 scalar)
  off 77  float  pos_x   off 81 pos_y   off 85 pos_z
  off 97  float  yaw
  off101  float  vel_x   off105 vel_y   off109 vel_z
  off113  float  ang_vel_x  off117 ang_vel_y  off121 ang_vel_z
"""

import struct

EGO_PACKET_SIZE = 229
_OFF_SEC = 27
_OFF_NSEC = 31
_OFF_SIGNED_VEL = 37
_OFF_POS = 77       # 3 x float
_OFF_YAW = 97
_OFF_VEL = 101      # 3 x float
_OFF_ANG_VEL = 113  # 3 x float


class EgoStatusParseError(Exception):
    """EgoVehicleStatus 패킷을 해석할 수 없을 때 발생한다."""
    pass


class EgoStatusReading(object):
    """파싱된 EgoVehicleStatus raw 값. 단위/좌표계 해석은 ROS 노드가 담당한다."""

    __slots__ = ("sec", "nsec", "signed_vel",
                 "vel_x", "vel_y", "vel_z",
                 "ang_vel_x", "ang_vel_y", "ang_vel_z",
                 "pos_x", "pos_y", "pos_z", "yaw")

    def __init__(self, sec, nsec, signed_vel, vel, ang_vel, pos, yaw):
        self.sec = sec
        self.nsec = nsec
        self.signed_vel = signed_vel
        self.vel_x, self.vel_y, self.vel_z = vel
        self.ang_vel_x, self.ang_vel_y, self.ang_vel_z = ang_vel
        self.pos_x, self.pos_y, self.pos_z = pos
        self.yaw = yaw

    def __repr__(self):
        return ("EgoStatusReading(stamp=%d.%09d, signed_vel=%.4f, "
                "vel=(%.4f,%.4f,%.4f), ang_vel=(%.4f,%.4f,%.4f), yaw=%.4f)"
                % (self.sec, self.nsec, self.signed_vel, self.vel_x, self.vel_y,
                   self.vel_z, self.ang_vel_x, self.ang_vel_y, self.ang_vel_z,
                   self.yaw))


# 함수이름: parse_ego_status_packet
# 기능: MORAI EgoVehicleStatus UDP 바이너리를 EgoStatusReading으로 파싱한다.
# 인자: data - 수신한 bytes
# 반환값: EgoStatusReading
# 예외: EgoStatusParseError (길이 부족 등)
def parse_ego_status_packet(data):
    # 고정 229B struct이므로 정확 길이 검증. header/data_lenght/tail 값은
    # 자료에 없어(수신 struct placeholder) 추측 검사하지 않는다.
    if data is None or len(data) != EGO_PACKET_SIZE:
        raise EgoStatusParseError(
            "EgoStatus 패킷 길이 오류: %d != %d"
            % (0 if data is None else len(data), EGO_PACKET_SIZE))
    try:
        sec = struct.unpack_from("<i", data, _OFF_SEC)[0]
        nsec = struct.unpack_from("<i", data, _OFF_NSEC)[0]
        signed_vel = struct.unpack_from("<f", data, _OFF_SIGNED_VEL)[0]
        pos = struct.unpack_from("<3f", data, _OFF_POS)
        yaw = struct.unpack_from("<f", data, _OFF_YAW)[0]
        vel = struct.unpack_from("<3f", data, _OFF_VEL)
        ang_vel = struct.unpack_from("<3f", data, _OFF_ANG_VEL)
    except struct.error as error:
        raise EgoStatusParseError("struct 언패킹 실패: %s" % error)
    return EgoStatusReading(sec, nsec, signed_vel, vel, ang_vel, pos, yaw)


# 함수이름: build_ego_status_packet
# 기능: 테스트/모의 송신용으로 유효한 229바이트 EgoVehicleStatus 패킷을 만든다.
def build_ego_status_packet(sec=0, nsec=0, signed_vel=0.0,
                            vel=(0.0, 0.0, 0.0), ang_vel=(0.0, 0.0, 0.0),
                            pos=(0.0, 0.0, 0.0), yaw=0.0,
                            header=b"MoraiEgoSt\x00"):
    if len(header) != 11:
        header = (header + b"\x00" * 11)[:11]
    packet = bytearray(EGO_PACKET_SIZE)
    packet[0:11] = header
    struct.pack_into("<i", packet, 11, EGO_PACKET_SIZE)      # data_lenght
    struct.pack_into("<i", packet, _OFF_SEC, sec)
    struct.pack_into("<i", packet, _OFF_NSEC, nsec)
    struct.pack_into("<f", packet, _OFF_SIGNED_VEL, signed_vel)
    struct.pack_into("<3f", packet, _OFF_POS, *pos)
    struct.pack_into("<f", packet, _OFF_YAW, yaw)
    struct.pack_into("<3f", packet, _OFF_VEL, *vel)
    struct.pack_into("<3f", packet, _OFF_ANG_VEL, *ang_vel)
    packet[227:229] = b"\x00\x00"                            # tail
    return bytes(packet)

