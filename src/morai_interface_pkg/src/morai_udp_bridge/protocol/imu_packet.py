# -*- coding: utf-8 -*-
"""
imu_packet.py
- 역할: MORAI IMU UDP 바이너리 패킷을 파싱해 raw 값 구조체로 변환한다.
        ROS에 의존하지 않으므로 단위 테스트가 가능하다.
- 근거: MORAI-NetworkModule lib/define/IMU.py (ctypes, _pack_=1, little-endian).
        오프셋은 ctypes.sizeof/offset으로 실측 확인했다.

패킷 레이아웃(총 115바이트, little-endian, 패딩 없음):
  off  0  char[9]  header
  off  9  int      data_lenght
  off 13  int[3]   aux_data
  off 25  int      sec        (packet timestamp)
  off 29  int      nsec
  off 33  double   ori_w      (orientation 순서: W, X, Y, Z)
  off 41  double   ori_x
  off 49  double   ori_y
  off 57  double   ori_z
  off 65  double   ang_vel_x  (단위 미확정 -> 노드에서 scale param 적용)
  off 73  double   ang_vel_y
  off 81  double   ang_vel_z
  off 89  double   lin_acc_x  (단위 미확정 -> 노드에서 scale param 적용)
  off 97  double   lin_acc_y
  off105  double   lin_acc_z
  off113  char[2]  tail
"""

import struct

IMU_PACKET_SIZE = 115
_OFF_SEC = 25
_OFF_NSEC = 29
_OFF_ORI = 33      # 4 x double: w, x, y, z
_OFF_ANG_VEL = 65  # 3 x double
_OFF_LIN_ACC = 89  # 3 x double


class ImuParseError(Exception):
    """IMU 패킷을 해석할 수 없을 때 발생한다."""
    pass


class ImuReading(object):
    """파싱된 IMU raw 값. 단위 변환/공분산은 ROS 노드가 담당한다."""

    __slots__ = ("sec", "nsec",
                 "ori_w", "ori_x", "ori_y", "ori_z",
                 "ang_vel_x", "ang_vel_y", "ang_vel_z",
                 "lin_acc_x", "lin_acc_y", "lin_acc_z")

    def __init__(self, sec, nsec, ori, ang_vel, lin_acc):
        self.sec = sec
        self.nsec = nsec
        self.ori_w, self.ori_x, self.ori_y, self.ori_z = ori
        self.ang_vel_x, self.ang_vel_y, self.ang_vel_z = ang_vel
        self.lin_acc_x, self.lin_acc_y, self.lin_acc_z = lin_acc

    def __repr__(self):
        return ("ImuReading(stamp=%d.%09d, ori_wxyz=(%.5f,%.5f,%.5f,%.5f), "
                "ang_vel=(%.5f,%.5f,%.5f), lin_acc=(%.5f,%.5f,%.5f))"
                % (self.sec, self.nsec, self.ori_w, self.ori_x, self.ori_y,
                   self.ori_z, self.ang_vel_x, self.ang_vel_y, self.ang_vel_z,
                   self.lin_acc_x, self.lin_acc_y, self.lin_acc_z))


# 함수이름: parse_imu_packet
# 기능: MORAI IMU UDP 바이너리를 ImuReading으로 파싱한다.
# 인자: data - 수신한 bytes
# 반환값: ImuReading
# 예외: ImuParseError (길이 부족 등)
def parse_imu_packet(data):
    # 고정 115B struct이므로 정확 길이 검증(수신 datagram 1개=패킷 1개).
    # 길이가 다르면 aux 헤더 크기/프로토콜 불일치 신호 -> 폐기.
    # header/data_lenght/tail 값은 자료에 없어(추측 금지) 검사하지 않는다.
    if data is None or len(data) != IMU_PACKET_SIZE:
        raise ImuParseError("IMU 패킷 길이 오류: %d != %d"
                            % (0 if data is None else len(data), IMU_PACKET_SIZE))
    try:
        sec = struct.unpack_from("<i", data, _OFF_SEC)[0]
        nsec = struct.unpack_from("<i", data, _OFF_NSEC)[0]
        ori = struct.unpack_from("<4d", data, _OFF_ORI)
        ang_vel = struct.unpack_from("<3d", data, _OFF_ANG_VEL)
        lin_acc = struct.unpack_from("<3d", data, _OFF_LIN_ACC)
    except struct.error as error:
        raise ImuParseError("struct 언패킹 실패: %s" % error)
    return ImuReading(sec, nsec, ori, ang_vel, lin_acc)


# 함수이름: build_imu_packet
# 기능: 테스트/모의 송신용으로 유효한 115바이트 IMU 패킷을 만든다.
def build_imu_packet(sec=0, nsec=0, ori=(1.0, 0.0, 0.0, 0.0),
                     ang_vel=(0.0, 0.0, 0.0), lin_acc=(0.0, 0.0, 0.0),
                     header=b"MoraiIMU\x00"):
    if len(header) != 9:
        header = (header + b"\x00" * 9)[:9]
    packet = bytearray(IMU_PACKET_SIZE)
    packet[0:9] = header
    struct.pack_into("<i", packet, 9, IMU_PACKET_SIZE)       # data_lenght
    struct.pack_into("<i", packet, _OFF_SEC, sec)
    struct.pack_into("<i", packet, _OFF_NSEC, nsec)
    struct.pack_into("<4d", packet, _OFF_ORI, *ori)
    struct.pack_into("<3d", packet, _OFF_ANG_VEL, *ang_vel)
    struct.pack_into("<3d", packet, _OFF_LIN_ACC, *lin_acc)
    packet[113:115] = b"\x00\x00"                            # tail
    return bytes(packet)
