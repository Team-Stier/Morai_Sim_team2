# -*- coding: utf-8 -*-
"""Competition Vehicle Status packet parser.

Live MORAI competition traffic captured on 2026-10-01 uses a 181-byte UDP
payload:

- 0..10:  b"#MoraiInfo$"
- 11..14: little-endian data_length == 152
- 27..178: Competition Vehicle Status data
- 179..180: CRLF

The field offsets through front steering match the MORAI EgoVehicleStatus
example. Competition-withheld position, lateral/vertical velocity and linear
acceleration slots remain on the wire but are not exposed by this parser.
The rear-steer slot and the tire/boundary/CTE tail from the full example are
not present in the observed competition packet.
"""

import struct

COMPETITION_STATUS_HEADER = b"#MoraiInfo$"
COMPETITION_STATUS_DATA_LENGTH = 152
COMPETITION_STATUS_PACKET_SIZE = 181
COMPETITION_STATUS_TRAILER = b"\r\n"

_OFF_DATA_LENGTH = 11
_OFF_SEC = 27
_OFF_NSEC = 31
_OFF_CTRL_MODE = 35
_OFF_GEAR = 36
_OFF_SIGNED_VEL = 37
_OFF_ACCEL = 45
_OFF_BRAKE = 49
_OFF_ROLL = 89
_OFF_PITCH = 93
_OFF_YAW = 97
_OFF_VEL_X = 101
_OFF_ANG_VEL = 113
_OFF_STEER = 137


class CompetitionVehicleStatusParseError(Exception):
    pass


class CompetitionVehicleStatusReading(object):
    __slots__ = (
        "sec", "nsec", "ctrl_mode", "gear", "signed_vel",
        "accel", "brake", "roll", "pitch", "yaw", "vel_x",
        "ang_vel_x", "ang_vel_y", "ang_vel_z", "steer",
    )

    def __init__(self, sec, nsec, ctrl_mode, gear, signed_vel,
                 accel, brake, roll, pitch, yaw, vel_x, ang_vel, steer):
        self.sec = sec
        self.nsec = nsec
        self.ctrl_mode = ctrl_mode
        self.gear = gear
        self.signed_vel = signed_vel
        self.accel = accel
        self.brake = brake
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.vel_x = vel_x
        self.ang_vel_x, self.ang_vel_y, self.ang_vel_z = ang_vel
        self.steer = steer

    def __repr__(self):
        return (
            "CompetitionVehicleStatusReading(stamp=%d.%09d, ctrl_mode=%d, "
            "gear=%d, signed_vel=%.4f, vel_x=%.4f, "
            "ang_vel=(%.4f,%.4f,%.4f), steer=%.4f)"
            % (
                self.sec, self.nsec, self.ctrl_mode, self.gear,
                self.signed_vel, self.vel_x, self.ang_vel_x,
                self.ang_vel_y, self.ang_vel_z, self.steer,
            )
        )


def parse_competition_vehicle_status_packet(data):
    if data is None or len(data) != COMPETITION_STATUS_PACKET_SIZE:
        raise CompetitionVehicleStatusParseError(
            "Competition Vehicle Status 패킷 길이 오류: %d != %d"
            % (
                0 if data is None else len(data),
                COMPETITION_STATUS_PACKET_SIZE,
            )
        )

    if data[:len(COMPETITION_STATUS_HEADER)] != COMPETITION_STATUS_HEADER:
        raise CompetitionVehicleStatusParseError(
            "Competition Vehicle Status 헤더 오류"
        )

    data_length = struct.unpack_from("<i", data, _OFF_DATA_LENGTH)[0]
    if data_length != COMPETITION_STATUS_DATA_LENGTH:
        raise CompetitionVehicleStatusParseError(
            "Competition Vehicle Status data_length 오류: %d != %d"
            % (data_length, COMPETITION_STATUS_DATA_LENGTH)
        )

    if data[-2:] != COMPETITION_STATUS_TRAILER:
        raise CompetitionVehicleStatusParseError(
            "Competition Vehicle Status CRLF trailer 오류"
        )

    try:
        sec = struct.unpack_from("<i", data, _OFF_SEC)[0]
        nsec = struct.unpack_from("<i", data, _OFF_NSEC)[0]
        ctrl_mode = struct.unpack_from("<b", data, _OFF_CTRL_MODE)[0]
        gear = struct.unpack_from("<b", data, _OFF_GEAR)[0]
        signed_vel = struct.unpack_from("<f", data, _OFF_SIGNED_VEL)[0]
        accel = struct.unpack_from("<f", data, _OFF_ACCEL)[0]
        brake = struct.unpack_from("<f", data, _OFF_BRAKE)[0]
        roll = struct.unpack_from("<f", data, _OFF_ROLL)[0]
        pitch = struct.unpack_from("<f", data, _OFF_PITCH)[0]
        yaw = struct.unpack_from("<f", data, _OFF_YAW)[0]
        vel_x = struct.unpack_from("<f", data, _OFF_VEL_X)[0]
        ang_vel = struct.unpack_from("<3f", data, _OFF_ANG_VEL)
        steer = struct.unpack_from("<f", data, _OFF_STEER)[0]
    except struct.error as error:
        raise CompetitionVehicleStatusParseError(
            "Competition Vehicle Status struct 언패킹 실패: %s" % error
        )

    if nsec < 0 or nsec >= 1000000000:
        raise CompetitionVehicleStatusParseError(
            "Competition Vehicle Status nsec 범위 오류: %d" % nsec
        )

    return CompetitionVehicleStatusReading(
        sec, nsec, ctrl_mode, gear, signed_vel, accel, brake,
        roll, pitch, yaw, vel_x, ang_vel, steer,
    )


def build_competition_vehicle_status_packet(
        sec=0, nsec=0, ctrl_mode=2, gear=4, signed_vel=0.0,
        accel=0.0, brake=0.0, roll=0.0, pitch=0.0, yaw=0.0,
        vel_x=0.0, ang_vel=(0.0, 0.0, 0.0), steer=0.0):
    """테스트용 181-byte Competition Vehicle Status packet을 만든다."""
    packet = bytearray(COMPETITION_STATUS_PACKET_SIZE)
    packet[:len(COMPETITION_STATUS_HEADER)] = COMPETITION_STATUS_HEADER
    struct.pack_into("<i", packet, _OFF_DATA_LENGTH, COMPETITION_STATUS_DATA_LENGTH)
    struct.pack_into("<i", packet, _OFF_SEC, sec)
    struct.pack_into("<i", packet, _OFF_NSEC, nsec)
    struct.pack_into("<b", packet, _OFF_CTRL_MODE, ctrl_mode)
    struct.pack_into("<b", packet, _OFF_GEAR, gear)
    struct.pack_into("<f", packet, _OFF_SIGNED_VEL, signed_vel)
    struct.pack_into("<f", packet, _OFF_ACCEL, accel)
    struct.pack_into("<f", packet, _OFF_BRAKE, brake)
    struct.pack_into("<f", packet, _OFF_ROLL, roll)
    struct.pack_into("<f", packet, _OFF_PITCH, pitch)
    struct.pack_into("<f", packet, _OFF_YAW, yaw)
    struct.pack_into("<f", packet, _OFF_VEL_X, vel_x)
    struct.pack_into("<3f", packet, _OFF_ANG_VEL, *ang_vel)
    struct.pack_into("<f", packet, _OFF_STEER, steer)
    packet[-2:] = COMPETITION_STATUS_TRAILER
    return bytes(packet)
