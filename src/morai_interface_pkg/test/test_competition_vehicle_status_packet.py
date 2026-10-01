import unittest

from morai_udp_bridge.protocol.competition_vehicle_status_packet import (
    COMPETITION_STATUS_DATA_LENGTH,
    COMPETITION_STATUS_PACKET_SIZE,
    CompetitionVehicleStatusParseError,
    build_competition_vehicle_status_packet,
    parse_competition_vehicle_status_packet,
)


# 2026-10-01 live capture:
# 192.168.0.1:9088 -> 192.168.0.10:9099, UDP payload length 181.
_LIVE_PACKET_HEX = (
    "234d6f726169496e666f2498000000000000000000000000000000"
    "39f8bd6a8012ca25020464a1a7b8860000002a233b3e00000000"
    "ec5194400e2df23fa8c61b40ec51583f00004040713d4a3f0000"
    "000000000000000000002bb9b3c3a1c8ec3fa20531c364a1a7b8"
    "0000000000000000f5ba97399fe48a3cc3a93c3b000000000000"
    "0000000000005442a8bf4132323536573030313038360000000000"
    "0000000000000000000000000000000000000000000d0a"
)


class CompetitionVehicleStatusPacketTest(unittest.TestCase):
    def test_only_allowed_subset_is_exposed(self):
        packet = build_competition_vehicle_status_packet(
            sec=1, nsec=2, ctrl_mode=2, gear=4, signed_vel=3.5,
            accel=0.2, brake=0.1, roll=0.01, pitch=0.02, yaw=0.03,
            vel_x=3.4, ang_vel=(0.1, 0.2, 0.3), steer=0.4,
        )
        self.assertEqual(len(packet), COMPETITION_STATUS_PACKET_SIZE)
        self.assertEqual(COMPETITION_STATUS_DATA_LENGTH, 152)

        reading = parse_competition_vehicle_status_packet(packet)
        self.assertEqual(reading.ctrl_mode, 2)
        self.assertEqual(reading.gear, 4)
        self.assertAlmostEqual(reading.vel_x, 3.4, places=5)
        self.assertAlmostEqual(reading.ang_vel_z, 0.3, places=5)

        for forbidden in (
            "pos_x", "pos_y", "pos_z", "vel_y", "vel_z",
            "accel_x", "accel_y", "accel_z",
        ):
            self.assertFalse(hasattr(reading, forbidden))

    def test_live_181_byte_capture_is_parsed(self):
        packet = bytes.fromhex(_LIVE_PACKET_HEX)
        self.assertEqual(len(packet), 181)

        reading = parse_competition_vehicle_status_packet(packet)

        self.assertEqual(reading.sec, 1790834745)
        self.assertEqual(reading.nsec, 634000000)
        self.assertEqual(reading.ctrl_mode, 2)
        self.assertEqual(reading.gear, 4)
        self.assertAlmostEqual(reading.signed_vel, -0.0000799324, places=7)
        self.assertAlmostEqual(reading.accel, 0.18275133, places=6)
        self.assertAlmostEqual(reading.brake, 0.0, places=6)
        self.assertAlmostEqual(reading.vel_x, -0.0000799324, places=7)
        self.assertAlmostEqual(reading.ang_vel_x, 0.000289403, places=6)
        self.assertAlmostEqual(reading.ang_vel_y, 0.016954718, places=6)
        self.assertAlmostEqual(reading.ang_vel_z, 0.002878771, places=6)
        self.assertAlmostEqual(reading.steer, -1.31452417, places=6)

    def test_wrong_wire_size_is_rejected(self):
        with self.assertRaises(CompetitionVehicleStatusParseError):
            parse_competition_vehicle_status_packet(b"\x00" * 100)

    def test_wrong_header_is_rejected(self):
        packet = bytearray(build_competition_vehicle_status_packet())
        packet[0] = 0
        with self.assertRaises(CompetitionVehicleStatusParseError):
            parse_competition_vehicle_status_packet(bytes(packet))

    def test_wrong_data_length_is_rejected(self):
        packet = bytearray(build_competition_vehicle_status_packet())
        packet[11:15] = (151).to_bytes(4, byteorder="little", signed=True)
        with self.assertRaises(CompetitionVehicleStatusParseError):
            parse_competition_vehicle_status_packet(bytes(packet))


if __name__ == "__main__":
    unittest.main()
