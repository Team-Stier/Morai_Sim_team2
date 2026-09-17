import math
import unittest

from morai_udp_bridge.protocol.competition_vehicle_status_packet import (
    COMPETITION_STATUS_PACKET_SIZE,
    CompetitionVehicleStatusParseError,
    build_competition_vehicle_status_packet,
    parse_competition_vehicle_status_packet,
)


class CompetitionVehicleStatusPacketTest(unittest.TestCase):
    def test_only_allowed_subset_is_exposed(self):
        packet = build_competition_vehicle_status_packet(
            sec=1, nsec=2, ctrl_mode=2, gear=4, signed_vel=3.5,
            accel=0.2, brake=0.1, roll=0.01, pitch=0.02, yaw=0.03,
            vel_x=3.4, ang_vel=(0.1, 0.2, 0.3), steer=0.4,
        )
        self.assertEqual(len(packet), COMPETITION_STATUS_PACKET_SIZE)
        reading = parse_competition_vehicle_status_packet(packet)
        self.assertEqual(reading.ctrl_mode, 2)
        self.assertEqual(reading.gear, 4)
        self.assertAlmostEqual(reading.vel_x, 3.4, places=5)
        self.assertAlmostEqual(reading.ang_vel_z, 0.3, places=5)
        for forbidden in ('pos_x', 'pos_y', 'pos_z', 'vel_y', 'vel_z',
                          'accel_x', 'accel_y', 'accel_z'):
            self.assertFalse(hasattr(reading, forbidden))

    def test_unverified_wire_size_mismatch_is_rejected(self):
        with self.assertRaises(CompetitionVehicleStatusParseError):
            parse_competition_vehicle_status_packet(b'\x00' * 100)


if __name__ == '__main__':
    unittest.main()
