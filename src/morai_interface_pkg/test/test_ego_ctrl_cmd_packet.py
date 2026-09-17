import unittest

from morai_udp_bridge.protocol.ego_ctrl_cmd_packet import (
    COMPETITION_CMD_TYPE,
    COMPETITION_CTRL_MODE,
    EGO_CTRL_CMD_PACKET_SIZE,
    EgoCtrlCmdPacketError,
    parse_ego_ctrl_cmd,
    serialize_ego_ctrl_cmd,
)


class EgoCtrlCmdPacketTest(unittest.TestCase):
    def test_competition_fields_and_size(self):
        packet = serialize_ego_ctrl_cmd(gear=4, accel=0.25, brake=0.0, steer_normalized=0.0)
        self.assertEqual(len(packet), EGO_CTRL_CMD_PACKET_SIZE)
        parsed = parse_ego_ctrl_cmd(packet)
        self.assertEqual(parsed['ctrl_mode'], COMPETITION_CTRL_MODE)
        self.assertEqual(parsed['cmd_type'], COMPETITION_CMD_TYPE)
        self.assertEqual(parsed['gear'], 4)
        self.assertAlmostEqual(parsed['accel'], 0.25, places=6)
        self.assertEqual(parsed['velocity'], 0.0)
        self.assertEqual(parsed['acceleration'], 0.0)

    def test_out_of_range_command_is_rejected(self):
        with self.assertRaises(EgoCtrlCmdPacketError):
            serialize_ego_ctrl_cmd(gear=4, accel=1.1, brake=0.0, steer_normalized=0.0)
        with self.assertRaises(EgoCtrlCmdPacketError):
            serialize_ego_ctrl_cmd(gear=4, accel=0.0, brake=0.0, steer_normalized=1.1)


if __name__ == '__main__':
    unittest.main()
