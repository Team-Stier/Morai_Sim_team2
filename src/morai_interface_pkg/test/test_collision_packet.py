import unittest

from morai_udp_bridge.protocol.collision_packet import (
    COLLISION_PACKET_SIZE,
    build_collision_packet,
    parse_collision_packet,
)


class CollisionPacketTest(unittest.TestCase):
    def test_parse_five_slot_layout(self):
        packet = build_collision_packet(
            sec=12,
            nsec=34,
            slots=[(2, 7, (1.0, 2.0, 3.0), (4.0, 5.0, 6.0))],
        )
        self.assertEqual(len(packet), COLLISION_PACKET_SIZE)
        parsed = parse_collision_packet(packet)
        self.assertEqual(parsed.sec, 12)
        self.assertEqual(parsed.nsec, 34)
        self.assertEqual(len(parsed.slots), 5)
        self.assertEqual(len(parsed.active_slots), 1)
        self.assertEqual(parsed.active_slots[0].obj_type, 2)
        self.assertEqual(parsed.active_slots[0].obj_id, 7)

    def test_empty_packet_has_no_active_collision(self):
        parsed = parse_collision_packet(build_collision_packet())
        self.assertEqual(parsed.active_slots, [])


if __name__ == '__main__':
    unittest.main()
