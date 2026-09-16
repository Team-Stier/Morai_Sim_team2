#!/usr/bin/env python3

import unittest

from hd_map_pkg.speed_policy import (
    competition_speed_policy,
    competition_speed_tags,
)


class _Dataset(object):
    links = {
        "START": {"road_id": "R1", "points": [[0, 0, 0], [1, 0, 0]]},
        "MIDDLE": {"road_id": "R2", "points": [[1, 0, 0], [2, 0, 0]]},
        "END": {"road_id": "R2", "points": [[2, 0, 0], [3, 0, 0]]},
        "OUTSIDE": {"road_id": "R3", "points": [[3, 0, 0], [4, 0, 0]]},
    }


class CompetitionSpeedPolicyTest(unittest.TestCase):
    def setUp(self):
        self.config = {
            "competition_speed_policy": {
                "default_limit_kph": 60,
                "exemption": {
                    "id": "zone",
                    "label": "test zone",
                    "start_link_id": "START",
                    "end_link_id": "END",
                    "road_ids": ["R1", "R2"],
                },
            },
        }

    def test_resolves_parallel_road_links_and_boundary_points(self):
        policy = competition_speed_policy(_Dataset(), self.config)

        self.assertEqual(policy["link_ids"], ("END", "MIDDLE", "START"))
        self.assertEqual(policy["start_point"], [0, 0, 0])
        self.assertEqual(policy["end_point"], [3, 0, 0])
        self.assertEqual(competition_speed_tags("MIDDLE", policy), {
            "molit:competition_speed_limit_kph": "60",
            "molit:competition_speed_limit_exempt": "yes",
            "molit:competition_speed_zone": "zone",
        })
        self.assertEqual(competition_speed_tags("OUTSIDE", policy), {
            "molit:competition_speed_limit_kph": "60",
            "molit:competition_speed_limit_exempt": "no",
        })

    def test_rejects_missing_official_boundary_link(self):
        self.config["competition_speed_policy"]["exemption"][
            "start_link_id"] = "ABSENT"
        with self.assertRaisesRegex(ValueError, "ABSENT"):
            competition_speed_policy(_Dataset(), self.config)


if __name__ == "__main__":
    unittest.main()
