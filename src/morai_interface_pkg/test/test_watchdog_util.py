# -*- coding: utf-8 -*-
"""lidar watchdog staleness 판정 단위 테스트. ROS 없이 실행 가능."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from morai_udp_bridge.watchdog_util import is_stale  # noqa: E402


class WatchdogUtilTest(unittest.TestCase):
    def test_not_stale_within_timeout(self):
        self.assertFalse(is_stale(now_sec=10.0, reference_sec=9.5, timeout_sec=1.0))

    def test_stale_after_timeout(self):
        self.assertTrue(is_stale(now_sec=11.5, reference_sec=10.0, timeout_sec=1.0))

    def test_boundary_not_stale(self):
        # 정확히 timeout이면 아직 stale 아님(초과여야 함)
        self.assertFalse(is_stale(now_sec=11.0, reference_sec=10.0, timeout_sec=1.0))


if __name__ == "__main__":
    unittest.main()
