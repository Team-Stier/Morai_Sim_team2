# -*- coding: utf-8 -*-
"""Camera source packet timestamp fail-closed 정책 테스트."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from morai_udp_bridge.timestamp_guard import PacketTimestampGuard  # noqa: E402


class PacketTimestampGuardTest(unittest.TestCase):
    def setUp(self):
        self.guard = PacketTimestampGuard()

    def test_increasing_stamp_is_accepted(self):
        self.assertIsNone(self.guard.check(10, 1, 10, 2))
        self.assertIsNone(self.guard.check(10, 2, 10, 3))

    def test_invalid_and_future_stamp_are_rejected(self):
        self.assertEqual(self.guard.check(0, 0, 10, 0), "invalid")
        self.assertEqual(self.guard.check(11, 0, 10, 0), "future")

    def test_duplicate_is_rejected_without_reset(self):
        self.assertIsNone(self.guard.check(10, 1, 10, 2))
        self.assertEqual(self.guard.check(10, 1, 10, 3), "duplicate")
        self.assertEqual(self.guard.reset_count, 0)

    def test_regression_is_rejected_then_guard_reinitializes(self):
        self.assertIsNone(self.guard.check(20, 0, 21, 0))
        self.assertEqual(self.guard.check(10, 0, 21, 0), "regression_reset")
        self.assertEqual(self.guard.reset_count, 1)
        self.assertIsNone(self.guard.check(10, 1, 21, 0))


if __name__ == "__main__":
    unittest.main()
