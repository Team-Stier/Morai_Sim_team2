"""Q is a temporary transmission inhibit, never an inferred mode toggle."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from morai_udp_bridge.local_handover_guard import LocalHandoverGuard


class LocalHandoverTest(unittest.TestCase):
    def guard(self):
        g = LocalHandoverGuard.__new__(LocalHandoverGuard)
        g.hold_sec, g.until = 0.5, 0.0
        return g

    def test_inhibits_press_hold_and_release_grace_without_toggle(self):
        g = self.guard()
        self.assertFalse(g._update_key(False, True, 1.0))
        self.assertTrue(g._update_key(True, True, 2.0))
        self.assertTrue(g._update_key(True, True, 3.0))
        self.assertTrue(g._update_key(False, True, 3.49))
        self.assertFalse(g._update_key(False, True, 3.5))
        # A second press behaves identically; parity never changes mode.
        self.assertTrue(g._update_key(True, True, 4.0))
        self.assertFalse(g._update_key(False, True, 4.5))
        self.assertFalse(hasattr(g, 'autonomous'))

    def test_other_window_q_has_no_effect(self):
        g = self.guard()
        self.assertFalse(g._update_key(True, False, 1.0))
        self.assertFalse(g._update_key(False, True, 1.1))

    def test_invalid_duration_rejected_before_opening_display(self):
        for duration in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):LocalHandoverGuard(duration)

if __name__ == '__main__':unittest.main()
