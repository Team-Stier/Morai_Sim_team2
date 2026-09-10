#!/usr/bin/env python3

import unittest

from system_bringup_pkg.readiness import (
    COMPONENT_BITS,
    STATE_DEGRADED,
    STATE_FAULT,
    STATE_INITIALIZING,
    STATE_READY,
    evaluate,
)


class ReadinessLogicTest(unittest.TestCase):
    def _ready_samples(self, stamp=10.0):
        return {
            name: {
                "stamp_sec": stamp,
                "state": STATE_READY,
                "ready": True,
                "stop_required": False,
            }
            for name in COMPONENT_BITS
        }

    def test_all_required_components_ready(self):
        required = list(COMPONENT_BITS.keys())
        result = evaluate(required, self._ready_samples(), 10.2, 1.0)
        self.assertTrue(result["ready"])
        self.assertEqual(result["state"], STATE_READY)
        self.assertEqual(result["ready_mask"], result["required_mask"])
        self.assertEqual(result["missing_mask"], 0)
        self.assertEqual(result["stale_mask"], 0)
        self.assertEqual(result["not_ready_mask"], 0)
        self.assertEqual(result["fault_mask"], 0)

    def test_missing_component_is_initializing_and_fail_closed(self):
        result = evaluate(["interface", "map"], {}, 10.0, 1.0)
        self.assertFalse(result["ready"])
        self.assertEqual(result["state"], STATE_INITIALIZING)
        self.assertEqual(
            result["missing_mask"],
            COMPONENT_BITS["interface"] | COMPONENT_BITS["map"],
        )

    def test_stale_component_is_degraded(self):
        samples = self._ready_samples(stamp=5.0)
        result = evaluate(["interface"], samples, 10.0, 1.0)
        self.assertFalse(result["ready"])
        self.assertEqual(result["state"], STATE_DEGRADED)
        self.assertEqual(result["stale_mask"], COMPONENT_BITS["interface"])

    def test_fresh_but_not_ready_is_separate_from_fault(self):
        samples = self._ready_samples()
        samples["planning"]["ready"] = False
        samples["planning"]["state"] = STATE_INITIALIZING
        result = evaluate(["planning"], samples, 10.1, 1.0)
        self.assertFalse(result["ready"])
        self.assertEqual(result["state"], STATE_DEGRADED)
        self.assertEqual(result["not_ready_mask"], COMPONENT_BITS["planning"])
        self.assertEqual(result["fault_mask"], 0)

    def test_stop_required_is_fault(self):
        samples = self._ready_samples()
        samples["control"]["stop_required"] = True
        result = evaluate(["control"], samples, 10.1, 1.0)
        self.assertFalse(result["ready"])
        self.assertEqual(result["state"], STATE_FAULT)
        self.assertEqual(result["fault_mask"], COMPONENT_BITS["control"])

    def test_future_stamp_is_rejected(self):
        samples = self._ready_samples(stamp=11.0)
        result = evaluate(["localization"], samples, 10.0, 1.0)
        self.assertFalse(result["ready"])
        self.assertEqual(result["state"], STATE_FAULT)
        self.assertEqual(result["stale_mask"], COMPONENT_BITS["localization"])
        self.assertEqual(result["fault_mask"], COMPONENT_BITS["localization"])

    def test_unknown_required_component_is_rejected(self):
        with self.assertRaises(ValueError):
            evaluate(["not_a_component"], {}, 10.0, 1.0)


if __name__ == "__main__":
    unittest.main()
