"""Same-host internal MORAI mode gate; no key/window inference."""
import math
import threading
import time

CONTROL_MODE_TOPIC = "/molit/internal/morai_interface/control_mode"
AUTO_MODE = 2


class ControlModeGuard:
    def __init__(self, timeout_sec):
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError("control_mode_timeout_sec must be finite and positive")
        self.timeout_ns = int(timeout_sec * 1e9)
        self._state = (0, 0)
        self._lock = threading.Lock()

    def update(self, data):
        # UInt64MultiArray: [raw ctrl_mode (unknown=0), ingress monotonic_ns].
        # Both nodes must run on the same host/boot. Never compare with ROS time.
        state = tuple(data) if len(data) == 2 else (0, 0)
        with self._lock:
            self._state = state

    def blocked(self, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        with self._lock:
            mode, ingress_ns = self._state
        age_ns = now_ns - ingress_ns
        return mode != AUTO_MODE or ingress_ns <= 0 or not 0 <= age_ns < self.timeout_ns
