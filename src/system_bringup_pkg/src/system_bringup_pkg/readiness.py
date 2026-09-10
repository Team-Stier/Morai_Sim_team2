# -*- coding: utf-8 -*-
"""Pure readiness aggregation logic used by system_readiness_node."""

from collections import OrderedDict


COMPONENT_BITS = OrderedDict([
    ("interface", 1),
    ("map", 2),
    ("camera_perception", 4),
    ("lidar_perception", 8),
    ("localization", 16),
    ("route", 32),
    ("world_model", 64),
    ("planning", 128),
    ("control", 256),
])

STATE_UNKNOWN = 0
STATE_INITIALIZING = 1
STATE_READY = 2
STATE_DEGRADED = 3
STATE_FAULT = 4
STATE_DISABLED = 5

READY_STATES = (STATE_READY, STATE_DEGRADED)
FAULT_STATES = (STATE_FAULT, STATE_DISABLED)


def component_mask(names):
    """Return the OR mask for a validated component-name sequence."""
    mask = 0
    for name in names:
        if name not in COMPONENT_BITS:
            raise ValueError("unknown readiness component: %s" % name)
        mask |= COMPONENT_BITS[name]
    return mask


def evaluate(required_components, samples, now_sec, timeout_sec):
    """Evaluate normalized component samples into a fail-closed readiness snapshot.

    samples maps component name to dictionaries with:
      stamp_sec, state, ready, stop_required
    """
    required_components = list(required_components)
    required_mask = component_mask(required_components)
    ready_mask = 0
    missing_mask = 0
    stale_mask = 0
    not_ready_mask = 0
    fault_mask = 0
    ages = []
    reasons = []

    for name in required_components:
        bit = COMPONENT_BITS[name]
        sample = samples.get(name)
        if sample is None:
            missing_mask |= bit
            reasons.append("%s:missing" % name)
            continue

        stamp_sec = float(sample.get("stamp_sec", 0.0))
        state = int(sample.get("state", STATE_UNKNOWN))
        ready = bool(sample.get("ready", False))
        stop_required = bool(sample.get("stop_required", False))

        if stamp_sec <= 0.0:
            stale_mask |= bit
            reasons.append("%s:zero_stamp" % name)
            continue

        age = float(now_sec) - stamp_sec
        if age < 0.0:
            stale_mask |= bit
            fault_mask |= bit
            reasons.append("%s:future_stamp" % name)
            continue

        ages.append(age)
        if age > float(timeout_sec):
            stale_mask |= bit
            reasons.append("%s:stale" % name)
            continue

        if stop_required or state in FAULT_STATES:
            fault_mask |= bit
            reasons.append("%s:fault" % name)
            continue

        if ready and state in READY_STATES:
            ready_mask |= bit
            continue

        not_ready_mask |= bit
        reasons.append("%s:not_ready" % name)

    all_ready = (
        required_mask != 0
        and (ready_mask & required_mask) == required_mask
        and missing_mask == 0
        and stale_mask == 0
        and not_ready_mask == 0
        and fault_mask == 0
    )

    if fault_mask:
        state = STATE_FAULT
    elif missing_mask:
        state = STATE_INITIALIZING
    elif stale_mask or not_ready_mask:
        state = STATE_DEGRADED
    elif all_ready:
        state = STATE_READY
    else:
        state = STATE_UNKNOWN

    return {
        "state": state,
        "ready": all_ready,
        "required_mask": required_mask,
        "ready_mask": ready_mask,
        "missing_mask": missing_mask,
        "stale_mask": stale_mask,
        "not_ready_mask": not_ready_mask,
        "fault_mask": fault_mask,
        "max_status_age_sec": max(ages) if ages else -1.0,
        "reason": "ready" if all_ready else ",".join(reasons),
    }
