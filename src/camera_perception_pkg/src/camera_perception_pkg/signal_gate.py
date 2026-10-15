"""Observation windows use approved RouteContext progress, never estimated pose here."""
import math


class SignalRouteGate:
    def __init__(self, policy, enabled=True):
        self.policy = policy
        self.enabled = enabled
        self.context = None
        self.received_at = None
        self.revision = 0
        self.scope = None
        self.clock_regressed = False

    def update(self, context, received_at):
        # A regression/map change invalidates any inference from the previous epoch.
        self.clock_regressed = (self.context is not None and
                                context.header.stamp < self.context.header.stamp)
        if self.context is not None and (
                context.header.stamp < self.context.header.stamp or
                context.map_id != self.context.map_id or
                context.progress < self.context.progress):
            self.revision += 1
        self.context, self.received_at = context, received_at

    def select(self, now, monotonic_now, image_stamp=None):
        if not self.enabled:
            return dict(stop_line_id='', signal_ids=[], number=0)
        context = self.context
        timeout = self.policy['context_timeout_sec']
        if context is None or self.received_at is None or self.clock_regressed:
            return None
        stamp = context.header.stamp.to_sec()
        if (not context.map_id or context.header.frame_id != 'map' or
                context.reference_sha256 != self.policy['reference_sha256'] or
                context.route_complete or not math.isfinite(context.progress) or
                context.progress < 0 or stamp <= 0 or
                not 0 <= now - stamp <= timeout or
                not 0 <= monotonic_now - self.received_at <= timeout or
                (image_stamp is not None and
                 (image_stamp <= 0 or not 0 <= now-image_stamp <= timeout or
                  abs(image_stamp-stamp) > timeout))):
            return None
        for target in self.policy['targets']:
            remaining = target['stop_station_m'] - context.progress
            if 0 <= remaining <= self.policy['approach_distance_m']:
                return dict(target, distance_to_stop_m=remaining)
        return None

    def snapshot(self, now, monotonic_now, image_stamp=None):
        target = self.select(now, monotonic_now, image_stamp)
        scope = target['stop_line_id'] if target is not None else None
        if scope != self.scope:
            self.revision += 1
            self.scope = scope
        return target, self.revision
