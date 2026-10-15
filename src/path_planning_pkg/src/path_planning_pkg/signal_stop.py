"""Route-scoped stop/go from selected World Model lamps; never raw perception."""
import math
import numpy as np


class SignalStopPolicy:
    def __init__(self, policy, config):
        self.policy, self.config = policy, config
        self.committed = set()
        self.epoch = None
        self.previous_progress = None
        self.reason = 'signal=outside_approach'

    def caps(self, stations, route, world, now, reset_id):
        result = np.full(len(stations), math.inf)
        if self.epoch != reset_id or (route is not None and self.previous_progress is not None
                                      and route.progress < self.previous_progress-50.):
            self.committed.clear()
        self.epoch = reset_id
        if route is None:
            return result
        self.previous_progress = route.progress
        target = next((t for t in self.policy['targets'] if
                       0 <= t['stop_station_m']-route.progress <= self.policy['approach_distance_m']),None)
        if target is None:
            self.reason = 'signal=outside_approach'
            return result
        key=target['stop_line_id'];remaining=target['stop_station_m']-route.progress
        if key in self.committed:
            self.reason='signal=committed; signal_number=%d' % target['number']
            return result
        fresh_route=(route.reference_sha256==self.policy['reference_sha256'] and
                     0<=now-route.header.stamp.to_sec()<=self.policy['context_timeout_sec'])
        selected=[] if world is None or world.localization_reset_id!=reset_id else [
            s for s in world.traffic_signals if s.stop_line_id==key and s.valid and
            set(s.signal_ids).issubset(set(target['signal_ids'])) and s.signal_ids and
            0<=now-s.header.stamp.to_sec()<=self.config['signal_observation_timeout_sec']]
        signal=selected[0] if len(selected)==1 and fresh_route else None
        mask=signal.lamp_mask if signal else 0
        go=bool(mask & 8 and not mask & 2) if target['maneuver']=='left' else bool(mask & 4 and not mask & 3)
        label='LEFT' if mask & 8 else 'RED' if mask & 1 else 'YELLOW' if mask & 2 else 'GREEN' if mask & 4 else 'UNKNOWN'
        self.reason='signal=%s; signal_number=%d; signal_action=%s; signal_distance_m=%.2f' % (
            label,target['number'],'GO' if go else 'STOP',remaining)
        if go:
            if remaining<=self.config['signal_left_commit_distance_m' if target['maneuver']=='left' else 'signal_commit_distance_m']:
                self.committed.add(key)
            return result
        stop_station=target['stop_station_m']-self.config['signal_left_stop_buffer_m' if target['maneuver']=='left' else 'signal_stop_buffer_m']
        return np.sqrt(2*self.config['signal_deceleration_mps2']*np.maximum(0.,stop_station-np.asarray(stations)))
