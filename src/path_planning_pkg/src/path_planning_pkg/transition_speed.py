"""Route-station speed caps around changes of motion planner."""

import math

import numpy as np

from .planner_mode_manager import FRENET, HYBRID_ASTAR, PlannerModeManager


class TransitionSpeedPolicy:
    """Add a continuous speed cap on both sides of each planner-mode boundary.

    The cap is an upper bound in m/s, not a replacement for lane, curvature,
    collision, or acceleration limits.  A Frenet point outside a transition
    returns infinity, while a Hybrid A* point returns its normal mode cap.
    """

    def __init__(self, config):
        manager = PlannerModeManager.from_mapping(config['planner_mode'])
        self.zones = manager.zones
        self.route_length_m = manager.route_length_m
        self.boundaries = tuple(
            (left.end_s, left.planner, right.planner)
            for left, right in zip(self.zones[:-1], self.zones[1:])
            if left.planner != right.planner
        )
        self.boundary_speed_mps = float(config['mode_transition_speed_kph']) / 3.6
        self.hybrid_speed_mps = float(config['hybrid_speed_cap_kph']) / 3.6
        self.high_cruise_mps = float(config['high_cruise_kph']) / 3.6
        self.normal_margin_mps = float(config['normal_cruise_margin_kph']) / 3.6
        self.acceleration_mps2 = float(config['acceleration_mps2'])
        self.deceleration_mps2 = float(config['deceleration_mps2'])
        self.minimum_distance_m = float(config['mode_transition_min_distance_m'])
        self.reaction_sec = float(config['mode_transition_reaction_sec'])
        if (not all(math.isfinite(value) for value in (
                self.boundary_speed_mps, self.hybrid_speed_mps,
                self.high_cruise_mps, self.normal_margin_mps,
                self.acceleration_mps2, self.deceleration_mps2,
                self.minimum_distance_m, self.reaction_sec)) or
                min(self.boundary_speed_mps, self.hybrid_speed_mps,
                    self.high_cruise_mps, self.acceleration_mps2,
                    self.deceleration_mps2) <= 0.0 or
                min(self.normal_margin_mps, self.minimum_distance_m,
                    self.reaction_sec) < 0.0):
            raise ValueError('Transition speed settings must be finite and physically valid')

    def _nominal(self, mode, boundary_s, side, reference_lane):
        if mode == HYBRID_ASTAR:
            return self.hybrid_speed_mps
        station = np.asarray(reference_lane.s, dtype=float)
        limits = np.asarray(reference_lane.limits, dtype=float)
        if (station.ndim != 1 or limits.shape != station.shape or not len(station) or
                not np.isfinite(station).all() or not np.isfinite(limits).all() or
                np.any(np.diff(station) <= 0.0)):
            raise ValueError('Reference lane needs ordered stations and matching finite limits')
        index = int(np.searchsorted(station, boundary_s, side='left'))
        index = min(max(index - (side == 'before'), 0), len(station) - 1)
        limit = limits[index]
        return (self.high_cruise_mps if limit < 0.0 else
                max(0.0, limit - self.normal_margin_mps))

    def _distance(self, start_speed, end_speed):
        acceleration = (self.acceleration_mps2 if end_speed >= start_speed else
                        self.deceleration_mps2)
        maximum_speed = max(start_speed, end_speed)
        return max(self.minimum_distance_m,
                   maximum_speed * abs(end_speed - start_speed) / acceleration +
                   self.reaction_sec * maximum_speed)

    def caps(self, route_s, mode, reference_lane):
        """Return one speed cap per route station, including the adjacent zone tail."""
        if mode not in (FRENET, HYBRID_ASTAR):
            raise ValueError('Unsupported planner mode: {}'.format(mode))
        station = np.asarray(route_s, dtype=float)
        if station.ndim != 1 or not np.isfinite(station).all():
            raise ValueError('Route stations must be a finite one-dimensional array')
        if not len(station):
            return np.empty(0, dtype=float)

        ends = np.array([zone.end_s for zone in self.zones])
        zone_indices = np.clip(np.searchsorted(ends, station, side='right'),
                               0, len(self.zones) - 1)
        point_modes = np.array([self.zones[index].planner for index in zone_indices])
        base_caps = np.where(point_modes == HYBRID_ASTAR,
                             self.hybrid_speed_mps, math.inf)
        ramp_caps = np.full(len(station), math.inf)

        for boundary_s, left_mode, right_mode in self.boundaries:
            before_speed = self._nominal(left_mode, boundary_s, 'before', reference_lane)
            after_speed = self._nominal(right_mode, boundary_s, 'after', reference_lane)
            before_distance = self._distance(before_speed, self.boundary_speed_mps)
            after_distance = self._distance(self.boundary_speed_mps, after_speed)

            # A same-mode zone may precede the immediately adjacent zone.  Let
            # its points join a long braking ramp (e.g. 150 km/h to 30 km/h).
            before = ((station >= boundary_s - before_distance) &
                      (station < boundary_s) & (point_modes == left_mode))
            fraction = (station[before] - (boundary_s - before_distance)) / before_distance
            ramp = before_speed + (self.boundary_speed_mps - before_speed) * fraction
            ramp_caps[before] = np.minimum(ramp_caps[before], ramp)

            after = ((station >= boundary_s) &
                     (station <= boundary_s + after_distance) &
                     (point_modes == right_mode))
            fraction = (station[after] - boundary_s) / after_distance
            ramp = self.boundary_speed_mps + (after_speed - self.boundary_speed_mps) * fraction
            ramp_caps[after] = np.minimum(ramp_caps[after], ramp)

        return np.where(np.isfinite(ramp_caps), ramp_caps, base_caps)
