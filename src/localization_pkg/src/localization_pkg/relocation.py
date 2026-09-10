"""Conservative, measurement-time detection of a stationary GPS discontinuity.

This is sensor evidence of a relocation, not a simulator respawn event. Missing
GPS/IMU breaks the evidence chain; ordinary blackout recovery cannot arm it.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class RelocationConfig:
    enabled: bool = True
    min_jump_m: float = 15.0
    confirmation_samples: int = 5
    min_confirmation_sec: float = 0.6
    max_confirmation_sec: float = 2.0
    max_gps_gap_sec: float = 0.65
    cluster_radius_m: float = 1.5
    max_imu_displacement_m: float = 3.0
    max_linear_accel_mps2: float = 2.0
    max_angular_rate_radps: float = 0.5
    max_gps_stddev_m: float = 2.0

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('relocation.enabled must be boolean')
        if type(self.confirmation_samples) is not int or self.confirmation_samples < 3:
            raise ValueError('relocation needs at least three distinct GPS samples')
        for key, value in vars(self).items():
            if key != 'enabled' and (not math.isfinite(float(value)) or value <= 0):
                raise ValueError('relocation thresholds must be finite and positive')
        if self.min_confirmation_sec >= self.max_confirmation_sec:
            raise ValueError('invalid relocation confirmation time bounds')
        if self.cluster_radius_m >= self.min_jump_m or self.max_imu_displacement_m >= self.min_jump_m:
            raise ValueError('relocation jump must exceed cluster and IMU motion bounds')


class RelocationDetector:
    def __init__(self, config=None):
        self.config = config or RelocationConfig()
        self.clear()

    def clear(self):
        self.previous = None
        self.candidate = None
        self.last_imu_stamp = None
        self.quiet_since_gps = False
        self.diagnostic = ''

    @property
    def pending(self):
        return self.candidate is not None

    def invalidate(self, reason):
        self.previous = None
        self.candidate = None
        self.quiet_since_gps = False
        self.diagnostic = reason

    def expire(self, stamp):
        if self.previous and stamp - self.previous[0] > self.config.max_gps_gap_sec:
            self.invalidate('GPS continuity lost; relocation evidence cleared')
        elif self.candidate and stamp - self.candidate[0] > self.config.max_confirmation_sec:
            self.invalidate('relocation confirmation timed out')

    def observe_imu(self, stamp, world_acceleration, angular_velocity, max_gap):
        self.expire(stamp)
        if self.last_imu_stamp is not None and not 0 < stamp - self.last_imu_stamp <= max_gap:
            self.invalidate('IMU continuity lost; relocation evidence cleared')
        self.last_imu_stamp = stamp
        quiet = (np.linalg.norm(world_acceleration) <= self.config.max_linear_accel_mps2
                 and np.linalg.norm(angular_velocity) <= self.config.max_angular_rate_radps)
        if not quiet:
            self.invalidate('IMU motion inconsistent with stationary relocation')
        # An invalidated interval cannot become valid again until a new GPS anchor.
        return quiet

    def observe_gps(self, stamp, position, local_position, residual, noise, quiet):
        c = self.config
        if not c.enabled:
            return 'none'
        self.expire(stamp)
        if (self.last_imu_stamp is None or not quiet
                or np.sqrt(max(np.linalg.eigvalsh(noise))) > c.max_gps_stddev_m):
            self.invalidate('GPS or IMU quality insufficient for relocation')
            return 'none'
        previous = self.previous
        self.previous = (stamp, position.copy(), local_position.copy())
        continuous = (previous is not None and 0 < stamp - previous[0] <= c.max_gps_gap_sec
                      and self.quiet_since_gps)
        self.quiet_since_gps = True
        if not continuous:
            self.candidate = None
            return 'none'
        if self.candidate is not None:
            start, center, count = self.candidate
            if np.linalg.norm(position - center) > c.cluster_radius_m:
                self.candidate = None
                self.diagnostic = 'GPS relocation cluster inconsistent'
                return 'none'
            count += 1
            self.candidate = (start, center, count)
            if count >= c.confirmation_samples and stamp - start >= c.min_confirmation_sec:
                self.candidate = None
                self.diagnostic = 'sensor relocation confirmed; stationary GPS cluster'
                return 'confirmed'
            self.diagnostic = 'GPS relocation candidate %d/%d' % (count, c.confirmation_samples)
            return 'candidate'
        gps_step = position - previous[1]
        imu_step = local_position - previous[2]
        if (np.linalg.norm(residual[:2]) >= c.min_jump_m
                and np.linalg.norm((gps_step - imu_step)[:2]) >= c.min_jump_m
                and np.linalg.norm(imu_step) <= c.max_imu_displacement_m):
            self.candidate = (stamp, position.copy(), 1)
            self.diagnostic = 'GPS relocation candidate 1/%d' % c.confirmation_samples
            return 'candidate'
        return 'none'
