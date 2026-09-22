"""Measurement-time GPS/IMU position filter and continuous local odometry.

The nine Kalman states are map position, velocity and body accelerometer bias.
Attitude is an external IMU observation, not an estimated attitude. GPS position
corrections affect map pose; local position accumulates prediction increments
only. Its deliberately conservative uncertainty budget does not shrink when
GPS corrects map position. This development estimator is not a driving gate.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from pyproj import Transformer


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError('expected finite vector of size %d' % size)
    return result.copy()


def quaternion(value):
    result = vector(value, 4)
    norm = np.linalg.norm(result)
    if norm < 1e-8 or abs(norm - 1.0) > 0.1:
        raise ValueError('invalid IMU quaternion norm')
    return result / norm


def rotation(value):
    x, y, z, w = quaternion(value)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def skew(value):
    x, y, z = value
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def slerp(first, second, fraction):
    first, second = quaternion(first), quaternion(second)
    dot = float(first.dot(second))
    if dot < 0:
        second, dot = -second, -dot
    dot = np.clip(dot, -1., 1.)
    if dot > 0.9995:
        result = first + fraction * (second-first)
        return result / np.linalg.norm(result)
    angle = math.acos(dot)
    return (math.sin((1-fraction)*angle)*first +
            math.sin(fraction*angle)*second) / math.sin(angle)


def covariance(value, size, floor_stddev=0.):
    result = np.asarray(value, dtype=float).reshape(size, size)
    if not np.isfinite(result).all() or not np.allclose(result, result.T, atol=1e-8):
        raise ValueError('nonfinite or asymmetric covariance')
    values, basis = np.linalg.eigh((result+result.T)*0.5)
    if values.min() < -1e-8:
        raise ValueError('covariance is not positive semidefinite')
    return (basis * np.maximum(values, floor_stddev**2)).dot(basis.T)


@dataclass
class EstimatorConfig:
    gravity_mps2: float = 9.80665
    max_integration_step_sec: float = 0.30
    gps_position_stddev_floor_m: float = 0.25
    gps_altitude_stddev_floor_m: float = 0.5
    orientation_stddev_floor_rad: float = 0.02
    accelerometer_noise_stddev_mps2: float = 0.5
    initial_velocity_stddev_mps: float = 2.0
    gps_innovation_gate_chi2: float = 25.0
    ingress_timing_stddev_sec: float = 0.03
    initial_accelerometer_bias_stddev_mps2: float = 0.3
    accelerometer_bias_random_walk_mps2_sqrt_sec: float = 0.005
    stationary_window_sec: float = 2.0
    stationary_gps_radius_m: float = 0.08
    stationary_speed_mps: float = 0.15
    stationary_acceleration_mps2: float = 0.15
    stationary_angular_rate_radps: float = 0.02
    stationary_velocity_stddev_mps: float = 0.10
    stationary_gps_stddev_m: float = 0.6

    def __post_init__(self):
        if any(not math.isfinite(float(v)) or v <= 0 for v in vars(self).values()):
            raise ValueError('estimator parameters must be finite and positive')


@dataclass
class ImuObservation:
    stamp: float
    orientation_xyzw: object
    acceleration_mps2: object
    angular_velocity_radps: object
    orientation_covariance: object = None
    angular_velocity_covariance: object = None


@dataclass
class GpsObservation:
    stamp: float
    position_m: object
    position_covariance: object = None


class MapProjector:
    def __init__(self, epsg, origin_utm_m):
        if int(epsg) != 32652:
            raise ValueError('this development map contract requires EPSG:32652')
        self.origin = vector(origin_utm_m, 3)
        self.transformer = Transformer.from_crs(4326, int(epsg), always_xy=True)

    def project(self, latitude, longitude, altitude):
        if not all(math.isfinite(float(v)) for v in (latitude, longitude, altitude)):
            raise ValueError('nonfinite GPS fix')
        if not (0 <= latitude <= 84 and 126 <= longitude <= 132):
            raise ValueError('GPS fix outside the approved UTM zone 52N')
        east, north = self.transformer.transform(longitude, latitude, errcheck=True)
        return vector([east, north, altitude], 3) - self.origin


class GpsImuEstimator:
    def __init__(self, config=None, gps_translation_m=(0., 0., 1.3)):
        self.config = config or EstimatorConfig()
        self.gps_translation = vector(gps_translation_m, 3)
        self.reset()

    def reset(self):
        self.gps_reinitialized = False
        self.gps_diagnostic = ""
        self.initialized = False
        self.stamp = None
        self.last_imu_stamp = None
        self.last_gps_stamp = None
        self.last_seen_gps_stamp = None
        self.last_imu = None
        self.pending_gps = None
        self.state = np.zeros(9)
        self.P = np.eye(9)
        self.stationary_gps = deque(maxlen=256)
        self.quiet_since = None
        self.last_stationary_update = None
        self.local_position = np.zeros(3)
        self.local_position_stddev = np.zeros(3)
        self.q = np.array([0., 0., 0., 1.])
        self.acceleration = np.zeros(3)
        self.omega = np.zeros(3)
        self.attitude_cov = np.eye(3)
        self.omega_cov = np.eye(3)
        self.last_rejection = ''

    def _imu_values(self, sample):
        if not math.isfinite(sample.stamp) or sample.stamp <= 0:
            raise ValueError('invalid IMU stamp')
        q = quaternion(sample.orientation_xyzw)
        acceleration = vector(sample.acceleration_mps2, 3)
        omega = vector(sample.angular_velocity_radps, 3)
        floor = self.config.orientation_stddev_floor_rad
        attitude_cov = covariance(np.eye(3)*floor**2 if sample.orientation_covariance is None
                                  else sample.orientation_covariance, 3, floor)
        omega_cov = covariance(np.eye(3)*0.03**2 if sample.angular_velocity_covariance is None
                               else sample.angular_velocity_covariance, 3, 0.01)
        return q, acceleration, omega, attitude_cov, omega_cov

    def _gps_values(self, sample):
        if not math.isfinite(sample.stamp) or sample.stamp <= 0:
            raise ValueError('invalid GPS stamp')
        position = vector(sample.position_m, 3)
        c = self.config
        floors = np.array([c.gps_position_stddev_floor_m**2]*2 +
                          [c.gps_altitude_stddev_floor_m**2])
        noise = np.zeros((3, 3)) if sample.position_covariance is None else covariance(
            sample.position_covariance, 3)
        # Floors are additive model error, including NMEA quantization/datum.
        return position, noise + np.diag(floors)

    def _initialize(self, gps, imu_values):
        q, acceleration, omega, attitude_cov, omega_cov = imu_values
        antenna, noise = self._gps_values(gps)
        R = rotation(q)
        arm_jacobian = -R.dot(skew(self.gps_translation))
        noise += arm_jacobian.dot(attitude_cov).dot(arm_jacobian.T)
        self.state[:3] = antenna - R.dot(self.gps_translation)
        self.state[3:] = 0.
        self.P = np.zeros((9, 9))
        self.P[:3, :3] = noise
        self.P[3:6, 3:6] = np.eye(3)*self.config.initial_velocity_stddev_mps**2
        self.P[6:, 6:] = np.eye(3)*self.config.initial_accelerometer_bias_stddev_mps2**2
        # A relocation is not evidence that the vehicle stopped. Discard all
        # previous motion/calibration evidence, while preserving local odometry.
        self.stationary_gps.clear()
        self.quiet_since = None
        self.last_stationary_update = None
        self.q, self.acceleration, self.omega = q, acceleration, omega
        self.attitude_cov, self.omega_cov = attitude_cov, omega_cov
        self.stamp, self.last_gps_stamp = gps.stamp, gps.stamp
        self.initialized = True
        self.pending_gps = None

    def _advance(self, stamp, values):
        dt = stamp-self.stamp
        if dt < -1e-9 or dt > self.config.max_integration_step_sec:
            raise ValueError('state integration interval invalid; reset required')
        q, acceleration, omega, attitude_cov, omega_cov = values
        if dt > 0:
            R_mid = rotation(slerp(self.q, q, 0.5))
            specific_force = 0.5*(self.acceleration+acceleration)-self.state[6:]
            world_acceleration = R_mid.dot(specific_force) - np.array(
                [0., 0., self.config.gravity_mps2])
            delta_position = self.state[3:6]*dt + world_acceleration*dt*dt*0.5
            candidate = self.state.copy()
            candidate[:3] += delta_position
            candidate[3:6] += world_acceleration*dt
            F = np.eye(9)
            F[:3, 3:6] = np.eye(3)*dt
            F[:3, 6:] = -R_mid*0.5*dt*dt
            F[3:6, 6:] = -R_mid*dt
            G = np.vstack((np.eye(3)*0.5*dt*dt, np.eye(3)*dt, np.zeros((3, 3))))
            attitude_acceleration_jacobian = -R_mid.dot(skew(specific_force))
            accel_noise = np.eye(3)*self.config.accelerometer_noise_stddev_mps2**2
            accel_noise += attitude_acceleration_jacobian.dot(
                0.5*(self.attitude_cov+attitude_cov)).dot(attitude_acceleration_jacobian.T)
            process_noise = G.dot(accel_noise).dot(G.T)
            # Continuous body-bias random walk integrated through position and
            # velocity. Retain cross-covariances; unknown bias must grow blackout P.
            B = np.eye(9)
            B[:3, :3] = B[3:6, 3:6] = -R_mid
            moments = np.array([[dt**5/20, dt**4/8, dt**3/6],
                                [dt**4/8, dt**3/3, dt**2/2],
                                [dt**3/6, dt**2/2, dt]])
            process_noise += (B.dot(np.kron(moments, np.eye(3))).dot(B.T) *
                              self.config.accelerometer_bias_random_walk_mps2_sqrt_sec**2)
            candidate_cov = covariance(F.dot(self.P).dot(F.T)+process_noise, 9)
            if not np.isfinite(candidate).all():
                raise ValueError('nonfinite filter state')
            # Fully correlated integral bound, not an overconfident cloned map P.
            self.local_position_stddev += np.sqrt(np.maximum(
                np.diag(self.P)[3:6], 0))*dt + np.sqrt(np.diag(
                    accel_noise+R_mid.dot(self.P[6:, 6:]).dot(R_mid.T)))*dt*dt*0.5
            self.local_position += delta_position
            self.state, self.P = candidate, candidate_cov
        self.q, self.acceleration, self.omega = q, acceleration, omega
        self.attitude_cov, self.omega_cov = attitude_cov, omega_cov
        self.stamp = stamp
        c = self.config
        residual_accel = rotation(q).dot(acceleration-self.state[6:]) - np.array(
            [0., 0., c.gravity_mps2])
        if (np.linalg.norm(residual_accel) > c.stationary_acceleration_mps2 or
                np.linalg.norm(omega) > c.stationary_angular_rate_radps):
            self.quiet_since = None
            self.stationary_gps.clear()
        elif self.quiet_since is None:
            self.quiet_since = stamp

    def _correct_stationary(self, position, noise):
        """GPS-supported soft ZUPT, once per new fix, never from quiet IMU alone.

        No position pinning and no repeated stale zero-speed observations during
        blackout. Motion in a tunnel remains inertial until another independently
        validated speed source is available.
        """
        c = self.config
        if (self.quiet_since is None or
                np.sqrt(np.linalg.eigvalsh(noise).max()) > c.stationary_gps_stddev_m):
            self.stationary_gps.clear()
            return
        # Only contiguous fixes count; integration bound comes from central timing.
        if self.stationary_gps and self.stamp-self.stationary_gps[-1][0] > c.max_integration_step_sec:
            self.stationary_gps.clear()
        self.stationary_gps.append((self.stamp, position.copy()))
        while len(self.stationary_gps) > 1 and self.stamp-self.stationary_gps[1][0] >= c.stationary_window_sec:
            self.stationary_gps.popleft()
        if (len(self.stationary_gps) < 5 or
                self.stamp-self.stationary_gps[0][0] < c.stationary_window_sec or
                self.stamp-self.quiet_since < c.stationary_window_sec or
                np.linalg.norm(self.state[3:6]) > c.stationary_speed_mps):
            return
        positions = np.array([p for _, p in self.stationary_gps])
        if np.max(np.linalg.norm(positions-positions[0], axis=1)) > c.stationary_gps_radius_m:
            return
        H = np.zeros((3, 9))
        H[:, 3:6] = np.eye(3)
        noise = np.eye(3)*c.stationary_velocity_stddev_mps**2
        innovation = H.dot(self.P).dot(H.T)+noise
        gain = np.linalg.solve(innovation, H.dot(self.P)).T
        candidate = self.state-gain.dot(self.state[3:6])
        A = np.eye(9)-gain.dot(H)
        candidate_cov = covariance(A.dot(self.P).dot(A.T)+gain.dot(noise).dot(gain.T), 9)
        self.state, self.P = candidate, candidate_cov
        self.last_stationary_update = self.stamp

    def process_imu(self, sample):
        try:
            values = self._imu_values(sample)
            if self.last_imu_stamp is not None and sample.stamp <= self.last_imu_stamp:
                raise ValueError('duplicate or regressing IMU stamp')
            if self.initialized and sample.stamp < self.stamp:
                raise ValueError('IMU older than filter state')
            if not self.initialized:
                self.last_imu, self.last_imu_stamp = sample, sample.stamp
                if self.pending_gps is None:
                    return False
                dt = sample.stamp-self.pending_gps.stamp
                if dt < 0 or dt > self.config.max_integration_step_sec:
                    self.pending_gps = None
                    return False
                self._initialize(self.pending_gps, values)
            self._advance(sample.stamp, values)
            self.last_imu, self.last_imu_stamp = sample, sample.stamp
            self.last_rejection = ''
            return True
        except (ValueError, np.linalg.LinAlgError) as error:
            self.quiet_since = None
            self.stationary_gps.clear()
            self.last_rejection = str(error)
            return False

    def process_gps(self, sample, interpolated_imu=None):
        self.gps_reinitialized = False
        try:
            antenna, noise = self._gps_values(sample)
            if self.last_seen_gps_stamp is not None and sample.stamp <= self.last_seen_gps_stamp:
                raise ValueError('duplicate or regressing GPS stamp')
            if self.initialized and sample.stamp < self.stamp:
                raise ValueError('delayed GPS older than processed state')
            values = self._imu_values(interpolated_imu or self.last_imu) if (
                interpolated_imu is not None or self.last_imu is not None) else None
            if values is None:
                self.pending_gps = sample
                self.last_seen_gps_stamp = sample.stamp
                return False
            if self.last_imu_stamp is not None and abs(sample.stamp-self.last_imu_stamp) > self.config.max_integration_step_sec:
                raise ValueError('GPS has no recent IMU attitude')
            if not self.initialized:
                self._initialize(sample, values)
                self.last_seen_gps_stamp = sample.stamp
                self.last_rejection = ''
                return True
            self._advance(sample.stamp, values)
            R = rotation(self.q)
            arm_jacobian = -R.dot(skew(self.gps_translation))
            noise += arm_jacobian.dot(self.attitude_cov).dot(arm_jacobian.T)
            noise += np.outer(self.state[3:6], self.state[3:6])*self.config.ingress_timing_stddev_sec**2
            position = antenna-R.dot(self.gps_translation)
            residual = position-self.state[:3]
            innovation = self.P[:3, :3]+noise
            distance = float(residual.dot(np.linalg.solve(innovation, residual)))
            self.last_seen_gps_stamp = sample.stamp
            if not math.isfinite(distance):
                raise ValueError('GPS innovation rejected (chi2=%.3f)' % distance)
            if distance > self.config.gps_innovation_gate_chi2:
                # User-selected simulator policy: accept this fix as the new map anchor.
                self._initialize(sample, values)
                self.gps_reinitialized = True
                self.gps_diagnostic = 'GPS innovation reset (chi2=%.3f > %.3f)' % (
                    distance, self.config.gps_innovation_gate_chi2)
                self.last_rejection = ''
                return True
            gain = np.linalg.solve(innovation, self.P[:, :3].T).T
            candidate = self.state+gain.dot(residual)
            H = np.hstack((np.eye(3), np.zeros((3, 6))))
            A = np.eye(9)-gain.dot(H)
            candidate_cov = covariance(A.dot(self.P).dot(A.T)+gain.dot(noise).dot(gain.T), 9)
            if not np.isfinite(candidate).all():
                raise ValueError('nonfinite GPS correction')
            self.state, self.P = candidate, candidate_cov
            self._correct_stationary(position, noise)
            self.last_gps_stamp = sample.stamp
            self.gps_diagnostic = ""
            self.last_rejection = ''
            return True
        except (ValueError, np.linalg.LinAlgError) as error:
            self.last_rejection = self.gps_diagnostic = str(error)
            return False

    def snapshot(self):
        if not self.initialized:
            return None
        R = rotation(self.q)
        velocity_body = R.T.dot(self.state[3:6])
        map_cov = np.zeros((6, 6))
        map_cov[:3, :3], map_cov[3:, 3:] = self.P[:3, :3], self.attitude_cov
        local_cov = np.zeros((6, 6))
        local_cov[:3, :3] = np.diag(np.maximum(self.local_position_stddev, 1e-3)**2)
        local_cov[3:, 3:] = self.attitude_cov
        twist_cov = np.zeros((6, 6))
        J = skew(velocity_body)
        twist_cov[:3, :3] = R.T.dot(self.P[3:6, 3:6]).dot(R)+J.dot(self.attitude_cov).dot(J.T)
        twist_cov[3:, 3:] = self.omega_cov
        # Odom axes remain aligned with map ENU, so its rotation is identity.
        # This is exactly T_map_base * inverse(T_odom_base), with both poses
        # represented at the same measurement stamp and including full attitude.
        return dict(stamp=self.stamp, map_position=self.state[:3].copy(),
                    local_position=self.local_position.copy(), orientation=self.q.copy(),
                    map_odom_translation=(self.state[:3]-self.local_position).copy(),
                    map_odom_orientation=np.array([0., 0., 0., 1.]),
                    velocity_body=velocity_body, angular_velocity=self.omega.copy(),
                    map_pose_covariance=map_cov, local_pose_covariance=local_cov,
                    twist_covariance=twist_cov,
                    map_position_stddev=float(np.sqrt(self.P[0, 0] + self.P[1, 1])),
                    local_position_stddev=float(np.linalg.norm(self.local_position_stddev[:2])),
                    yaw_stddev=float(math.sqrt(self.attitude_cov[2, 2])))
