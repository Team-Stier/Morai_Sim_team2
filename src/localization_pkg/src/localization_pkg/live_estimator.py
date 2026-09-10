"""Measurement-time GPS/IMU position filter and continuous local odometry.

The six Kalman states are map position and velocity. Attitude is an external
IMU observation, not an unobservable attitude/bias estimate. GPS position
corrections affect map pose; local position accumulates prediction increments
only. Its deliberately conservative uncertainty budget does not shrink when
GPS corrects map position. This development estimator is not a driving gate.
"""
from dataclasses import dataclass
import math

import numpy as np
from pyproj import Transformer
from .relocation import RelocationDetector


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
    def __init__(self, config=None, gps_translation_m=(0., 0., 1.3), relocation_config=None):
        self.config = config or EstimatorConfig()
        self.relocation = RelocationDetector(relocation_config)
        self.gps_translation = vector(gps_translation_m, 3)
        self.reset()

    def reset(self):
        self.relocation.clear()
        self.gps_reinitialized = False
        self.gps_diagnostic = ""
        self.initialized = False
        self.stamp = None
        self.last_imu_stamp = None
        self.last_gps_stamp = None
        self.last_seen_gps_stamp = None
        self.last_imu = None
        self.pending_gps = None
        self.state = np.zeros(6)
        self.P = np.eye(6)
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
        self.P = np.zeros((6, 6))
        self.P[:3, :3] = noise
        self.P[3:, 3:] = np.eye(3)*self.config.initial_velocity_stddev_mps**2
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
            specific_force = 0.5*(self.acceleration+acceleration)
            world_acceleration = R_mid.dot(specific_force) - np.array(
                [0., 0., self.config.gravity_mps2])
            delta_position = self.state[3:]*dt + world_acceleration*dt*dt*0.5
            candidate = self.state.copy()
            candidate[:3] += delta_position
            candidate[3:] += world_acceleration*dt
            F = np.eye(6)
            F[:3, 3:] = np.eye(3)*dt
            G = np.vstack((np.eye(3)*0.5*dt*dt, np.eye(3)*dt))
            attitude_acceleration_jacobian = -R_mid.dot(skew(specific_force))
            accel_noise = np.eye(3)*self.config.accelerometer_noise_stddev_mps2**2
            accel_noise += attitude_acceleration_jacobian.dot(
                0.5*(self.attitude_cov+attitude_cov)).dot(attitude_acceleration_jacobian.T)
            candidate_cov = covariance(F.dot(self.P).dot(F.T)+G.dot(accel_noise).dot(G.T), 6)
            if not np.isfinite(candidate).all():
                raise ValueError('nonfinite filter state')
            # Fully correlated integral bound, not an overconfident cloned map P.
            self.local_position_stddev += np.sqrt(np.maximum(
                np.diag(self.P)[3:], 0))*dt + np.sqrt(np.diag(accel_noise))*dt*dt*0.5
            self.local_position += delta_position
            self.state, self.P = candidate, candidate_cov
        self.q, self.acceleration, self.omega = q, acceleration, omega
        self.attitude_cov, self.omega_cov = attitude_cov, omega_cov
        self.stamp = stamp

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
            self.relocation.observe_imu(sample.stamp, rotation(values[0]).dot(values[1]) -
                                        np.array([0., 0., self.config.gravity_mps2]), values[2],
                                        self.config.max_integration_step_sec)
            self.last_rejection = ''
            return True
        except (ValueError, np.linalg.LinAlgError) as error:
            self.last_rejection = str(error)
            self.relocation.invalidate('IMU rejected: ' + str(error))
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
                self.relocation.last_imu_stamp = self.last_imu_stamp
                self.relocation.observe_gps(sample.stamp, self.state[:3], self.local_position,
                                            np.zeros(3), noise, True)
                return True
            self._advance(sample.stamp, values)
            R = rotation(self.q)
            arm_jacobian = -R.dot(skew(self.gps_translation))
            noise += arm_jacobian.dot(self.attitude_cov).dot(arm_jacobian.T)
            noise += np.outer(self.state[3:], self.state[3:])*self.config.ingress_timing_stddev_sec**2
            position = antenna-R.dot(self.gps_translation)
            residual = position-self.state[:3]
            world_accel = rotation(values[0]).dot(values[1]) - np.array([0., 0., self.config.gravity_mps2])
            quiet = (np.linalg.norm(world_accel) <= self.relocation.config.max_linear_accel_mps2
                     and np.linalg.norm(values[2]) <= self.relocation.config.max_angular_rate_radps)
            decision = self.relocation.observe_gps(sample.stamp, position, self.local_position,
                                                   residual, noise, quiet)
            self.last_seen_gps_stamp = sample.stamp
            if decision == 'confirmed':
                # Keep the odom frame continuous; re-anchor map pose and clear old velocity.
                self._initialize(sample, values)
                self.gps_reinitialized = True
                self.gps_diagnostic = self.relocation.diagnostic
                self.last_rejection = ''
                return True
            if decision == 'candidate':
                self.gps_diagnostic = self.relocation.diagnostic
                return False
            innovation = self.P[:3, :3]+noise
            distance = float(residual.dot(np.linalg.solve(innovation, residual)))
            self.last_seen_gps_stamp = sample.stamp
            if not math.isfinite(distance) or distance > self.config.gps_innovation_gate_chi2:
                raise ValueError('GPS innovation rejected (chi2=%.3f)' % distance)
            gain = np.linalg.solve(innovation, self.P[:, :3].T).T
            candidate = self.state+gain.dot(residual)
            H = np.hstack((np.eye(3), np.zeros((3, 3))))
            A = np.eye(6)-gain.dot(H)
            candidate_cov = covariance(A.dot(self.P).dot(A.T)+gain.dot(noise).dot(gain.T), 6)
            if not np.isfinite(candidate).all():
                raise ValueError('nonfinite GPS correction')
            self.state, self.P = candidate, candidate_cov
            self.last_gps_stamp = sample.stamp
            self.gps_diagnostic = ""
            self.last_rejection = ''
            return True
        except (ValueError, np.linalg.LinAlgError) as error:
            self.last_rejection = self.gps_diagnostic = str(error)
            self.relocation.invalidate('GPS rejected: ' + str(error))
            return False

    def snapshot(self):
        if not self.initialized:
            return None
        R = rotation(self.q)
        velocity_body = R.T.dot(self.state[3:])
        map_cov = np.zeros((6, 6))
        map_cov[:3, :3], map_cov[3:, 3:] = self.P[:3, :3], self.attitude_cov
        local_cov = np.zeros((6, 6))
        local_cov[:3, :3] = np.diag(np.maximum(self.local_position_stddev, 1e-3)**2)
        local_cov[3:, 3:] = self.attitude_cov
        twist_cov = np.zeros((6, 6))
        J = skew(velocity_body)
        twist_cov[:3, :3] = R.T.dot(self.P[3:, 3:]).dot(R)+J.dot(self.attitude_cov).dot(J.T)
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
