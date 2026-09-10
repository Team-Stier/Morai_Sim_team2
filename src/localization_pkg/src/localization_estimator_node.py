#!/usr/bin/env python3
"""Development GPS/IMU adapter. Sensor clocks and central contracts are authoritative."""
import bisect
import math
from pathlib import Path
import threading
import time

import numpy as np
import rospkg
import rospy
import tf2_ros
import yaml
from common_msgs_pkg.msg import EgoState, LocalizationStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from localization_pkg.relocation import RelocationConfig
from localization_pkg.live_estimator import (
    EstimatorConfig, GpsImuEstimator, GpsObservation, ImuObservation,
    MapProjector, covariance, slerp)


def load(path):
    with path.open() as stream:
        return yaml.safe_load(stream)


class LocalizationNode:
    def __init__(self):
        root = Path(rospkg.RosPack().get_path('ros_architecture_pkg')) / 'config'
        frames = load(root / 'tf/frame_contract.yaml')
        mounts = load(root / 'tf/sensor_extrinsics.yaml')
        projection = load(root / 'tf/map_projection.yaml')
        self.timing = load(root / 'timestamp/timestamp_contract.yaml')['development_localization_profile']
        dynamic = [t for t in frames['transforms'] if t['type'] == 'dynamic']
        if {(t['parent'], t['child']) for t in dynamic} != {('map', 'odom'), ('odom', 'base_link')}:
            raise ValueError('unexpected dynamic TF tree')
        for entry in dynamic:
            if not (entry['publish_enabled'] and entry['activation_scope'] == 'development_only'
                    and entry['runtime_publisher_owner'] == 'localization_pkg'):
                raise ValueError('dynamic TF gate closed')
        sensor = {m['key']: m for m in mounts['sensor_mounts']}
        for key in ('imu', 'gps'):
            if not sensor[key]['publish_enabled']:
                raise ValueError('sensor TF gate closed')
            if sensor[key]['candidate_ros_pose']['rotation_rpy_rad'] != [0., 0., 0.]:
                raise ValueError('adapter requires aligned sensor axes')
        if sensor['imu']['candidate_ros_pose']['translation_m'] != [0., 0., 0.]:
            raise ValueError('adapter requires IMU at body origin')
        if not projection['runtime_activation_allowed']:
            raise ValueError('projection gate closed')
        self.projector = MapProjector(projection['epsg'], projection['origin_utm_m'])
        config = EstimatorConfig(**{key: rospy.get_param('~' + key, value)
                                   for key, value in vars(EstimatorConfig()).items()})
        config.max_integration_step_sec = self.timing['max_integration_step_sec']
        self.core = GpsImuEstimator(config, sensor['gps']['candidate_ros_pose']['translation_m'],
                                    RelocationConfig(**rospy.get_param('~relocation', {})))
        self.lock = threading.RLock()
        self.epoch = 0
        self.status_wall = -math.inf
        self.clock_ns = 0
        self.clock_wall = time.monotonic()
        self.clear()
        self.ego_pub = rospy.Publisher('/molit/localization/ego_state', EgoState, queue_size=10)
        self.odom_pub = rospy.Publisher('/molit/localization/local/odometry', Odometry, queue_size=10)
        self.status_pub = rospy.Publisher('/molit/localization/status', LocalizationStatus, queue_size=10, latch=True)
        self.tf = tf2_ros.TransformBroadcaster()
        self.subs = [rospy.Subscriber('/molit/sensors/imu/data', Imu, self.imu, queue_size=100),
                     rospy.Subscriber('/molit/sensors/gps/fix', NavSatFix, self.gps, queue_size=30)]

    def clear(self):
        self.core.reset()
        self.queue = []
        self.imu_history = []
        self.seen = {'imu': 0, 'gps': 0}
        self.arrival = {'imu': None, 'gps': None}
        self.latest = None
        self.output_stamp = rospy.Time(0)
        self.reason = 'waiting for GPS and IMU'

    def clock(self):
        now = rospy.Time.now()
        wall = time.monotonic()
        ns = now.to_nsec()
        if ns < self.clock_ns:
            self.epoch += 1
            self.clear()
            self.reason = 'ROS clock regression: buffers cleared'
        if ns != self.clock_ns:
            self.clock_wall = wall
        self.clock_ns = ns
        return now, wall

    def accept(self, message, kind, frame):
        now, wall = self.clock()
        stamp = message.header.stamp.to_nsec()
        if (message.header.frame_id != frame or stamp <= 0 or stamp > now.to_nsec()
                or stamp <= self.seen[kind]
                or (now-message.header.stamp).to_sec() > self.timing[kind + '_timeout_sec']):
            raise ValueError('invalid frame, stale, future, zero, duplicate or regressing ' + kind)
        if len(self.queue) >= 256:
            raise ValueError('bounded reorder queue full')
        return stamp, wall

    def imu(self, message):
        with self.lock:
            try:
                ns, wall = self.accept(message, 'imu', 'imu_link')
                q, a, w = message.orientation, message.linear_acceleration, message.angular_velocity
                # -1 means unavailable; zero matrix means unknown and receives a model floor.
                covariance(message.linear_acceleration_covariance, 3)
                observation = ImuObservation(message.header.stamp.to_sec(),
                    [q.x, q.y, q.z, q.w], [a.x, a.y, a.z], [w.x, w.y, w.z],
                    message.orientation_covariance, message.angular_velocity_covariance)
                self.core._imu_values(observation)
                self.seen['imu'], self.arrival['imu'] = ns, wall
                self.imu_history.append((ns, observation))
                self.imu_history = self.imu_history[-100:]
                bisect.insort(self.queue, (ns, 0, observation))
            except (ValueError, np.linalg.LinAlgError) as error:
                self.reason = str(error)
                self.core.relocation.invalidate(self.reason)

    def gps(self, message):
        with self.lock:
            try:
                ns, wall = self.accept(message, 'gps', 'gps_link')
                if message.status.status < 0:
                    raise ValueError('GPS no fix')
                if message.position_covariance_type not in range(4):
                    raise ValueError('unknown covariance type')
                checked = covariance(message.position_covariance, 3)
                noise = None if message.position_covariance_type == 0 else checked
                observation = GpsObservation(message.header.stamp.to_sec(),
                    self.projector.project(message.latitude, message.longitude, message.altitude), noise)
                self.core._gps_values(observation)
                self.seen['gps'], self.arrival['gps'] = ns, wall
                bisect.insort(self.queue, (ns, 1, observation))
            except (ValueError, np.linalg.LinAlgError) as error:
                self.reason = str(error)
                self.core.relocation.invalidate(self.reason)

    def attitude_at(self, ns):
        before = [item for item in self.imu_history if item[0] <= ns]
        after = [item for item in self.imu_history if item[0] >= ns]
        if not before or not after:
            return None
        left_ns, left = before[-1]
        right_ns, right = after[0]
        if (right_ns-left_ns)*1e-9 > self.timing['max_integration_step_sec']:
            return None
        fraction = (ns-left_ns)/(right_ns-left_ns) if right_ns != left_ns else 0.
        def blend(a, b):
            return np.asarray(a)*(1-fraction)+np.asarray(b)*fraction
        return ImuObservation(ns*1e-9, slerp(left.orientation_xyzw, right.orientation_xyzw, fraction),
            blend(left.acceleration_mps2, right.acceleration_mps2),
            blend(left.angular_velocity_radps, right.angular_velocity_radps),
            blend(left.orientation_covariance, right.orientation_covariance),
            blend(left.angular_velocity_covariance, right.angular_velocity_covariance))

    @staticmethod
    def pose(target, position, orientation):
        target.position.x, target.position.y, target.position.z = position
        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = orientation

    def publish_estimate(self, ns):
        state = self.core.snapshot()
        # Construct from integer source stamp; floating filter time is never serialized.
        stamp = rospy.Time(ns // 1000000000, ns % 1000000000)
        ego, odom = EgoState(), Odometry()
        ego.header.stamp = odom.header.stamp = stamp
        ego.header.frame_id, odom.header.frame_id = 'map', 'odom'
        ego.child_frame_id = odom.child_frame_id = 'base_link'
        ego.reset_id = self.epoch
        ego.pose_valid = ego.twist_valid = [True]*6
        for message, position, cov in ((ego, 'map_position', 'map_pose_covariance'),
                                       (odom, 'local_position', 'local_pose_covariance')):
            self.pose(message.pose.pose, state[position], state['orientation'])
            message.pose.covariance = state[cov].reshape(-1).tolist()
            twist = message.twist.twist
            twist.linear.x, twist.linear.y, twist.linear.z = state['velocity_body']
            twist.angular.x, twist.angular.y, twist.angular.z = state['angular_velocity']
            message.twist.covariance = state['twist_covariance'].reshape(-1).tolist()
        transforms = []
        for parent, child, position, orientation in (
                ('map', 'odom', state['map_odom_translation'], state['map_odom_orientation']),
                ('odom', 'base_link', state['local_position'], state['orientation'])):
            transform = TransformStamped()
            transform.header.stamp, transform.header.frame_id, transform.child_frame_id = stamp, parent, child
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = position
            q = transform.transform.rotation
            q.x, q.y, q.z, q.w = orientation
            transforms.append(transform)
        self.ego_pub.publish(ego)
        self.odom_pub.publish(odom)
        self.tf.sendTransform(transforms)
        self.latest, self.output_stamp = state, stamp

    def tick(self):
        with self.lock:
            now, wall = self.clock()
            if now.to_nsec() == 0:
                return
            stalled = wall-self.clock_wall > self.timing['clock_stall_sec']
            cutoff = now.to_nsec()-int(self.timing['reorder_delay_sec']*1e9)
            while self.queue and self.queue[0][0] <= cutoff and not stalled:
                ns, kind, observation = self.queue[0]
                # Wait for the right IMU endpoint when a packet arrives slowly.
                # The bound is central integration timeout, never stale extrapolation.
                if (kind == 1 and self.imu_history and self.imu_history[-1][0] < ns
                        and (now.to_nsec()-ns)*1e-9 < self.timing['max_integration_step_sec']):
                    break
                self.queue.pop(0)
                if kind == 0:
                    if (self.core.last_imu_stamp is not None and
                            observation.stamp-self.core.last_imu_stamp > self.timing['max_integration_step_sec']):
                        self.epoch += 1
                        self.core.reset()
                        self.latest, self.output_stamp = None, rospy.Time(0)
                    if self.core.process_imu(observation):
                        if (not self.core.relocation.pending and
                                (now.to_nsec()-ns)*1e-9 <= self.timing['estimate_timeout_sec']):
                            self.publish_estimate(ns)
                            self.publish_status(rospy.Time.now(), time.monotonic(), stalled)
                else:
                    attitude = self.attitude_at(ns)
                    if attitude is not None:
                        if self.core.process_gps(observation, attitude):
                            if self.core.gps_reinitialized:
                                self.epoch += 1
                                self.latest, self.output_stamp = None, rospy.Time(0)
                                self.reason = 'sensor relocation confirmed; map re-anchored; velocity reset'
                                rospy.logwarn('%s (reset_id=%d)', self.reason, self.epoch)
                            else:
                                self.reason = 'GPS correction accepted'
                    else:
                        self.reason = 'GPS lacks bracketing IMU: rejected'
                        self.core.relocation.invalidate(self.reason)
            if wall-self.status_wall < 1.0/self.timing['status_publish_rate_hz']:
                return
            self.publish_status(now, wall, stalled)

    def publish_status(self, now, wall, stalled):
        self.core.relocation.expire(now.to_sec())
        self.status_wall = wall
        # A wall loop continues during a paused ROS clock and input loss.
        status = LocalizationStatus()
        status.header.stamp = now
        status.reset_id, status.stop_required = self.epoch, True
        gps_stamp = self.core.last_gps_stamp
        status.gps_age_sec = now.to_sec()-gps_stamp if gps_stamp is not None else -1.
        imu_fresh = (self.arrival['imu'] is not None and
                     wall-self.arrival['imu'] <= self.timing['imu_timeout_sec'])
        gps_fresh = (self.arrival['gps'] is not None and
                     wall-self.arrival['gps'] <= self.timing['gps_timeout_sec'])
        relocating = self.core.relocation.pending
        status.gps_fix_valid = bool(not relocating and not stalled and gps_fresh and
            0 <= status.gps_age_sec <= self.timing['gps_timeout_sec'])
        valid = bool(not relocating and self.latest is not None and not stalled and imu_fresh and
            (now-self.output_stamp).to_sec() <= self.timing['estimate_timeout_sec'] and
            0 <= status.gps_age_sec <= self.timing['max_dead_reckoning_sec'])
        status.map_pose_valid = status.local_odometry_valid = valid
        status.ego_state_stamp = status.local_odometry_stamp = self.output_stamp
        status.map_position_stddev_m = self.latest['map_position_stddev'] if self.latest else -1.
        status.local_position_stddev_m = self.latest['local_position_stddev'] if self.latest else -1.
        status.yaw_stddev_rad = self.latest['yaw_stddev'] if self.latest else -1.
        status.mode = ((status.TRACKING if status.gps_fix_valid else status.DEAD_RECKONING)
                       if valid else (status.LOST if self.latest or stalled else status.INITIALIZING))
        if relocating and imu_fresh and not stalled:
            status.mode = status.RELOCALIZING
        status.reason = ('development only; ingress fallback; physical alignment unverified; ' +
                         ('clock stalled' if stalled else
                          'IMU input stale' if not imu_fresh else
                          self.core.relocation.diagnostic if relocating else
                          'estimate stale or dead reckoning budget expired' if self.latest and not valid else
                          'GPS blackout; inertial prediction only' if valid and not status.gps_fix_valid else
                          (self.core.last_rejection or self.reason)))
        if self.core.gps_diagnostic:
            status.reason += '; ' + self.core.gps_diagnostic
        self.status_pub.publish(status)

    def run(self):
        # Status accompanies each estimate; the central heartbeat continues during input loss.
        period = 0.01
        while not rospy.is_shutdown():
            start = time.monotonic()
            self.tick()
            time.sleep(max(0.001, period-(time.monotonic()-start)))


if __name__ == '__main__':
    rospy.init_node('localization_node', anonymous=False)
    LocalizationNode().run()
