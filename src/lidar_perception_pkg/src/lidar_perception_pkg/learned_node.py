"""ROS boundary with one inference worker and a wall-clock health heartbeat."""
import copy
import math
import threading
import time

import numpy as np
import rospy
from common_msgs_pkg.msg import ComponentStatus, LidarObservationArray
from common_msgs_pkg.lidar_validation import validate_lidar
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool

from .learned import PointPillars, read_xyzi, roi_points, observations_from_predictions


class LearnedNode:
    def __init__(self, backend_factory=PointPillars):
        self.condition = threading.Condition(threading.RLock())
        self.stop = threading.Event()
        self.pending = None
        self.epoch = 0
        self.last_clock = rospy.Time.now()
        self.clock_changed = time.monotonic()
        self.accepted_stamp = rospy.Time()
        self.received = self.transport_received = 0.0
        self.transport_ok = False
        self.inference_start = None
        self.model_ready = False
        self.health = ComponentStatus(component='lidar_perception_pkg', ready=False,
                                      stop_required=True, processing_latency_sec=-1,
                                      state=ComponentStatus.DEGRADED, reason='loading pretrained model')
        self.watchdog = self.number('contract/development_watchdog_sec', positive=True)
        self.period = self.number('contract/status_period_sec', positive=True)
        self.max_age = self.number('contract/max_scan_age_sec')
        self.budget = self.number('processing_budget_sec', positive=True)
        self.threshold = self.number('score_threshold')
        if self.threshold > 1:
            raise ValueError('score_threshold must be <=1')
        self.calibration = rospy.get_param('~contract/calibration_id')
        if not self.calibration:
            raise ValueError('missing central calibration identity')
        self.bounds = np.array([rospy.get_param('~'+axis+'_'+side)
                                for side in ('min','max') for axis in 'xyz'], dtype=float)
        if not np.isfinite(self.bounds).all() or np.any(self.bounds[:3] >= self.bounds[3:]):
            raise ValueError('invalid ROI bounds')
        self.model_args = [rospy.get_param('~'+n) for n in ('model_repository','checkpoint','checkpoint_sha256')]
        self.output = rospy.Publisher('/molit/perception/lidar/observations', LidarObservationArray, queue_size=2)
        self.status = rospy.Publisher('/molit/perception/lidar/status', ComponentStatus, queue_size=1, latch=True)
        self.debug = rospy.Publisher('~filtered_points', PointCloud2, queue_size=1)
        self.subs = [rospy.Subscriber('/molit/sensors/lidar/points', PointCloud2, self.scan,
                                      queue_size=1, buff_size=8*1024*1024),
                     rospy.Subscriber('/molit/sensors/lidar/status', Bool, self.transport, queue_size=1)]
        rospy.on_shutdown(self.shutdown)
        self.worker = threading.Thread(target=self.run, args=(backend_factory,), daemon=True)
        self.timer = threading.Thread(target=self.heartbeat, daemon=True)
        self.worker.start()
        self.timer.start()

    @staticmethod
    def number(name, positive=False):
        value = float(rospy.get_param('~'+name))
        if not math.isfinite(value) or value < 0 or (positive and value == 0):
            raise ValueError('invalid ' + name)
        return value

    def shutdown(self):
        self.stop.set()
        with self.condition:
            self.condition.notify_all()

    def fault(self, reason):
        self.health.state, self.health.reason = ComponentStatus.FAULT, reason

    def clock(self, now, wall):
        if now < self.last_clock:
            self.epoch += 1
            self.pending = None
            self.accepted_stamp = rospy.Time()
            self.health.data_stamp = rospy.Time()
            self.transport_ok = False
            self.fault('clock reset; waiting for new transport and scan')
        if now != self.last_clock:
            self.clock_changed = wall
        self.last_clock = now

    def transport(self, msg):
        with self.condition:
            self.transport_ok, self.transport_received = msg.data, time.monotonic()
            if not msg.data:
                self.epoch += 1
                self.pending = None
                self.fault('LiDAR transport reports no points')

    def scan(self, msg):
        with self.condition:
            now, wall = rospy.Time.now(), time.monotonic()
            self.clock(now, wall)
            if (msg.header.frame_id != 'lidar_link' or msg.header.stamp == rospy.Time() or
                    msg.header.stamp > now or msg.header.stamp <= self.accepted_stamp):
                self.health.invalid_count += 1
                self.epoch += 1
                self.pending = None
                self.fault('wrong frame or zero/future/duplicate/regressing scan stamp')
                return
            self.accepted_stamp, self.received = msg.header.stamp, wall
            if self.pending is not None:
                self.health.dropped_count += 1
            self.pending = (msg, wall, self.epoch)
            self.condition.notify()

    def check_input(self, msg, received, epoch):
        now, wall = rospy.Time.now(), time.monotonic()
        self.clock(now, wall)
        if epoch != self.epoch or now < msg.header.stamp:
            raise ValueError('scan invalidated during processing')
        if not self.transport_ok or wall-self.transport_received > self.watchdog:
            raise ValueError('missing/stale/false transport status')
        if wall-received > self.budget:
            raise ValueError('development processing budget exceeded')
        if wall-self.clock_changed > self.watchdog:
            raise ValueError('ROS clock stalled')
        if self.max_age > 0 and (now-msg.header.stamp).to_sec() > self.max_age:
            raise ValueError('scan exceeds central data-age limit')

    def run(self, factory):
        try:
            backend = factory(*self.model_args)
            with self.condition:
                self.model_ready = True
                self.health.reason = 'model loaded; waiting for valid XYZI scan'
        except Exception as error:
            with self.condition:
                self.fault('model load failed: '+str(error))
            rospy.logerr('%s', self.health.reason)
            return
        while not self.stop.is_set():
            with self.condition:
                self.condition.wait_for(lambda: self.pending is not None or self.stop.is_set())
                if self.stop.is_set():
                    return
                msg, received, epoch = self.pending
                self.pending = None
                self.inference_start = time.monotonic()
            output = LidarObservationArray(header=copy.deepcopy(msg.header),
                                           calibration_id=self.calibration,
                                           timestamp_provenance='ingress_fallback')
            try:
                with self.condition:
                    self.check_input(msg, received, epoch)
                points = read_xyzi(msg)
                boxes, scores, labels = backend.infer(points)
                output.objects = observations_from_predictions(points, boxes, scores, labels,
                                                                 backend.classes, self.bounds, self.threshold)
                output.objects_valid = True
                validate_lidar(output)
                with self.condition:
                    self.check_input(msg, received, epoch)
                    self.health.processed_count += 1
                    self.health.state = ComponentStatus.DEGRADED
                    self.health.reason = 'pretrained single scan; calibration/freshness/accuracy unverified'
                    self.debug.publish(create_cloud_xyz32(msg.header, roi_points(points, self.bounds)[:, :3]))
            except Exception as error:
                output.objects, output.objects_valid = [], False
                with self.condition:
                    self.health.invalid_count += 1
                    self.fault(str(error))
                rospy.logwarn_throttle(2, 'Learned LiDAR rejected scan: %s', error)
            with self.condition:
                self.health.processing_latency_sec = time.monotonic()-self.inference_start
                self.inference_start = None
                # No result from a prior clock/transport epoch may escape.
                self.clock(rospy.Time.now(), time.monotonic())
                if epoch == self.epoch and rospy.Time.now() >= msg.header.stamp:
                    self.health.data_stamp = msg.header.stamp
                    self.output.publish(output)
                else:
                    self.health.dropped_count += 1

    def heartbeat(self):
        while not self.stop.wait(self.period):
            with self.condition:
                now, wall = rospy.Time.now(), time.monotonic()
                self.clock(now, wall)
                if self.model_ready:
                    if (not self.transport_ok or wall-self.transport_received > self.watchdog or
                            wall-self.received > self.watchdog):
                        self.fault('no recent LiDAR transport/scan')
                    elif wall-self.clock_changed > self.watchdog:
                        self.fault('ROS clock stalled')
                    elif self.inference_start is not None and wall-self.inference_start > self.budget:
                        self.fault('model inference stalled/exceeded processing budget')
                self.health.header.stamp = now
                stamp = self.health.data_stamp
                self.health.data_age_sec = (now-stamp).to_sec() if stamp != rospy.Time() and now >= stamp else -1
                if now != rospy.Time():
                    self.status.publish(copy.deepcopy(self.health))
