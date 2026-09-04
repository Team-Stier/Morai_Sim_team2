#!/usr/bin/python3
"""두 EKF와 두 보조 node로 구성된 전체 localization ROS graph를 검사한다."""

import copy
import math
import threading
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rosgraph
import rosnode
import rospy
import rosservice
import rostest
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


OFFICIAL_PROJECTION_POINT = (37.24290592317244, 126.77453339001822)


class LocalizationIntegrationTest(unittest.TestCase):
    """실제 네 node의 연결, GPS anchor, relay와 장애 상태 계약을 검증한다."""

    @classmethod
    def setUpClass(cls):
        """통합 테스트용 ROS node를 한 번 초기화한다."""
        rospy.init_node("test_localization_integration", anonymous=True)

    def setUp(self):
        """합성 센서 publisher, 출력 subscriber와 20 Hz 입력 thread를 준비한다."""
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._imu_enabled = threading.Event()
        self._twist_enabled = threading.Event()
        self._gps_enabled = threading.Event()
        self._imu_enabled.set()
        self._twist_enabled.set()
        self._diagnostics = []
        self._final_messages = []
        self._global_messages = {}

        self._imu_topic = rospy.get_param("topics/input_imu")
        self._twist_topic = rospy.get_param("topics/input_vehicle_twist")
        self._gps_topic = rospy.get_param("topics/input_gps_fix")
        self._global_topic = rospy.get_param(
            "topics/global_filtered_odometry")
        self._final_topic = rospy.get_param("topics/output_odometry")
        self._status_topic = rospy.get_param("topics/status")
        self._imu_frame = rospy.get_param("sensor_frames/imu")
        self._twist_frame = rospy.get_param("sensor_frames/vehicle_twist")
        self._map_frame = rospy.get_param("frames/map")
        self._base_link_frame = rospy.get_param("frames/base_link")

        self._imu_publisher = rospy.Publisher(
            self._imu_topic, Imu, queue_size=20)
        self._twist_publisher = rospy.Publisher(
            self._twist_topic, TwistWithCovarianceStamped, queue_size=20)
        self._gps_publisher = rospy.Publisher(
            self._gps_topic, NavSatFix, queue_size=20)
        self._diagnostic_subscriber = rospy.Subscriber(
            self._status_topic, DiagnosticArray, self._diagnostic_callback,
            queue_size=50)
        self._global_subscriber = rospy.Subscriber(
            self._global_topic, Odometry, self._global_callback,
            queue_size=50)
        self._final_subscriber = rospy.Subscriber(
            self._final_topic, Odometry, self._final_callback, queue_size=50)

        self._publisher_thread = threading.Thread(
            target=self._publish_inputs, daemon=True)
        self._publisher_thread.start()

    def tearDown(self):
        """각 테스트가 끝나면 합성 센서 발행 thread를 안전하게 종료한다."""
        self._stop.set()
        self._publisher_thread.join(2.0)

    @staticmethod
    def _stamp_key(stamp):
        """ROS timestamp를 dictionary key로 쓸 수 있는 정수 tuple로 바꾼다."""
        return stamp.secs, stamp.nsecs

    @staticmethod
    def _finite_covariance(dimension, diagonal):
        """모든 값이 유한하고 지정된 대각 분산을 가진 정사각 covariance를 만든다."""
        covariance = [0.001] * (dimension * dimension)
        for index in range(dimension):
            covariance[index * dimension + index] = diagonal
        return covariance

    def _make_imu(self, stamp, frame_id=None):
        """정상 orientation·yaw rate·covariance를 가진 합성 IMU 메시지를 만든다."""
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = (
            self._imu_frame if frame_id is None else frame_id)
        message.orientation.w = 1.0
        message.angular_velocity.z = 0.01
        message.orientation_covariance = self._finite_covariance(3, 0.05)
        message.angular_velocity_covariance = self._finite_covariance(
            3, 0.02)
        message.linear_acceleration_covariance = self._finite_covariance(
            3, 0.10)
        return message

    def _make_twist(self, stamp):
        """base_link 기준 전진 속도와 유효 covariance를 가진 차량 twist를 만든다."""
        message = TwistWithCovarianceStamped()
        message.header.stamp = stamp
        message.header.frame_id = self._twist_frame
        message.twist.twist.linear.x = 0.2
        message.twist.twist.linear.y = 0.0
        message.twist.covariance = self._finite_covariance(6, 0.10)
        return message

    @staticmethod
    def _make_gps(stamp):
        """공식 원점에서 local (10, -5)에 해당하는 유효한 GPS fix를 만든다."""
        latitude, longitude = OFFICIAL_PROJECTION_POINT
        message = NavSatFix()
        message.header.stamp = stamp
        message.status.status = NavSatStatus.STATUS_FIX
        message.status.service = NavSatStatus.SERVICE_GPS
        message.latitude = latitude
        message.longitude = longitude
        message.altitude = 100.0
        message.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN)
        message.position_covariance[0] = 1.0
        message.position_covariance[4] = 1.0
        message.position_covariance[8] = 4.0
        return message

    def _publish_inputs(self):
        """활성화된 GPS·IMU·twist 입력을 공통 timestamp로 20 Hz 발행한다."""
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and not self._stop.is_set():
            stamp = rospy.Time.now()
            if stamp != rospy.Time():
                if self._imu_enabled.is_set():
                    imu = self._make_imu(stamp)
                    self._imu_publisher.publish(imu)
                if self._twist_enabled.is_set():
                    self._twist_publisher.publish(self._make_twist(stamp))
                if self._gps_enabled.is_set():
                    self._gps_publisher.publish(self._make_gps(stamp))
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                return

    def _diagnostic_callback(self, message):
        """첫 diagnostic status를 key/value dictionary와 수신 시간으로 기록한다."""
        if not message.status:
            return
        status = message.status[0]
        values = {value.key: value.value for value in status.values}
        with self._lock:
            self._diagnostics.append((time.monotonic(), values))

    def _global_callback(self, message):
        """Global EKF odometry를 timestamp별로 보관해 최종 relay와 비교한다."""
        with self._lock:
            self._global_messages[self._stamp_key(message.header.stamp)] = (
                copy.deepcopy(message))

    def _final_callback(self, message):
        """최종 Global Route Manager용 odometry와 monotonic 수신 시간을 기록한다."""
        with self._lock:
            self._final_messages.append(
                (time.monotonic(), copy.deepcopy(message)))

    @staticmethod
    def _wait_for(predicate, timeout):
        """predicate가 참이 될 때까지 monotonic deadline 안에서 기다린다."""
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return True
            rospy.sleep(0.01)
        return predicate()

    def _diagnostic_matching(self, state, reason=None, since=0.0):
        """지정 시각 이후의 최신 diagnostic 중 state와 선택적 reason이 맞는 값을 찾는다."""
        with self._lock:
            for receipt, values in reversed(self._diagnostics):
                if receipt < since:
                    break
                if values.get("state") != state:
                    continue
                if reason is not None and values.get(
                        "rejection_reason") != reason:
                    continue
                return receipt, dict(values)
        return None

    def _wait_for_diagnostic(self, state, timeout, reason=None, since=0.0):
        """원하는 diagnostic 상태를 기다리고 없으면 테스트를 실패시킨다."""
        self.assertTrue(
            self._wait_for(
                lambda: self._diagnostic_matching(
                    state, reason, since) is not None,
                timeout),
            "diagnostic did not reach {} ({})".format(state, reason))
        return self._diagnostic_matching(state, reason, since)

    def _final_count(self):
        """지금까지 수신한 최종 odometry 개수를 반환한다."""
        with self._lock:
            return len(self._final_messages)

    def _latest_final(self):
        """가장 최근 최종 odometry의 복사본을 반환한다."""
        with self._lock:
            return copy.deepcopy(self._final_messages[-1][1])

    def _first_final(self):
        """GPS anchor 확인 뒤 처음 relay된 odometry의 복사본을 반환한다."""
        with self._lock:
            return copy.deepcopy(self._final_messages[0][1])

    def _assert_graph_contracts(self):
        """node, service, publisher 소유권과 최종 topic type의 실제 ROS 계약을 검사한다."""
        expected_nodes = {
            "/molit_local_ekf",
            "/molit_global_ekf",
            "/molit_gps_projector",
            "/molit_localization_supervisor",
        }
        self.assertTrue(self._wait_for(
            lambda: expected_nodes.issubset(set(rosnode.get_node_names())),
            8.0), "required localization nodes did not all start")

        local_service = "/molit_local_ekf/set_pose"
        global_service = rospy.get_param("services/global_set_pose")
        rospy.wait_for_service(local_service, timeout=5.0)
        rospy.wait_for_service(global_service, timeout=5.0)
        self.assertEqual(
            rosservice.get_service_type(local_service),
            "robot_localization/SetPose")
        self.assertEqual(
            rosservice.get_service_type(global_service),
            "robot_localization/SetPose")

        master = rosgraph.Master(rospy.get_name())

        def publication_map():
            """ROS master 상태를 topic별 publisher node 목록으로 바꾼다."""
            publishers, _, _ = master.getSystemState()
            return {topic: nodes for topic, nodes in publishers}

        local_topic = rospy.get_param(
            "topics/local_filtered_odometry")
        publications = publication_map()
        self.assertEqual(publications.get(local_topic), ["/molit_local_ekf"])
        self.assertEqual(publications.get(self._global_topic),
                         ["/molit_global_ekf"])
        self.assertNotIn("/molit_local_ekf",
                         publications.get("/odometry/filtered", []))
        self.assertNotIn("/molit_global_ekf",
                         publications.get("/odometry/filtered", []))

        published_types = dict(master.getPublishedTopics(""))
        self.assertEqual(published_types.get(self._final_topic),
                         "nav_msgs/Odometry")

    def _assert_valid_final(self, message):
        """최종 odometry의 frame, 유한값, quaternion과 covariance를 검사한다."""
        self.assertEqual(message.header.frame_id, self._map_frame)
        self.assertEqual(message.child_frame_id, self._base_link_frame)
        values = [
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
            message.twist.twist.angular.x,
            message.twist.twist.angular.y,
            message.twist.twist.angular.z,
        ]
        values.extend(message.pose.covariance)
        values.extend(message.twist.covariance)
        self.assertTrue(all(math.isfinite(value) for value in values))
        quaternion = message.pose.pose.orientation
        quaternion_norm = math.sqrt(
            quaternion.x ** 2 + quaternion.y ** 2 +
            quaternion.z ** 2 + quaternion.w ** 2)
        self.assertGreater(quaternion_norm, 0.0)
        self.assertAlmostEqual(quaternion_norm, 1.0, places=3)
        for index in (0, 7, 14, 21, 28, 35):
            self.assertGreaterEqual(message.pose.covariance[index], 0.0)
            self.assertGreaterEqual(message.twist.covariance[index], 0.0)

    def _assert_unchanged_relay(self, message):
        """최종 메시지가 같은 timestamp의 Global EKF 메시지를 변경 없이 relay했는지 확인한다."""
        key = self._stamp_key(message.header.stamp)
        self.assertTrue(self._wait_for(
            lambda: key in self._global_messages, 1.0),
            "relayed stamp was not observed on the global EKF output")
        with self._lock:
            upstream = copy.deepcopy(self._global_messages[key])
        # roscpp는 publisher마다 별도의 transport sequence 번호를 부여한다.
        # 이 번호를 제외한 timestamp, frame, 상태와 covariance는 정확히 같아야 한다.
        message.header.seq = upstream.header.seq
        self.assertEqual(message, upstream)

    def _assert_relay_stopped(self, fault_receipt):
        """FAULT 경계 뒤 새로운 최종 odometry가 더 이상 발행되지 않는지 확인한다."""
        deadline = time.monotonic() + 0.40
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                late_receipts = [
                    receipt for receipt, _ in self._final_messages
                    if receipt > fault_receipt + 0.10
                ]
            self.assertFalse(
                late_receipts,
                "fresh final relay occurred after FAULT boundary")
            time.sleep(0.01)

    def test_dual_ekf_pipeline_health_and_blackout_contracts(self):
        """시작 anchor, 정상 relay, 입력 거부, GPS blackout과 FAULT 차단을 검증한다."""
        self.assertFalse(rospy.get_param("/use_sim_time", False))
        self.assertTrue(self._wait_for(
            lambda: self._imu_publisher.get_num_connections() >= 3 and
            self._twist_publisher.get_num_connections() >= 3 and
            self._gps_publisher.get_num_connections() >= 1,
            8.0), "synthetic publishers did not connect to the runtime graph")
        self._assert_graph_contracts()
        self.assertEqual(self._final_topic, "/localization/odometry")

        # motion 입력이 먼저 시작되므로 두 EKF는 처음에 map 원점 근처를 예측한다.
        # GPS가 Global EKF 기준점을 만들고 odometry가 그 위치와 일치하기 전까지는
        # Global Route Manager용 최종 출력을 차단해야 한다.
        self._wait_for_diagnostic(
            "INITIALIZING", 5.0, "waiting_for_gps_fix")
        self.assertTrue(self._wait_for(
            lambda: bool(self._global_messages), 3.0),
            "global EKF did not produce the pre-GPS prediction")
        rospy.sleep(0.4)
        self.assertEqual(
            self._final_count(), 0,
            "unanchored global EKF prediction reached the final output")

        gps_start_boundary = time.monotonic()
        self._gps_enabled.set()
        self._wait_for_diagnostic("TRACKING", 10.0)
        self.assertTrue(self._wait_for(
            lambda: self._final_count() > 0, 2.0),
            "final output did not begin after global GPS anchoring")
        first_final = self._first_final()
        expected_x, expected_y = 10.0, -5.0
        anchor_error = math.hypot(
            first_final.pose.pose.position.x - expected_x,
            first_final.pose.pose.position.y - expected_y)
        self.assertLessEqual(
            anchor_error,
            rospy.get_param(
                "validation/global_anchor_max_error_m", 2.0) + 0.05,
            "first final odometry was not aligned to the accepted GPS anchor")
        self.assertTrue(self._diagnostic_matching(
            "TRACKING", since=gps_start_boundary))

        start_count = self._final_count()
        self.assertTrue(self._wait_for(
            lambda: self._final_count() >= start_count + 5, 0.5),
            "fewer than five final odometry messages arrived in 0.5 seconds")
        final_message = self._latest_final()
        self._assert_valid_final(final_message)
        self._assert_unchanged_relay(final_message)

        # 실제 callback에서 frame 의미 오류를 거부하고 relay가 멈추는지 확인한다.
        self._imu_enabled.clear()
        rospy.sleep(0.10)
        rejection_boundary = time.monotonic()
        self._imu_publisher.publish(
            self._make_imu(rospy.Time.now(), frame_id="wrong_imu_frame"))
        fault_receipt, _ = self._wait_for_diagnostic(
            "FAULT", 1.5, "imu_frame_mismatch", rejection_boundary)
        self._assert_relay_stopped(fault_receipt)
        recovery_boundary = time.monotonic()
        self._imu_enabled.set()
        self._wait_for_diagnostic(
            "TRACKING", 2.0, since=recovery_boundary)

        # 중복 timestamp는 거부되며 마지막 승인 sample의 freshness를 갱신할 수 없다.
        self._imu_enabled.clear()
        rospy.sleep(0.10)
        accepted = self._make_imu(rospy.Time.now())
        accepted_boundary = time.monotonic()
        self._imu_publisher.publish(accepted)
        self._wait_for_diagnostic(
            "TRACKING", 1.5, since=accepted_boundary)
        duplicate_boundary = time.monotonic()
        self._imu_publisher.publish(copy.deepcopy(accepted))
        duplicate_fault_receipt, _ = self._wait_for_diagnostic(
            "FAULT", 1.5, "imu_stamp_not_monotonic", duplicate_boundary)

        def duplicate_did_not_refresh():
            """중복 IMU가 승인 수신 시간을 갱신하지 않아 age가 timeout을 넘었는지 확인한다."""
            match = self._diagnostic_matching(
                "FAULT", "imu_stamp_not_monotonic", duplicate_boundary)
            if match is None:
                return False
            try:
                return float(match[1]["imu_age_sec"]) > rospy.get_param(
                    "runtime/sensor_timeout_sec")
            except (KeyError, ValueError):
                return False

        self.assertTrue(self._wait_for(duplicate_did_not_refresh, 1.5),
                        "duplicate IMU unexpectedly refreshed freshness")
        self._assert_relay_stopped(duplicate_fault_receipt)
        recovery_boundary = time.monotonic()
        self._imu_enabled.set()
        self._wait_for_diagnostic(
            "TRACKING", 2.0, since=recovery_boundary)

        # GPS blackout에서는 DEGRADED로 바뀌지만 motion 기반 dead reckoning은 계속된다.
        blackout_boundary = time.monotonic()
        self._gps_enabled.clear()
        self._wait_for_diagnostic(
            "DEGRADED", 2.0, "gps_degraded", blackout_boundary)
        degraded_start_count = self._final_count()
        self.assertTrue(self._wait_for(
            lambda: self._final_count() >= degraded_start_count + 5, 0.5),
            "final odometry stopped during GPS blackout")
        degraded_message = self._latest_final()
        self._assert_valid_final(degraded_message)
        self._assert_unchanged_relay(degraded_message)

        recovery_boundary = time.monotonic()
        self._gps_enabled.set()
        self._wait_for_diagnostic(
            "TRACKING", 3.0, since=recovery_boundary)

        # 필수 IMU가 stale해지면 FAULT가 되고 이후의 새로운 relay 출력을 막아야 한다.
        imu_stop_boundary = time.monotonic()
        self._imu_enabled.clear()
        fault_receipt, _ = self._wait_for_diagnostic(
            "FAULT", 1.5, "stale_imu", imu_stop_boundary)
        self._assert_relay_stopped(fault_receipt)


if __name__ == "__main__":
    rostest.rosrun(
        "localization_pkg", "localization_integration",
        LocalizationIntegrationTest)


