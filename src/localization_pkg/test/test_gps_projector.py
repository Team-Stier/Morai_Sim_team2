#!/usr/bin/python3
"""GPS map pose 투영과 제어된 Global EKF reset을 ROS graph에서 검사한다."""

import math
import threading
import time
import unittest

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import rospy
from robot_localization.srv import SetPose, SetPoseResponse
import rosservice
import rostest
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


LOCAL_X_10_Y_MINUS_5 = (37.24290592317244, 126.77453339001822)
LOCAL_X_20_Y_0 = (37.24295307772507, 126.77464472940534)
LOCAL_X_19_5_Y_0_5 = (37.24295747543199, 126.77463896371258)
LOCAL_X_20_5_Y_MINUS_0_5 = (37.24294868001784, 126.77465049509742)


class GpsProjectorTest(unittest.TestCase):
    """GpsProjectorNode의 입력 검증, 상태 전이, timeout과 SetPose 흐름을 검증한다."""

    @classmethod
    def setUpClass(cls):
        """테스트용 ROS node를 한 번 초기화한다."""
        rospy.init_node("test_gps_projector", anonymous=True)

    def setUp(self):
        """각 테스트에 필요한 topic 연결, 기록 목록과 가짜 SetPose service를 준비한다."""
        self._lock = threading.Lock()
        self._poses = []
        self._states = []
        self._reset_requests = []
        self._service_mode = "success"
        self._service_called = threading.Event()
        self._release_service = threading.Event()

        self._gps_topic = rospy.get_param("topics/input_gps_fix")
        self._pose_topic = rospy.get_param("topics/gps_map_pose")
        self._state_topic = rospy.get_param("topics/gps_state")
        self._odometry_topic = rospy.get_param(
            "topics/global_filtered_odometry")
        self._set_pose_service = rospy.get_param("services/global_set_pose")
        self._map_frame = rospy.get_param("frames/map")
        self._base_link_frame = rospy.get_param("frames/base_link")

        self._service = rospy.Service(
            self._set_pose_service, SetPose, self._handle_set_pose)
        self._pose_subscriber = rospy.Subscriber(
            self._pose_topic, type(self)._pose_type(), self._pose_callback,
            queue_size=20)
        self._state_subscriber = rospy.Subscriber(
            self._state_topic, String, self._state_callback, queue_size=20)
        self._gps_publisher = rospy.Publisher(
            self._gps_topic, NavSatFix, queue_size=20)
        self._odometry_publisher = rospy.Publisher(
            self._odometry_topic, Odometry, queue_size=10)

        self.assertTrue(self._wait_for(
            lambda: self._gps_publisher.get_num_connections() > 0 and
            self._odometry_publisher.get_num_connections() > 0,
            8.0), "GPS projector subscribers did not connect")
        self.assertTrue(self._wait_for_state("WAITING_FOR_FIX", 5.0))

    @staticmethod
    def _pose_type():
        """subscriber 생성 시 사용할 GPS map pose 메시지 타입을 지연 import한다."""
        from geometry_msgs.msg import PoseWithCovarianceStamped
        return PoseWithCovarianceStamped

    def _pose_callback(self, message):
        """projector가 발행한 GPS map pose를 thread-safe 목록에 기록한다."""
        with self._lock:
            self._poses.append(message)

    def _state_callback(self, message):
        """projector의 GPS gate 상태 heartbeat를 순서대로 기록한다."""
        with self._lock:
            self._states.append(message.data)

    def _handle_set_pose(self, request):
        """SetPose 요청을 기록하고 설정된 성공·지연·실패 동작을 재현한다."""
        with self._lock:
            self._reset_requests.append(request)
        self._service_called.set()
        if self._service_mode == "delayed_success":
            if not self._release_service.wait(5.0):
                raise rospy.ServiceException("test did not release SetPose")
        elif self._service_mode == "delayed_failure":
            if not self._release_service.wait(5.0):
                raise rospy.ServiceException("test did not release SetPose")
            raise rospy.ServiceException("injected delayed SetPose failure")
        elif self._service_mode == "failure":
            raise rospy.ServiceException("injected SetPose failure")
        return SetPoseResponse()

    @staticmethod
    def _wait_for(predicate, timeout):
        """predicate가 참이 될 때까지 monotonic deadline 안에서 기다린다."""
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return True
            rospy.sleep(0.01)
        return predicate()

    def _wait_for_state(self, expected, timeout):
        """마지막 GPS gate 상태가 expected가 될 때까지 기다린다."""
        def has_state():
            """현재 기록의 마지막 상태가 목표 상태인지 확인한다."""
            with self._lock:
                return bool(self._states) and self._states[-1] == expected
        return self._wait_for(has_state, timeout)

    def _pose_count(self):
        """지금까지 받은 GPS map pose 개수를 반환한다."""
        with self._lock:
            return len(self._poses)

    def _last_pose(self):
        """가장 최근 GPS map pose를 반환한다."""
        with self._lock:
            return self._poses[-1]

    def _last_reset_request(self):
        """가짜 service가 받은 가장 최근 SetPose 요청을 반환한다."""
        with self._lock:
            return self._reset_requests[-1]

    def _reset_request_count(self):
        """SetPose 요청이 발생한 누적 횟수를 반환한다."""
        with self._lock:
            return len(self._reset_requests)

    def _state_count(self):
        """수신한 GPS 상태 heartbeat의 누적 개수를 반환한다."""
        with self._lock:
            return len(self._states)

    def _has_seen_state_since(self, index, expected):
        """지정한 기록 위치 이후 expected 상태가 한 번이라도 있었는지 확인한다."""
        with self._lock:
            return expected in self._states[index:]

    @staticmethod
    def _make_fix(latitude, longitude, stamp=None,
                  covariance_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
                  variance_x=4.0, variance_y=9.0, altitude=100.0):
        """좌표·시간·covariance를 조절할 수 있는 유효한 NavSatFix 기본 메시지를 만든다."""
        message = NavSatFix()
        message.header.stamp = stamp if stamp is not None else rospy.Time.now()
        message.status.status = NavSatStatus.STATUS_FIX
        message.status.service = NavSatStatus.SERVICE_GPS
        message.latitude = latitude
        message.longitude = longitude
        message.altitude = altitude
        message.position_covariance_type = covariance_type
        message.position_covariance[0] = variance_x
        message.position_covariance[4] = variance_y
        message.position_covariance[8] = 16.0
        return message

    def _publish_fix(self, message):
        """GPS fix를 발행하고 callback이 처리할 짧은 시간을 준다."""
        self._gps_publisher.publish(message)
        rospy.sleep(0.03)

    def _assert_fix_not_forwarded(self, message):
        """특정 GPS fix가 map pose 발행 개수를 늘리지 않는지 확인한다."""
        count_before = self._pose_count()
        self._publish_fix(message)
        rospy.sleep(0.03)
        self.assertEqual(self._pose_count(), count_before)

    def _publish_prediction(self, x, y, orientation, stamp=None,
                            child_frame=None, frame_id=None, settle_sec=0.05):
        """위치·자세·frame·시간을 조절한 Global EKF prediction을 발행한다."""
        message = Odometry()
        message.header.stamp = stamp if stamp is not None else rospy.Time.now()
        message.header.frame_id = (frame_id if frame_id is not None
                                   else self._map_frame)
        message.child_frame_id = (child_frame if child_frame is not None
                                  else self._base_link_frame)
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation = orientation
        self._odometry_publisher.publish(message)
        rospy.sleep(settle_sec)

    def test_projection_validation_timeout_and_controlled_reset(self):
        """GPS 투영부터 timeout, 안정 복귀와 비동기 SetPose 예외 경로까지 검증한다."""
        heartbeat_start = self._state_count()
        heartbeat_period = rospy.get_param("runtime/timeout_check_period_sec")
        self.assertTrue(self._wait_for(
            lambda: self._state_count() >= heartbeat_start + 2,
            max(1.0, heartbeat_period * 6.0)),
            "GPS state did not publish a periodic liveness heartbeat")
        self.assertTrue(self._wait_for_state("WAITING_FOR_FIX", 1.0))

        latitude, longitude = LOCAL_X_10_Y_MINUS_5
        gps_max_age = rospy.get_param(
            "runtime/gps_message_max_age_sec",
            rospy.get_param("runtime/gps_timeout_sec"))
        stale_start = (rospy.Time.now() -
                       rospy.Duration.from_sec(gps_max_age + 1.0))
        for offset in (0.0, 0.01, 0.02):
            self._assert_fix_not_forwarded(self._make_fix(
                latitude, longitude,
                stamp=stale_start + rospy.Duration.from_sec(offset)))
        self.assertTrue(self._wait_for_state("WAITING_FOR_FIX", 1.0))
        self.assertEqual(self._reset_request_count(), 0)

        initial_fix = self._make_fix(latitude, longitude)
        initial_count = self._pose_count()
        self._publish_fix(initial_fix)
        self.assertTrue(self._wait_for(
            lambda: self._pose_count() == initial_count + 1, 3.0))
        pose = self._last_pose()
        self.assertEqual(pose.header.frame_id, self._map_frame)
        self.assertAlmostEqual(pose.pose.pose.position.x, 10.0, places=2)
        self.assertAlmostEqual(pose.pose.pose.position.y, -5.0, places=2)
        self.assertEqual(pose.pose.covariance[0], 4.0)
        self.assertEqual(pose.pose.covariance[7], 9.0)
        self.assertTrue(self._wait_for_state("TRACKING", 2.0))

        fallback_fix = self._make_fix(
            latitude, longitude,
            covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN,
            variance_x=float("nan"), variance_y=float("nan"),
            altitude=float("nan"))
        self._publish_fix(fallback_fix)
        self.assertTrue(self._wait_for(
            lambda: self._pose_count() == initial_count + 2, 3.0))
        fallback_pose = self._last_pose()
        self.assertEqual(
            fallback_pose.pose.covariance[0],
            rospy.get_param("gps_covariance/fallback_xy_variance_m2"))
        self.assertEqual(
            fallback_pose.pose.covariance[7],
            rospy.get_param("gps_covariance/fallback_xy_variance_m2"))
        self.assertTrue(math.isfinite(fallback_pose.pose.pose.position.z))

        duplicate = self._make_fix(
            latitude, longitude, stamp=fallback_fix.header.stamp)
        self._assert_fix_not_forwarded(duplicate)

        no_fix = self._make_fix(latitude, longitude)
        no_fix.status.status = NavSatStatus.STATUS_NO_FIX
        self._assert_fix_not_forwarded(no_fix)

        zero_stamp = self._make_fix(latitude, longitude, stamp=rospy.Time())
        self._assert_fix_not_forwarded(zero_stamp)
        future_stamp = rospy.Time.now() + rospy.Duration.from_sec(
            rospy.get_param("runtime/max_future_stamp_sec") + 1.0)
        self._assert_fix_not_forwarded(
            self._make_fix(latitude, longitude, stamp=future_stamp))
        self._assert_fix_not_forwarded(
            self._make_fix(float("nan"), longitude))
        self._assert_fix_not_forwarded(
            self._make_fix(latitude, float("inf")))
        self._assert_fix_not_forwarded(
            self._make_fix(latitude, longitude, variance_x=float("nan")))
        self._assert_fix_not_forwarded(
            self._make_fix(latitude, longitude, variance_y=-1.0))

        # 형식은 유효하지만 거부되는 이상치가 연속되어도 GPS timeout을 가리면 안 된다.
        state_index = self._state_count()
        outlier_deadline = time.monotonic() + rospy.get_param(
            "runtime/gps_timeout_sec") + 0.35
        outlier_index = 0
        while time.monotonic() < outlier_deadline:
            self._publish_prediction(
                100.0, 100.0, Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
            coordinates = (LOCAL_X_10_Y_MINUS_5
                           if outlier_index % 2 == 0 else LOCAL_X_20_Y_0)
            self._publish_fix(self._make_fix(*coordinates))
            outlier_index += 1
        self.assertTrue(
            self._has_seen_state_since(state_index, "DEGRADED"),
            "continuous rejected outliers masked the accepted-fix timeout")
        self.assertTrue(self._wait_for_state("DEGRADED", 3.0))

        # 순서는 증가하지만 오래된 odometry timestamp는 prediction을 갱신하면 안 된다.
        old_prediction_stamp = rospy.Time.now()
        self._publish_prediction(
            0.0, 0.0, Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            stamp=old_prediction_stamp)
        rospy.sleep(rospy.get_param("runtime/global_odometry_timeout_sec") +
                    0.1)
        stale_stamp = old_prediction_stamp + rospy.Duration.from_sec(0.001)
        self._publish_prediction(
            0.0, 0.0, Quaternion(x=0.0, y=0.0, z=-0.5, w=0.5),
            stamp=stale_stamp)
        self._service_called.clear()
        stale_pose_count = self._pose_count()
        self._publish_fix(self._make_fix(*LOCAL_X_20_Y_0))
        self._publish_fix(self._make_fix(*LOCAL_X_19_5_Y_0_5))
        self._publish_fix(self._make_fix(*LOCAL_X_20_5_Y_MINUS_0_5))
        self.assertFalse(
            self._service_called.wait(0.2),
            "stale odometry was incorrectly refreshed for SetPose")
        self.assertTrue(self._wait_for(
            lambda: self._pose_count() == stale_pose_count + 1, 2.0))
        self.assertTrue(self._wait_for_state("TRACKING", 2.0))
        self.assertTrue(self._wait_for_state("DEGRADED", 3.0))

        # 수신 시간은 fresh하지만 저장된 ROS timestamp가 reset 전에 만료되는 경우를 검사한다.
        odometry_timeout = rospy.get_param(
            "runtime/global_odometry_timeout_sec")
        near_boundary_stamp = (
            rospy.Time.now() -
            rospy.Duration.from_sec(odometry_timeout * 0.6))
        self._publish_prediction(
            0.0, 0.0, Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            stamp=near_boundary_stamp)
        rospy.sleep(odometry_timeout * 0.5)
        boundary_request_count = self._reset_request_count()
        boundary_pose_count = self._pose_count()
        self._service_called.clear()
        self._publish_fix(self._make_fix(*LOCAL_X_20_Y_0))
        self._publish_fix(self._make_fix(*LOCAL_X_19_5_Y_0_5))
        self._publish_fix(self._make_fix(*LOCAL_X_20_5_Y_MINUS_0_5))
        self.assertFalse(
            self._service_called.wait(0.2),
            "ROS-time-stale odometry incorrectly triggered SetPose")
        self.assertEqual(self._reset_request_count(), boundary_request_count)
        self.assertTrue(self._wait_for(
            lambda: self._pose_count() == boundary_pose_count + 1, 2.0))
        self.assertTrue(self._wait_for_state("TRACKING", 2.0))
        self.assertTrue(self._wait_for_state("DEGRADED", 3.0))

        # 방향은 유효하지만 크기만 다른 quaternion은 reset 요청에서 정규화돼야 한다.
        yaw = 1.0
        orientation = Quaternion(
            x=0.0, y=0.0, z=2.0 * math.sin(yaw / 2.0),
            w=2.0 * math.cos(yaw / 2.0))
        prediction_stamp = rospy.Time.now()
        self._publish_prediction(
            0.0, 0.0, orientation, stamp=prediction_stamp)
        stale_yaw = -1.0
        stale_orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(stale_yaw / 2.0),
            w=math.cos(stale_yaw / 2.0))
        self._publish_prediction(
            0.0, 0.0, stale_orientation, stamp=prediction_stamp,
            settle_sec=0.0)
        self._publish_prediction(
            0.0, 0.0, stale_orientation,
            child_frame="wrong_base_link", settle_sec=0.0)
        self._publish_prediction(
            0.0, 0.0, stale_orientation, frame_id="wrong_map",
            settle_sec=0.0)
        self._publish_prediction(
            0.0, 0.0, stale_orientation, stamp=rospy.Time(),
            settle_sec=0.0)
        self._publish_prediction(
            0.0, 0.0, stale_orientation,
            stamp=(rospy.Time.now() + rospy.Duration.from_sec(
                rospy.get_param("runtime/max_future_stamp_sec") + 1.0)),
            settle_sec=0.0)
        self._publish_prediction(
            0.0, 0.0, Quaternion(x=0.0, y=0.0, z=0.0, w=0.0),
            settle_sec=0.0)
        rospy.sleep(0.05)

        self._service_mode = "delayed_success"
        self._service_called.clear()
        self._release_service.clear()
        success_request_count = self._reset_request_count()
        pose_count_before_reset = self._pose_count()
        far_1 = self._make_fix(*LOCAL_X_20_Y_0)
        self._publish_fix(far_1)
        self.assertTrue(self._wait_for_state("RELOCALIZING", 2.0))

        # 같은 timestamp의 sample을 반복 전달해도 안정 복귀 횟수를 채우면 안 된다.
        self._assert_fix_not_forwarded(far_1)
        self._assert_fix_not_forwarded(far_1)
        self.assertFalse(self._service_called.is_set())

        self._publish_fix(self._make_fix(*LOCAL_X_19_5_Y_0_5))
        self._gps_publisher.publish(
            self._make_fix(*LOCAL_X_20_5_Y_MINUS_0_5))
        self.assertTrue(self._service_called.wait(3.0))
        # RPC 진행 중 도착한 불안정 fix가 reset_pending 기준 pose를 바꾸면 안 된다.
        self._publish_fix(self._make_fix(*LOCAL_X_10_Y_MINUS_5))
        self._publish_fix(self._make_fix(*LOCAL_X_20_Y_0))
        self._publish_fix(self._make_fix(*LOCAL_X_19_5_Y_0_5))
        self.assertEqual(
            self._reset_request_count(), success_request_count + 1)
        rospy.sleep(0.1)
        self.assertEqual(self._pose_count(), pose_count_before_reset)
        self.assertTrue(self._wait_for_state("RELOCALIZING", 1.0))

        reset_request = self._last_reset_request().pose
        reset_q = reset_request.pose.pose.orientation
        reset_norm = math.sqrt(reset_q.x ** 2 + reset_q.y ** 2 +
                               reset_q.z ** 2 + reset_q.w ** 2)
        self.assertAlmostEqual(reset_norm, 1.0, places=12)
        self.assertAlmostEqual(reset_q.z, math.sin(yaw / 2.0), places=12)
        self.assertAlmostEqual(reset_q.w, math.cos(yaw / 2.0), places=12)
        self.assertGreaterEqual(
            reset_request.pose.covariance[0],
            rospy.get_param("reacquisition/reset_xy_variance_m2"))
        self.assertGreaterEqual(
            reset_request.pose.covariance[7],
            rospy.get_param("reacquisition/reset_xy_variance_m2"))

        self._release_service.set()
        self.assertTrue(self._wait_for(
            lambda: self._pose_count() == pose_count_before_reset + 1, 3.0))
        self.assertTrue(self._wait_for_state("TRACKING", 2.0))

        self.assertTrue(self._wait_for_state("DEGRADED", 3.0))
        self._publish_prediction(0.0, 0.0, orientation)
        self._service_mode = "delayed_success"
        self._service_called.clear()
        self._release_service.clear()
        overdue_request_count = self._reset_request_count()
        hung_pose_count = self._pose_count()
        self._publish_fix(self._make_fix(*LOCAL_X_20_Y_0))
        self._publish_fix(self._make_fix(*LOCAL_X_19_5_Y_0_5))
        self._publish_fix(self._make_fix(*LOCAL_X_20_5_Y_MINUS_0_5))
        self.assertTrue(self._service_called.wait(3.0))
        state_count_at_call = self._state_count()
        callbacks_continued = self._wait_for(
            lambda: self._state_count() > state_count_at_call and
            self._wait_for_state("RELOCALIZING", 0.0),
            rospy.get_param("reacquisition/set_pose_call_timeout_sec") + 1.0)
        self.assertEqual(self._pose_count(), hung_pose_count)
        self._publish_fix(self._make_fix(*LOCAL_X_20_Y_0))
        self._publish_fix(self._make_fix(*LOCAL_X_19_5_Y_0_5))
        self._publish_fix(self._make_fix(*LOCAL_X_20_5_Y_MINUS_0_5))
        self.assertEqual(
            self._reset_request_count(), overdue_request_count + 1)
        rospy.sleep(
            rospy.get_param("runtime/gps_message_max_age_sec") + 0.15)
        state_index_before_stale_response = self._state_count()
        self._release_service.set()
        self.assertTrue(
            callbacks_continued,
            "hung SetPose call blocked the response-timeout callback")
        self.assertTrue(self._wait_for_state("DEGRADED", 2.0))
        rospy.sleep(0.1)
        self.assertEqual(
            self._pose_count(), hung_pose_count,
            "stale late SetPose response republished an expired GPS pose")
        self.assertFalse(
            self._has_seen_state_since(
                state_index_before_stale_response, "TRACKING"),
            "stale late SetPose response incorrectly restored TRACKING")

        self.assertTrue(self._wait_for_state("DEGRADED", 3.0))
        self._publish_prediction(0.0, 0.0, orientation)
        self._service_mode = "delayed_failure"
        self._service_called.clear()
        self._release_service.clear()
        failure_request_count = self._reset_request_count()
        failure_pose_count = self._pose_count()
        self._publish_fix(self._make_fix(*LOCAL_X_20_Y_0))
        self._publish_fix(self._make_fix(*LOCAL_X_19_5_Y_0_5))
        self._publish_fix(self._make_fix(*LOCAL_X_20_5_Y_MINUS_0_5))
        self.assertTrue(self._service_called.wait(3.0))
        state_count_before_failure_timeout = self._state_count()
        self.assertTrue(self._wait_for(
            lambda: self._state_count() > state_count_before_failure_timeout,
            rospy.get_param("reacquisition/set_pose_call_timeout_sec") + 1.0))
        self.assertEqual(
            self._reset_request_count(), failure_request_count + 1)
        self._release_service.set()
        self._service_mode = "success"
        self._service_called.clear()
        retry_deadline = time.monotonic() + 3.0
        while (not self._service_called.is_set() and
               time.monotonic() < retry_deadline):
            self._publish_prediction(0.0, 0.0, orientation)
            self._publish_fix(self._make_fix(*LOCAL_X_20_Y_0))
        self.assertTrue(
            self._service_called.is_set(),
            "late SetPose failure was not reconciled to permit one retry")
        self.assertEqual(
            self._reset_request_count(), failure_request_count + 2)
        self.assertTrue(self._wait_for(
            lambda: self._pose_count() == failure_pose_count + 1, 3.0))
        self.assertTrue(self._wait_for_state("TRACKING", 2.0))

        # service가 늦게 발견되어도 만료된 reset snapshot으로 RPC를 보내면 안 된다.
        # 이후 새 snapshot으로 시작한 시도만 새로 사용 가능한 service를 한 번 호출한다.
        self.assertTrue(self._wait_for_state("DEGRADED", 3.0))
        self._service.shutdown("exercise unavailable SetPose service")
        self.assertTrue(self._wait_for(
            lambda: self._set_pose_service not in
            rosservice.get_service_list(), 2.0))

        unavailable_request_count = self._reset_request_count()
        unavailable_pose_count = self._pose_count()
        self._service_called.clear()
        self._publish_prediction(0.0, 0.0, orientation)
        odometry_timeout = rospy.get_param(
            "runtime/global_odometry_timeout_sec")

        def register_service_after_freshness_expires():
            """reset snapshot이 만료된 뒤 SetPose service를 다시 등록한다."""
            rospy.sleep(odometry_timeout + 0.1)
            self._service = rospy.Service(
                self._set_pose_service, SetPose, self._handle_set_pose)

        registrar = threading.Thread(
            target=register_service_after_freshness_expires)
        registrar.start()
        self._publish_fix(self._make_fix(*LOCAL_X_20_Y_0))
        self._publish_fix(self._make_fix(*LOCAL_X_19_5_Y_0_5))
        self._publish_fix(self._make_fix(*LOCAL_X_20_5_Y_MINUS_0_5))
        registrar.join(odometry_timeout + 2.0)
        self.assertFalse(registrar.is_alive())
        self.assertTrue(self._wait_for(
            lambda: self._set_pose_service in rosservice.get_service_list(),
            2.0))
        self.assertFalse(
            self._service_called.wait(0.2),
            "expired reset snapshot reached the late SetPose service")
        self.assertEqual(
            self._reset_request_count(), unavailable_request_count)
        self.assertEqual(self._pose_count(), unavailable_pose_count)

        self._service_called.clear()
        retry_coordinates = (
            LOCAL_X_20_Y_0,
            LOCAL_X_19_5_Y_0_5,
            LOCAL_X_20_5_Y_MINUS_0_5,
        )
        for coordinates in retry_coordinates:
            self._publish_prediction(0.0, 0.0, orientation)
            self._publish_fix(self._make_fix(*coordinates))
            if self._service_called.is_set():
                break
        self.assertTrue(self._service_called.wait(2.0))
        self.assertEqual(
            self._reset_request_count(), unavailable_request_count + 1)
        self.assertTrue(self._wait_for(
            lambda: self._pose_count() == unavailable_pose_count + 1, 3.0))
        self.assertTrue(self._wait_for_state("TRACKING", 2.0))


if __name__ == "__main__":
    rostest.rosrun("localization_pkg", "gps_projector", GpsProjectorTest)

