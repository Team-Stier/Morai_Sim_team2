# -*- coding: utf-8 -*-
"""
imu_node.py
- 역할: MORAI IMU UDP 바이너리를 수신·파싱해 sensor_msgs/Imu로 발행한다.
- 주요 클래스: MoraiImuBridge
인터페이스
- pub ~topic (기본 /sensors/imu/data): sensor_msgs/Imu
- 입력: MORAI IMU UDP(115바이트 바이너리), bind_ip:port로 수신

설계 메모(자료로 미확정 -> 하드코딩하지 않고 파라미터화)
- 단위: ang_vel/lin_acc 단위가 자료에 없어 ~angular_velocity_scale,
  ~linear_acceleration_scale로 변환한다(기본 1.0 = rad/s, m/s² 가정).
- timestamp: ~timestamp_source = receive(수신 시각) 또는 packet(sec/nsec).
  기본 receive(패킷 stamp의 sim-time 의미 미확인).
- covariance: ~*_covariance 파라미터. 실제 센서 정확도가 아니라 placeholder.
- orientation 순서는 패킷 정의상 W,X,Y,Z이며 그대로 매핑한다.
"""

import rospy
from sensor_msgs.msg import Imu

from morai_interface_pkg.protocol import imu_packet
from morai_interface_pkg.udp_receiver import UdpReceiver


class MoraiImuBridge(object):
    """IMU UDP를 받아 sensor_msgs/Imu로 발행하는 ROS 노드."""

    def __init__(self):
        self.bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
        if not rospy.has_param("~port"):
            raise rospy.ROSInitException(
                "필수 파라미터 ~port가 없습니다. config/imu_bridge.yaml에서 지정하세요.")
        self.port = int(rospy.get_param("~port"))
        self.topic = rospy.get_param("~topic", "/sensors/imu/data")
        self.frame_id = rospy.get_param("~frame_id", "base_link")

        self.timestamp_source = str(rospy.get_param("~timestamp_source", "receive"))
        self.angular_velocity_scale = float(
            rospy.get_param("~angular_velocity_scale", 1.0))
        self.linear_acceleration_scale = float(
            rospy.get_param("~linear_acceleration_scale", 1.0))

        # 공분산(placeholder, 실제 정확도 아님). 길이 9 리스트.
        self.orientation_covariance = self._param_covariance(
            "~orientation_covariance", 9)
        self.angular_velocity_covariance = self._param_covariance(
            "~angular_velocity_covariance", 9)
        self.linear_acceleration_covariance = self._param_covariance(
            "~linear_acceleration_covariance", 9)

        self.buffer_bytes = int(rospy.get_param("~receive_buffer_bytes", 2048))
        self.socket_timeout_sec = float(rospy.get_param("~socket_timeout_sec", 0.5))
        self.stats_log_period_sec = float(rospy.get_param("~stats_log_period_sec", 5.0))
        self.log_raw = bool(rospy.get_param("~log_raw", False))
        self.log_raw_period_sec = float(rospy.get_param("~log_raw_period_sec", 2.0))

        self._publisher = rospy.Publisher(self.topic, Imu, queue_size=10)
        self._receiver = UdpReceiver(self.bind_ip, self.port,
                                     self.buffer_bytes, self.socket_timeout_sec)

        self._recv_count = 0
        self._parse_ok_count = 0
        self._parse_fail_count = 0
        self._last_recv_time = None
        self._first_packet_logged = False

        rospy.on_shutdown(self._on_shutdown)
        if self.stats_log_period_sec > 0.0:
            rospy.Timer(rospy.Duration(self.stats_log_period_sec), self._log_stats)

        rospy.loginfo("[morai_imu_bridge] UDP %s:%d -> topic '%s' (frame_id=%s, "
                      "ts=%s, ang_scale=%.4f, acc_scale=%.4f)",
                      self.bind_ip, self.port, self.topic, self.frame_id,
                      self.timestamp_source, self.angular_velocity_scale,
                      self.linear_acceleration_scale)

    def _param_covariance(self, name, length):
        value = rospy.get_param(name, [0.0] * length)
        value = [float(v) for v in value]
        if len(value) != length:
            raise rospy.ROSInitException("%s 길이는 %d 여야 합니다(현재 %d)."
                                         % (name, length, len(value)))
        return value

    # 함수이름: spin
    # 기능: shutdown까지 UDP를 수신하며 파싱/발행한다.
    def spin(self):
        while not rospy.is_shutdown():
            received = self._receiver.receive()
            if received is None:
                continue
            data, _sender = received
            self._recv_count += 1
            self._last_recv_time = rospy.Time.now()

            if not self._first_packet_logged:
                self._first_packet_logged = True
                rospy.loginfo("[morai_imu_bridge] 첫 패킷 %d bytes (기대 %d)",
                              len(data), imu_packet.IMU_PACKET_SIZE)

            try:
                reading = imu_packet.parse_imu_packet(data)
            except imu_packet.ImuParseError as error:
                self._parse_fail_count += 1
                rospy.logwarn_throttle(2.0, "[morai_imu_bridge] 파싱 실패: %s" % error)
                continue
            except Exception as error:
                self._parse_fail_count += 1
                rospy.logwarn_throttle(2.0,
                                       "[morai_imu_bridge] 예상치 못한 오류: %s" % error)
                continue

            self._parse_ok_count += 1
            if self.log_raw:
                rospy.loginfo_throttle(self.log_raw_period_sec,
                                       "[morai_imu_bridge] raw %r" % reading)
            self._publish(reading)

    # 함수이름: _publish
    # 기능: ImuReading을 sensor_msgs/Imu로 변환해 발행한다.
    def _publish(self, reading):
        message = Imu()
        message.header.stamp = self._stamp(reading)
        message.header.frame_id = self.frame_id

        message.orientation.w = reading.ori_w
        message.orientation.x = reading.ori_x
        message.orientation.y = reading.ori_y
        message.orientation.z = reading.ori_z

        message.angular_velocity.x = reading.ang_vel_x * self.angular_velocity_scale
        message.angular_velocity.y = reading.ang_vel_y * self.angular_velocity_scale
        message.angular_velocity.z = reading.ang_vel_z * self.angular_velocity_scale

        message.linear_acceleration.x = reading.lin_acc_x * self.linear_acceleration_scale
        message.linear_acceleration.y = reading.lin_acc_y * self.linear_acceleration_scale
        message.linear_acceleration.z = reading.lin_acc_z * self.linear_acceleration_scale

        message.orientation_covariance = list(self.orientation_covariance)
        message.angular_velocity_covariance = list(self.angular_velocity_covariance)
        message.linear_acceleration_covariance = list(self.linear_acceleration_covariance)

        self._publisher.publish(message)

    # 함수이름: _stamp
    # 기능: timestamp_source에 따라 header.stamp를 만든다.
    def _stamp(self, reading):
        if self.timestamp_source == "packet":
            return rospy.Time(reading.sec, reading.nsec)
        return rospy.Time.now()

    def _log_stats(self, _event):
        last = "N/A"
        if self._last_recv_time is not None:
            last = "%.2fs ago" % (rospy.Time.now() - self._last_recv_time).to_sec()
        rospy.loginfo("[morai_imu_bridge] recv=%d parse_ok=%d parse_fail=%d last_recv=%s",
                      self._recv_count, self._parse_ok_count,
                      self._parse_fail_count, last)

    def _on_shutdown(self):
        self._receiver.close()
        rospy.loginfo("[morai_imu_bridge] 종료. recv=%d parse_ok=%d parse_fail=%d",
                      self._recv_count, self._parse_ok_count, self._parse_fail_count)


def main():
    rospy.init_node("morai_imu_bridge")
    try:
        bridge = MoraiImuBridge()
    except rospy.ROSInitException as error:
        rospy.logfatal("[morai_imu_bridge] 초기화 실패: %s", error)
        return
    bridge.spin()


if __name__ == "__main__":
    main()


