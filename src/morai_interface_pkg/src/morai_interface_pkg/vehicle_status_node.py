# -*- coding: utf-8 -*-
"""
vehicle_status_node.py
- 역할: MORAI EgoVehicleStatus UDP 바이너리를 수신·파싱해
        geometry_msgs/TwistWithCovarianceStamped로 발행한다.
- 주요 클래스: MoraiVehicleStatusBridge
인터페이스
- pub ~topic (기본 /vehicle/twist): geometry_msgs/TwistWithCovarianceStamped
- 입력: MORAI EgoVehicleStatus UDP(229바이트 바이너리), bind_ip:port로 수신

설계 메모(자료로 미확정 -> 하드코딩하지 않고 파라미터화)
- 속도 source: ~velocity_source = vel_xy(vel_x,vel_y 벡터) 또는 signed_vel(scalar 종방향).
  좌표계(body/global)와 단위(m/s vs km/h)가 자료에 없어 확정 불가.
- 속도 scale: ~velocity_scale (예: km/h면 0.2777..). 기본 1.0.
- 각속도: ~publish_angular 시 ang_vel_z -> twist.angular.z, ~angular_velocity_scale 적용.
- timestamp: ~timestamp_source = receive 또는 packet. 기본 receive.
- covariance: ~twist_covariance 파라미터(길이 36). 실제 정확도 아님(placeholder).
- 주의: EgoVehicleStatus의 position/heading은 ground-truth이므로 사용하지 않는다
  (localization 입력 계약상 twist 속도 성분만 사용).
"""

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped

from morai_interface_pkg.protocol import ego_status_packet
from morai_interface_pkg.udp_receiver import UdpReceiver

_VELOCITY_SOURCES = ("vel_xy", "signed_vel")


class MoraiVehicleStatusBridge(object):
    """EgoVehicleStatus UDP를 받아 TwistWithCovarianceStamped로 발행하는 노드."""

    def __init__(self):
        self.bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
        if not rospy.has_param("~port"):
            raise rospy.ROSInitException(
                "필수 파라미터 ~port가 없습니다. config/vehicle_status_bridge.yaml에서 지정하세요.")
        self.port = int(rospy.get_param("~port"))
        self.topic = rospy.get_param("~topic", "/vehicle/twist")
        self.frame_id = rospy.get_param("~frame_id", "base_link")

        self.velocity_source = str(rospy.get_param("~velocity_source", "vel_xy"))
        if self.velocity_source not in _VELOCITY_SOURCES:
            raise rospy.ROSInitException(
                "~velocity_source는 %s 중 하나여야 합니다(현재 %r)."
                % (_VELOCITY_SOURCES, self.velocity_source))
        self.velocity_scale = float(rospy.get_param("~velocity_scale", 1.0))
        self.publish_angular = bool(rospy.get_param("~publish_angular", True))
        self.angular_velocity_scale = float(
            rospy.get_param("~angular_velocity_scale", 1.0))

        self.twist_covariance = self._param_covariance("~twist_covariance", 36)

        self.buffer_bytes = int(rospy.get_param("~receive_buffer_bytes", 2048))
        self.socket_timeout_sec = float(rospy.get_param("~socket_timeout_sec", 0.5))
        self.stats_log_period_sec = float(rospy.get_param("~stats_log_period_sec", 5.0))
        self.log_raw = bool(rospy.get_param("~log_raw", False))
        self.log_raw_period_sec = float(rospy.get_param("~log_raw_period_sec", 2.0))
        self.timestamp_source = str(rospy.get_param("~timestamp_source", "receive"))

        self._publisher = rospy.Publisher(self.topic, TwistWithCovarianceStamped,
                                          queue_size=10)
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

        rospy.loginfo("[morai_vehicle_status_bridge] UDP %s:%d -> topic '%s' "
                      "(frame_id=%s, ts=%s, vel_source=%s, vel_scale=%.4f, "
                      "publish_angular=%s)",
                      self.bind_ip, self.port, self.topic, self.frame_id,
                      self.timestamp_source, self.velocity_source,
                      self.velocity_scale, self.publish_angular)

    def _param_covariance(self, name, length):
        value = rospy.get_param(name, [0.0] * length)
        value = [float(v) for v in value]
        if len(value) != length:
            raise rospy.ROSInitException("%s 길이는 %d 여야 합니다(현재 %d)."
                                         % (name, length, len(value)))
        return value

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
                rospy.loginfo("[morai_vehicle_status_bridge] 첫 패킷 %d bytes (기대 %d)",
                              len(data), ego_status_packet.EGO_PACKET_SIZE)

            try:
                reading = ego_status_packet.parse_ego_status_packet(data)
            except ego_status_packet.EgoStatusParseError as error:
                self._parse_fail_count += 1
                rospy.logwarn_throttle(
                    2.0, "[morai_vehicle_status_bridge] 파싱 실패: %s" % error)
                continue
            except Exception as error:
                self._parse_fail_count += 1
                rospy.logwarn_throttle(
                    2.0, "[morai_vehicle_status_bridge] 예상치 못한 오류: %s" % error)
                continue

            self._parse_ok_count += 1
            if self.log_raw:
                rospy.loginfo_throttle(self.log_raw_period_sec,
                                       "[morai_vehicle_status_bridge] raw %r" % reading)
            self._publish(reading)

    # 함수이름: _publish
    # 기능: EgoStatusReading을 TwistWithCovarianceStamped로 변환해 발행한다.
    def _publish(self, reading):
        message = TwistWithCovarianceStamped()
        message.header.stamp = self._stamp(reading)
        message.header.frame_id = self.frame_id

        if self.velocity_source == "vel_xy":
            message.twist.twist.linear.x = reading.vel_x * self.velocity_scale
            message.twist.twist.linear.y = reading.vel_y * self.velocity_scale
        else:  # signed_vel: 종방향 scalar만
            message.twist.twist.linear.x = reading.signed_vel * self.velocity_scale
            message.twist.twist.linear.y = 0.0

        if self.publish_angular:
            message.twist.twist.angular.z = (reading.ang_vel_z
                                             * self.angular_velocity_scale)

        message.twist.covariance = list(self.twist_covariance)
        self._publisher.publish(message)

    def _stamp(self, reading):
        if self.timestamp_source == "packet":
            return rospy.Time(reading.sec, reading.nsec)
        return rospy.Time.now()

    def _log_stats(self, _event):
        last = "N/A"
        if self._last_recv_time is not None:
            last = "%.2fs ago" % (rospy.Time.now() - self._last_recv_time).to_sec()
        rospy.loginfo("[morai_vehicle_status_bridge] recv=%d parse_ok=%d "
                      "parse_fail=%d last_recv=%s", self._recv_count,
                      self._parse_ok_count, self._parse_fail_count, last)

    def _on_shutdown(self):
        self._receiver.close()
        rospy.loginfo("[morai_vehicle_status_bridge] 종료. recv=%d parse_ok=%d parse_fail=%d",
                      self._recv_count, self._parse_ok_count, self._parse_fail_count)


def main():
    rospy.init_node("morai_vehicle_status_bridge")
    try:
        bridge = MoraiVehicleStatusBridge()
    except rospy.ROSInitException as error:
        rospy.logfatal("[morai_vehicle_status_bridge] 초기화 실패: %s", error)
        return
    bridge.spin()


if __name__ == "__main__":
    main()


