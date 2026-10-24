# -*- coding: utf-8 -*-
"""Competition Vehicle Status UDP -> ROS twist bridge.

The official competition guide directs teams to use MORAI 24.R2.0's
EgoVehicleStatus UDP example but explicitly withholds position, vel_y/vel_z,
linear acceleration and tire-dynamics fields. This node therefore parses only
the allowed subset and never reads the prohibited fields.

Until a live competition datagram is observed, the 229-byte wire size remains a
development candidate inferred from the referenced EgoVehicleStatus example.
A different live datagram size is rejected and logged instead of guessed.
"""

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped

from morai_udp_bridge.protocol import competition_vehicle_status_packet
from morai_udp_bridge.udp_receiver import UdpReceiver

_VELOCITY_SOURCES = ("vel_x", "signed_vel")


class MoraiVehicleStatusBridge(object):
    def __init__(self):
        self.bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
        if not rospy.has_param("~port"):
            raise rospy.ROSInitException(
                "필수 파라미터 ~port가 없습니다. config/vehicle_status_bridge.yaml에서 지정하세요."
            )
        self.port = int(rospy.get_param("~port"))
        self.frame_id = "base_link"

        self.velocity_source = str(rospy.get_param("~velocity_source", "vel_x"))
        if self.velocity_source not in _VELOCITY_SOURCES:
            raise rospy.ROSInitException(
                "~velocity_source는 %s 중 하나여야 합니다(현재 %r)."
                % (_VELOCITY_SOURCES, self.velocity_source)
            )
        self.velocity_scale = float(rospy.get_param("~velocity_scale", 1.0))
        self.angular_velocity_scale = float(
            rospy.get_param("~angular_velocity_scale", 1.0)
        )
        self.twist_covariance = self._param_covariance("~twist_covariance", 36)

        self.buffer_bytes = int(rospy.get_param("~receive_buffer_bytes", 2048))
        self.socket_timeout_sec = float(rospy.get_param("~socket_timeout_sec", 0.5))
        self.stats_log_period_sec = float(rospy.get_param("~stats_log_period_sec", 5.0))
        self.log_raw = bool(rospy.get_param("~log_raw", False))
        self.log_raw_period_sec = float(rospy.get_param("~log_raw_period_sec", 2.0))
        self.timestamp_source = str(rospy.get_param("~timestamp_source", "receive"))
        if self.timestamp_source not in ("receive", "packet"):
            raise rospy.ROSInitException(
                "~timestamp_source는 receive 또는 packet이어야 합니다."
            )

        self._publisher = rospy.Publisher(
            "/molit/vehicle/twist", TwistWithCovarianceStamped, queue_size=10
        )
        self._receiver = UdpReceiver(
            self.bind_ip, self.port, self.buffer_bytes, self.socket_timeout_sec
        )

        self._recv_count = 0
        self._parse_ok_count = 0
        self._parse_fail_count = 0
        self._last_recv_time = None
        self._first_packet_logged = False

        rospy.on_shutdown(self._on_shutdown)
        if self.stats_log_period_sec > 0.0:
            rospy.Timer(rospy.Duration(self.stats_log_period_sec), self._log_stats)

        rospy.loginfo(
            "[morai_vehicle_status_bridge] Competition Vehicle Status UDP "
            "%s:%d -> /molit/vehicle/twist "
            "(frame_id=%s, ts=%s, vel_source=%s, vel_scale=%.4f)",
            self.bind_ip, self.port, self.frame_id, self.timestamp_source,
            self.velocity_source, self.velocity_scale,
        )

    def _param_covariance(self, name, length):
        value = rospy.get_param(name, [0.0] * length)
        value = [float(v) for v in value]
        if len(value) != length:
            raise rospy.ROSInitException(
                "%s 길이는 %d 여야 합니다(현재 %d)."
                % (name, length, len(value))
            )
        return value

    def spin(self):
        while not rospy.is_shutdown():
            received = self._receiver.receive()
            if received is None:
                continue
            data, _sender = received
            self._recv_count += 1
            ingress_stamp = rospy.Time.now()
            self._last_recv_time = ingress_stamp

            if not self._first_packet_logged:
                self._first_packet_logged = True
                rospy.loginfo(
                    "[morai_vehicle_status_bridge] 첫 패킷 %d bytes "
                    "(EgoVehicleStatus 예제 기반 기대 %d)",
                    len(data),
                    competition_vehicle_status_packet.COMPETITION_STATUS_PACKET_SIZE,
                )

            try:
                reading = (
                    competition_vehicle_status_packet
                    .parse_competition_vehicle_status_packet(data)
                )
            except competition_vehicle_status_packet.CompetitionVehicleStatusParseError as error:
                self._parse_fail_count += 1
                rospy.logwarn_throttle(
                    2.0,
                    "[morai_vehicle_status_bridge] Competition Status 파싱 실패: %s"
                    % error,
                )
                continue
            except Exception as error:
                self._parse_fail_count += 1
                rospy.logwarn_throttle(
                    2.0,
                    "[morai_vehicle_status_bridge] 예상치 못한 오류: %s" % error,
                )
                continue

            self._parse_ok_count += 1
            if self.log_raw:
                rospy.loginfo_throttle(
                    self.log_raw_period_sec,
                    "[morai_vehicle_status_bridge] raw %r" % reading,
                )
            self._publish(reading, ingress_stamp)

    def _publish(self, reading, ingress_stamp):
        message = TwistWithCovarianceStamped()
        message.header.stamp = self._stamp(reading, ingress_stamp)
        message.header.frame_id = self.frame_id

        longitudinal = (
            reading.vel_x if self.velocity_source == "vel_x" else reading.signed_vel
        )
        message.twist.twist.linear.x = longitudinal * self.velocity_scale

        # Competition Vehicle Status는 vel_y / vel_z를 제공하지 않는다.
        # NaN으로 unavailable을 명시해 0 측정값처럼 오해하지 않게 한다.
        message.twist.twist.linear.y = float("nan")
        message.twist.twist.linear.z = float("nan")

        message.twist.twist.angular.x = reading.ang_vel_x * self.angular_velocity_scale
        message.twist.twist.angular.y = reading.ang_vel_y * self.angular_velocity_scale
        message.twist.twist.angular.z = reading.ang_vel_z * self.angular_velocity_scale
        message.twist.covariance = list(self.twist_covariance)
        self._publisher.publish(message)

    def _stamp(self, reading, ingress_stamp):
        if self.timestamp_source == "packet":
            return rospy.Time(reading.sec, reading.nsec)
        return ingress_stamp

    def _log_stats(self, _event):
        last = "N/A"
        if self._last_recv_time is not None:
            last = "%.2fs ago" % (rospy.Time.now() - self._last_recv_time).to_sec()
        rospy.loginfo(
            "[morai_vehicle_status_bridge] recv=%d parse_ok=%d parse_fail=%d last_recv=%s",
            self._recv_count, self._parse_ok_count, self._parse_fail_count, last,
        )

    def _on_shutdown(self):
        self._receiver.close()
        rospy.loginfo(
            "[morai_vehicle_status_bridge] 종료. recv=%d parse_ok=%d parse_fail=%d",
            self._recv_count, self._parse_ok_count, self._parse_fail_count,
        )


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
