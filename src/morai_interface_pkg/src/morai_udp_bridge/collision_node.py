# -*- coding: utf-8 -*-
"""MORAI CollisionData UDP -> common_msgs_pkg/CollisionEvent bridge."""

import rospy

from common_msgs_pkg.msg import CollisionEvent
from morai_udp_bridge.protocol import collision_packet
from morai_udp_bridge.udp_receiver import UdpReceiver


class MoraiCollisionBridge(object):
    def __init__(self):
        self.bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
        if not rospy.has_param("~port"):
            raise rospy.ROSInitException(
                "필수 파라미터 ~port가 없습니다. config/collision_bridge.yaml에서 지정하세요."
            )
        self.port = int(rospy.get_param("~port"))
        self.buffer_bytes = int(rospy.get_param("~receive_buffer_bytes", 2048))
        self.socket_timeout_sec = float(rospy.get_param("~socket_timeout_sec", 0.5))
        self.stats_log_period_sec = float(rospy.get_param("~stats_log_period_sec", 5.0))

        self._publisher = rospy.Publisher(
            "/molit/events/collision", CollisionEvent, queue_size=10
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
            "[morai_collision_bridge] UDP %s:%d -> /molit/events/collision",
            self.bind_ip,
            self.port,
        )

    def spin(self):
        while not rospy.is_shutdown():
            received = self._receiver.receive()
            if received is None:
                continue
            data, _sender = received
            ingress_stamp = rospy.Time.now()
            self._recv_count += 1
            self._last_recv_time = ingress_stamp

            if not self._first_packet_logged:
                self._first_packet_logged = True
                rospy.loginfo(
                    "[morai_collision_bridge] 첫 패킷 %d bytes (기대 %d)",
                    len(data), collision_packet.COLLISION_PACKET_SIZE,
                )

            try:
                parsed = collision_packet.parse_collision_packet(data)
            except collision_packet.CollisionPacketParseError as error:
                self._parse_fail_count += 1
                rospy.logwarn_throttle(
                    2.0, "[morai_collision_bridge] 파싱 실패: %s" % error
                )
                continue

            self._parse_ok_count += 1
            active = parsed.active_slots
            message = CollisionEvent()
            message.header.stamp = ingress_stamp
            message.header.frame_id = ""
            message.source_stamp = rospy.Time(parsed.sec, parsed.nsec)
            message.collision_detected = bool(active)
            message.object_count = len(active)
            message.object_types = [slot.obj_type for slot in active]
            message.object_ids = [slot.obj_id for slot in active]
            message.raw_data_length = len(data)
            self._publisher.publish(message)

    def _log_stats(self, _event):
        last = "N/A"
        if self._last_recv_time is not None:
            last = "%.2fs ago" % (rospy.Time.now() - self._last_recv_time).to_sec()
        rospy.loginfo(
            "[morai_collision_bridge] recv=%d parse_ok=%d parse_fail=%d last_recv=%s",
            self._recv_count, self._parse_ok_count, self._parse_fail_count, last,
        )

    def _on_shutdown(self):
        self._receiver.close()


def main():
    rospy.init_node("morai_collision_bridge")
    try:
        bridge = MoraiCollisionBridge()
    except rospy.ROSInitException as error:
        rospy.logfatal("[morai_collision_bridge] 초기화 실패: %s", error)
        return
    bridge.spin()


if __name__ == "__main__":
    main()
