# -*- coding: utf-8 -*-
"""
gps_node.py
- 역할: MORAI GPS UDP(NMEA)를 수신·파싱해 sensor_msgs/NavSatFix로 발행한다.
- 주요 클래스: MoraiGpsBridge
인터페이스
- pub ~topic (기본 /molit/sensors/gps/fix): sensor_msgs/NavSatFix
- 입력: MORAI GPS UDP(NMEA $GPGGA/$GPRMC), bind_ip:port로 수신

설계 메모
- timestamp: MORAI GPS UDP(NMEA)에는 ROS clock으로 바로 쓸 완전한 절대 시각이
  없으므로 datagram ingress 시각을 header.stamp로 보존한다.
- covariance: NMEA로는 정확도를 알 수 없어 position_covariance_type을
  COVARIANCE_TYPE_UNKNOWN으로 두고 position_covariance는 0으로 채운다.
  (localization의 GPS projector가 UNKNOWN이면 fallback 분산을 적용한다.)
- 같은 epoch에 연속 수신되는 RMC/GGA 중 기본값은 GGA만 발행해 한 측정을
  NavSatFix 두 건으로 중복 융합하지 않는다.
- port/ip/frame_id/topic은 모두 파라미터이며 코드에 하드코딩하지 않는다.
"""

import math

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus

from morai_udp_bridge.protocol import nmea_gps
from morai_udp_bridge.udp_receiver import UdpReceiver


# 파서 상수 -> NavSatStatus 값 매핑(값 자체는 동일하게 맞춰 두었으나 명시적으로 변환).
_STATUS_TO_NAVSAT = {
    nmea_gps.STATUS_NO_FIX: NavSatStatus.STATUS_NO_FIX,
    nmea_gps.STATUS_FIX: NavSatStatus.STATUS_FIX,
    nmea_gps.STATUS_SBAS_FIX: NavSatStatus.STATUS_SBAS_FIX,
    nmea_gps.STATUS_GBAS_FIX: NavSatStatus.STATUS_GBAS_FIX,
}


class MoraiGpsBridge(object):
    """GPS UDP를 받아 NavSatFix로 발행하는 ROS 노드."""

    # 함수이름: __init__
    # 기능: 파라미터를 읽고 publisher, UDP 수신기, 통계 timer를 초기화한다.
    def __init__(self):
        self.bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
        # port는 의도적으로 기본값을 두지 않는다(실제 MORAI port는 설정에서 지정).
        if not rospy.has_param("~port"):
            raise rospy.ROSInitException(
                "필수 파라미터 ~port가 없습니다. config/gps_bridge.yaml에서 지정하세요.")
        self.port = int(rospy.get_param("~port"))
        self.topic = rospy.get_param("~topic", "/molit/sensors/gps/fix")
        self.frame_id = rospy.get_param("~frame_id", "gps_link")
        self.require_checksum = bool(rospy.get_param("~require_checksum", True))
        self.sentence_policy = str(rospy.get_param(
            "~sentence_policy", "gga_only"))
        if self.sentence_policy not in nmea_gps.SUPPORTED_SENTENCE_POLICIES:
            raise rospy.ROSInitException(
                "지원하지 않는 ~sentence_policy: %s" % self.sentence_policy)
        self.buffer_bytes = int(rospy.get_param("~receive_buffer_bytes", 2048))
        self.socket_timeout_sec = float(rospy.get_param("~socket_timeout_sec", 0.5))
        self.stats_log_period_sec = float(rospy.get_param("~stats_log_period_sec", 5.0))

        self._publisher = rospy.Publisher(self.topic, NavSatFix, queue_size=10)
        self._receiver = UdpReceiver(self.bind_ip, self.port,
                                     self.buffer_bytes, self.socket_timeout_sec)

        # 통계/상태
        self._recv_count = 0
        self._parse_ok_count = 0
        self._parse_fail_count = 0
        self._filtered_count = 0
        self._publish_count = 0
        self._last_recv_time = None
        self._first_packet_logged = False
        self._last_altitude = None  # GGA에서 갱신, RMC 발행 시 재사용

        rospy.on_shutdown(self._on_shutdown)
        if self.stats_log_period_sec > 0.0:
            rospy.Timer(rospy.Duration(self.stats_log_period_sec), self._log_stats)

        rospy.loginfo("[morai_gps_bridge] UDP %s:%d -> topic '%s' (frame_id=%s, "
                      "require_checksum=%s, sentence_policy=%s)",
                      self.bind_ip, self.port, self.topic, self.frame_id,
                      self.require_checksum, self.sentence_policy)

    # 함수이름: spin
    # 기능: shutdown까지 UDP를 수신하며 파싱/발행한다.
    def spin(self):
        while not rospy.is_shutdown():
            received = self._receiver.receive()
            if received is None:
                continue  # timeout: 종료 여부만 확인하고 계속
            data, _sender = received
            self._recv_count += 1
            ingress_stamp = rospy.Time.now()
            self._last_recv_time = ingress_stamp

            if not self._first_packet_logged:
                self._first_packet_logged = True
                rospy.loginfo("[morai_gps_bridge] 첫 패킷 %d bytes: %r",
                              len(data), data[:120])

            try:
                text = data.decode("ascii", errors="replace")
                fix = nmea_gps.parse_nmea_sentence(
                    text, require_checksum=self.require_checksum)
            except nmea_gps.NmeaParseError as error:
                self._parse_fail_count += 1
                rospy.logwarn_throttle(2.0,
                                       "[morai_gps_bridge] 파싱 실패: %s" % error)
                continue
            except Exception as error:  # 어떤 예외로도 노드가 죽지 않도록 방어
                self._parse_fail_count += 1
                rospy.logwarn_throttle(2.0,
                                       "[morai_gps_bridge] 예상치 못한 파싱 오류: %s"
                                       % error)
                continue

            self._parse_ok_count += 1
            if not nmea_gps.sentence_allowed(fix, self.sentence_policy):
                self._filtered_count += 1
                continue
            self._publish(fix, ingress_stamp)

    # 함수이름: _publish
    # 기능: GpsFix를 NavSatFix로 변환해 발행한다.
    def _publish(self, fix, ingress_stamp):
        message = NavSatFix()
        message.header.stamp = ingress_stamp
        message.header.frame_id = self.frame_id

        message.status.status = _STATUS_TO_NAVSAT.get(fix.status,
                                                      NavSatStatus.STATUS_NO_FIX)
        message.status.service = NavSatStatus.SERVICE_GPS

        message.latitude = fix.latitude if fix.latitude is not None else float("nan")
        message.longitude = fix.longitude if fix.longitude is not None else float("nan")

        if fix.altitude is not None:
            self._last_altitude = fix.altitude
            message.altitude = fix.altitude
        elif self._last_altitude is not None:
            message.altitude = self._last_altitude  # RMC: 마지막 GGA 고도 재사용
        else:
            message.altitude = float("nan")

        # 정확도 불명 -> UNKNOWN. localization이 fallback 분산을 적용한다.
        message.position_covariance = [0.0] * 9
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self._publisher.publish(message)
        self._publish_count += 1

    # 함수이름: _log_stats
    # 기능: 수신/파싱 통계를 주기적으로 로그로 남긴다.
    def _log_stats(self, _event):
        last = "N/A"
        if self._last_recv_time is not None:
            last = "%.2fs ago" % (rospy.Time.now() - self._last_recv_time).to_sec()
        rospy.loginfo("[morai_gps_bridge] recv=%d parse_ok=%d parse_fail=%d "
                      "filtered=%d published=%d last_recv=%s",
                      self._recv_count, self._parse_ok_count,
                      self._parse_fail_count, self._filtered_count,
                      self._publish_count, last)

    # 함수이름: _on_shutdown
    # 기능: 소켓을 닫고 최종 통계를 출력한다.
    def _on_shutdown(self):
        self._receiver.close()
        rospy.loginfo("[morai_gps_bridge] 종료. recv=%d parse_ok=%d parse_fail=%d "
                      "filtered=%d published=%d",
                      self._recv_count, self._parse_ok_count,
                      self._parse_fail_count, self._filtered_count,
                      self._publish_count)


# 함수이름: main
# 기능: 노드를 초기화하고 spin한다.
def main():
    rospy.init_node("morai_gps_bridge")
    try:
        bridge = MoraiGpsBridge()
    except rospy.ROSInitException as error:
        rospy.logfatal("[morai_gps_bridge] 초기화 실패: %s", error)
        return
    bridge.spin()


if __name__ == "__main__":
    main()
