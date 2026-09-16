# -*- coding: utf-8 -*-
"""
camera_node.py
- 역할: MORAI Camera UDP 청크를 수신·재조립해 sensor_msgs/CompressedImage(jpeg)로
        발행한다. 한 노드가 카메라 1대를 담당하며, 여러 인스턴스로 실행할 수 있다.
- 주요 클래스: MoraiCameraBridge
인터페이스
- pub ~topic (예 /molit/sensors/camera/front/image/compressed): sensor_msgs/CompressedImage
- 입력: MORAI Camera UDP(MOR 청크), bind_ip:port로 수신

설계 메모
- 완성된 JPEG bytes를 그대로 발행한다(OpenCV 재인코딩 안 함).
- MOR 청크만 처리하고 BOX(bounding box)는 무시한다.
- 프레임 종료는 공식 예제의 tail=='EI' 규칙을 따른다.
- 유실/순서오류/크기초과/잘못된 header/timeout은 해당 프레임만 폐기하고 계속 실행.
- timestamp: ~timestamp_source = receive(완성 청크 수신시각) | packet(프레임 sec/nsec).
- 실제 MORAI 포트와 카메라 이름은 아직 확정되지 않았다(YAML로 지정).
"""

import time

import rospy
from sensor_msgs.msg import CompressedImage

from morai_udp_bridge.protocol import camera_packet
from morai_udp_bridge.timestamp_guard import PacketTimestampGuard
from morai_udp_bridge.udp_receiver import UdpReceiver


class MoraiCameraBridge(object):
    """Camera UDP를 받아 CompressedImage로 발행하는 ROS 노드(카메라 1대)."""

    def __init__(self):
        self.bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
        if not rospy.has_param("~port"):
            raise rospy.ROSInitException(
                "필수 파라미터 ~port가 없습니다. camera config에서 지정하세요.")
        self.port = int(rospy.get_param("~port"))
        self.topic = rospy.get_param("~topic",
                                     "/molit/sensors/camera/image/compressed")
        self.frame_id = rospy.get_param("~frame_id", "camera_front_optical_frame")

        self.timestamp_source = str(rospy.get_param("~timestamp_source", "receive"))
        self.max_frame_bytes = int(rospy.get_param("~max_frame_bytes", 2000000))
        self.frame_timeout_sec = float(rospy.get_param("~frame_timeout_sec", 0.5))
        self.check_chunk_index = bool(rospy.get_param("~check_chunk_index", False))

        # Camera 패킷은 최대 65000바이트이므로 수신 버퍼를 넉넉히 잡는다.
        self.buffer_bytes = int(rospy.get_param("~receive_buffer_bytes", 65536))
        self.socket_timeout_sec = float(rospy.get_param("~socket_timeout_sec", 0.5))
        self.stats_log_period_sec = float(rospy.get_param("~stats_log_period_sec", 5.0))

        self._publisher = rospy.Publisher(self.topic, CompressedImage, queue_size=2)
        self._receiver = UdpReceiver(self.bind_ip, self.port,
                                     self.buffer_bytes, self.socket_timeout_sec)
        self._assembler = camera_packet.JpegFrameAssembler(
            max_frame_bytes=self.max_frame_bytes,
            frame_timeout_sec=self.frame_timeout_sec,
            check_chunk_index=self.check_chunk_index)
        self._timestamp_guard = PacketTimestampGuard()

        self._recv_count = 0
        self._box_count = 0
        self._parse_fail_count = 0
        self._timestamp_reject_count = 0
        self._last_recv_time = None
        self._first_packet_logged = False

        rospy.on_shutdown(self._on_shutdown)
        if self.stats_log_period_sec > 0.0:
            rospy.Timer(rospy.Duration(self.stats_log_period_sec), self._log_stats)

        rospy.loginfo("[morai_camera_bridge] UDP %s:%d -> topic '%s' (frame_id=%s, "
                      "ts=%s, max_frame=%dB, timeout=%.2fs, check_index=%s)",
                      self.bind_ip, self.port, self.topic, self.frame_id,
                      self.timestamp_source, self.max_frame_bytes,
                      self.frame_timeout_sec, self.check_chunk_index)

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
                header = bytes(data[:camera_packet.HEADER_LEN])
                rospy.loginfo("[morai_camera_bridge] 첫 패킷 %d bytes, header=%r",
                              len(data), header)

            try:
                kind, chunk = camera_packet.parse_camera_packet(data)
            except camera_packet.CameraParseError as error:
                self._parse_fail_count += 1
                rospy.logwarn_throttle(2.0,
                                       "[morai_camera_bridge] 패킷 폐기: %s" % error)
                continue
            except Exception as error:
                self._parse_fail_count += 1
                rospy.logwarn_throttle(2.0,
                                       "[morai_camera_bridge] 예상치 못한 오류: %s"
                                       % error)
                continue

            if kind == "BOX":
                self._box_count += 1
                continue  # BOX는 이번 단계에서 무시

            result = self._assembler.add_chunk(chunk, time.monotonic())
            if result is not None:
                frame, sec, nsec = result
                self._publish(frame, sec, nsec, ingress_stamp)

    # 함수이름: _publish
    # 기능: 완성된 JPEG bytes를 CompressedImage로 발행한다(재인코딩 없음).
    def _publish(self, frame_bytes, sec, nsec, ingress_stamp):
        message = CompressedImage()
        if self.timestamp_source == "packet":
            reject_reason = self._timestamp_guard.check(
                sec, nsec, ingress_stamp.secs, ingress_stamp.nsecs)
            if reject_reason is not None:
                self._timestamp_reject_count += 1
                rospy.logwarn_throttle(
                    2.0, "[morai_camera_bridge] packet timestamp 폐기: "
                    "%d.%09d reason=%s" % (sec, nsec, reject_reason))
                return
            message.header.stamp = rospy.Time(sec, nsec)
        else:
            message.header.stamp = ingress_stamp
        message.header.frame_id = self.frame_id
        message.format = "jpeg"
        message.data = frame_bytes
        self._publisher.publish(message)

    def _log_stats(self, _event):
        last = "N/A"
        if self._last_recv_time is not None:
            last = "%.2fs ago" % (rospy.Time.now() - self._last_recv_time).to_sec()
        rospy.loginfo("[morai_camera_bridge] recv=%d completed=%d discarded=%d "
                      "box=%d parse_fail=%d timestamp_reject=%d last_recv=%s "
                      "(last_discard=%s)",
                      self._recv_count, self._assembler.completed_frames,
                      self._assembler.discarded_frames, self._box_count,
                      self._parse_fail_count, self._timestamp_reject_count, last,
                      self._assembler.last_discard_reason)

    def _on_shutdown(self):
        self._receiver.close()
        rospy.loginfo("[morai_camera_bridge] 종료. recv=%d completed=%d discarded=%d",
                      self._recv_count, self._assembler.completed_frames,
                      self._assembler.discarded_frames)


def main():
    rospy.init_node("morai_camera_bridge")
    try:
        bridge = MoraiCameraBridge()
    except rospy.ROSInitException as error:
        rospy.logfatal("[morai_camera_bridge] 초기화 실패: %s", error)
        return
    bridge.spin()


if __name__ == "__main__":
    main()
