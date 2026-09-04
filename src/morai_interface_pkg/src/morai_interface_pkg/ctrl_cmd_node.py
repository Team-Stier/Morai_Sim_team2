# -*- coding: utf-8 -*-
"""
ctrl_cmd_node.py
- 역할: ROS 제어 명령(morai_msgs/CtrlCmd)을 구독해 MORAI EgoCtrlCmd UDP로 송신한다.
- 주요 클래스: MoraiCtrlCmdBridge
인터페이스
- sub ~topic (기본 /control/final_command): morai_msgs/CtrlCmd
- 송신: MORAI EgoCtrlCmd UDP(55B), destination_ip:destination_port

데이터 흐름: /control/final_command -> subscriber -> EgoCtrlCmd packer -> UDP -> MORAI

설계 메모(조건 반영)
- destination_ip/port는 파라미터. 코드에 포트 하드코딩 없음.
- cmd_type <- CtrlCmd.longlCmdType. 규정 기본 제어모드는 longi type 1(~default_cmd_type).
- ctrl_mode/gear는 YAML 기본값. CtrlCmd에 해당 필드가 있으면 우선 사용(getattr).
- accel/brake/steer 범위 검증·clamp는 ~clamp_enabled로 on/off.
- ~publish_rate_hz로 주기 송신. ~command_timeout_sec 초과 시 안전 명령
  (accel=0, brake=safe_brake, steer=0) 송신.
- 실제 MORAI destination port는 미확정. 실제 제어 성공을 주장하지 않는다.
"""

import rospy

try:
    from morai_msgs.msg import CtrlCmd
except Exception:  # morai_msgs 미빌드 시 import 오류를 노드 초기화에서 안내
    CtrlCmd = None

from morai_interface_pkg.protocol import ego_ctrl_cmd_packet
from morai_interface_pkg.udp_sender import UdpSender


class MoraiCtrlCmdBridge(object):
    """CtrlCmd를 구독해 EgoCtrlCmd UDP로 송신하는 ROS 노드."""

    def __init__(self):
        if CtrlCmd is None:
            raise rospy.ROSInitException(
                "morai_msgs/CtrlCmd import 실패. morai_msgs 패키지를 빌드/설치하세요.")

        if not rospy.has_param("~destination_ip"):
            raise rospy.ROSInitException("필수 파라미터 ~destination_ip 누락.")
        if not rospy.has_param("~destination_port"):
            raise rospy.ROSInitException("필수 파라미터 ~destination_port 누락.")
        self.destination_ip = str(rospy.get_param("~destination_ip"))
        self.destination_port = int(rospy.get_param("~destination_port"))

        self.topic = rospy.get_param("~topic", "/control/final_command")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 0.5))

        # 규정 기본 제어모드 longi type 1(YAML 변경 가능), ctrl_mode/gear 기본값.
        # MORAI NetworkModule 공식 예제 기준: ctrl_mode 2=AutoMode, gear 4=Drive
        # (자율 전진 주행). YAML/launch argument로 override 가능.
        self.default_cmd_type = int(rospy.get_param("~default_cmd_type", 1))
        self.default_ctrl_mode = int(rospy.get_param("~default_ctrl_mode", 2))
        self.default_gear = int(rospy.get_param("~default_gear", 4))

        # 안전 명령
        self.safe_brake = float(rospy.get_param("~safe_brake", 1.0))

        # 범위 검증/clamp
        self.clamp_enabled = bool(rospy.get_param("~clamp_enabled", False))
        self.accel_min = float(rospy.get_param("~accel_min", 0.0))
        self.accel_max = float(rospy.get_param("~accel_max", 1.0))
        self.brake_min = float(rospy.get_param("~brake_min", 0.0))
        self.brake_max = float(rospy.get_param("~brake_max", 1.0))
        self.steer_min = float(rospy.get_param("~steer_min", -1.0))
        self.steer_max = float(rospy.get_param("~steer_max", 1.0))

        self._sender = UdpSender(self.destination_ip, self.destination_port)
        self._subscriber = rospy.Subscriber(self.topic, CtrlCmd,
                                            self._command_callback, queue_size=10)

        self._latest = None
        self._latest_time = None
        self._send_count = 0
        self._timeout_count = 0
        self._pack_fail_count = 0
        self._first_command_logged = False

        self.stats_log_period_sec = float(rospy.get_param("~stats_log_period_sec", 5.0))

        rospy.on_shutdown(self._on_shutdown)
        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0 else 0.05
        self._timer = rospy.Timer(rospy.Duration(period), self._send_timer)
        if self.stats_log_period_sec > 0.0:
            rospy.Timer(rospy.Duration(self.stats_log_period_sec), self._log_stats)

        rospy.loginfo("[morai_ctrl_cmd_bridge] sub '%s' -> UDP %s:%d "
                      "(rate=%.1fHz, timeout=%.2fs, default_cmd_type=%d, "
                      "clamp=%s, safe_brake=%.2f)",
                      self.topic, self.destination_ip, self.destination_port,
                      self.publish_rate_hz, self.command_timeout_sec,
                      self.default_cmd_type, self.clamp_enabled, self.safe_brake)

    # 함수이름: _command_callback
    # 기능: 최신 CtrlCmd와 수신 시각을 보관한다.
    def _command_callback(self, message):
        self._latest = message
        self._latest_time = rospy.Time.now()
        if not self._first_command_logged:
            self._first_command_logged = True
            rospy.loginfo("[morai_ctrl_cmd_bridge] 첫 명령: longlCmdType=%d "
                          "accel=%.3f brake=%.3f steering=%.3f velocity=%.3f "
                          "acceleration=%.3f",
                          int(message.longlCmdType), message.accel, message.brake,
                          message.steering, message.velocity, message.acceleration)

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))

    # 함수이름: _send_timer
    # 기능: 주기적으로 최신 명령(또는 timeout 시 안전 명령)을 UDP로 송신한다.
    def _send_timer(self, _event):
        now = rospy.Time.now()
        timed_out = (self._latest is None or self._latest_time is None or
                     (now - self._latest_time).to_sec() > self.command_timeout_sec)

        if timed_out:
            self._timeout_count += 1
            packet = self._build_safe_packet()
        else:
            packet = self._build_command_packet(self._latest)

        if packet is None:
            return
        try:
            self._sender.send(packet)
            self._send_count += 1
        except OSError as error:
            rospy.logwarn_throttle(2.0, "[morai_ctrl_cmd_bridge] 송신 실패: %s" % error)

    # 함수이름: _build_command_packet
    # 기능: CtrlCmd -> EgoCtrlCmd 55바이트. 실패 시 None(카운트).
    def _build_command_packet(self, message):
        accel, brake, steer = message.accel, message.brake, message.steering
        if self.clamp_enabled:
            accel = self._clamp(accel, self.accel_min, self.accel_max)
            brake = self._clamp(brake, self.brake_min, self.brake_max)
            steer = self._clamp(steer, self.steer_min, self.steer_max)

        # ctrl_mode/gear: 메시지에 해당 필드가 있으면 우선, 없으면 YAML 기본값.
        ctrl_mode = int(getattr(message, "ctrl_mode", self.default_ctrl_mode))
        gear = int(getattr(message, "gear", self.default_gear))
        cmd_type = int(message.longlCmdType)

        try:
            return ego_ctrl_cmd_packet.build_ego_ctrl_cmd_packet(
                cmd_type=cmd_type, velocity=message.velocity,
                acceleration=message.acceleration, accel=accel, brake=brake,
                steer=steer, ctrl_mode=ctrl_mode, gear=gear)
        except ego_ctrl_cmd_packet.EgoCtrlCmdPackError as error:
            self._pack_fail_count += 1
            rospy.logwarn_throttle(2.0, "[morai_ctrl_cmd_bridge] pack 실패: %s" % error)
            return None

    # 함수이름: _build_safe_packet
    # 기능: 안전 명령(accel=0, brake=safe_brake, steer=0)을 packing한다.
    def _build_safe_packet(self):
        try:
            return ego_ctrl_cmd_packet.build_ego_ctrl_cmd_packet(
                cmd_type=self.default_cmd_type, velocity=0.0, acceleration=0.0,
                accel=0.0, brake=self.safe_brake, steer=0.0,
                ctrl_mode=self.default_ctrl_mode, gear=self.default_gear)
        except ego_ctrl_cmd_packet.EgoCtrlCmdPackError as error:
            self._pack_fail_count += 1
            rospy.logwarn_throttle(2.0,
                                   "[morai_ctrl_cmd_bridge] safe pack 실패: %s" % error)
            return None

    def _log_stats(self, _event):
        rospy.loginfo("[morai_ctrl_cmd_bridge] send=%d timeout=%d pack_fail=%d "
                      "(has_command=%s)", self._send_count, self._timeout_count,
                      self._pack_fail_count, self._latest is not None)

    def _on_shutdown(self):
        self._sender.close()
        rospy.loginfo("[morai_ctrl_cmd_bridge] 종료. send=%d timeout=%d pack_fail=%d",
                      self._send_count, self._timeout_count, self._pack_fail_count)


def main():
    rospy.init_node("morai_ctrl_cmd_bridge")
    try:
        bridge = MoraiCtrlCmdBridge()
    except rospy.ROSInitException as error:
        rospy.logfatal("[morai_ctrl_cmd_bridge] 초기화 실패: %s", error)
        return
    rospy.spin()


if __name__ == "__main__":
    main()


