# -*- coding: utf-8 -*-
"""Safety final command -> MORAI EgoCtrlCmd UDP sender.

Development communication-probe implementation. It never repeats the last
command on a timer. Network transmission and motion-capable commands are both
opt-in. Non-zero steering is rejected until an explicit rad-to-normalized
conversion is marked verified.
"""

import math

import rospy

from common_msgs_pkg.msg import ActuatorCommand
from morai_udp_bridge.protocol import ego_ctrl_cmd_packet
from morai_udp_bridge.udp_sender import UdpSender
from morai_udp_bridge.q_key_guard import QKeyGuard


class MoraiControlSender(object):
    def __init__(self):
        self.destination_ip = str(rospy.get_param("~destination_ip", "127.0.0.1"))
        if not rospy.has_param("~port"):
            raise rospy.ROSInitException(
                "필수 파라미터 ~port가 없습니다. config/control_sender.yaml에서 지정하세요."
            )
        self.port = int(rospy.get_param("~port"))
        self.dry_run = bool(rospy.get_param("~dry_run", True))
        self.allow_motion_commands = bool(rospy.get_param("~allow_motion_commands", False))
        self.max_command_age_sec = float(rospy.get_param("~max_command_age_sec", 0.25))
        self.future_tolerance_sec = float(rospy.get_param("~future_tolerance_sec", 0.05))
        self.steering_conversion_verified = bool(
            rospy.get_param("~steering_conversion_verified", False)
        )
        self.steering_normalized_per_rad = float(
            rospy.get_param("~steering_normalized_per_rad", 1.0)
        )

        if self.max_command_age_sec <= 0.0:
            raise rospy.ROSInitException("~max_command_age_sec는 양수여야 합니다.")
        if not math.isfinite(self.steering_normalized_per_rad):
            raise rospy.ROSInitException(
                "~steering_normalized_per_rad는 유한값이어야 합니다."
            )

        self._sender = None if self.dry_run else UdpSender(
            self.destination_ip, self.port
        )
        self._sent_count = 0
        self._rejected_count = 0
        self._q_guard = None
        if not self.dry_run and rospy.get_param('~local_q_guard_enabled', False):
            self._q_guard = QKeyGuard(float(rospy.get_param('~local_q_handover_sec', 0.5)))
        self._subscriber = rospy.Subscriber(
            "/molit/safety/final_command",
            ActuatorCommand,
            self._on_command,
            queue_size=2,
        )
        rospy.on_shutdown(self._on_shutdown)

        rospy.logwarn(
            "[morai_control_sender] development probe: dst=%s:%d dry_run=%s "
            "allow_motion=%s steering_conversion_verified=%s",
            self.destination_ip, self.port, self.dry_run,
            self.allow_motion_commands, self.steering_conversion_verified,
        )

    def _on_command(self, message):
        if self._q_guard is not None and self._q_guard.blocked():
            rospy.loginfo_throttle(5.0, "[morai_control_sender] Q manual/handover: UDP paused")
            return
        self._send_command(message)

    def _send_command(self, message):
        try:
            normalized_steer = self._validate_and_convert(message)
            packet = ego_ctrl_cmd_packet.serialize_ego_ctrl_cmd(
                gear=message.gear,
                accel=message.accel,
                brake=message.brake,
                steer_normalized=normalized_steer,
            )
        except (ValueError, ego_ctrl_cmd_packet.EgoCtrlCmdPacketError) as error:
            self._rejected_count += 1
            rospy.logwarn_throttle(
                1.0, "[morai_control_sender] 명령 거부: %s" % error
            )
            return

        if self.dry_run:
            rospy.loginfo_throttle(
                1.0,
                "[morai_control_sender] dry-run packet=%dB gear=%d accel=%.3f "
                "brake=%.3f steer_norm=%.3f",
                len(packet), message.gear, message.accel,
                message.brake, normalized_steer,
            )
            return

        self._sender.send(packet)
        self._sent_count += 1
        rospy.loginfo_throttle(
            5.0, "[morai_control_sender] Auto UDP sent=%d", self._sent_count
        )

    def _validate_and_convert(self, message):
        if not message.valid:
            raise ValueError("valid=false")
        if message.header.stamp == rospy.Time():
            raise ValueError("zero command stamp")
        if message.header.frame_id not in ("", "base_link"):
            raise ValueError("command frame_id must be base_link")

        now = rospy.Time.now()
        age = (now - message.header.stamp).to_sec()
        if age < -self.future_tolerance_sec:
            raise ValueError("future command stamp: %.6fs" % age)
        if age > self.max_command_age_sec:
            raise ValueError("expired command: %.6fs" % age)

        values = (message.steering_rad, message.accel, message.brake)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("NaN/Inf command field")
        if message.accel < 0.0 or message.accel > 1.0:
            raise ValueError("accel outside 0..1")
        if message.brake < 0.0 or message.brake > 1.0:
            raise ValueError("brake outside 0..1")
        if int(message.gear) < 0 or int(message.gear) > 5:
            raise ValueError("gear outside 0..5")

        if not self.allow_motion_commands:
            if message.accel > 0.0 or abs(message.steering_rad) > 1e-12:
                raise ValueError(
                    "motion command blocked; allow_motion_commands is false"
                )

        if abs(message.steering_rad) <= 1e-12:
            normalized = 0.0
        else:
            if not self.steering_conversion_verified:
                raise ValueError(
                    "non-zero steering blocked until rad-to-normalized conversion is verified"
                )
            normalized = message.steering_rad * self.steering_normalized_per_rad

        if not math.isfinite(normalized) or normalized < -1.0 or normalized > 1.0:
            raise ValueError("normalized steer outside -1..1")
        return normalized

    def _on_shutdown(self):
        if self._q_guard is not None:
            self._q_guard.close()
        if self._sender is not None:
            self._sender.close()
        rospy.loginfo(
            "[morai_control_sender] 종료. sent=%d rejected=%d",
            self._sent_count, self._rejected_count,
        )


def main():
    rospy.init_node("morai_control_sender")
    try:
        sender = MoraiControlSender()
    except rospy.ROSInitException as error:
        rospy.logfatal("[morai_control_sender] 초기화 실패: %s", error)
        return
    rospy.spin()


if __name__ == "__main__":
    main()
