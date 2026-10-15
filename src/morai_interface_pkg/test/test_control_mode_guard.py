"""Mode handover, stale transport and producer-consumer wire regression tests."""
import socket
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/morai_interface_pkg/src"))

import rospy
from common_msgs_pkg.msg import ActuatorCommand
from std_msgs.msg import UInt64MultiArray
from morai_udp_bridge.control_mode_guard import ControlModeGuard, CONTROL_MODE_TOPIC
from morai_udp_bridge.control_node import MoraiControlSender
from morai_udp_bridge.vehicle_status_node import MoraiVehicleStatusBridge
from morai_udp_bridge.protocol.competition_vehicle_status_packet import (
    build_competition_vehicle_status_packet, parse_competition_vehicle_status_packet,
)


class ControlModeGuardTest(unittest.TestCase):
    def test_actual_mode_and_timeout(self):
        g = ControlModeGuard(0.5)
        t = 10_000_000_000
        self.assertTrue(g.blocked(t))  # no initial assumption of Auto
        for mode, blocked in ((2, False), (1, True), (2, False), (0, True), (3, True)):
            g.update([mode, t])
            self.assertEqual(g.blocked(t), blocked)
        g.update([2, t])
        self.assertFalse(g.blocked(t + 499_999_999))
        self.assertTrue(g.blocked(t + 500_000_000))
        self.assertTrue(g.blocked(t - 1))
        # A fresh status restores Auto after transport loss.
        g.update([2, t + 1_000_000_000])
        self.assertFalse(g.blocked(t + 1_000_000_000))

    def test_invalid_payload_clears_auto(self):
        g = ControlModeGuard(0.5)
        for data in ([], [2], [2, 10, 1], [2, 0]):
            g.update([2, 10])
            g.update(data)
            self.assertTrue(g.blocked(10))
        for value in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                ControlModeGuard(value)

    def test_producer_to_sender_and_udp_suppression(self):
        bridge = MoraiVehicleStatusBridge.__new__(MoraiVehicleStatusBridge)
        bridge._mode_publisher = Mock()
        sender = MoraiControlSender.__new__(MoraiControlSender)
        sender._mode_guard = ControlModeGuard(0.5)
        sender._send_command = Mock()
        command = object()
        with patch('rospy.loginfo_throttle'):
            sender._on_command(command)
            sender._send_command.assert_not_called()
            # Mode is read from real parser output, then ROS serialized/deserialized.
            for mode, expected_calls in ((1, 0), (2, 1), (1, 1), (2, 2), (3, 2)):
                raw = build_competition_vehicle_status_packet(ctrl_mode=mode)
                reading = parse_competition_vehicle_status_packet(raw)
                bridge._publish_mode(reading.ctrl_mode, time.monotonic_ns())
                message = bridge._mode_publisher.publish.call_args[0][0]
                import io
                buff = io.BytesIO()
                message.serialize(buff)
                sender._on_control_mode(UInt64MultiArray().deserialize(buff.getvalue()))
                sender._on_command(command)
                self.assertEqual(sender._send_command.call_count, expected_calls)
            # Delayed queued Auto must not refresh its ingress timestamp.
            sender._on_control_mode(UInt64MultiArray(data=[2, time.monotonic_ns() - 600_000_000]))
            sender._on_command(command)
            self.assertEqual(sender._send_command.call_count, 2)

    def test_real_control_udp_obeys_mode_with_frozen_ros_time(self):
        from morai_udp_bridge.protocol.ego_ctrl_cmd_packet import parse_ego_ctrl_cmd
        sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sink.bind(('127.0.0.1', 0))
        sink.settimeout(0.03)
        params = {'~destination_ip': '127.0.0.1', '~port': sink.getsockname()[1],
                  '~dry_run': False, '~allow_motion_commands': True}
        def get_param(key, default=None):
            return params.get(key, default)
        with patch('rospy.get_param', side_effect=get_param), \
             patch('rospy.has_param', return_value=True), \
             patch('rospy.Subscriber'), patch('rospy.on_shutdown'), \
             patch('rospy.logwarn'), patch('rospy.loginfo'), \
             patch('rospy.loginfo_throttle'), patch('rospy.logwarn_throttle'), \
             patch('rospy.Time.now', return_value=rospy.Time(10)):
            sender = MoraiControlSender()
            command = ActuatorCommand()
            command.header.stamp = rospy.Time(10)
            command.header.frame_id = 'base_link'
            command.valid = True
            command.gear = 4
            command.accel = 0.2
            try:
                def expect_suppressed():
                    sender._on_command(command)
                    with self.assertRaises(socket.timeout):
                        sink.recvfrom(2048)
                expect_suppressed()  # startup before any status
                for mode in (1, 0, 3):
                    sender._on_control_mode(UInt64MultiArray(data=[mode, time.monotonic_ns()]))
                    expect_suppressed()
                sender._on_control_mode(UInt64MultiArray(data=[2, time.monotonic_ns()]))
                sender._on_command(command)
                packet, _ = sink.recvfrom(2048)
                self.assertEqual(parse_ego_ctrl_cmd(packet)['ctrl_mode'], 2)
                sender._on_control_mode(UInt64MultiArray(data=[1, time.monotonic_ns()]))
                expect_suppressed()  # handover stops actual network traffic
                sender._on_control_mode(UInt64MultiArray(data=[2, time.monotonic_ns() - 600_000_000]))
                expect_suppressed()  # monotonic watchdog works with frozen ROS time
                sender._on_control_mode(UInt64MultiArray(data=[2, time.monotonic_ns()]))
                command.valid = False
                expect_suppressed()  # upstream validation is retained
            finally:
                sender._on_shutdown()
                sink.close()

    def test_internal_contract_and_sender_profiles(self):
        central = yaml.safe_load((ROOT / 'src/ros_architecture_pkg/config/morai_interface/udp_ros_bridge.yaml').read_text())
        contract = central['channels']['vehicle_status']['internal_control_mode']
        self.assertEqual(contract['topic'], CONTROL_MODE_TOPIC)
        self.assertEqual(contract['message_type'], 'std_msgs/UInt64MultiArray')
        self.assertFalse(contract['cross_package_consumers_allowed'])
        for name in ('control_sender.yaml', 'global_path_demo.yaml', 'competition/control.yaml'):
            config = yaml.safe_load((ROOT / 'src/morai_interface_pkg/config' / name).read_text())
            self.assertEqual(config['control_mode_timeout_sec'], contract['watchdog_timeout_sec'])
            self.assertNotIn('local_q_guard_enabled', config)


if __name__ == '__main__':
    unittest.main()
