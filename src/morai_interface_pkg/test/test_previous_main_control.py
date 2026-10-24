"""Previous-main handover and Safety command -> real loopback UDP regressions."""
import io
import socket
import unittest
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch

import rospy
import roslaunch.config
import yaml
from common_msgs_pkg.msg import ActuatorCommand
from morai_udp_bridge.control_node import MoraiControlSender
from morai_udp_bridge.protocol.ego_ctrl_cmd_packet import parse_ego_ctrl_cmd
from morai_udp_bridge.q_key_guard import QKeyGuard


ROOT = Path(__file__).resolve().parents[3]


class PreviousMainControlTest(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.sink.close)
        self.sink.bind(('127.0.0.1', 0))
        self.sink.settimeout(0.03)
        self.params = {
            '~destination_ip': '127.0.0.1', '~port': self.sink.getsockname()[1],
            '~dry_run': False, '~allow_motion_commands': True,
            '~steering_conversion_verified': True,
            '~steering_normalized_per_rad': 1.432394487827058,
        }
        self.stack.enter_context(patch('rospy.get_param',
            side_effect=lambda key, default=None: self.params.get(key, default)))
        self.stack.enter_context(patch('rospy.has_param', return_value=True))
        self.subscriber = self.stack.enter_context(patch('rospy.Subscriber'))
        for name in ('on_shutdown', 'logwarn', 'loginfo', 'loginfo_throttle',
                     'logwarn_throttle'):
            self.stack.enter_context(patch('rospy.' + name))
        self.stack.enter_context(patch('rospy.Time.now', return_value=rospy.Time(10)))

    def sender(self):
        sender = MoraiControlSender()
        self.addCleanup(sender._on_shutdown)
        return sender

    def command(self):
        # Exercise the same generated ROS wire type used by Safety and the sender.
        message = ActuatorCommand()
        message.header.stamp = rospy.Time(10)
        message.header.frame_id = 'base_link'
        message.valid, message.gear = True, ActuatorCommand.GEAR_DRIVE
        message.accel, message.steering_rad = 0.2, 0.1
        stream = io.BytesIO()
        message.serialize(stream)
        return ActuatorCommand().deserialize(stream.getvalue())

    def assert_no_packet(self):
        with self.assertRaises(socket.timeout):
            self.sink.recvfrom(2048)

    def test_fresh_safety_command_sends_auto_without_vehicle_status(self):
        sender = self.sender()
        self.assertEqual(self.subscriber.call_args.args[0], '/molit/safety/final_command')
        self.assertEqual(self.subscriber.call_count, 1)
        sender._on_command(self.command())
        packet, _ = self.sink.recvfrom(2048)
        decoded = parse_ego_ctrl_cmd(packet)
        self.assertEqual(len(packet), 55)
        self.assertEqual((decoded['ctrl_mode'], decoded['cmd_type'], decoded['gear']), (2, 1, 4))
        self.assertAlmostEqual(decoded['accel'], 0.2, places=6)
        self.assertAlmostEqual(decoded['steer_normalized'], 0.1432394487827058, places=6)
        self.assertEqual(decoded['brake'], 0.)
        self.assert_no_packet()  # No timer may repeat the previous motion command.

    def test_q_manual_hold_and_second_press_resume_udp(self):
        sender = self.sender()
        guard = QKeyGuard.__new__(QKeyGuard)
        guard.hold_sec, guard.until = 0.5, 0.
        guard.autonomous, guard.was_down = True, False
        sender._q_guard = self.stack.enter_context(patch.object(
            sender, '_q_guard', create=True))
        message = self.command()
        for down, focused, now, blocked in (
            (False, True, 1., False),
            (True, True, 2., True),
            (True, True, 3., True),
            (False, True, 4., True),
            (True, True, 5., True),
            (False, True, 5.49, True),
            (False, True, 5.5, False),
        ):
            with self.subTest(down=down, time=now):
                value = guard._update_key(down, focused, now)
                self.assertEqual(value, blocked)
                sender._q_guard.blocked.return_value = value
                sender._on_command(message)
                if blocked:
                    self.assert_no_packet()
                else:
                    self.assertEqual(parse_ego_ctrl_cmd(self.sink.recvfrom(2048)[0])['ctrl_mode'], 2)

    def test_q_in_another_window_does_not_toggle(self):
        guard = QKeyGuard.__new__(QKeyGuard)
        guard.hold_sec, guard.until = 0.5, 0.
        guard.autonomous, guard.was_down = True, False
        self.assertFalse(guard._update_key(True, False, 1.))
        self.assertFalse(guard._update_key(False, True, 2.))
        self.assertTrue(guard.autonomous)

    def test_invalid_or_expired_final_commands_do_not_reach_udp(self):
        sender = self.sender()
        for field, value in (
            ('valid', False), ('accel', float('nan')), ('accel', 1.1),
            ('brake', -0.1), ('steering_rad', float('inf')), ('gear', 6),
            ('stamp', rospy.Time(0)), ('stamp', rospy.Time.from_sec(9.7)),
            ('stamp', rospy.Time.from_sec(10.1)),
        ):
            with self.subTest(field=field, value=value):
                message = self.command()
                if field == 'stamp':
                    message.header.stamp = value
                else:
                    setattr(message, field, value)
                sender._on_command(message)
                self.assert_no_packet()

    def test_unapproved_motion_is_rejected(self):
        self.params['~allow_motion_commands'] = False
        self.sender()._on_command(self.command())
        self.assert_no_packet()

    def test_unverified_steering_is_rejected(self):
        self.params['~steering_conversion_verified'] = False
        self.sender()._on_command(self.command())
        self.assert_no_packet()

    def test_default_and_dry_run_profiles_do_not_send_udp(self):
        config = yaml.safe_load((ROOT / 'src/morai_interface_pkg/config/control_sender.yaml').read_text())
        self.assertTrue(config['dry_run'])
        self.assertFalse(config['allow_motion_commands'])
        self.params['~dry_run'] = True
        sender = self.sender()
        sender._on_command(self.command())
        self.assertIsNone(sender._sender)
        self.assert_no_packet()

    def test_full_stack_uses_previous_main_local_handover(self):
        launch = ROOT / 'src/system_bringup_pkg/launch/frenet_all.launch'
        config = roslaunch.config.load_config_default([(str(launch), [])], None)
        names = {node.name for node in config.nodes}
        self.assertIn('morai_control_sender', names)
        self.assertNotIn('morai_vehicle_status_bridge', names)
        self.assertTrue(config.params['/morai_control_sender/local_q_guard_enabled'].value)
        self.assertEqual(config.params['/morai_control_sender/local_q_handover_sec'].value, 0.5)
        self.assertNotIn('/morai_control_sender/control_mode_timeout_sec', config.params)


if __name__ == '__main__':
    unittest.main()
