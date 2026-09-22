import io
import re
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / 'src/common_msgs_pkg'
CONFIG = ROOT / 'src/ros_architecture_pkg/config'


class ControllerContractTest(unittest.TestCase):
    def test_wire_schema_and_build_registry(self):
        contract = yaml.safe_load((CONFIG / 'interface_contract.yaml').read_text())
        module = contract['contract_modules']['controller_messages']
        schema = yaml.safe_load((CONFIG / module['path']).read_text())
        self.assertEqual(module['required_contract_version'], schema['contract_version'])
        registry = {entry['type']: entry for entry in contract['messages']}
        cmake = (PACKAGE / 'CMakeLists.txt').read_text()
        self.assertEqual(set(schema['messages']), {'Trajectory', 'ControllerStatus'})
        for name, entry in schema['messages'].items():
            self.assertEqual((PACKAGE / 'msg' / (name+'.msg')).read_text(), entry['wire_definition'])
            self.assertIn(name+'.msg', cmake)
            self.assertIn('schema', registry['common_msgs_pkg/'+name])

    def test_producer_consumer_serialization(self):
        from common_msgs_pkg.msg import Trajectory, ControllerStatus
        from geometry_msgs.msg import Pose
        import rospy
        trajectory = Trajectory()
        trajectory.header.stamp = rospy.Time(10, 123)
        trajectory.header.frame_id = 'odom'
        trajectory.reset_id = 9
        trajectory.valid = True
        trajectory.valid_for = rospy.Duration(0.5)
        pose = Pose()
        pose.orientation.w = 1.0
        trajectory.poses = [pose, pose]
        trajectory.speed_mps = [2.0, 0.0]
        trajectory.time_from_start = [rospy.Duration(0), rospy.Duration(1)]
        status = ControllerStatus()
        status.header.stamp = rospy.Time(10, 456)
        status.trajectory_stamp = trajectory.header.stamp
        status.command_valid = True
        status.mode = ControllerStatus.PURE_PURSUIT
        for message in (trajectory, status):
            buffer = io.BytesIO()
            message.serialize(buffer)
            decoded = type(message)().deserialize(buffer.getvalue())
            self.assertEqual(message, decoded)


if __name__ == '__main__':
    unittest.main()
