import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / 'src/common_msgs_pkg'
CONFIG = ROOT / 'src/ros_architecture_pkg/config'


class CompetitionIoContractTest(unittest.TestCase):
    def test_candidate_wire_definitions_match_messages(self):
        schema = yaml.safe_load((CONFIG / 'messages/competition_io_messages.yaml').read_text())
        self.assertEqual(set(schema['messages']), {'ActuatorCommand', 'CollisionEvent'})
        for name, entry in schema['messages'].items():
            self.assertEqual((PACKAGE / 'msg' / (name + '.msg')).read_text(),
                             entry['wire_definition'])

    def test_public_type_names_are_reserved_centrally(self):
        contract = yaml.safe_load((CONFIG / 'interface_contract.yaml').read_text())
        statuses = {entry['type']: entry['status'] for entry in contract['messages']}
        self.assertIn('common_msgs_pkg/ActuatorCommand', statuses)
        self.assertIn('common_msgs_pkg/CollisionEvent', statuses)

    def test_competition_command_constraints_are_locked(self):
        schema = yaml.safe_load((CONFIG / 'messages/competition_io_messages.yaml').read_text())
        constraints = schema['source_evidence']['competition_rule']['ego_ctrl_cmd_constraints']
        self.assertEqual(constraints['command_type'], 1)
        self.assertEqual(constraints['control_mode'], 2)


if __name__ == '__main__':
    unittest.main()
