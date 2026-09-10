#!/usr/bin/env python3
"""Guard the imported EKF's isolation and the future producer/consumer boundary."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / 'src/localization_pkg'


class InterfaceContractTest(unittest.TestCase):
    def test_reserved_outputs_have_approved_consumers_and_types(self):
        contract = yaml.safe_load((ROOT / 'src/ros_architecture_pkg/config/interface_contract.yaml').read_text())
        boundary = contract['package_boundaries']['localization_pkg']
        self.assertEqual(boundary['public_nodes'], ['localization_node'])
        expected = {
            '/molit/localization/local/odometry': 'nav_msgs/Odometry',
            '/molit/localization/ego_state': 'common_msgs_pkg/EgoState',
            '/molit/localization/status': 'common_msgs_pkg/LocalizationStatus',
        }
        self.assertEqual(set(boundary['outputs']), set(expected))
        topics = {entry['name']: entry for entry in contract['topics']}
        for name, message_type in expected.items():
            entry = topics[name]
            self.assertEqual(entry['data_type'], message_type)
            self.assertEqual(entry['producers'], ['localization_node'])
            self.assertTrue(entry['consumers'])
            for consumer in entry['consumers']:
                packages = [b for b in contract['package_boundaries'].values()
                            if consumer in b['public_nodes']]
                self.assertEqual(len(packages), 1)
                self.assertIn(name, packages[0]['inputs'])

    def test_legacy_adapter_cannot_be_launched_or_built(self):
        for path in (PACKAGE / 'launch').glob('*.launch'):
            root = ET.parse(path).getroot()
            self.assertEqual(list(root.iter('node')), [])
            self.assertEqual(list(root.iter('include')), [])
        cmake = '\n'.join(line for line in (PACKAGE / 'CMakeLists.txt').read_text().splitlines()
                          if not line.lstrip().startswith('#'))
        self.assertNotIn('ego_state_estimator', cmake)
        self.assertNotIn('add_executable', cmake)


if __name__ == '__main__':
    unittest.main()
