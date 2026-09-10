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
        launch_nodes = []
        for path in (PACKAGE / 'launch').glob('*.launch'):
            root = ET.parse(path).getroot()
            for param in root.iter('param'):
                self.assertNotEqual(param.attrib.get('name', '').lstrip('/'), 'use_sim_time',
                                    'Clock mode belongs to bringup/replay, not package launch')
            for include_node in root.iter('include'):
                self.fail(f'Included launch is forbidden in package launch: {include_node}')
            for node in root.iter('node'):
                launch_nodes.append((path.name, node.attrib.get('name'), node.attrib.get('type')))
                # legacy adapter reference must never be present.
                self.assertNotIn('ego_state_estimator', (node.attrib.get('name', '') + node.attrib.get('type', '')))

        # Localization launch remains the single active public runtime node.
        self.assertTrue((PACKAGE / 'launch/localization_pkg.launch').exists())
        active_nodes = [entry for entry in launch_nodes if entry[1] == 'localization_node' and entry[2] == 'localization_node']
        self.assertEqual(1, len(active_nodes))

        cmake = (PACKAGE / 'CMakeLists.txt').read_text()
        self.assertIn('add_executable(localization_node', cmake)
        self.assertIn('target_link_libraries(localization_node', cmake)
        self.assertNotIn('ego_state_estimator', cmake)


if __name__ == '__main__':
    unittest.main()
