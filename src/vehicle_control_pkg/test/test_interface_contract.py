import hashlib
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / 'src/vehicle_control_pkg'
CONFIG = ROOT / 'src/ros_architecture_pkg/config'


class InterfaceContractTest(unittest.TestCase):
    def test_cpp_connects_exact_approved_boundary(self):
        contract = yaml.safe_load((CONFIG / 'interface_contract.yaml').read_text())
        boundary = contract['package_boundaries']['vehicle_control_pkg']
        source = (PACKAGE / 'src/vehicle_controller_node.cpp').read_text()
        topics = set(re.findall(r'"(/molit/[^"]+)"', source))
        self.assertEqual(topics, set(boundary['inputs'] + boundary['outputs']))
        self.assertNotIn('/molit/safety/final_command', source)
        launch = ET.parse(PACKAGE / 'launch/vehicle_control_pkg.launch').getroot()
        node, = launch.findall('node')
        self.assertEqual(node.attrib['name'], 'vehicle_controller_node')
        self.assertEqual(node.attrib['ns'], '/')
        self.assertFalse(launch.findall('.//remap'))

    def test_original_core_and_tests_are_unchanged(self):
        manifest = yaml.safe_load((PACKAGE / 'docs/upstream_manifest.yaml').read_text())
        self.assertEqual(manifest['commit'], 'ff8eb424585183790165ef2f50042ec7c8517e4b')
        for item in manifest['files']:
            self.assertEqual(hashlib.sha256((PACKAGE / item['destination']).read_bytes()).hexdigest(), item['sha256'])

    def test_pi_class_body_matches_original_excerpt(self):
        manifest = yaml.safe_load((PACKAGE / 'docs/upstream_manifest.yaml').read_text())
        entry = manifest['extracted_pi']
        source = (PACKAGE / entry['destination']).read_text()
        excerpt = source[source.index('class BoundedPiController {'):source.index('}  // namespace longitudinal_control')]
        self.assertEqual(hashlib.sha256(excerpt.encode()).hexdigest(), entry['class_sha256'])

    def test_rates_and_queues_follow_central_contract(self):
        c = yaml.safe_load((CONFIG / 'interface_contract.yaml').read_text())
        topics = {t['name']: t for t in c['topics']}
        r = yaml.safe_load((CONFIG / 'messages/controller_messages.yaml').read_text())['runtime']
        self.assertEqual(r['control_rate_hz'], topics['/molit/control/nominal_command']['expected_rate_hz'])
        self.assertEqual(r['status_rate_hz'], topics['/molit/control/status']['expected_rate_hz'])
        self.assertEqual(r['command_queue_size'], 2)
        self.assertEqual(r['status_queue_size'], 1)
        self.assertTrue(r['status_latched'])

    def test_existing_bringup_includes_controller_launch(self):
        launch = ET.parse(ROOT / 'src/system_bringup_pkg/launch/system_bringup_pkg.launch')
        group = launch.find(".//group[@if='$(arg start_vehicle_control)']")
        self.assertEqual(group.find('include').attrib['file'],
                         '$(find vehicle_control_pkg)/launch/vehicle_control_pkg.launch')


if __name__ == '__main__':
    unittest.main()
