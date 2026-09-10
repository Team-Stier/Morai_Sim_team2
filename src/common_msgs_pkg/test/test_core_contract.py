import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / 'src/common_msgs_pkg'
CONFIG = ROOT / 'src/ros_architecture_pkg/config'


def read(path):
    return yaml.safe_load(path.read_text())


class CoreContractTest(unittest.TestCase):
    def test_wire_definitions_exactly_match_central_contract(self):
        schema = read(CONFIG / 'messages/core_messages.yaml')
        self.assertEqual(set(schema['messages']), {'ComponentStatus', 'EgoState', 'LocalizationStatus'})
        for name, entry in schema['messages'].items():
            self.assertEqual((PACKAGE / 'msg' / (name + '.msg')).read_text(),
                             entry['wire_definition'])

    def test_schema_registry_and_topic_states(self):
        contract = read(CONFIG / 'interface_contract.yaml')
        schema = read(CONFIG / contract['contract_modules']['core_messages']['path'])
        self.assertEqual(schema['contract_version'],
                         contract['contract_modules']['core_messages']['required_contract_version'])
        types = {'common_msgs_pkg/' + name for name in schema['messages']}
        for entry in contract['messages']:
            if entry['type'] in types:
                self.assertEqual(entry['status'], 'schema_implemented_runtime_not_implemented')
                self.assertIn('schema', entry)
        for topic in contract['topics']:
            if topic['data_type'] in types:
                self.assertEqual(topic['status'], 'schema_implemented_runtime_not_implemented')

    def test_no_forbidden_raw_fields_in_ego(self):
        schema = read(CONFIG / 'messages/core_messages.yaml')
        fields = [l.split()[1] for l in schema['messages']['EgoState']['wire_definition'].splitlines()
                  if l and not l.startswith('#')]
        forbidden = schema['regulation_source']['competition_vehicle_status_missing']
        self.assertFalse(set(fields).intersection(forbidden))
        self.assertIn('pose_valid', fields)
        self.assertIn('twist_valid', fields)
        self.assertFalse(any('accel' in field or 'tire' in field for field in fields))
        self.assertFalse(schema['regulation_source']['gps_imu_artificial_noise_applied_2026'])
        self.assertTrue(schema['regulation_source']['gps_blackout_still_required'])

    def test_build_dependencies_and_exact_message_set(self):
        cmake = (PACKAGE / 'CMakeLists.txt').read_text()
        names = set(re.findall(r'\b\w+\.msg\b', cmake))
        self.assertEqual(names, {'ComponentStatus.msg', 'EgoState.msg', 'LocalizationStatus.msg'})
        manifest = ET.parse(PACKAGE / 'package.xml').getroot()
        self.assertIn('message_generation', [e.text for e in manifest.findall('build_depend')])
        self.assertIn('message_runtime', [e.text for e in manifest.findall('exec_depend')])
        for dependency in ('std_msgs', 'geometry_msgs'):
            self.assertIn(dependency, [e.text for e in manifest.findall('depend')])

    def test_producers_consumers_depend_on_shared_package(self):
        contract = read(CONFIG / 'interface_contract.yaml')
        owners = {n['name']: n['owner_package'] for n in contract['nodes']}
        for topic in contract['topics']:
            if topic['data_type'] in ('common_msgs_pkg/EgoState', 'common_msgs_pkg/LocalizationStatus',
                                      'common_msgs_pkg/ComponentStatus'):
                for node in topic['producers'] + topic['consumers']:
                    package = ROOT / 'src' / owners[node]
                    deps = [e.text for e in ET.parse(package / 'package.xml').findall('depend')]
                    self.assertIn('common_msgs_pkg', deps)
                    self.assertIn('core_messages.md', (package / 'README.md').read_text())

    def test_tf_and_udp_gates_remain_closed(self):
        for tf in read(CONFIG / 'tf/frame_contract.yaml')['transforms']:
            self.assertFalse(tf['publish_enabled'])
        channels = read(CONFIG / 'morai_interface/udp_ros_bridge.yaml')['channels']
        for name in ('control', 'collision'):
            self.assertFalse(channels[name]['runtime_activation_allowed'])

    def test_launch_xml_remains_well_formed(self):
        for path in (ROOT / 'src').rglob('*.launch'):
            ET.parse(path)


if __name__ == '__main__':
    unittest.main()
