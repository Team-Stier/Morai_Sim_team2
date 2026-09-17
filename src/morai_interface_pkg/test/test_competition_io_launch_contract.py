import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / 'src/morai_interface_pkg'


class CompetitionIoLaunchContractTest(unittest.TestCase):
    def test_probe_launches_default_disabled(self):
        for name, expected_node in (
            ('vehicle_status_bridge.launch', 'morai_vehicle_status_bridge'),
            ('collision_bridge.launch', 'morai_collision_bridge'),
            ('control_sender.launch', 'morai_control_sender'),
        ):
            root = ET.parse(PKG / 'launch' / name).getroot()
            args = {entry.attrib['name']: entry.attrib.get('default')
                    for entry in root.findall('arg')}
            self.assertEqual(args.get('enable'), 'false')
            nodes = root.findall('node')
            self.assertEqual(len(nodes), 1)
            self.assertEqual(nodes[0].attrib['name'], expected_node)
            self.assertEqual(nodes[0].attrib.get('if'), '$(arg enable)')

    def test_probe_ports_match_persisted_simulator_defaults(self):
        expected = {
            'vehicle_status_bridge.yaml': 803,
            'collision_bridge.yaml': 9092,
            'control_sender.yaml': 9094,
        }
        for name, expected_port in expected.items():
            config = yaml.safe_load((PKG / 'config' / name).read_text())
            self.assertEqual(int(config['port']), expected_port, name)

    def test_control_defaults_fail_closed(self):
        config = yaml.safe_load((PKG / 'config/control_sender.yaml').read_text())
        self.assertTrue(config['dry_run'])
        self.assertFalse(config['allow_motion_commands'])
        self.assertFalse(config['steering_conversion_verified'])


if __name__ == '__main__':
    unittest.main()
