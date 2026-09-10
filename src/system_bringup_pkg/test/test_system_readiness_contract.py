#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "system_readiness.launch"
TOP_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "system_bringup_pkg.launch"
CONFIG_PATH = PACKAGE_ROOT / "config" / "system_readiness.yaml"
SCRIPT_PATH = PACKAGE_ROOT / "scripts" / "system_readiness_node"


class SystemReadinessContractTest(unittest.TestCase):
    def test_public_node_name_and_executable_are_exact(self):
        root = ET.parse(LAUNCH_PATH).getroot()
        nodes = root.findall("node")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].attrib.get("pkg"), "system_bringup_pkg")
        self.assertEqual(nodes[0].attrib.get("type"), "system_readiness_node")
        self.assertEqual(nodes[0].attrib.get("name"), "system_readiness_node")
        self.assertEqual(nodes[0].attrib.get("required"), "true")

    def test_default_required_component_set_is_complete(self):
        profile = yaml.safe_load(CONFIG_PATH.read_text())
        self.assertEqual(
            profile["required_components"],
            [
                "interface",
                "map",
                "camera_perception",
                "lidar_perception",
                "localization",
                "route",
                "world_model",
                "planning",
                "control",
            ],
        )
        self.assertGreater(profile["publish_rate_hz"], 0.0)
        self.assertGreater(profile["status_timeout_sec"], 0.0)

    def test_script_uses_only_approved_public_status_topics(self):
        text = SCRIPT_PATH.read_text()
        expected = {
            "/molit/interface/status",
            "/molit/map/status",
            "/molit/perception/camera/status",
            "/molit/perception/lidar/status",
            "/molit/localization/status",
            "/molit/route/status",
            "/molit/world_model/status",
            "/molit/planning/status",
            "/molit/control/status",
            "/molit/system/readiness",
        }
        for topic in expected:
            self.assertIn('"%s"' % topic, text)

    def test_top_level_readiness_gate_stays_opt_in(self):
        root = ET.parse(TOP_LAUNCH_PATH).getroot()
        args = {
            item.attrib.get("name"): item.attrib.get("default")
            for item in root.findall("arg")
        }
        self.assertEqual(args.get("start_system_readiness"), "false")


if __name__ == "__main__":
    unittest.main()
