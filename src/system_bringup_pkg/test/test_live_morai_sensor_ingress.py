#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
PROFILE_PATH = PACKAGE_ROOT / "config" / "live_morai_sensor_ingress.yaml"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "system_bringup_pkg.launch"
UDP_CONTRACT_PATH = (
    REPOSITORY_ROOT
    / "src"
    / "ros_architecture_pkg"
    / "config"
    / "morai_interface"
    / "udp_ros_bridge.yaml"
)


class LiveMoraiSensorIngressProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with PROFILE_PATH.open("r", encoding="utf-8") as stream:
            cls.profile = yaml.safe_load(stream)
        with UDP_CONTRACT_PATH.open("r", encoding="utf-8") as stream:
            cls.udp_contract = yaml.safe_load(stream)
        cls.launch_root = ET.parse(LAUNCH_PATH).getroot()

    def test_profile_declares_phase1_channel_sets(self):
        self.assertEqual(
            self.profile["required_channels"],
            ["camera_front", "camera_left", "camera_right", "gps"],
        )
        self.assertEqual(self.profile["optional_channels"], [])

    def test_every_active_profile_channel_is_runtime_allowed(self):
        active_channels = (
            self.profile["required_channels"] + self.profile["optional_channels"]
        )
        for channel_name in active_channels:
            with self.subTest(channel=channel_name):
                self.assertIn(channel_name, self.udp_contract["channels"])
                self.assertTrue(
                    self.udp_contract["channels"][channel_name][
                        "runtime_activation_allowed"
                    ]
                )

    def test_excluded_channels_remain_runtime_disabled(self):
        for channel_name in self.profile["excluded_channels"]:
            with self.subTest(channel=channel_name):
                self.assertIn(channel_name, self.udp_contract["channels"])
                channel = self.udp_contract["channels"][channel_name]
                if channel_name == "imu":
                    # Excluded from phase1, but centrally approved for localization development.
                    self.assertTrue(channel["runtime_activation_allowed"])
                    self.assertEqual(channel["activation_scope"], "development_only")
                    self.assertFalse(channel["autonomous_driving_ready"])
                else:
                    self.assertFalse(channel["runtime_activation_allowed"])

    def test_launch_matches_phase1_profile(self):
        use_sim_time = next(
            param
            for param in self.launch_root.findall("param")
            if param.attrib.get("name") == "/use_sim_time"
        )
        self.assertEqual(use_sim_time.attrib.get("value"), "false")

        includes = [
            include
            for include in self.launch_root.findall("include")
            if include.attrib.get("file")
            == "$(find morai_interface_pkg)/launch/morai_interface_pkg.launch"
        ]
        self.assertEqual(len(includes), 1)

        launch_args = {
            arg.attrib["name"]: arg.attrib["value"]
            for arg in includes[0].findall("arg")
        }
        self.assertEqual(
            launch_args,
            {
                "start_cameras": "true",
                "start_gps": "true",
                "start_imu": "false",
                "start_lidar": "false",
                "start_lidar_watchdog": "false",
            },
        )


if __name__ == "__main__":
    unittest.main()
