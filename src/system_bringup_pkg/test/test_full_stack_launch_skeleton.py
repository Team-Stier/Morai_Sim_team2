#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "system_bringup_pkg.launch"


class FullStackLaunchSkeletonTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(LAUNCH_PATH).getroot()

    def test_downstream_integration_gates_default_to_false(self):
        expected_args = {
            "start_hd_map",
            "start_camera_perception",
            "start_lidar_perception",
            "start_localization",
            "start_global_route_manager",
            "start_world_model",
            "start_path_planning",
            "start_vehicle_control",
            "start_system_readiness",
            "start_safety_supervisor",
            "start_runtime_evaluation",
        }
        args = {
            item.attrib["name"]: item.attrib.get("default")
            for item in self.root.findall("arg")
            if item.attrib.get("name") in expected_args
        }
        self.assertEqual(set(args), expected_args)
        self.assertTrue(all(value == "false" for value in args.values()))

    def test_every_downstream_gate_points_to_owned_package_launch(self):
        expected = {
            "start_hd_map": "$(find hd_map_pkg)/launch/hd_map_pkg.launch",
            "start_camera_perception": "$(find camera_perception_pkg)/launch/camera_perception_pkg.launch",
            "start_lidar_perception": "$(find lidar_perception_pkg)/launch/lidar_perception_pkg.launch",
            "start_localization": "$(find localization_pkg)/launch/localization_pkg.launch",
            "start_global_route_manager": "$(find global_route_manager_pkg)/launch/global_route_manager_pkg.launch",
            "start_world_model": "$(find world_model_pkg)/launch/world_model_pkg.launch",
            "start_path_planning": "$(find path_planning_pkg)/launch/path_planning_pkg.launch",
            "start_vehicle_control": "$(find vehicle_control_pkg)/launch/vehicle_control_pkg.launch",
            "start_system_readiness": "$(find system_bringup_pkg)/launch/system_readiness.launch",
            "start_safety_supervisor": "$(find safety_supervisor_pkg)/launch/safety_supervisor_pkg.launch",
            "start_runtime_evaluation": "$(find runtime_evaluation_pkg)/launch/runtime_evaluation_pkg.launch",
        }

        actual = {}
        for group in self.root.findall("group"):
            condition = group.attrib.get("if", "")
            if not condition.startswith("$(arg "):
                continue
            arg_name = condition[len("$(arg "):-1]
            include = group.find("include")
            if include is not None:
                actual[arg_name] = include.attrib.get("file")

        self.assertEqual(actual, expected)

    def test_non_runtime_packages_are_not_launched(self):
        include_files = [
            include.attrib.get("file", "") for include in self.root.iter("include")
        ]
        self.assertFalse(any("common_msgs_pkg" in item for item in include_files))
        self.assertFalse(any("ros_architecture_pkg" in item for item in include_files))


if __name__ == "__main__":
    unittest.main()
