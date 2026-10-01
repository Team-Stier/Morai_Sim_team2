#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import roslaunch.config
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "competition.launch"
PROFILE_PATH = PACKAGE_ROOT / "config" / "competition_network.yaml"
MORAI_CONFIG_ROOT = (
    REPOSITORY_ROOT / "src" / "morai_interface_pkg" / "config" / "competition"
)


class CompetitionNetworkBringupTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch_root = ET.parse(LAUNCH_PATH).getroot()
        with PROFILE_PATH.open("r", encoding="utf-8") as stream:
            cls.profile = yaml.safe_load(stream)

    def load_launch(self, args=()):
        return roslaunch.config.load_config_default(
            [(str(LAUNCH_PATH), list(args))], None
        )

    def test_all_competition_io_gates_default_enabled(self):
        expected = {
            "start_camera_left",
            "start_camera_right",
            "start_imu",
            "start_lidar",
            "start_lidar_watchdog",
            "start_vehicle_status",
            "start_collision",
            "start_control",
        }
        args = {
            item.attrib["name"]: item.attrib.get("default")
            for item in self.launch_root.findall("arg")
            if item.attrib.get("name") in expected
        }
        self.assertEqual(set(args), expected)
        self.assertTrue(all(value == "true" for value in args.values()))

    def test_full_stack_runtime_options(self):
        args = {
            item.attrib["name"]: item.attrib.get("default")
            for item in self.launch_root.findall("arg")
        }
        self.assertEqual(args["rviz"], "true")
        self.assertEqual(args["test_speed_cap_kph"], "0.0")

    def test_launch_uses_only_competition_network_configs(self):
        includes = list(self.launch_root.findall("include"))
        self.assertEqual(len(includes), 6)

        sensor_include = next(
            item for item in includes
            if item.attrib.get("file")
            == "$(find morai_interface_pkg)/launch/morai_interface_pkg.launch"
        )
        sensor_args = {
            item.attrib["name"]: item.attrib["value"]
            for item in sensor_include.findall("arg")
        }
        for key in (
            "camera_front_config",
            "camera_left_config",
            "camera_right_config",
            "gps_config",
            "imu_config",
            "lidar_config",
        ):
            self.assertIn("/config/competition/", sensor_args[key])

        expected_extra = {
            "$(find morai_interface_pkg)/launch/vehicle_status_bridge.launch":
                "$(find morai_interface_pkg)/config/competition/vehicle_status.yaml",
            "$(find morai_interface_pkg)/launch/collision_bridge.launch":
                "$(find morai_interface_pkg)/config/competition/collision.yaml",
            "$(find morai_interface_pkg)/launch/control_sender.launch":
                "$(find morai_interface_pkg)/config/competition/control.yaml",
        }
        for include in includes:
            path = include.attrib.get("file")
            if path not in expected_extra:
                continue
            args = {
                item.attrib["name"]: item.attrib["value"]
                for item in include.findall("arg")
            }
            self.assertEqual(args["config"], expected_extra[path])

    def test_competition_launch_reuses_algorithms_without_development_sender(self):
        includes = {
            item.attrib.get("file"): {
                arg.attrib["name"]: arg.attrib["value"]
                for arg in item.findall("arg")
            }
            for item in self.launch_root.findall("include")
        }

        localization = "$(find system_bringup_pkg)/launch/localization_visualization.launch"
        frenet = "$(find system_bringup_pkg)/launch/frenet_rddf.launch"
        self.assertIn(localization, includes)
        self.assertIn(frenet, includes)
        self.assertEqual(includes[localization]["rviz"], "$(arg rviz)")
        self.assertEqual(includes[frenet]["send_to_morai"], "false")
        self.assertEqual(
            includes[frenet]["test_speed_cap_kph"],
            "$(arg test_speed_cap_kph)",
        )

    def test_expanded_competition_stack_has_single_runtime_nodes(self):
        config = self.load_launch()
        names = [node.namespace + node.name for node in config.nodes]
        self.assertEqual(len(names), len(set(names)))

        for name in (
            "morai_camera_front",
            "morai_camera_left",
            "morai_camera_right",
            "morai_gps_bridge",
            "morai_imu_bridge",
            "morai_velodyne_cloud",
            "morai_vehicle_status_bridge",
            "morai_collision_bridge",
            "localization_node",
            "sensor_tf_publisher",
            "vehicle_rviz",
            "hd_map_server_node",
            "global_route_manager_node",
            "lidar_perception_node",
            "world_model_node",
            "path_planner_node",
            "vehicle_controller_node",
            "safety_supervisor_node",
            "morai_control_sender",
        ):
            self.assertIn("/" + name, names)

        self.assertFalse(config.params["/use_sim_time"].value)
        self.assertEqual(
            config.params["/path_planner_node/test_speed_cap_kph"].value, 0.0
        )

        # Exactly one sender must exist, and it must use the competition endpoint.
        self.assertEqual(names.count("/morai_control_sender"), 1)
        self.assertEqual(config.params["/morai_control_sender/port"].value, 9093)
        self.assertEqual(
            config.params["/morai_control_sender/destination_ip"].value,
            "192.168.0.1",
        )
        self.assertFalse(config.params["/morai_control_sender/dry_run"].value)

    def test_runtime_options_reach_full_stack(self):
        config = self.load_launch(["rviz:=false", "test_speed_cap_kph:=10.0"])
        names = {node.namespace + node.name for node in config.nodes}
        self.assertNotIn("/vehicle_rviz", names)
        self.assertIn("/vehicle_controller_node", names)
        self.assertIn("/morai_control_sender", names)
        self.assertEqual(
            config.params["/path_planner_node/test_speed_cap_kph"].value, 10.0
        )

    def test_profile_ports_match_runtime_configs(self):
        config_files = {
            "camera_front": "camera_front.yaml",
            "camera_left": "camera_left.yaml",
            "camera_right": "camera_right.yaml",
            "gps": "gps.yaml",
            "imu": "imu.yaml",
            "lidar": "lidar.yaml",
            "vehicle_status": "vehicle_status.yaml",
            "collision": "collision.yaml",
            "control": "control.yaml",
        }
        for channel, filename in config_files.items():
            with self.subTest(channel=channel):
                with (MORAI_CONFIG_ROOT / filename).open("r", encoding="utf-8") as stream:
                    config = yaml.safe_load(stream)
                self.assertEqual(
                    int(config["port"]),
                    int(self.profile["channels"][channel]["port"]),
                )

        with (MORAI_CONFIG_ROOT / "control.yaml").open("r", encoding="utf-8") as stream:
            control = yaml.safe_load(stream)
        self.assertEqual(
            control["destination_ip"],
            self.profile["channels"]["control"]["destination_ip"],
        )

    def test_competition_network_table_matches_central_contract(self):
        central_path = (
            REPOSITORY_ROOT / "src/ros_architecture_pkg/config/morai_interface"
            / "udp_ros_bridge.yaml"
        )
        central = yaml.safe_load(central_path.read_text(encoding="utf-8"))
        self.assertEqual(self.profile["network"]["client_pc_ipv4"], "192.168.0.1")
        self.assertEqual(self.profile["network"]["team_pc_ipv4"], "192.168.0.10")
        for key in ("client_pc_ipv4", "team_pc_ipv4"):
            self.assertEqual(
                self.profile["network"][key],
                central["competition_network"][key],
            )

        expected = {
            "vehicle_status": {"host_port": 9088, "destination_port": 9099},
            "collision": {"host_port": 9091, "destination_port": 9092},
            "control": {"host_port": 9093, "destination_port": 9094},
        }
        self.assertEqual(self.profile["simulator_channels"], expected)
        self.assertEqual(
            central["competition_network"]["simulator_channels"], expected
        )
        for name, ports in expected.items():
            key = "host_port" if name == "control" else "destination_port"
            self.assertEqual(self.profile["channels"][name]["port"], ports[key])

        ports = [channel["port"] for channel in self.profile["channels"].values()]
        self.assertEqual(len(ports), len(set(ports)))
        for name, channel in self.profile["channels"].items():
            self.assertEqual(
                channel["port"], central["channels"][name]["port"], name
            )
        self.assertEqual(
            central["channels"]["control"]["destination_ip"], "192.168.0.1"
        )

    def test_control_is_enabled_for_morai_simulator_validation(self):
        with (MORAI_CONFIG_ROOT / "control.yaml").open("r", encoding="utf-8") as stream:
            control = yaml.safe_load(stream)

        self.assertFalse(control["dry_run"])
        self.assertTrue(control["allow_motion_commands"])
        self.assertTrue(control["steering_conversion_verified"])
        self.assertEqual(float(control["steering_normalized_per_rad"]), 1.0)


if __name__ == "__main__":
    unittest.main()
