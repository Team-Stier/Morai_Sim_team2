# -*- coding: utf-8 -*-
"""MORAI adapter의 config/launch가 중앙 ROS 계약과 일치하는지 검사한다."""

import os
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WORKSPACE_SRC = os.path.dirname(PACKAGE_ROOT)
ARCHITECTURE_ROOT = os.path.join(WORKSPACE_SRC, "ros_architecture_pkg")
BRIDGE_CONTRACT = os.path.join(
    ARCHITECTURE_ROOT, "config", "morai_interface", "udp_ros_bridge.yaml")
TOP_LEVEL_CONTRACT = os.path.join(
    ARCHITECTURE_ROOT, "config", "interface_contract.yaml")


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class InterfaceContractAlignmentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = load_yaml(BRIDGE_CONTRACT)
        cls.top_level_contract = load_yaml(TOP_LEVEL_CONTRACT)

    def test_import_source_is_pinned(self):
        source = self.contract["source_import"]
        self.assertEqual(
            source["repository"],
            "https://github.com/Team-Stier/Morai_Sim-Stier-Team2")
        self.assertEqual(
            source["commit"],
            "f889f7f5ae47b51c4c5c0211c6a92a62398ca269")

    def test_local_configs_match_central_channel_contract(self):
        config_by_channel = {
            "camera_front": "camera_front.yaml",
            "camera_left": "camera_left.yaml",
            "camera_right": "camera_right.yaml",
            "gps": "gps_bridge.yaml",
            "imu": "imu_bridge.yaml",
            "vehicle_status": "vehicle_status_bridge.yaml",
            "lidar": "lidar_bridge.yaml",
        }
        for channel_name, filename in config_by_channel.items():
            channel = self.contract["channels"][channel_name]
            local = load_yaml(os.path.join(PACKAGE_ROOT, "config", filename))
            self.assertEqual(local["port"], channel["port"], channel_name)
            self.assertEqual(local["frame_id"], channel["frame_id"], channel_name)
            if "topic" in local:
                self.assertEqual(local["topic"], channel["topic"], channel_name)

    def test_camera_measurement_stamp_is_selected(self):
        mapping = self.contract["runtime_timestamp_parameter_mapping"]["packet"]
        for role in ("front", "left", "right"):
            local = load_yaml(os.path.join(
                PACKAGE_ROOT, "config", "camera_%s.yaml" % role))
            self.assertEqual(local["timestamp_source"], "packet")
            self.assertTrue(local["check_chunk_index"])
            channel_name = "camera_%s" % role
            self.assertIn(channel_name, mapping["allowed_channels"])
            self.assertEqual(
                self.contract["channels"][channel_name]["timestamp_source"],
                mapping["central_timestamp_source"],
            )

    def test_receive_timestamp_runtime_enum_maps_to_central_source(self):
        mapping = self.contract["runtime_timestamp_parameter_mapping"]["receive"]
        for channel_name, filename in (
            ("imu", "imu_bridge.yaml"),
            ("vehicle_status", "vehicle_status_bridge.yaml"),
        ):
            local = load_yaml(os.path.join(PACKAGE_ROOT, "config", filename))
            self.assertEqual(local["timestamp_source"], "receive")
            self.assertIn(channel_name, mapping["allowed_channels"])
            self.assertEqual(
                self.contract["channels"][channel_name]["timestamp_source"],
                mapping["central_timestamp_source"],
            )

    def test_gps_filters_duplicate_epoch_rmc(self):
        local = load_yaml(os.path.join(
            PACKAGE_ROOT, "config", "gps_bridge.yaml"))
        self.assertEqual(local["sentence_policy"], "gga_only")
        self.assertEqual(
            self.contract["channels"]["gps"]["sentence_policy"], "gga_only")

    def test_launches_use_architecture_owner_package(self):
        allowed_packages = {
            "morai_interface_pkg", "nodelet", "velodyne_driver",
            "velodyne_pointcloud"}
        launch_dir = os.path.join(PACKAGE_ROOT, "launch")
        for filename in os.listdir(launch_dir):
            if not filename.endswith(".launch"):
                continue
            root = ET.parse(os.path.join(launch_dir, filename)).getroot()
            for node in root.iter("node"):
                self.assertIn(node.attrib["pkg"], allowed_packages, filename)

    def test_unverified_channels_are_gated_off(self):
        expected_gate = {
            "imu_bridge.launch": ("enable", "false"),
            "lidar_bridge.launch": ("enable", "false"),
            "lidar_watchdog.launch": ("enable", "false"),
            "vehicle_status_bridge.launch": ("allow_legacy", "false"),
        }
        for filename, expected in expected_gate.items():
            root = ET.parse(os.path.join(PACKAGE_ROOT, "launch", filename)).getroot()
            args = {arg.attrib["name"]: arg.attrib.get("default")
                    for arg in root.findall("arg")}
            self.assertEqual(args.get(expected[0]), expected[1], filename)

    def test_lidar_internal_packet_topic_is_scoped(self):
        root = ET.parse(os.path.join(
            PACKAGE_ROOT, "launch", "lidar_bridge.launch")).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in root.findall("arg")}
        expected = "/molit/internal/morai_interface/lidar/packets"
        self.assertEqual(args["packets_topic"], expected)
        remap_targets = [remap.attrib.get("to")
                         for remap in root.iter("remap")]
        self.assertGreaterEqual(remap_targets.count("$(arg packets_topic)"), 2)

    def test_public_launch_node_and_points_topic_are_not_remappable(self):
        camera_root = ET.parse(os.path.join(
            PACKAGE_ROOT, "launch", "camera_bridge.launch")).getroot()
        camera_args = {arg.attrib["name"] for arg in camera_root.findall("arg")}
        self.assertNotIn("name", camera_args)
        camera_nodes = list(camera_root.iter("node"))
        self.assertEqual(len(camera_nodes), 1)
        self.assertEqual(camera_nodes[0].attrib["name"], "morai_camera_front")

        lidar_root = ET.parse(os.path.join(
            PACKAGE_ROOT, "launch", "lidar_bridge.launch")).getroot()
        lidar_args = {arg.attrib["name"] for arg in lidar_root.findall("arg")}
        self.assertNotIn("points_topic", lidar_args)
        self.assertNotIn("frame_id", lidar_args)
        point_targets = [
            remap.attrib.get("to")
            for remap in lidar_root.iter("remap")
            if remap.attrib.get("from") == "velodyne_points"
        ]
        self.assertEqual(point_targets, ["/molit/sensors/lidar/points"])

    def test_detail_and_top_level_topic_contracts_match(self):
        topics = {item["name"]: item
                  for item in self.top_level_contract["topics"]}
        for channel_name in ("camera_front", "camera_left", "camera_right",
                             "gps", "imu", "vehicle_status", "lidar",
                             "lidar_status"):
            channel = self.contract["channels"][channel_name]
            topic = topics[channel["topic"]]
            self.assertEqual(topic["producers"], [channel["node_name"]],
                             channel_name)
            self.assertEqual(topic["data_type"], channel["message_type"],
                             channel_name)
            self.assertEqual(topic["frame"], channel["frame_id"], channel_name)
            self.assertEqual(topic["timestamp_source"],
                             channel["timestamp_source"], channel_name)
            self.assertEqual(topic["expected_rate_hz"],
                             channel["configured_rate_hz"], channel_name)

        lidar = self.contract["channels"]["lidar"]
        packet_topic = topics[lidar["internal_packet_topic"]]
        self.assertEqual(packet_topic["data_type"],
                         "velodyne_msgs/VelodyneScan")
        self.assertEqual(packet_topic["runtime_graph_caller"],
                         lidar["runtime_graph_caller"])

        nodes = {item["name"] for item in self.top_level_contract["nodes"]}
        for channel in self.contract["channels"].values():
            self.assertIn(channel["node_name"], nodes)
        self.assertIn(lidar["driver_node_name"], nodes)
        self.assertIn(lidar["nodelet_manager_name"], nodes)

        message_types = {item["type"]
                         for item in self.top_level_contract["messages"]}
        for topic in topics.values():
            self.assertIn(topic["data_type"], message_types, topic["name"])

    def test_ros_to_morai_control_files_are_absent(self):
        forbidden = (
            "scripts/ctrl_cmd_sender_node",
            "src/morai_udp_bridge/ctrl_cmd_node.py",
            "src/morai_udp_bridge/udp_sender.py",
            "src/morai_udp_bridge/protocol/ego_ctrl_cmd_packet.py",
            "launch/ctrl_cmd_bridge.launch",
            "config/ctrl_cmd_bridge.yaml",
        )
        for relative_path in forbidden:
            self.assertFalse(os.path.exists(os.path.join(PACKAGE_ROOT, relative_path)),
                             relative_path)

    def test_entrypoints_fail_closed_on_namespace_collision(self):
        scripts_dir = os.path.join(PACKAGE_ROOT, "scripts")
        for filename in os.listdir(scripts_dir):
            with open(os.path.join(scripts_dir, filename), "r",
                      encoding="utf-8") as stream:
                source = stream.read()
            self.assertIn("RUNTIME_OWNER_PACKAGE", source, filename)
            self.assertIn('!= "morai_interface_pkg"', source, filename)


if __name__ == "__main__":
    unittest.main()
