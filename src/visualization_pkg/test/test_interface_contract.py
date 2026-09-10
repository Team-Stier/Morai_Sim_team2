#!/usr/bin/env python3
"""Keep RViz a consumer of the centrally approved localization estimates."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "src/visualization_pkg"


class VisualizationContractTest(unittest.TestCase):
    def test_localization_producer_and_visualizer_consumer_agree(self):
        contract = yaml.safe_load((ROOT / "src/ros_architecture_pkg/config/interface_contract.yaml").read_text())
        boundaries = contract["package_boundaries"]
        visualizer = boundaries["visualization_pkg"]
        expected = {
            "/molit/localization/ego_state": "common_msgs_pkg/EgoState",
            "/molit/localization/local/odometry": "nav_msgs/Odometry",
            "/molit/localization/status": "common_msgs_pkg/LocalizationStatus",
        }
        self.assertEqual(visualizer["public_nodes"], ["vehicle_visualizer_node"])
        self.assertEqual(set(visualizer["inputs"]), set(expected))
        self.assertEqual(visualizer["outputs"], [])
        topics = {entry["name"]: entry for entry in contract["topics"]}
        for name, data_type in expected.items():
            self.assertEqual(topics[name]["data_type"], data_type)
            self.assertIn("localization_node", topics[name]["producers"])
            self.assertIn("vehicle_visualizer_node", topics[name]["consumers"])
            self.assertIn(name, boundaries["localization_pkg"]["outputs"])

    def test_standalone_launch_has_no_tf_clock_or_driving_publishers(self):
        nodes = []
        for path in (PACKAGE / "launch").glob("*.launch"):
            launch = ET.parse(path).getroot()
            self.assertEqual(list(launch.iter("remap")), [], str(path))
            for param in launch.iter("param"):
                self.assertNotEqual(param.get("name", "").lstrip("/"), "use_sim_time")
            for node in launch.iter("node"):
                nodes.append(node)
                self.assertNotIn(node.get("pkg"), {"tf", "tf2_ros", "localization_pkg", "morai_interface_pkg"})
                self.assertNotIn("static_transform", node.get("type", ""))
        visualizers = [node for node in nodes if node.get("name") == "vehicle_visualizer_node"]
        self.assertEqual(len(visualizers), 1)
        self.assertEqual(visualizers[0].get("pkg"), "visualization_pkg")
        self.assertEqual(visualizers[0].get("ns", "/"), "/")


if __name__ == "__main__":
    unittest.main()
