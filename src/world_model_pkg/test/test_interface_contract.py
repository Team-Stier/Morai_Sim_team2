import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]


class WorldModelInterfaceContractTest(unittest.TestCase):
    def setUp(self):
        self.contract = yaml.safe_load(
            (ROOT / "src/ros_architecture_pkg/config/interface_contract.yaml").read_text(encoding="utf-8")
        )

    def test_exact_public_node_and_topics(self):
        node = next(item for item in self.contract["nodes"] if item["name"] == "world_model_node")
        self.assertEqual(node["owner_package"], "world_model_pkg")
        self.assertEqual(node["executable"], "world_model_node")
        scene = next(item for item in self.contract["topics"] if item["name"] == "/molit/world_model/scene")
        status = next(item for item in self.contract["topics"] if item["name"] == "/molit/world_model/status")
        self.assertEqual(scene["data_type"], "common_msgs_pkg/WorldModel")
        self.assertEqual(scene["frame"], "map")
        self.assertEqual(scene["timestamp_source"], "fusion_reference_time")
        self.assertEqual(status["data_type"], "common_msgs_pkg/ComponentStatus")

    def test_node_uses_exact_scan_time_tf_and_no_latest_fallback(self):
        source = (ROOT / "src/world_model_pkg/scripts/world_model_node").read_text(encoding="utf-8")
        self.assertIn("message.header.stamp", source)
        self.assertIn("lookup_transform", source)
        self.assertNotIn('lookup_transform(\n                    self._reference_frame,\n                    self._lidar_frame,\n                    rospy.Time(0)', source)

    def test_launch_uses_exact_public_name(self):
        launch = (ROOT / "src/world_model_pkg/launch/world_model_pkg.launch").read_text(encoding="utf-8")
        self.assertIn('name="world_model_node"', launch)
        self.assertIn('type="world_model_node"', launch)


if __name__ == "__main__":
    unittest.main()
