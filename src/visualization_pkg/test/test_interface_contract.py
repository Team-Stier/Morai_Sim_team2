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
        self.assertEqual(visualizer["public_nodes"], ["vehicle_visualizer_node", "vehicle_rviz"])
        self.assertEqual(
            set(visualizer["inputs"]),
            set(expected) | {"/molit/perception/lidar/observations", "/molit/world_model/scene", "/molit/planning/status",
                             "/molit/world_model/painted_points", "/molit/sensors/camera/front/image/compressed"},
        )
        lidar = next(t for t in contract["topics"] if t["name"] == "/molit/perception/lidar/observations")
        self.assertEqual(lidar["data_type"], "common_msgs_pkg/LidarObservationArray")
        self.assertEqual(lidar["producers"], ["lidar_perception_node"])
        self.assertIn("vehicle_visualizer_node", lidar["consumers"])
        world_model = next(t for t in contract["topics"] if t["name"] == "/molit/world_model/scene")
        self.assertEqual(world_model["data_type"], "common_msgs_pkg/WorldModel")
        self.assertIn("vehicle_visualizer_node", world_model["consumers"])
        self.assertEqual(visualizer["outputs"], [])
        topics = {entry["name"]: entry for entry in contract["topics"]}
        planning = topics["/molit/planning/status"]
        self.assertEqual(planning["data_type"], "common_msgs_pkg/ComponentStatus")
        self.assertEqual(planning["producers"], ["path_planner_node"])
        self.assertIn("vehicle_visualizer_node", planning["consumers"])
        internal = topics["/molit/internal/visualization/planner_status"]
        self.assertEqual(internal["producers"], ["vehicle_visualizer_node"])
        self.assertEqual(internal["consumers"], ["vehicle_rviz"])
        self.assertEqual(internal["data_type"], planning["data_type"])
        config = yaml.safe_load((PACKAGE / "config/vehicle.rviz").read_text())
        self.assertTrue(any(display["Class"] == "visualization_pkg/PlannerStatus" and display["Enabled"]
                            for display in config["Visualization Manager"]["Displays"]))
        plugin = ET.parse(PACKAGE / "rviz_plugins.xml").getroot().find("class")
        self.assertEqual(plugin.get("name"), "visualization_pkg/PlannerStatus")
        for name, data_type in expected.items():
            self.assertEqual(topics[name]["data_type"], data_type)
            self.assertIn("localization_node", topics[name]["producers"])
            self.assertIn("vehicle_visualizer_node", topics[name]["consumers"])
            self.assertIn(name, boundaries["localization_pkg"]["outputs"])

    def test_pidnet_rviz_shows_foreground_and_selected_signal_state(self):
        config = yaml.safe_load((PACKAGE / 'config/pidnet_vehicle.rviz').read_text())
        displays = config['Visualization Manager']['Displays']
        cloud = next(d for d in displays if d['Class'] == 'rviz/PointCloud2')
        self.assertTrue(cloud['Enabled'])
        self.assertEqual(cloud['Topic'], '/molit/world_model/painted_points')
        self.assertEqual(cloud['Color Transformer'], 'RGB8')
        self.assertGreater(cloud['Decay Time'], 0)
        for item in displays:
            if item.get('Marker Topic') == '/molit/internal/visualization/lidar_markers':
                self.assertFalse(item['Enabled'])
                self.assertFalse(item['Value'])
            if item.get('Marker Topic') == '/molit/internal/visualization/world_model_markers':
                self.assertTrue(item['Enabled'])
                self.assertTrue(item['Value'])

    def test_standalone_launch_has_no_tf_clock_or_driving_publishers(self):
        nodes = []
        for path in (PACKAGE / "launch").glob("*.launch"):
            launch = ET.parse(path).getroot()
            for remap in launch.iter('remap'):
                # Only redirect an upstream GUI helper into this node's private namespace.
                self.assertEqual(path.name, 'pointpainting.launch')
                self.assertEqual(remap.attrib, {'from': '/molit/sensors/camera/front/image/mouse_click',
                                               'to': '~front_image_click'})
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

    def test_pointpainting_rviz_consumes_approved_fields_and_topics(self):
        contract = yaml.safe_load((ROOT / 'src/ros_architecture_pkg/config/interface_contract.yaml').read_text())
        topics = {t['name']: t for t in contract['topics']}
        config = yaml.safe_load((PACKAGE / 'config/pointpainting.rviz').read_text())['Visualization Manager']
        self.assertEqual(config['Global Options']['Fixed Frame'], topics['/molit/world_model/painted_points']['frame'])
        cloud = next(d for d in config['Displays'] if d['Name'] == 'Painted points')
        self.assertEqual(cloud['Color Transformer'], 'RGB8')
        self.assertIn('vehicle_rviz', topics[cloud['Topic']]['consumers'])
        image = next(d for d in config['Displays'] if d['Class'] == 'rviz/Image')
        self.assertEqual(image['Transport Hint'], 'compressed')
        self.assertIn('vehicle_rviz', topics[image['Image Topic'] + '/compressed']['consumers'])
        schema = yaml.safe_load((ROOT / 'src/ros_architecture_pkg/config/messages/pointpainting.yaml').read_text())
        self.assertIn('rgb', schema['painted_cloud']['append_float32'])
        self.assertEqual(len(schema['display_palette_rgb']), len(schema['classes']) + 1)


if __name__ == "__main__":
    unittest.main()
