#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ElementTree


class RvizBringupContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package_root = pathlib.Path(__file__).resolve().parents[1]
        cls.planning_launch = ElementTree.parse(
            cls.package_root / "launch" / "planning_stack.launch"
        ).getroot()
        cls.path_control_launch = ElementTree.parse(
            cls.package_root / "launch" / "path_control_test.launch"
        ).getroot()
        cls.rviz_text = (
            cls.package_root / "rviz" / "planning_stack.rviz"
        ).read_text(encoding="utf-8")

    @staticmethod
    def _arguments(root):
        return {argument.get("name"): argument for argument in root.findall("arg")}

    def test_path_control_entry_point_enables_and_forwards_rviz_by_default(self):
        arguments = self._arguments(self.path_control_launch)
        self.assertEqual(arguments["use_rviz"].get("default"), "true")

        planning_include = next(
            include
            for include in self.path_control_launch.findall("include")
            if include.get("file")
            == "$(find system_bringup_pkg)/launch/planning_stack.launch"
        )
        forwarded = {
            argument.get("name"): argument.get("value")
            for argument in planning_include.findall("arg")
        }
        self.assertEqual(forwarded["use_rviz"], "$(arg use_rviz)")
        self.assertEqual(forwarded["rviz_config"], "$(arg rviz_config)")

    def test_planning_stack_starts_the_expected_rviz_profile_conditionally(self):
        arguments = self._arguments(self.planning_launch)
        self.assertEqual(arguments["use_rviz"].get("default"), "true")

        rviz_nodes = [
            node
            for node in self.planning_launch.findall("node")
            if node.get("pkg") == "rviz" and node.get("type") == "rviz"
        ]
        self.assertEqual(len(rviz_nodes), 1)
        node = rviz_nodes[0]
        self.assertEqual(node.get("if"), "$(arg use_rviz)")
        self.assertEqual(node.get("args"), "-d $(arg rviz_config)")
        self.assertEqual(node.get("required"), "false")

    def test_rviz_profile_contains_route_plan_vehicle_and_hard_wall_topics(self):
        for topic in (
            "/hd_map/markers",
            "/planning/global_path",
            "/planning/local_path",
            "/planning/ego_trace",
            "/planning/vehicle_marker",
            "/planning/corridor_markers",
        ):
            self.assertIn("Topic: {}".format(topic), self.rviz_text)


if __name__ == "__main__":
    unittest.main()
