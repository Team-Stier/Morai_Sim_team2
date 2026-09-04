#!/usr/bin/env python3

import pathlib
import re
import subprocess
import unittest
import xml.etree.ElementTree as ElementTree


class Team1IntegrationContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package_root = pathlib.Path(__file__).resolve().parents[1]
        cls.repository_root = cls.package_root.parents[1]
        cls.component_launch_path = (
            cls.package_root / "launch" / "team1_controller_test.launch"
        )
        cls.component_launch_text = cls.component_launch_path.read_text(
            encoding="utf-8"
        )
        cls.component_launch = ElementTree.parse(cls.component_launch_path).getroot()
        cls.system_launch_path = (
            cls.repository_root
            / "src"
            / "system_bringup_pkg"
            / "launch"
            / "path_control_test.launch"
        )

    def test_only_adapter_and_path_tracker_are_launched(self):
        nodes = self.component_launch.findall(".//node")
        identities = {(node.get("pkg"), node.get("type")) for node in nodes}
        self.assertEqual(
            identities,
            {
                ("vehicle_control_pkg", "team1_trajectory_adapter"),
                ("morai_path_tracking", "path_tracking_controller_node"),
            },
        )
        forbidden = {
            "control_sender_node",
            "competition_vehicle_status_receiver_node",
            "molit_2026_autonomous.launch",
            "morai_bringup",
        }
        for value in forbidden:
            self.assertNotIn(value, self.component_launch_text)

    def test_vendor_controller_is_confined_to_control_test_topics(self):
        controller = next(
            node
            for node in self.component_launch.findall(".//node")
            if node.get("type") == "path_tracking_controller_node"
        )
        params = {
            parameter.get("name"): parameter.get("value")
            for parameter in controller.findall("param")
        }
        self.assertEqual(
            params["odometry_topic"], "/control_test/team1/odometry"
        )
        self.assertEqual(
            params["local_path_topic"], "/control_test/team1/local_path"
        )
        for key in (
            "vehicle_status_topic",
            "command_topic",
            "controller_status_topic",
            "lookahead_point_topic",
            "stanley_projection_point_topic",
        ):
            self.assertTrue(params[key].startswith("/control_test/team1/"), key)
        self.assertNotIn("/planning/local_path", params.values())
        adapter_config = (
            self.package_root / "config" / "team1_adapter.yaml"
        ).read_text(encoding="utf-8")
        speed_match = re.search(
            r"^controller_target_speed_mps:\s*([0-9.]+)\s*$",
            adapter_config,
            re.MULTILINE,
        )
        self.assertIsNotNone(speed_match)
        self.assertAlmostEqual(
            float(params["target_speed_kph"]),
            float(speed_match.group(1)) * 3.6,
        )
        speed_label = "{:g} km/h".format(float(params["target_speed_kph"]))
        documentation = (
            self.package_root / "docs" / "team1_temporary_integration.md"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "If any trajectory point requests less than {}".format(speed_label),
            documentation,
        )
        self.assertEqual(float(params["maximum_input_skew_sec"]), 0.0)

    def test_upstream_yaml_loads_once_into_controller_private_namespace(self):
        group = next(
            candidate
            for candidate in self.component_launch.findall("group")
            if candidate.get("ns") == "control_test/team1"
        )
        controller = next(
            node
            for node in group.findall("node")
            if node.get("type") == "path_tracking_controller_node"
        )
        direct_rosparams = group.findall("rosparam")
        self.assertEqual(len(direct_rosparams), 1)
        self.assertEqual(direct_rosparams[0].get("command"), "load")
        self.assertIsNone(controller.find("rosparam"))

        config_text = (
            self.package_root
            / "config"
            / "team1_molit_2026_path_tracking.yaml"
        ).read_text(encoding="utf-8")
        top_level_keys = re.findall(
            r"^([A-Za-z_][A-Za-z0-9_]*):\s*$", config_text, re.MULTILINE
        )
        self.assertEqual(top_level_keys, [controller.get("name")])

    def test_adapter_uses_authoritative_trajectory_and_exact_odom_stamp(self):
        source = (
            self.package_root / "scripts" / "team1_trajectory_adapter"
        ).read_text(encoding="utf-8")
        self.assertIn('TRAJECTORY_TOPIC = "/planning/trajectory"', source)
        self.assertIn(
            'CONTROL_ODOMETRY_TOPIC = "/control_test/team1/odometry"', source
        )
        self.assertNotIn('TRAJECTORY_TOPIC = "/planning/local_path"', source)
        self.assertRegex(
            source,
            re.compile(
                r"_controller_path\(trajectory, odometry\.header\.stamp\)",
                re.MULTILINE,
            ),
        )
        self.assertRegex(
            source,
            re.compile(
                r"validate_trajectory\(\s*trajectory,\s*now_sec,\s*"
                r"self\._config,\s*MAP_FRAME\s*\)",
                re.MULTILINE,
            ),
        )
        self.assertIn("self._empty_path(odometry.header.stamp)", source)
        self.assertIn("with self._output_lock:", source)

    def test_visualization_path_remains_outside_control(self):
        contract = (
            self.repository_root
            / "src"
            / "ros_architecture_pkg"
            / "config"
            / "interface_contract.yaml"
        ).read_text(encoding="utf-8")
        local_path_block = contract.split(
            "  - name: /planning/local_path\n", 1
        )[1].split("\n  - name:", 1)[0]
        self.assertIn("consumers: [runtime_evaluation_pkg, rviz]", local_path_block)
        self.assertNotIn("vehicle_control_pkg", local_path_block)
        self.assertIn("  - name: /control_test/team1/odometry\n", contract)

    def test_system_launch_composes_planning_without_editing_it(self):
        root = ElementTree.parse(self.system_launch_path).getroot()
        includes = {include.get("file") for include in root.findall("include")}
        self.assertIn(
            "$(find system_bringup_pkg)/launch/planning_stack.launch", includes
        )
        self.assertIn(
            "$(find vehicle_control_pkg)/launch/team1_controller_test.launch",
            includes,
        )
        text = self.system_launch_path.read_text(encoding="utf-8")
        self.assertNotIn("control_sender_node", text)

    def test_submodule_source_and_revision_are_auditable(self):
        modules = (self.repository_root / ".gitmodules").read_text(encoding="utf-8")
        self.assertIn("vendor/team1_mpc_controller", modules)
        self.assertIn(
            "https://github.com/Team-Stier/Morai_SIM_2026_Team1_MPC_Controller.git",
            modules,
        )
        documentation = (
            self.package_root / "docs" / "team1_temporary_integration.md"
        ).read_text(encoding="utf-8")
        self.assertIn("11c076b2e464697d86c76f968999cec58d0ffd69", documentation)
        self.assertIn("not MPC", documentation)
        checked_out_revision = subprocess.check_output(
            [
                "git",
                "-C",
                str(self.repository_root / "vendor" / "team1_mpc_controller"),
                "rev-parse",
                "HEAD",
            ],
            text=True,
        ).strip()
        self.assertEqual(
            checked_out_revision, "11c076b2e464697d86c76f968999cec58d0ffd69"
        )

    def test_filtered_snapshot_matches_upstream_except_declared_integration_patch(self):
        runtime = (
            self.repository_root / "src" / "vendor" / "team1_controller_runtime"
        )
        local_tracker = runtime / "morai_path_tracking"
        upstream_tracker = (
            self.repository_root
            / "vendor"
            / "team1_mpc_controller"
            / "src"
            / "morai_path_tracking"
        )
        frame_patch = (
            "      output.header.frame_id = "
            "config_.expected_velocity_frame_id;\n"
        )
        compared = 0
        for tree_name in ("include", "src"):
            for local_path in sorted((local_tracker / tree_name).rglob("*")):
                if not local_path.is_file():
                    continue
                relative = local_path.relative_to(local_tracker)
                upstream_path = upstream_tracker / relative
                self.assertTrue(upstream_path.is_file(), str(relative))
                local_text = local_path.read_text(encoding="utf-8")
                upstream_text = upstream_path.read_text(encoding="utf-8")
                if relative.as_posix() == "src/nodes/path_tracking_controller_node.cpp":
                    self.assertEqual(local_text.count(frame_patch), 2)
                    local_text = local_text.replace(frame_patch, "")
                    replacements = {
                        '"common_msgs_pkg/ControllerVehicleState.h"': (
                            '"morai_udp_bridge/CompetitionVehicleStatus.h"'
                        ),
                        '"common_msgs_pkg/RawActuatorCommand.h"': (
                            '"morai_udp_bridge/ActuatorCommand.h"'
                        ),
                        '"common_msgs_pkg/Team1ControllerStatus.h"': (
                            '"morai_path_tracking/ControllerStatus.h"'
                        ),
                        "common_msgs_pkg::ControllerVehicleState": (
                            "morai_udp_bridge::CompetitionVehicleStatus"
                        ),
                        "common_msgs_pkg::RawActuatorCommand": (
                            "morai_udp_bridge::ActuatorCommand"
                        ),
                        "common_msgs_pkg::Team1ControllerStatus": "ControllerStatus",
                    }
                    for current, upstream in replacements.items():
                        self.assertGreater(
                            local_text.count(current),
                            0,
                            "missing declared integration token: {}".format(current),
                        )
                        local_text = local_text.replace(current, upstream)
                self.assertEqual(local_text, upstream_text, str(relative))
                compared += 1
        self.assertEqual(compared, 17)

        self.assertEqual(
            (self.package_root / "config" / "team1_molit_2026_path_tracking.yaml").read_bytes(),
            (
                upstream_tracker
                / "config"
                / "controllers"
                / "molit_2026_path_tracking.yaml"
            ).read_bytes(),
        )
        self.assertEqual(
            (
                self.repository_root
                / "src"
                / "common_msgs_pkg"
                / "msg"
                / "RawActuatorCommand.msg"
            ).read_bytes(),
            (
                self.repository_root
                / "vendor"
                / "team1_mpc_controller"
                / "src"
                / "morai_udp_bridge"
                / "msg"
                / "ActuatorCommand.msg"
            ).read_bytes(),
        )
        self.assertEqual(
            (
                self.repository_root
                / "src"
                / "common_msgs_pkg"
                / "msg"
                / "Team1ControllerStatus.msg"
            ).read_bytes(),
            (upstream_tracker / "msg" / "ControllerStatus.msg").read_bytes(),
        )
        upstream_vehicle_state = (
            self.repository_root
            / "vendor"
            / "team1_mpc_controller"
            / "src"
            / "morai_udp_bridge"
            / "msg"
            / "CompetitionVehicleStatus.msg"
        ).read_text(encoding="utf-8")
        expected_vehicle_state = "\n".join(
            line
            for line in upstream_vehicle_state.splitlines()
            if line and line not in {"uint8 control_mode", "uint8 gear"}
        ) + "\n"
        self.assertEqual(
            (
                self.repository_root
                / "src"
                / "common_msgs_pkg"
                / "msg"
                / "ControllerVehicleState.msg"
            ).read_text(encoding="utf-8"),
            expected_vehicle_state,
        )

    def test_filtered_source_tree_contains_no_udp_runtime(self):
        runtime = (
            self.repository_root / "src" / "vendor" / "team1_controller_runtime"
        )
        packages = {
            ElementTree.parse(path).getroot().findtext("name")
            for path in runtime.rglob("package.xml")
        }
        self.assertEqual(packages, {"morai_path_tracking"})
        self.assertFalse((runtime / "morai_udp_bridge").exists())
        all_paths = {path.name for path in runtime.rglob("*") if path.is_file()}
        self.assertNotIn("control_sender_node.cpp", all_paths)
        self.assertNotIn("competition_vehicle_status_receiver_node.cpp", all_paths)
        self.assertFalse(list(runtime.rglob("*.launch")))

    def test_rviz_exposes_all_controller_geometry_topics(self):
        rviz = (
            self.repository_root
            / "src"
            / "system_bringup_pkg"
            / "rviz"
            / "planning_stack.rviz"
        ).read_text(encoding="utf-8")
        for topic in (
            "/control_test/team1/local_path",
            "/control_test/team1/lookahead_point",
            "/control_test/team1/stanley_projection_point",
        ):
            self.assertIn("Topic: {}".format(topic), rviz)


if __name__ == "__main__":
    unittest.main()
