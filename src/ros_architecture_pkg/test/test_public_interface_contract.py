#!/usr/bin/env python3

import copy
import importlib.util
import re
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
GENERATOR_PATH = PACKAGE_ROOT / "scripts" / "generate_interface_diagrams.py"
CONTRACT_PATH = PACKAGE_ROOT / "config" / "interface_contract.yaml"


def load_generator():
    spec = importlib.util.spec_from_file_location("interface_diagram_generator", GENERATOR_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PublicInterfaceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.generator = load_generator()
        cls.contract = cls.generator.load_contract(CONTRACT_PATH)

    def test_contract_forms_complete_package_boundary_graph(self):
        errors = self.generator.validate_contract(self.contract, REPOSITORY_ROOT)
        self.assertEqual(errors, [], "\n".join(errors))
        self.assertTrue(self.contract["package_boundaries"])

    def test_validator_requires_all_versioned_contract_modules(self):
        required_paths = {
            "package_registry": ("path",),
            "tf": ("frame_contract", "sensor_extrinsics"),
            "timestamp": ("timestamp_contract",),
            "morai_interface": ("udp_ros_bridge",),
        }
        for module_name, path_fields in required_paths.items():
            for path_field in path_fields:
                altered = copy.deepcopy(self.contract)
                del altered["contract_modules"][module_name][path_field]
                errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
                self.assertIn(
                    "contract_modules.{}.{} is missing".format(
                        module_name, path_field
                    ),
                    errors,
                )

            altered = copy.deepcopy(self.contract)
            altered["contract_modules"][module_name][
                "required_contract_version"
            ] = "9.9.9"
            errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
            for path_field in path_fields:
                self.assertIn(
                    "contract module {}.{} version must match "
                    "required_contract_version 9.9.9".format(
                        module_name, path_field
                    ),
                    errors,
                )

    def test_validator_aligns_disabled_collision_and_control_udp_stubs(self):
        altered = copy.deepcopy(self.contract)
        bridge_path = (
            PACKAGE_ROOT
            / "config"
            / altered["contract_modules"]["morai_interface"]["udp_ros_bridge"]
        )
        original_loader = self.generator.load_contract

        def validate_with_udp_mutation(mutate):
            udp_contract = original_loader(bridge_path)
            mutate(udp_contract)

            def load_contract(path):
                if path == bridge_path:
                    return udp_contract
                return original_loader(path)

            self.generator.load_contract = load_contract
            try:
                return self.generator.validate_contract(altered, REPOSITORY_ROOT)
            finally:
                self.generator.load_contract = original_loader

        errors = validate_with_udp_mutation(
            lambda udp: udp["channels"].pop("collision")
        )
        self.assertIn(
            "MORAI UDP contract needs a disabled collision channel stub", errors
        )

        errors = validate_with_udp_mutation(
            lambda udp: udp["channels"]["control"].__setitem__(
                "runtime_activation_allowed", True
            )
        )
        self.assertIn(
            "MORAI UDP control channel must remain runtime-disabled pending verification",
            errors,
        )

        errors = validate_with_udp_mutation(
            lambda udp: udp["channels"]["collision"].__setitem__(
                "message_type", "std_msgs/String"
            )
        )
        self.assertIn(
            "MORAI UDP collision message_type must match top-level "
            "/molit/events/collision",
            errors,
        )

        errors = validate_with_udp_mutation(
            lambda udp: udp["channels"]["collision"].__setitem__(
                "frame_id", "base_link"
            )
        )
        self.assertIn(
            "MORAI UDP collision frame_id must match top-level "
            "/molit/events/collision",
            errors,
        )

        errors = validate_with_udp_mutation(
            lambda udp: udp["channels"]["control"].__setitem__(
                "input_topic", "/molit/control/nominal_command"
            )
        )
        self.assertIn(
            "MORAI UDP control input_topic must be /molit/safety/final_command",
            errors,
        )

        errors = validate_with_udp_mutation(
            lambda udp: udp["channels"]["control"].__setitem__(
                "timestamp_source", "status_evaluation_time"
            )
        )
        self.assertIn(
            "MORAI UDP control timestamp_source must match top-level "
            "/molit/safety/final_command",
            errors,
        )

    def test_validator_locks_modular_planning_policy(self):
        mutations = []

        altered = copy.deepcopy(self.contract)
        altered["planning_policy"]["planner_class"] = "single_frame_policy"
        mutations.append(
            (
                altered,
                "planning_policy.planner_class must remain "
                "'modular_behavior_and_motion_planner'",
            )
        )

        altered = copy.deepcopy(self.contract)
        altered["planning_policy"]["owner_package"] = "camera_perception_pkg"
        mutations.append(
            (altered, "planning_policy.owner_package must remain 'path_planning_pkg'")
        )

        altered = copy.deepcopy(self.contract)
        altered["planning_policy"]["public_boundary_node"] = (
            "camera_perception_node"
        )
        mutations.append(
            (
                altered,
                "planning_policy.public_boundary_node must remain 'path_planner_node'",
            )
        )

        for input_group in ("localization", "route", "world_model"):
            altered = copy.deepcopy(self.contract)
            altered["planning_policy"]["input_groups"][input_group].pop()
            mutations.append(
                (
                    altered,
                    "planning_policy.input_groups must match the exact modular "
                    "planner inputs",
                )
            )

        altered = copy.deepcopy(self.contract)
        altered["planning_policy"]["approved_public_outputs"]["trajectory"][
            "topic"
        ] = (
            "/molit/control/nominal_command"
        )
        mutations.append(
            (
                altered,
                "planning_policy.approved_public_outputs must match trajectory "
                "and status",
            )
        )

        altered = copy.deepcopy(self.contract)
        altered["planning_policy"]["direct_actuator_output_allowed"] = True
        mutations.append(
            (
                altered,
                "planning_policy.direct_actuator_output_allowed must remain False",
            )
        )

        for altered, expected_error in mutations:
            errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
            self.assertIn(expected_error, errors, "\n".join(errors))

        altered = copy.deepcopy(self.contract)
        local_odometry = next(
            topic
            for topic in altered["topics"]
            if topic["name"] == "/molit/localization/local/odometry"
        )
        local_odometry["data_type"] = "geometry_msgs/PoseStamped"
        altered["messages"].append(
            {
                "type": "geometry_msgs/PoseStamped",
                "status": "approved_standard_type",
            }
        )
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "planning_policy input /molit/localization/local/odometry "
            "type must remain nav_msgs/Odometry",
            errors,
        )

    def test_validator_locks_runtime_readiness_policy(self):
        altered = copy.deepcopy(self.contract)
        del altered["runtime_readiness_policy"]
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn("runtime_readiness_policy must be a mapping", errors)

        altered = copy.deepcopy(self.contract)
        altered["runtime_readiness_policy"]["status"] = "runtime_verified"
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "runtime_readiness_policy.status must remain "
            "'semantics_approved_required_channel_sets_pending_runtime_validation'",
            errors,
        )

        for required_rule in self.generator.EXPECTED_RUNTIME_READINESS_RULES:
            altered = copy.deepcopy(self.contract)
            altered["runtime_readiness_policy"]["rules"].remove(required_rule)
            errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
            self.assertIn(
                "runtime_readiness_policy must include required v1 rule: {}".format(
                    required_rule
                ),
                errors,
            )

        for field, expected_value in self.generator.EXPECTED_GPS_BLACKOUT_POLICY.items():
            altered = copy.deepcopy(self.contract)
            altered["runtime_readiness_policy"]["gps_blackout"][field] = (
                not expected_value
            )
            errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
            self.assertIn(
                "runtime_readiness_policy.gps_blackout.{} must remain {!r}".format(
                    field, expected_value
                ),
                errors,
            )

    def test_planning_policy_matches_modular_topic_graph_and_package_boundary(self):
        expected_groups = {
            "localization": [
                "/molit/localization/local/odometry",
                "/molit/localization/ego_state",
                "/molit/localization/status",
            ],
            "route": [
                "/molit/route/context",
                "/molit/route/status",
            ],
            "world_model": [
                "/molit/world_model/scene",
                "/molit/world_model/status",
            ],
        }
        expected_inputs = [
            topic_name
            for group_name in ("localization", "route", "world_model")
            for topic_name in expected_groups[group_name]
        ]
        expected_outputs = [
            "/molit/planning/trajectory",
            "/molit/planning/status",
        ]
        policy = self.contract["planning_policy"]
        boundary = self.contract["package_boundaries"]["path_planning_pkg"]
        self.assertEqual(policy["input_groups"], expected_groups)
        self.assertEqual(boundary["inputs"], expected_inputs)
        self.assertEqual(boundary["outputs"], expected_outputs)
        self.assertEqual(
            policy["approved_public_outputs"],
            {
                "trajectory": {
                    "topic": "/molit/planning/trajectory",
                    "frame": "odom",
                },
                "status": {
                    "topic": "/molit/planning/status",
                    "frame": "not_applicable",
                },
            },
        )
        self.assertFalse(policy["direct_actuator_output_allowed"])

        planner_inputs = sorted(
            topic["name"]
            for topic in self.contract["topics"]
            if "path_planner_node" in topic["consumers"]
        )
        planner_outputs = sorted(
            topic["name"]
            for topic in self.contract["topics"]
            if "path_planner_node" in topic["producers"]
        )
        self.assertEqual(planner_inputs, sorted(expected_inputs))
        self.assertEqual(planner_outputs, sorted(expected_outputs))
        self.assertFalse(
            any(topic_name.startswith("/molit/control/") for topic_name in planner_outputs)
        )

        altered = copy.deepcopy(self.contract)
        topic = next(
            item
            for item in altered["topics"]
            if item["name"] == "/molit/localization/local/odometry"
        )
        topic["consumers"].remove("path_planner_node")
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "planning_policy inputs must exactly match path_planner_node topic consumers",
            errors,
        )

        altered = copy.deepcopy(self.contract)
        altered["package_boundaries"]["path_planning_pkg"]["inputs"].remove(
            "/molit/route/context"
        )
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "planning_policy inputs must exactly match path_planning_pkg boundary inputs",
            errors,
        )

        altered = copy.deepcopy(self.contract)
        altered["package_boundaries"]["path_planning_pkg"]["outputs"].reverse()
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "path_planning_pkg outputs must remain trajectory then planning status",
            errors,
        )

    def test_required_interface_fields_cannot_be_self_disabled(self):
        altered = copy.deepcopy(self.contract)
        altered["required_interface_fields"].remove("frame")
        del altered["topics"][0]["frame"]
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "required_interface_fields must match the complete v1 field set", errors
        )

    def test_validator_rejects_invalid_korean_diagram_copy(self):
        def public_node_target(contract):
            node = next(
                item
                for item in contract["nodes"]
                if item["visibility"] == "public_boundary"
            )
            return (
                node,
                "diagram_description_ko",
                "node {}.diagram_description_ko".format(node["name"]),
            )

        def public_topic_target(contract):
            topic = next(
                item
                for item in contract["topics"]
                if not self.generator._is_internal_topic(item)
            )
            return (
                topic,
                "diagram_description_ko",
                "topic {}.diagram_description_ko".format(topic["name"]),
            )

        def package_boundary_target(contract):
            package_name = next(iter(contract["package_boundaries"]))
            boundary = contract["package_boundaries"][package_name]
            return (
                boundary,
                "diagram_summary_ko",
                "package_boundaries.{}.diagram_summary_ko".format(package_name),
            )

        invalid_values = (
            ("missing", None, "must be a non-empty string"),
            ("empty", "", "must be a non-empty string"),
            ("english_only", "English only", "must include Korean text"),
            ("multiline", "첫 줄\n둘째 줄", "must be a single line"),
            ("too_long", "가" * 61, "must be at most 60 characters"),
        )
        targets = (
            ("public_node", public_node_target),
            ("public_topic", public_topic_target),
            ("package_boundary", package_boundary_target),
        )
        for target_name, target_getter in targets:
            for invalid_name, invalid_value, expected_suffix in invalid_values:
                with self.subTest(target=target_name, invalid=invalid_name):
                    altered = copy.deepcopy(self.contract)
                    target, field, field_path = target_getter(altered)
                    if invalid_name == "missing":
                        del target[field]
                    else:
                        target[field] = invalid_value
                    errors = self.generator.validate_contract(
                        altered, REPOSITORY_ROOT
                    )
                    self.assertIn(
                        "{} {}".format(field_path, expected_suffix),
                        errors,
                        "\n".join(errors),
                    )

        for package_name, boundary in self.contract["package_boundaries"].items():
            with self.subTest(package=package_name):
                self.assertTrue(boundary["diagram_summary_ko"])

    def test_generated_mermaid_is_up_to_date(self):
        documents = self.generator.build_documents(self.contract, REPOSITORY_ROOT)
        for path, expected in documents.items():
            self.assertTrue(path.is_file(), str(path.relative_to(REPOSITORY_ROOT)))
            self.assertEqual(
                path.read_text(encoding="utf-8"),
                expected,
                str(path.relative_to(REPOSITORY_ROOT)),
            )

    def test_package_interface_mermaid_projects_korean_details_only_locally(self):
        documents = self.generator.build_documents(self.contract, REPOSITORY_ROOT)
        node_by_name = {node["name"]: node for node in self.contract["nodes"]}
        topic_by_name = {
            topic["name"]: topic for topic in self.contract["topics"]
        }
        system_paths = {
            PACKAGE_ROOT / "docs" / "system_architecture.mmd",
        }
        system_paths.update(
            PACKAGE_ROOT
            / "docs"
            / "{}.mmd".format(view["output_basename"])
            for view in self.contract["architecture_views"].values()
        )
        system_contents = "\n".join(documents[path] for path in system_paths)

        for package_name, boundary in self.contract["package_boundaries"].items():
            package_path = (
                REPOSITORY_ROOT
                / "src"
                / package_name
                / "docs"
                / "interface_io.mmd"
            )
            contents = documents[package_path]
            with self.subTest(package=package_name):
                self.assertIn("입력 (구독)", contents)
                self.assertIn("패키지 처리", contents)
                self.assertIn("출력 (발행)", contents)

                summary = self.generator._escape_label(
                    boundary["diagram_summary_ko"]
                )
                self.assertIn(summary, contents)
                self.assertNotIn(summary, system_contents)

                for node_name in boundary["public_nodes"]:
                    role = "역할: {}".format(
                        self.generator._escape_label(
                            node_by_name[node_name]["diagram_description_ko"]
                        )
                    )
                    self.assertIn(role, contents)
                    self.assertNotIn(role, system_contents)

                for topic_name in boundary["inputs"] + boundary["outputs"]:
                    description = "설명: {}".format(
                        self.generator._escape_label(
                            topic_by_name[topic_name]["diagram_description_ko"]
                        )
                    )
                    self.assertIn(description, contents)
                    self.assertNotIn(description, system_contents)

    def test_public_namespace_policy_is_enforced(self):
        altered = copy.deepcopy(self.contract)
        altered["topics"][0]["name"] = "/wrong_namespace/camera"
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertTrue(
            any("must use namespace /molit/" in error for error in errors),
            "\n".join(errors),
        )
        altered = copy.deepcopy(self.contract)
        altered["public_interface_policy"]["topic_namespace"] = "/wrong"
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertTrue(
            any("must remain /molit" in error for error in errors),
            "\n".join(errors),
        )

        for policy_name, expected_value in self.generator.EXPECTED_PUBLIC_INTERFACE_POLICY.items():
            altered = copy.deepcopy(self.contract)
            altered["public_interface_policy"][policy_name] = (
                not expected_value
                if isinstance(expected_value, bool)
                else "/changed"
            )
            errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
            self.assertTrue(
                any("policy {} must remain".format(policy_name) in error for error in errors),
                "{}:\n{}".format(policy_name, "\n".join(errors)),
            )

    def test_noncanonical_or_cross_package_internal_names_are_rejected(self):
        mutations = []

        altered = copy.deepcopy(self.contract)
        altered["topics"][0]["name"] = "/molit//bad"
        mutations.append((altered, "is not a canonical absolute ROS name"))

        altered = copy.deepcopy(self.contract)
        altered["topics"][0]["name"] = "/molit/internal/bad/public"
        mutations.append((altered, "public topic /molit/internal/bad/public cannot"))

        altered = copy.deepcopy(self.contract)
        altered["topics"][0]["name"] = "/molit/foo/internal/public"
        mutations.append((altered, "public topic /molit/foo/internal/public cannot"))

        altered = copy.deepcopy(self.contract)
        altered["topics"][0]["name"] = "/molit/topic with space"
        mutations.append((altered, "is not a canonical absolute ROS name"))

        altered = copy.deepcopy(self.contract)
        internal_topic = next(
            topic for topic in altered["topics"] if topic["visibility"] == "package_internal"
        )
        internal_topic["name"] = "/molit/internal/wrong/packets"
        mutations.append((altered, "must use namespace /molit/internal/morai_interface/"))

        altered = copy.deepcopy(self.contract)
        internal_topic = next(
            topic for topic in altered["topics"] if topic["visibility"] == "package_internal"
        )
        internal_topic["consumers"].append("localization_node")
        mutations.append((altered, "crosses package boundary through localization_node"))

        altered = copy.deepcopy(self.contract)
        altered["nodes"][0]["name"] = "/remapped_node"
        mutations.append((altered, "must be an exact ROS basename"))

        for altered, expected_error in mutations:
            errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
            self.assertTrue(
                any(expected_error in error for error in errors),
                "expected {!r} in:\n{}".format(expected_error, "\n".join(errors)),
            )

    def test_topic_frames_are_exact_v1_contracts(self):
        topics = {topic["name"]: topic for topic in self.contract["topics"]}
        self.assertEqual(
            set(topics), set(self.generator.EXPECTED_TOPIC_FRAME_CONTRACT)
        )
        for topic_name, expected_frames in (
            self.generator.EXPECTED_TOPIC_FRAME_CONTRACT.items()
        ):
            actual_frames = {
                field: topics[topic_name][field]
                for field in ("frame", "child_frame", "motion_frame")
                if field in topics[topic_name]
            }
            self.assertEqual(actual_frames, expected_frames, topic_name)

        for topic_name, wrong_frame in (
            ("/molit/world_model/scene", "odom"),
            ("/molit/planning/trajectory", "map"),
            ("/molit/route/context", "odom"),
            ("/molit/safety/final_command", "map"),
            ("/molit/planning/trajectory", "pending_competition_packet_spec"),
            ("/molit/world_model/scene", "not_applicable"),
        ):
            altered = copy.deepcopy(self.contract)
            topic = next(
                item for item in altered["topics"] if item["name"] == topic_name
            )
            topic["frame"] = wrong_frame
            errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
            self.assertTrue(
                any("frame contract must remain" in error for error in errors),
                "{} -> {}:\n{}".format(topic_name, wrong_frame, "\n".join(errors)),
            )

    def test_timestamp_registry_rejects_unknown_or_misscoped_sources(self):
        altered = copy.deepcopy(self.contract)
        altered["topics"][0]["timestamp_source"] = "unknown_clock"
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertTrue(
            any("unregistered timestamp source unknown_clock" in error for error in errors),
            "\n".join(errors),
        )

        altered = copy.deepcopy(self.contract)
        altered["topics"][3]["timestamp_source"] = "morai_camera_packet_frame_time"
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertTrue(
            any("is not allowed on topic /molit/sensors/gps/fix" in error for error in errors),
            "\n".join(errors),
        )

        altered = copy.deepcopy(self.contract)
        altered["topics"][0]["timestamp_source"] = "steady_watchdog_time"
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertTrue(
            any("steady_watchdog_time is not allowed" in error for error in errors),
            "\n".join(errors),
        )

        altered = copy.deepcopy(self.contract)
        nominal = next(
            topic
            for topic in altered["topics"]
            if topic["name"] == "/molit/control/nominal_command"
        )
        nominal["timestamp_source"] = "fusion_reference_time"
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertTrue(
            any("fusion_reference_time is not allowed" in error for error in errors),
            "\n".join(errors),
        )

    def test_timestamp_sources_must_retain_exact_topic_scopes(self):
        timestamp_path = (
            PACKAGE_ROOT / "config" / "timestamp" / "timestamp_contract.yaml"
        )
        timestamp_contract = self.generator.load_contract(timestamp_path)
        for source_name, source_policy in timestamp_contract[
            "timestamp_source_registry"
        ].items():
            self.assertTrue(source_policy.get("allowed_topics"), source_name)

    def test_final_command_path_has_one_safety_gate(self):
        topics = {topic["name"]: topic for topic in self.contract["topics"]}
        final_command = topics["/molit/safety/final_command"]
        nominal_command = topics["/molit/control/nominal_command"]

        self.assertEqual(final_command["producers"], ["safety_supervisor_node"])
        self.assertIn("morai_control_sender", final_command["consumers"])
        self.assertNotIn("morai_control_sender", nominal_command["consumers"])
        self.assertEqual(
            self.contract["package_boundaries"]["morai_interface_pkg"]["inputs"],
            ["/molit/safety/final_command"],
        )
        self.assertEqual(
            final_command["consumer_watchdog_owner"], "morai_control_sender"
        )
        self.assertEqual(
            final_command["consumer_timeout_policy"],
            self.generator.EXPECTED_FINAL_COMMAND_TIMEOUT_POLICY,
        )

    def test_validator_rejects_direct_nominal_sender_bypass(self):
        altered = copy.deepcopy(self.contract)
        nominal_command = next(
            topic
            for topic in altered["topics"]
            if topic["name"] == "/molit/control/nominal_command"
        )
        nominal_command["consumers"].append("morai_control_sender")
        altered["package_boundaries"]["morai_interface_pkg"]["inputs"].append(
            "/molit/control/nominal_command"
        )

        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "morai_control_sender public inputs must be exactly "
            "/molit/safety/final_command",
            errors,
        )
        self.assertIn(
            "morai_interface_pkg public inputs must be exactly "
            "/molit/safety/final_command",
            errors,
        )

    def test_validator_enforces_final_command_consumer_watchdog(self):
        altered = copy.deepcopy(self.contract)
        final_command = next(
            topic
            for topic in altered["topics"]
            if topic["name"] == "/molit/safety/final_command"
        )
        del final_command["consumer_watchdog_timeout_sec"]
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn("final command is missing consumer_watchdog_timeout_sec", errors)

        altered = copy.deepcopy(self.contract)
        del altered["external_transport"]["control_constraints"][
            "final_command_consumer_gate"
        ]
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "external control constraints need final_command_consumer_gate", errors
        )

        altered = copy.deepcopy(self.contract)
        checks = altered["external_transport"]["control_constraints"][
            "final_command_consumer_gate"
        ]["required_checks"]
        del checks[0]
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "external final command consumer gate required_checks must match the v1 safety set",
            errors,
        )

        altered = copy.deepcopy(self.contract)
        sender = next(
            node for node in altered["nodes"] if node["name"] == "morai_control_sender"
        )
        sender["status"] = "runtime_verified"
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertTrue(
            any("live morai_control_sender needs" in error for error in errors),
            "\n".join(errors),
        )

        altered = copy.deepcopy(self.contract)
        altered["external_transport"]["control_constraints"][
            "final_command_consumer_gate"
        ]["enable_status"] = "runtime_enabled"
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "non-live morai_control_sender gate must remain prohibited pending verification",
            errors,
        )

        altered = copy.deepcopy(self.contract)
        final_command = next(
            topic
            for topic in altered["topics"]
            if topic["name"] == "/molit/safety/final_command"
        )
        final_command["consumer_watchdog_timeout_sec"] = -1.0
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "final command consumer watchdog timeout must be positive when numeric",
            errors,
        )

    def test_external_transport_direction_and_runtime_status_are_guarded(self):
        altered = copy.deepcopy(self.contract)
        altered["external_transport"]["diagram_bindings"]["egress"][
            "adapter_nodes"
        ].append("morai_camera_front")
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "external egress adapters must exactly match node transport roles", errors
        )

        altered = copy.deepcopy(self.contract)
        topic = next(
            topic
            for topic in altered["topics"]
            if topic["name"] == "/molit/world_model/scene"
        )
        topic["status"] = "runtime_verified"
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertTrue(
            any("live topic /molit/world_model/scene has non-live producer" in error for error in errors),
            "\n".join(errors),
        )

        altered = copy.deepcopy(self.contract)
        altered["topics"][0]["visibility"] = "publci"
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertTrue(
            any("has invalid visibility publci" in error for error in errors),
            "\n".join(errors),
        )

    def test_system_readiness_is_upstream_of_the_final_safety_state(self):
        topics = {topic["name"]: topic for topic in self.contract["topics"]}
        readiness = topics["/molit/system/readiness"]
        safety_state = topics["/molit/safety/state"]
        self.assertEqual(
            readiness["scope"],
            "upstream_dependencies_only_safety_state_is_final_driving_authority",
        )
        self.assertIn("safety_supervisor_node", readiness["consumers"])
        self.assertNotIn("system_readiness_node", safety_state["consumers"])

    def test_external_transport_is_visible_in_system_diagram(self):
        system_mmd = PACKAGE_ROOT / "docs" / "system_architecture.mmd"
        contents = system_mmd.read_text(encoding="utf-8")
        bindings = self.contract["external_transport"]["diagram_bindings"]
        self.assertIn("External MORAI transport boundary", contents)
        for direction in ("ingress", "egress"):
            self.assertIn(bindings[direction]["label"], contents)
            for node_name in bindings[direction]["adapter_nodes"]:
                self.assertIn(node_name, contents)

    def test_curated_architecture_views_are_exact_contract_projections(self):
        documents = self.generator.build_documents(self.contract, REPOSITORY_ROOT)
        topic_by_name = {
            topic["name"]: topic for topic in self.contract["topics"]
        }
        for view_name, view in self.contract["architecture_views"].items():
            source = (
                PACKAGE_ROOT
                / "docs"
                / "{}.mmd".format(view["output_basename"])
            )
            self.assertIn(source, documents)
            contents = documents[source]
            self.assertIn("flowchart TB", contents)
            projected_nodes = re.findall(
                r'view_node_\d+\["node: ([^<]+)<br/>package:', contents
            )
            projected_topics = re.findall(
                r'view_topic_\d+\["([^<]+)<br/>type:', contents
            )
            self.assertEqual(projected_nodes, view["nodes"], view_name)
            self.assertEqual(projected_topics, view["topics"], view_name)
            for topic_name in view["topics"]:
                self.assertIn(
                    "type: {}".format(topic_by_name[topic_name]["data_type"]),
                    contents,
                )
            for direction, channels in view["external_channels"].items():
                self.assertIn(
                    "External MORAI {} boundary".format(direction), contents
                )
                self.assertIn(", ".join(channels), contents)

    def test_curated_architecture_view_schema_is_guarded(self):
        altered = copy.deepcopy(self.contract)
        altered["architecture_views"]["nominal_data_control"]["topics"].append(
            "/molit/unknown"
        )
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "architecture view nominal_data_control references unknown topic /molit/unknown",
            errors,
        )

        altered = copy.deepcopy(self.contract)
        altered["architecture_views"]["nominal_data_control"][
            "output_basename"
        ] = "renamed"
        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "architecture view nominal_data_control output_basename must remain system_nominal_flow",
            errors,
        )

    def test_runtime_evaluation_is_read_only_from_driving_graph(self):
        evaluator = "runtime_evaluator_node"
        produced = [
            topic
            for topic in self.contract["topics"]
            if evaluator in topic["producers"]
        ]
        self.assertEqual(
            [topic["name"] for topic in produced],
            ["/molit/evaluation/metrics"],
        )
        self.assertEqual(produced[0]["consumers"], [])

    def test_validator_rejects_runtime_evaluation_feedback(self):
        altered = copy.deepcopy(self.contract)
        metrics = next(
            topic
            for topic in altered["topics"]
            if topic["name"] == "/molit/evaluation/metrics"
        )
        metrics["consumers"].append("path_planner_node")
        altered["package_boundaries"]["path_planning_pkg"]["inputs"].append(
            "/molit/evaluation/metrics"
        )

        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "runtime evaluation metrics must not feed any ROS node",
            errors,
        )

    def test_validator_rejects_runtime_evaluator_as_command_producer(self):
        altered = copy.deepcopy(self.contract)
        nominal_name = "/molit/control/nominal_command"
        nominal = next(
            topic for topic in altered["topics"] if topic["name"] == nominal_name
        )
        nominal["owner_package"] = "runtime_evaluation_pkg"
        nominal["producers"] = ["runtime_evaluator_node"]
        nominal["consumers"].remove("runtime_evaluator_node")
        altered["package_boundaries"]["vehicle_control_pkg"]["outputs"].remove(
            nominal_name
        )
        altered["package_boundaries"]["runtime_evaluation_pkg"]["inputs"].remove(
            nominal_name
        )
        altered["package_boundaries"]["runtime_evaluation_pkg"]["outputs"].append(
            nominal_name
        )

        errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
        self.assertIn(
            "runtime_evaluator_node outputs must be exactly "
            "/molit/evaluation/metrics",
            errors,
        )

    def test_validator_rejects_unapproved_services_and_actions(self):
        for interface_kind in ("services", "actions"):
            altered = copy.deepcopy(self.contract)
            altered[interface_kind] = [
                {
                    "name": "/molit/unapproved_rpc",
                    "owner_package": "path_planning_pkg",
                }
            ]
            errors = self.generator.validate_contract(altered, REPOSITORY_ROOT)
            self.assertIn(
                "{} must remain an empty list until a v1 interface is approved".format(
                    interface_kind
                ),
                errors,
            )

    def test_rendered_interface_images_exist_and_match_contract_labels(self):
        self.assertEqual(
            self.generator._check_render_manifest(
                self.generator.build_documents(self.contract, REPOSITORY_ROOT),
                REPOSITORY_ROOT,
            ),
            [],
        )
        system_svg = PACKAGE_ROOT / "docs" / "system_architecture.svg"
        system_png = PACKAGE_ROOT / "docs" / "system_architecture.png"
        self.assertIn("<svg", system_svg.read_text(encoding="utf-8"))
        self.assertTrue(system_png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))

        system_contents = system_svg.read_text(encoding="utf-8")
        for node in self.contract["nodes"]:
            self.assertIn(node["name"], system_contents)
        for topic in self.contract["topics"]:
            self.assertIn(topic["name"], system_contents)

        for view in self.contract["architecture_views"].values():
            basename = view["output_basename"]
            svg = PACKAGE_ROOT / "docs" / "{}.svg".format(basename)
            png = PACKAGE_ROOT / "docs" / "{}.png".format(basename)
            svg_contents = svg.read_text(encoding="utf-8")
            self.assertIn("<svg", svg_contents)
            self.assertTrue(png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))
            for node_name in view["nodes"]:
                self.assertIn(node_name, svg_contents)
            for topic_name in view["topics"]:
                self.assertIn(topic_name, svg_contents)

        for package_name, boundary in self.contract["package_boundaries"].items():
            docs = REPOSITORY_ROOT / "src" / package_name / "docs"
            svg = docs / "interface_io.svg"
            png = docs / "interface_io.png"
            svg_contents = svg.read_text(encoding="utf-8")
            self.assertIn("<svg", svg_contents)
            self.assertTrue(png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))
            for node_name in boundary["public_nodes"]:
                self.assertIn(node_name, svg_contents)
            for topic_name in boundary["inputs"] + boundary["outputs"]:
                self.assertIn(topic_name, svg_contents)

    def test_package_readmes_embed_generated_interface_image(self):
        topic_by_name = {
            topic["name"]: topic for topic in self.contract["topics"]
        }
        for package_name in self.contract["package_boundaries"]:
            boundary = self.contract["package_boundaries"][package_name]
            readme = REPOSITORY_ROOT / "src" / package_name / "README.md"
            self.assertTrue(readme.is_file(), str(readme))
            contents = readme.read_text(encoding="utf-8")
            self.assertIn(
                "docs/interface_io.svg",
                contents,
                "{} must embed or link its generated interface image".format(
                    readme.relative_to(REPOSITORY_ROOT)
                ),
            )
            image = readme.parent / "docs" / "interface_io.svg"
            self.assertTrue(
                image.is_file(),
                "missing rendered interface image: {}".format(
                    image.relative_to(REPOSITORY_ROOT)
                ),
            )
            node_projection = re.search(
                r"^\*\*공개 node \(exact\):\*\* (.+)$", contents, re.MULTILINE
            )
            self.assertIsNotNone(
                node_projection,
                "{} needs the exact public node projection".format(
                    readme.relative_to(REPOSITORY_ROOT)
                ),
            )
            projected_nodes = re.findall(r"`([^`]+)`", node_projection.group(1))
            self.assertEqual(projected_nodes, boundary["public_nodes"])
            if not boundary["public_nodes"]:
                self.assertEqual(node_projection.group(1), "없음")
            actual_rows = []
            for line in contents.splitlines():
                columns = [column.strip() for column in line.strip().split("|")]
                if len(columns) == 5 and columns[1] in ("입력", "출력"):
                    actual_rows.append((columns[1], columns[2], columns[3]))
            expected_rows = []
            for direction, topic_names in (
                ("입력", boundary["inputs"]),
                ("출력", boundary["outputs"]),
            ):
                for topic_name in topic_names:
                    expected_rows.append(
                        (
                            direction,
                            "`{}`".format(topic_name),
                            "`{}`".format(topic_by_name[topic_name]["data_type"]),
                        )
                    )
            self.assertEqual(
                actual_rows,
                expected_rows,
                "{} public I/O table must exactly project package_boundaries".format(
                    readme.relative_to(REPOSITORY_ROOT)
                ),
            )


if __name__ == "__main__":
    unittest.main()
