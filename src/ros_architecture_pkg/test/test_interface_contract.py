#!/usr/bin/env python3

"""Dependency-free structural checks for the central ROS interface contract.

The approved contract intentionally uses a small, regular YAML subset.  This
test reads only the indentation and scalar keys needed for governance checks, so
running the architecture package tests does not depend on PyYAML being available.
"""

import ast
import pathlib
import re
import unittest


TOP_LEVEL_PATTERN = re.compile(r"^([A-Za-z][A-Za-z0-9_]*):(.*)$")
FIELD_PATTERN = re.compile(r"^    ([A-Za-z][A-Za-z0-9_]*):\s*(.*)$")
NAME_ITEM_PATTERN = re.compile(r"^  - name:\s*(\S+)\s*$")
SCALAR_ITEM_PATTERN = re.compile(r"^  -\s+(\S+)\s*$")
SEMVER_PATTERN = re.compile(r"^[0-9]+\.[0-9]+\.[0-9]+$")

MANDATORY_TOPIC_FIELDS = {
    "owner_package",
    "producers",
    "consumers",
    "data_meaning",
    "data_type",
    "unit",
    "frame",
    "timestamp_source",
    "expected_rate_hz",
    "queue_policy",
    "timeout_sec",
    "invalid_data_policy",
}


class ContractParseError(ValueError):
    pass


class ContractText(object):
    """Parse the top level and regular named-record sections of the contract."""

    def __init__(self, text):
        self.sections = {}
        current = None
        for line_number, raw_line in enumerate(text.splitlines(), 1):
            if not raw_line.strip() or raw_line.lstrip().startswith("#"):
                continue
            if not raw_line.startswith(" "):
                match = TOP_LEVEL_PATTERN.match(raw_line)
                if match is None:
                    raise ContractParseError(
                        "line {} is not a top-level key".format(line_number))
                name = match.group(1)
                if name in self.sections:
                    raise ContractParseError(
                        "duplicate top-level key {!r}".format(name))
                current = {
                    "value": match.group(2).strip(),
                    "lines": [],
                    "line_number": line_number,
                }
                self.sections[name] = current
            elif current is None:
                raise ContractParseError(
                    "line {} appears before a top-level key".format(line_number))
            else:
                current["lines"].append((line_number, raw_line))

    def scalar(self, section_name):
        section = self.sections[section_name]
        if section["lines"]:
            raise ContractParseError(
                "section {!r} is not a scalar".format(section_name))
        if not section["value"]:
            raise ContractParseError(
                "section {!r} has an empty scalar".format(section_name))
        return section["value"]

    def scalar_items(self, section_name):
        section = self.sections[section_name]
        if section["value"]:
            raise ContractParseError(
                "section {!r} is not a block list".format(section_name))
        values = []
        for line_number, line in section["lines"]:
            match = SCALAR_ITEM_PATTERN.match(line)
            if match is None:
                raise ContractParseError(
                    "unexpected line {} in scalar list {!r}".format(
                        line_number, section_name))
            values.append(match.group(1))
        return values

    def records(self, section_name):
        section = self.sections[section_name]
        if section["value"] == "[]":
            if section["lines"]:
                raise ContractParseError(
                    "inline empty section {!r} has children".format(section_name))
            return []
        if section["value"]:
            raise ContractParseError(
                "named section {!r} must be a block list".format(section_name))
        records = []
        current = None
        for line_number, line in section["lines"]:
            name_match = NAME_ITEM_PATTERN.match(line)
            if name_match is not None:
                current = {"name": name_match.group(1)}
                records.append(current)
                continue
            field_match = FIELD_PATTERN.match(line)
            if field_match is None or current is None:
                raise ContractParseError(
                    "unexpected line {} in named section {!r}".format(
                        line_number, section_name))
            key, value = field_match.groups()
            if key in current:
                raise ContractParseError(
                    "duplicate field {!r} in {} {!r}".format(
                        key, section_name, current["name"]))
            if not value:
                raise ContractParseError(
                    "empty field {!r} in {} {!r}".format(
                        key, section_name, current["name"]))
            current[key] = value
        return records


def _inline_list(value):
    if not value.startswith("[") or not value.endswith("]"):
        raise ContractParseError("expected inline list, got {!r}".format(value))
    return tuple(
        item.strip() for item in value[1:-1].split(",") if item.strip())


class InterfaceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        package_root = pathlib.Path(__file__).resolve().parents[1]
        cls.contract_path = package_root / "config" / "interface_contract.yaml"
        cls.contract = ContractText(
            cls.contract_path.read_text(encoding="utf-8"))

    def test_required_top_level_schema_and_approved_version(self):
        expected_sections = {
            "schema_version",
            "contract_version",
            "status",
            "authority",
            "nodes",
            "topics",
            "messages",
            "services",
            "actions",
            "frames",
            "parameters",
            "required_interface_fields",
            "external_transport",
        }
        self.assertTrue(expected_sections.issubset(self.contract.sections))
        self.assertEqual(self.contract.scalar("schema_version"), "1")
        self.assertRegex(
            self.contract.scalar("contract_version"), SEMVER_PATTERN)
        self.assertTrue(self.contract.scalar("status").startswith("approved_"))

    def test_public_names_are_nonempty_absolute_and_unique_by_kind(self):
        for section_name in ("nodes", "topics", "parameters"):
            records = self.contract.records(section_name)
            names = [record["name"] for record in records]
            self.assertTrue(names, section_name)
            self.assertEqual(len(names), len(set(names)), section_name)
            self.assertTrue(
                all(name.startswith("/") for name in names), section_name)

        for section_name in ("messages", "frames"):
            records = self.contract.records(section_name)
            names = [record["name"] for record in records]
            self.assertTrue(names, section_name)
            self.assertEqual(len(names), len(set(names)), section_name)

        self.assertEqual(self.contract.records("services"), [])
        self.assertEqual(self.contract.records("actions"), [])

    def test_every_topic_has_the_declared_contract_fields(self):
        declared_items = self.contract.scalar_items("required_interface_fields")
        self.assertEqual(len(declared_items), len(set(declared_items)))
        declared = set(declared_items)
        self.assertTrue(MANDATORY_TOPIC_FIELDS.issubset(declared))
        for topic in self.contract.records("topics"):
            self.assertFalse(
                declared.difference(topic),
                "{} missing {}".format(
                    topic["name"], sorted(declared.difference(topic))),
            )
            producers = _inline_list(topic["producers"])
            consumers = _inline_list(topic["consumers"])
            self.assertTrue(producers, topic["name"])
            self.assertTrue(consumers, topic["name"])
            self.assertIn(topic["owner_package"], producers, topic["name"])
            self.assertGreaterEqual(float(topic["expected_rate_hz"]), 0.0)
            self.assertGreaterEqual(float(topic["timeout_sec"]), 0.0)

    def test_custom_topic_types_and_frames_resolve(self):
        message_names = {
            record["name"] for record in self.contract.records("messages")}
        frame_names = {
            record["name"] for record in self.contract.records("frames")}
        for topic in self.contract.records("topics"):
            data_type = topic["data_type"]
            if data_type.startswith("common_msgs_pkg/"):
                self.assertIn(data_type, message_names, topic["name"])
            for frame in (
                    value.strip() for value in topic["frame"].split("->")):
                self.assertIn(frame, frame_names, topic["name"])

    def test_route_context_timestamp_distinguishes_valid_and_invalid_output(self):
        route_context = next(
            topic for topic in self.contract.records("topics")
            if topic["name"] == "/planning/route_context")
        self.assertEqual(
            route_context["timestamp_source"],
            "timestamp of the accepted odometry sample when valid=true; "
            "publication time in ROS time when valid=false",
        )

    def test_package_configs_cannot_override_approved_frames(self):
        source_root = pathlib.Path(__file__).resolve().parents[2]
        local_configs = (
            source_root
            / "path_planning_pkg"
            / "config"
            / "hybrid_astar.yaml",
            source_root
            / "global_route_manager_pkg"
            / "config"
            / "competition_route.yaml",
        )
        forbidden = re.compile(r"^(?:frame_id|base_frame_id):", re.MULTILINE)
        for config_path in local_configs:
            with self.subTest(config_path=config_path):
                self.assertIsNone(
                    forbidden.search(config_path.read_text(encoding="utf-8"))
                )

    def test_planner_valid_publication_is_revision_gated_and_serialized(self):
        source_root = pathlib.Path(__file__).resolve().parents[2]
        node_path = (
            source_root / "path_planning_pkg" / "scripts" / "hybrid_astar_node"
        )
        source = node_path.read_text(encoding="utf-8")
        methods = {
            node.name: node
            for node in ast.walk(ast.parse(source))
            if isinstance(node, ast.FunctionDef)
        }

        def used_context_locks(method_name):
            result = set()
            for node in ast.walk(methods[method_name]):
                if not isinstance(node, ast.With):
                    continue
                for item in node.items:
                    expression = item.context_expr
                    if (
                        isinstance(expression, ast.Attribute)
                        and isinstance(expression.value, ast.Name)
                        and expression.value.id == "self"
                    ):
                        result.add(expression.attr)
            return result

        for callback in (
            "_on_odometry",
            "_on_route_context",
            "_on_lead_vehicle",
            "_on_global_path",
        ):
            callback_source = ast.get_source_segment(source, methods[callback])
            self.assertIn(
                "self._invalidate_live_trajectory_if_inputs_fault",
                callback_source,
            )
            self.assertEqual(
                used_context_locks(callback),
                {"_output_lock", "_state_lock"},
            )

        invalidation_source = ast.get_source_segment(
            source, methods["_invalidate_live_trajectory_if_inputs_fault"]
        )
        self.assertIn("self._input_fault_revision += 1", invalidation_source)

        success_source = ast.get_source_segment(
            source, methods["_publish_success"]
        )
        guard_source = ast.get_source_segment(
            source, methods["_guard_publication"]
        )
        self.assertIn("self._guard_publication", success_source)
        self.assertIn(
            "current_fault_revision != context.expected_fault_revision",
            guard_source,
        )
        self.assertIn("self._planner.validate_path", guard_source)
        self.assertIn("aligned_path_index", guard_source)
        self.assertEqual(
            used_context_locks("_publish_success"),
            {"_output_lock", "_state_lock"},
        )
        self.assertEqual(
            used_context_locks("_publish_failure"),
            {"_output_lock", "_state_lock"},
        )

    def test_named_record_minimum_fields(self):
        requirements = {
            "nodes": {"name", "owner_package", "responsibility"},
            "messages": {
                "name", "owner_package", "meaning", "invalid_representation"},
            "frames": {"name", "owner_package", "parent", "convention", "unit"},
            "parameters": {
                "name", "owner_package", "data_type", "unit", "default"},
        }
        for section_name, required in requirements.items():
            for record in self.contract.records(section_name):
                self.assertFalse(
                    required.difference(record),
                    "{} {!r} missing {}".format(
                        section_name,
                        record["name"],
                        sorted(required.difference(record)),
                    ),
                )


if __name__ == "__main__":
    unittest.main()
