#!/usr/bin/env python3

"""ROS-independent regression lock for the approved planning message schema."""

import pathlib
import re
import unittest


EXPECTED_SCHEMAS = {
    "LeadVehicleState.msg": {
        "constants": (),
        "fields": (
            ("std_msgs/Header", "header"),
            ("bool", "valid"),
            ("string", "lane_link_id"),
            ("geometry_msgs/Pose", "pose"),
            ("geometry_msgs/Twist", "twist"),
            ("float32", "length_m"),
            ("float32", "width_m"),
            ("float32", "longitudinal_distance_m"),
            ("float32", "confidence"),
        ),
    },
    "PlannedTrajectory.msg": {
        "constants": (
            ("uint8", "STATUS_VALID", "0"),
            ("uint8", "STATUS_STOP_REQUIRED", "1"),
            ("uint8", "STATUS_INPUT_STALE", "2"),
            ("uint8", "STATUS_NO_PATH", "3"),
            ("uint8", "STATUS_INVALID_START", "4"),
        ),
        "fields": (
            ("std_msgs/Header", "header"),
            ("time", "valid_until"),
            ("uint8", "status"),
            ("string", "reason"),
            ("bool", "lane_change_authorized"),
            ("float32", "minimum_boundary_clearance_m"),
            ("common_msgs_pkg/TrajectoryPoint[]", "points"),
        ),
    },
    "RouteContext.msg": {
        "constants": (),
        "fields": (
            ("std_msgs/Header", "header"),
            ("bool", "valid"),
            ("string", "reason"),
            ("float64", "progress_m"),
            ("float64", "route_length_m"),
            ("uint32", "nearest_route_index"),
            ("string", "current_link_id"),
            ("string[]", "horizon_link_ids"),
            ("bool", "speed_limit_exempt_zone"),
        ),
    },
    "TrajectoryPoint.msg": {
        "constants": (),
        "fields": (
            ("float64", "x_m"),
            ("float64", "y_m"),
            ("float64", "z_m"),
            ("float64", "yaw_rad"),
            ("float64", "curvature_1pm"),
            ("float64", "s_m"),
            ("float32", "target_speed_mps"),
            ("string", "lane_link_id"),
        ),
    },
}

EXPECTED_CONTRACT_MESSAGES = frozenset((
    "common_msgs_pkg/RouteContext",
    "common_msgs_pkg/LeadVehicleState",
    "common_msgs_pkg/TrajectoryPoint",
    "common_msgs_pkg/PlannedTrajectory",
))

MESSAGE_LINE_PATTERN = re.compile(
    r"^(?P<type>\S+)\s+(?P<name>[A-Za-z][A-Za-z0-9_]*)"
    r"(?:=(?P<value>\S+))?$"
)


def _parse_message(path):
    constants = []
    fields = []
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        match = MESSAGE_LINE_PATTERN.fullmatch(line)
        if match is None:
            raise AssertionError(
                "{}:{} has malformed message syntax".format(path, line_number))
        data_type = match.group("type")
        name = match.group("name")
        value = match.group("value")
        if value is None:
            fields.append((data_type, name))
        else:
            constants.append((data_type, name, value))
    return {"constants": tuple(constants), "fields": tuple(fields)}


class PlanningMessageSchemaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package_root = pathlib.Path(__file__).resolve().parents[1]
        cls.repository = cls.package_root.parents[1]
        cls.contract_path = (
            cls.repository
            / "src"
            / "ros_architecture_pkg"
            / "config"
            / "interface_contract.yaml"
        )

    def test_exact_constants_field_types_and_order(self):
        message_directory = self.package_root / "msg"
        actual_files = {
            path.name for path in message_directory.glob("*.msg")}
        self.assertEqual(actual_files, set(EXPECTED_SCHEMAS))
        for file_name, expected in EXPECTED_SCHEMAS.items():
            with self.subTest(message=file_name):
                self.assertEqual(
                    _parse_message(message_directory / file_name), expected)

    def test_central_contract_declares_exact_custom_message_set(self):
        contract = self.contract_path.read_text(encoding="utf-8")
        messages_block = contract.split("\nmessages:\n", 1)[1].split(
            "\nservices:", 1)[0]
        declared = set(re.findall(
            r"^  - name:\s*(common_msgs_pkg/\S+)\s*$",
            messages_block,
            re.MULTILINE,
        ))
        self.assertEqual(declared, EXPECTED_CONTRACT_MESSAGES)

        topic_types = set(re.findall(
            r"^\s+data_type:\s*(common_msgs_pkg/\S+)\s*$",
            contract,
            re.MULTILINE,
        ))
        self.assertEqual(
            topic_types,
            {
                "common_msgs_pkg/LeadVehicleState",
                "common_msgs_pkg/PlannedTrajectory",
                "common_msgs_pkg/RouteContext",
            },
        )

    def test_documented_type_and_critical_field_names_match_schema(self):
        documentation = "\n".join((
            (self.package_root / "README.md").read_text(encoding="utf-8"),
            (self.package_root / "docs" / "planning_messages.md").read_text(
                encoding="utf-8"),
        ))
        for file_name in EXPECTED_SCHEMAS:
            self.assertIn(file_name[:-4], documentation)

        documented_fields = (
            "pose",
            "longitudinal_distance_m",
            "lane_link_id",
            "points",
            "minimum_boundary_clearance_m",
            "header",
            "valid_until",
            "STATUS_VALID",
        )
        actual_names = {
            item[1]
            for schema in EXPECTED_SCHEMAS.values()
            for category in ("constants", "fields")
            for item in schema[category]
        }
        for name in documented_fields:
            with self.subTest(documented_name=name):
                self.assertRegex(
                    documentation,
                    r"\b{}\b".format(re.escape(name)),
                )
                self.assertIn(name, actual_names)


if __name__ == "__main__":
    unittest.main()
