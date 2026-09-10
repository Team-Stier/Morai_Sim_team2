import re
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "src/common_msgs_pkg"
CONFIG = ROOT / "src/ros_architecture_pkg/config"


def read(path):
    return yaml.safe_load(path.read_text())


class ReadinessContractTest(unittest.TestCase):
    def test_wire_definitions_match_central_candidate_schema(self):
        schema = read(CONFIG / "messages/readiness_messages.yaml")
        self.assertEqual(
            set(schema["messages"]),
            {"InterfaceStatus", "ControllerStatus", "SystemReadiness"},
        )
        for name, entry in schema["messages"].items():
            self.assertEqual(
                (PACKAGE / "msg" / (name + ".msg")).read_text(),
                entry["wire_definition"],
            )

    def test_type_names_are_already_reserved_by_public_contract(self):
        contract = read(CONFIG / "interface_contract.yaml")
        registered = {entry["type"] for entry in contract["messages"]}
        for name in ("InterfaceStatus", "ControllerStatus", "SystemReadiness"):
            self.assertIn("common_msgs_pkg/" + name, registered)

    def test_cmake_generates_readiness_messages(self):
        cmake = (PACKAGE / "CMakeLists.txt").read_text()
        generated = set(re.findall(r"\b\w+\.msg\b", cmake))
        for name in ("InterfaceStatus.msg", "ControllerStatus.msg", "SystemReadiness.msg"):
            self.assertIn(name, generated)

    def test_system_readiness_mask_bits_are_unique_powers_of_two(self):
        wire = read(CONFIG / "messages/readiness_messages.yaml")["messages"]["SystemReadiness"]["wire_definition"]
        values = []
        for line in wire.splitlines():
            match = re.match(r"uint32 COMPONENT_[A-Z_]+=(\d+)$", line)
            if match:
                values.append(int(match.group(1)))
        self.assertEqual(len(values), 9)
        self.assertEqual(len(set(values)), 9)
        self.assertTrue(all(value > 0 and value & (value - 1) == 0 for value in values))


if __name__ == "__main__":
    unittest.main()
