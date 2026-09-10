#!/usr/bin/env python3
"""Checks localization's centrally approved ROS contract entries without ROS runtime."""

from pathlib import Path
import unittest


class InterfaceContractTest(unittest.TestCase):
    def test_required_localization_interfaces_are_registered(self):
        repository_root = Path(__file__).resolve().parents[3]
        contract_path = repository_root / "src/ros_architecture_pkg/config/interface_contract.yaml"
        contract = contract_path.read_text()
        for required_text in (
            "/sensors/gps/fix",
            "/sensors/imu/data",
            "/localization/ego/odometry",
            "/localization/ego/quality",
            "common_msgs_pkg/LocalizationQuality",
        ):
            self.assertIn(required_text, contract)


if __name__ == "__main__":
    unittest.main()
