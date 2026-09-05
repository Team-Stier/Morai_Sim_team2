#!/usr/bin/env python3

import hashlib
import json
import math
import unittest
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
CONFIG_ROOT = PACKAGE_ROOT / "config"


def load_yaml(path):
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class TfTimestampContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.interface = load_yaml(CONFIG_ROOT / "interface_contract.yaml")
        cls.frames = load_yaml(CONFIG_ROOT / "tf" / "frame_contract.yaml")
        cls.extrinsics = load_yaml(CONFIG_ROOT / "tf" / "sensor_extrinsics.yaml")
        cls.timestamps = load_yaml(
            CONFIG_ROOT / "timestamp" / "timestamp_contract.yaml"
        )

    def test_interface_contract_references_existing_modules(self):
        modules = self.interface["contract_modules"]
        module_paths = [
            modules["tf"]["frame_contract"],
            modules["tf"]["sensor_extrinsics"],
            modules["timestamp"]["timestamp_contract"],
        ]
        for relative_path in module_paths:
            self.assertTrue((CONFIG_ROOT / relative_path).is_file(), relative_path)

        registry_names = [entry["name"] for entry in self.interface["frames"]]
        detailed_names = [entry["name"] for entry in self.frames["frames"]]
        self.assertEqual(registry_names, detailed_names)

    def test_frame_graph_has_one_parent_no_cycle_and_full_reachability(self):
        frame_names = [entry["name"] for entry in self.frames["frames"]]
        self.assertEqual(len(frame_names), len(set(frame_names)))

        root = self.frames["root_frame"]
        self.assertIn(root, frame_names)
        parent_by_child = {}
        children_by_parent = {name: [] for name in frame_names}
        for transform in self.frames["transforms"]:
            parent = transform["parent"]
            child = transform["child"]
            self.assertIn(parent, frame_names)
            self.assertIn(child, frame_names)
            self.assertNotIn(child, parent_by_child, f"multiple parents for {child}")
            parent_by_child[child] = parent
            children_by_parent[parent].append(child)

        self.assertNotIn(root, parent_by_child)
        self.assertEqual(set(parent_by_child), set(frame_names) - {root})

        visited = set()
        active = set()

        def visit(frame):
            self.assertNotIn(frame, active, f"cycle detected at {frame}")
            if frame in visited:
                return
            active.add(frame)
            for child in children_by_parent[frame]:
                visit(child)
            active.remove(frame)
            visited.add(frame)

        visit(root)
        self.assertEqual(visited, set(frame_names))

    def test_v1_dynamic_tf_chain_is_exact(self):
        self.assertEqual(self.frames["root_frame"], "map")
        dynamic_chain = [
            (transform["parent"], transform["child"])
            for transform in self.frames["transforms"]
            if transform["type"] == "dynamic"
        ]
        self.assertEqual(
            dynamic_chain,
            [("map", "odom"), ("odom", "base_link")],
        )

    def test_every_topic_frame_uses_the_registered_tf_vocabulary(self):
        frame_names = {entry["name"] for entry in self.frames["frames"]}
        non_frame_values = {"not_applicable", "pending_competition_packet_spec"}
        for topic in self.interface["topics"]:
            for field in ("frame", "child_frame", "motion_frame"):
                if field not in topic or topic[field] in non_frame_values:
                    continue
                self.assertIn(
                    topic[field],
                    frame_names,
                    "{} {}".format(topic["name"], field),
                )

        topics = {topic["name"]: topic for topic in self.interface["topics"]}
        odometry = topics["/molit/localization/local/odometry"]
        self.assertEqual(odometry["data_type"], "nav_msgs/Odometry")
        self.assertEqual(odometry["frame"], "odom")
        self.assertEqual(odometry["child_frame"], "base_link")
        ego_state = topics["/molit/localization/ego_state"]
        self.assertEqual(ego_state["frame"], "map")
        self.assertEqual(ego_state["motion_frame"], "base_link")

    def test_unverified_transforms_cannot_publish(self):
        for transform in self.frames["transforms"]:
            self.assertTrue(transform["verification_status"])
            if transform["verification_status"] != "runtime_verified":
                self.assertFalse(transform["publish_enabled"], transform["child"])

        for mount in self.extrinsics["sensor_mounts"]:
            self.assertTrue(mount["source_evidence"])
            self.assertTrue(mount["verification_status"])
            if mount["verification_status"] != "runtime_verified":
                self.assertFalse(mount["publish_enabled"], mount["key"])

    def test_candidate_extrinsics_are_finite_and_use_declared_units(self):
        source_convention = self.extrinsics["source_pose_convention"]
        ros_convention = self.extrinsics["candidate_ros_pose_convention"]
        self.assertEqual(source_convention["translation_unit"], "m")
        self.assertEqual(source_convention["rotation_unit"], "deg")
        self.assertEqual(ros_convention["translation_unit"], "m")
        self.assertEqual(ros_convention["rotation_unit"], "rad")
        self.assertFalse(ros_convention["assumption_verified"])

        for mount in self.extrinsics["sensor_mounts"]:
            for pose_name in ("source_pose", "candidate_ros_pose"):
                pose = mount[pose_name]
                self.assertEqual(len(pose["translation_m"]), 3)
                rotation_key = (
                    "rotation_rpy_deg"
                    if pose_name == "source_pose"
                    else "rotation_rpy_rad"
                )
                self.assertEqual(len(pose[rotation_key]), 3)
                for value in pose["translation_m"] + pose[rotation_key]:
                    self.assertTrue(math.isfinite(value), mount["key"])

    def test_repository_camera_values_and_hash_match_contract(self):
        evidence = self.extrinsics["source_evidence"]["repository_camera_reference"]
        source_path = REPOSITORY_ROOT / evidence["path"]
        digest = hashlib.sha256(source_path.read_bytes()).hexdigest()
        self.assertEqual(digest, evidence["sha256"])

        with source_path.open("r", encoding="utf-8") as stream:
            source = json.load(stream)
        source_by_id = {
            sensor["m_SensorUniqueID"]: sensor for sensor in source["cameraList"]
        }

        camera_mounts = {
            mount["key"]: mount
            for mount in self.extrinsics["sensor_mounts"]
            if mount["key"].startswith("camera_")
        }
        self.assertEqual(set(camera_mounts), set(evidence["scope"]))
        for mount in camera_mounts.values():
            sensor_id = mount["raw_identifiers"]["sensor_unique_id"]
            source_sensor = source_by_id[sensor_id]
            source_translation = [float(source_sensor["pos"][axis]) for axis in "xyz"]
            source_rotation = [
                float(source_sensor["rot"][axis]) for axis in ("roll", "pitch", "yaw")
            ]
            self.assertEqual(source_translation, mount["source_pose"]["translation_m"])
            self.assertEqual(source_rotation, mount["source_pose"]["rotation_rpy_deg"])
            self.assertEqual(
                source_sensor["cc"]["rosConfig"]["frameID"],
                mount["raw_identifiers"]["source_frame_id"],
            )

    def test_observed_launcher_profile_matches_all_recorded_mounts(self):
        evidence = self.extrinsics["source_evidence"]["launcher_saved_profile"]
        source_path = Path(evidence["observed_host_path"])
        if not source_path.is_file():
            self.skipTest("host-specific MORAI saved profile is not present")

        digest = hashlib.sha256(source_path.read_bytes()).hexdigest()
        self.assertEqual(digest, evidence["sha256"])
        with source_path.open("r", encoding="utf-8") as stream:
            source = json.load(stream)

        source_locations = {
            "camera_front": ("cameraList", "cc", 1),
            "camera_left": ("cameraList", "cc", 2),
            "camera_right": ("cameraList", "cc", 3),
            "lidar": ("Lidar3DList", "lc", 6),
            "gps": ("GPSList", "gc", 5),
            "imu": ("IMUList", "ic", 4),
        }
        mount_by_key = {
            mount["key"]: mount for mount in self.extrinsics["sensor_mounts"]
        }
        for key, (list_name, config_name, sensor_id) in source_locations.items():
            candidates = {
                sensor["m_SensorUniqueID"]: sensor for sensor in source[list_name]
            }
            source_sensor = candidates[sensor_id]
            mount = mount_by_key[key]
            source_translation = [float(source_sensor["pos"][axis]) for axis in "xyz"]
            source_rotation = [
                float(source_sensor["rot"][axis]) for axis in ("roll", "pitch", "yaw")
            ]
            sensor_config = source_sensor[config_name]
            self.assertEqual(source_translation, mount["source_pose"]["translation_m"])
            self.assertEqual(source_rotation, mount["source_pose"]["rotation_rpy_deg"])
            self.assertEqual(
                sensor_config["rosConfig"]["frameID"],
                mount["raw_identifiers"]["source_frame_id"],
            )
            self.assertEqual(
                sensor_config["sensorPeriod"], mount["configured_period_sec"]
            )

    def test_configured_rates_are_targets_and_match_periods(self):
        timing_targets = self.timestamps["freshness_policy"][
            "configured_targets_not_measurements"
        ]
        timing_key_by_mount = {
            "camera_front": "camera",
            "camera_left": "camera",
            "camera_right": "camera",
            "lidar": "lidar",
            "gps": "gps",
            "imu": "imu",
        }
        for mount in self.extrinsics["sensor_mounts"]:
            self.assertIsNone(mount["measured_rate_hz"])
            self.assertAlmostEqual(
                mount["configured_rate_hz"],
                1.0 / mount["configured_period_sec"],
                places=5,
            )
            target = timing_targets[timing_key_by_mount[mount["key"]]]
            self.assertEqual(target["configured_rate_hz"], mount["configured_rate_hz"])
            self.assertEqual(
                target["configured_period_sec"], mount["configured_period_sec"]
            )

    def test_timestamp_modes_and_measurement_stamp_policy(self):
        modes = self.timestamps["clock_modes"]
        self.assertFalse(modes["live_morai"]["ros_use_sim_time"])
        self.assertTrue(modes["rosbag_replay"]["ros_use_sim_time"])

        stamp = self.timestamps["header_stamp_contract"]
        self.assertEqual(stamp["semantic"], "measurement_time")
        self.assertFalse(stamp["downstream_overwrite_allowed"])
        self.assertFalse(stamp["processing_completion_time_allowed"])
        self.assertEqual(
            stamp["fallback_when_source_stamp_missing"]["provenance"],
            "ingress_fallback",
        )
        self.assertEqual(
            self.timestamps["freshness_policy"]["stale_threshold_status"],
            "pending_rate_and_jitter_measurement",
        )

    def test_interface_topics_use_the_canonical_timestamp_registry(self):
        registry = self.timestamps["timestamp_source_registry"]
        for topic in self.interface["topics"]:
            self.assertIn(
                topic["timestamp_source"], registry, topic["name"]
            )

        topics = {topic["name"]: topic for topic in self.interface["topics"]}
        expected = {
            "/molit/perception/camera/front/observations": "source_sensor_measurement_time",
            "/molit/perception/lidar/observations": "source_sensor_measurement_time",
            "/molit/localization/local/odometry": "estimate_valid_time",
            "/molit/localization/ego_state": "estimate_valid_time",
            "/molit/route/context": "ego_pose_time_used_for_route_matching",
            "/molit/world_model/scene": "fusion_reference_time",
            "/molit/planning/trajectory": "planning_reference_time",
            "/molit/control/nominal_command": "command_generation_time",
            "/molit/safety/final_command": "command_generation_time",
            "/molit/safety/state": "status_evaluation_time",
        }
        for topic_name, timestamp_source in expected.items():
            self.assertEqual(
                topics[topic_name]["timestamp_source"],
                timestamp_source,
                topic_name,
            )

        for policy in self.timestamps["derived_message_contract"].values():
            self.assertIn(policy["stamp"], registry)


if __name__ == "__main__":
    unittest.main()
