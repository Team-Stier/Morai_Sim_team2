#!/usr/bin/env python3

import copy
import hashlib
import importlib.util
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
        cls.projection = load_yaml(CONFIG_ROOT / "tf" / "map_projection.yaml")
        spec = importlib.util.spec_from_file_location("tf_contract_generator", PACKAGE_ROOT / "scripts" / "generate_interface_diagrams.py")
        cls.generator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.generator)
        cls.timestamps = load_yaml(
            CONFIG_ROOT / "timestamp" / "timestamp_contract.yaml"
        )

    def test_interface_contract_references_existing_modules(self):
        modules = self.interface["contract_modules"]
        module_paths = [
            modules["tf"]["frame_contract"],
            modules["tf"]["sensor_extrinsics"],
            modules["tf"]["map_projection"],
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

    def test_only_scoped_development_transforms_can_publish(self):
        allowed = {"odom", "base_link", "gps_link", "imu_link", "lidar_link"}
        enabled = {entry["child"] for entry in self.frames["transforms"] if entry["publish_enabled"]}
        self.assertEqual(enabled, allowed)
        self.assertEqual(self.generator.validate_tf_activation(self.frames, self.extrinsics, self.projection), [])
        for entry in self.frames["transforms"]:
            if entry["publish_enabled"]:
                self.assertEqual(entry["activation_scope"], "development_only")
                self.assertFalse(entry["physical_alignment_verified"])
                self.assertNotEqual(entry["verification_status"], "runtime_verified")

    def test_development_gate_rejects_camera_and_mismatched_mount(self):
        frames = copy.deepcopy(self.frames)
        camera = next(entry for entry in frames["transforms"] if entry["child"] == "camera_front_link")
        camera["publish_enabled"] = True
        self.assertTrue(self.generator.validate_tf_activation(frames, self.extrinsics, self.projection))
        extrinsics = copy.deepcopy(self.extrinsics)
        next(entry for entry in extrinsics["sensor_mounts"] if entry["key"] == "gps")["publish_enabled"] = False
        self.assertTrue(self.generator.validate_tf_activation(self.frames, extrinsics, self.projection))

    def test_development_gate_rejects_false_physical_claim_and_wrong_owner(self):
        frames = copy.deepcopy(self.frames)
        frames["transforms"][0]["physical_alignment_verified"] = True
        self.assertTrue(self.generator.validate_tf_activation(frames, self.extrinsics, self.projection))
        frames = copy.deepcopy(self.frames)
        frames["transforms"][0]["runtime_publisher_owner"] = "visualization_pkg"
        self.assertTrue(self.generator.validate_tf_activation(frames, self.extrinsics, self.projection))

    def test_development_projection_and_timing_are_explicitly_unverified(self):
        self.assertEqual(self.projection["origin_utm_m"], [302595.0, 4124145.0, 0.0])
        self.assertFalse(self.projection["physical_alignment_verified"])
        profile = self.timestamps["development_localization_profile"]
        self.assertFalse(profile["measurement_verified"])
        self.assertFalse(profile["autonomous_driving_ready"])
        self.assertEqual(profile["future_tolerance_sec"], 0.0)
        self.assertEqual(profile["provenance"], "ingress_fallback")

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

    def test_sensor_evidence_references_resolve_within_scope(self):
        evidence_by_key = self.extrinsics["source_evidence"]
        mounts = {mount["key"]: mount for mount in self.extrinsics["sensor_mounts"]}
        for key, mount in mounts.items():
            evidence_key = mount["source_evidence"]
            self.assertIn(evidence_key, evidence_by_key)
            self.assertIn(key, evidence_by_key[evidence_key]["scope"])
        for evidence_key, evidence in evidence_by_key.items():
            self.assertTrue(set(evidence["scope"]).issubset(mounts))

    def test_user_confirmed_gps_imu_placements_have_scoped_development_approval(self):
        mounts = {mount["key"]: mount for mount in self.extrinsics["sensor_mounts"]}
        evidence_key = "launcher_saved_profile_user_confirmed"
        evidence = self.extrinsics["source_evidence"][evidence_key]
        self.assertEqual(set(evidence["scope"]), {"gps", "imu"})
        self.assertTrue(evidence["user_confirmed_mount_position"])
        self.assertFalse(evidence["active_loadout_verified"])
        expected = {
            "gps": ([0.0, 0.0, 1.3], 4, "GPS-4"),
            "imu": ([0.0, 0.0, 0.0], 5, "IMU-5"),
        }
        for key, (position, sensor_id, raw_frame) in expected.items():
            mount = mounts[key]
            self.assertEqual(mount["source_evidence"], evidence_key)
            self.assertEqual(mount["source_pose"]["translation_m"], position)
            self.assertEqual(mount["candidate_ros_pose"]["translation_m"], position)
            self.assertEqual(mount["source_pose"]["rotation_rpy_deg"], [0.0] * 3)
            self.assertEqual(mount["candidate_ros_pose"]["rotation_rpy_rad"], [0.0] * 3)
            self.assertEqual(mount["raw_identifiers"]["sensor_unique_id"], sensor_id)
            self.assertEqual(mount["raw_identifiers"]["source_frame_id"], raw_frame)
            self.assertEqual(mount["child_frame"], key + "_link")
            self.assertTrue(mount["publish_enabled"])
            self.assertEqual(mount["activation_scope"], "development_only")
            self.assertFalse(mount["physical_alignment_verified"])

    def test_observed_user_confirmed_profile_matches_its_scope(self):
        self._assert_observed_profile_matches_scope("launcher_saved_profile_user_confirmed")

    def _assert_observed_profile_matches_scope(self, evidence_key):
        evidence = self.extrinsics["source_evidence"][evidence_key]
        source_path = Path(evidence["observed_host_path"])
        if not source_path.is_file():
            self.skipTest("host-specific MORAI saved profile is not present: " + evidence_key)

        digest = hashlib.sha256(source_path.read_bytes()).hexdigest()
        self.assertEqual(digest, evidence["sha256"])
        with source_path.open("r", encoding="utf-8") as stream:
            source = json.load(stream)

        source_locations = {
            "camera_front": ("cameraList", "cc"),
            "camera_left": ("cameraList", "cc"),
            "camera_right": ("cameraList", "cc"),
            "lidar": ("Lidar3DList", "lc"),
            "gps": ("GPSList", "gc"),
            "imu": ("IMUList", "ic"),
        }
        mount_by_key = {
            mount["key"]: mount for mount in self.extrinsics["sensor_mounts"]
        }
        for key in evidence["scope"]:
            list_name, config_name = source_locations[key]
            mount = mount_by_key[key]
            sensor_id = mount["raw_identifiers"]["sensor_unique_id"]
            candidates = {
                sensor["m_SensorUniqueID"]: sensor for sensor in source[list_name]
            }
            source_sensor = candidates[sensor_id]
            self.assertEqual(source_sensor["m_TargetUniqueID"],
                             mount["raw_identifiers"]["target_unique_id"])
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

    def test_user_approved_lidar_mount_and_publication_gate(self):
        lidar = next(m for m in self.extrinsics["sensor_mounts"] if m["key"] == "lidar")
        self.assertEqual(lidar["source_evidence"], "user_confirmed_lidar_mount")
        evidence = self.extrinsics["source_evidence"][lidar["source_evidence"]]
        self.assertEqual(evidence["translation_m"], [2.0, 0.0, 1.5])
        for field in ("source_pose", "candidate_ros_pose"):
            self.assertEqual(lidar[field]["translation_m"], [2.0, 0.0, 1.5])
        self.assertTrue(lidar["publish_enabled"])
        frames = yaml.safe_load((Path(__file__).resolve().parents[1] / "config/tf/frame_contract.yaml").read_text())
        transform = next(t for t in frames["transforms"] if t["child"] == "lidar_link")
        self.assertEqual(transform["parent"], "base_link")
        self.assertTrue(transform["publish_enabled"])

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
