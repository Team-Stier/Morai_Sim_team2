#!/usr/bin/env python3

"""Generate deterministic Mermaid views from the central ROS interface contract.

The YAML contract remains the only source of truth.  This tool deliberately does
not infer missing public interfaces from package source code or README files.
"""

import argparse
import difflib
import hashlib
import json
import math
import re
import sys
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
DEFAULT_CONTRACT_PATH = PACKAGE_ROOT / "config" / "interface_contract.yaml"
MERMAID_CLI_VERSION = "11.16.0"
APPROVED_TOPIC_NAMESPACE = "/molit"
EXPECTED_PUBLIC_INTERFACE_POLICY = {
    "topic_namespace": "/molit",
    "internal_topic_namespace_template": "/molit/internal/<package_name_without_pkg>/...",
    "integrated_launch_node_namespace": "/",
    "node_entries_are_basenames_under_integrated_namespace": True,
    "integrated_launch_node_names_are_exact": True,
    "public_topic_remap_allowed": False,
    "public_node_remap_allowed": False,
    "single_producer_per_topic": True,
}
EXPECTED_RUNTIME_READINESS_STATUS = (
    "semantics_approved_required_channel_sets_pending_runtime_validation"
)
EXPECTED_RUNTIME_READINESS_RULES = (
    "A channel with runtime_activation_allowed false cannot be required by an active bringup profile.",
    "Required and optional channel sets must be explicit in each system_bringup_pkg runtime profile.",
)
EXPECTED_GPS_BLACKOUT_POLICY = {
    "expected_operating_condition": True,
    "gps_stale_or_no_fix_alone_requires_stop": False,
    "motion_requires_fresh_local_odometry": True,
    "motion_requires_localization_quality_within_approved_bounds": True,
    "last_gps_fix_must_not_be_reused_as_current_position_truth": True,
}
EXPECTED_REQUIRED_INTERFACE_FIELDS = (
    "owner_package",
    "producers",
    "consumers",
    "status",
    "visibility",
    "data_meaning",
    "data_type",
    "unit",
    "frame",
    "timestamp_source",
    "expected_rate_hz",
    "queue_policy",
    "timeout_sec",
    "invalid_data_policy",
)
EXPECTED_FINAL_COMMAND_TIMEOUT_POLICY = (
    "never_repeat_last_nonzero_command_and_send_only_a_packet_verified_fail_closed_stop"
)
EXPECTED_TOPIC_FRAME_CONTRACT = {
    "/molit/sensors/camera/front/image/compressed": {"frame": "camera_front_optical_frame"},
    "/molit/sensors/camera/left/image/compressed": {"frame": "camera_left_optical_frame"},
    "/molit/sensors/camera/right/image/compressed": {"frame": "camera_right_optical_frame"},
    "/molit/sensors/gps/fix": {"frame": "gps_link"},
    "/molit/internal/morai_interface/lidar/packets": {"frame": "lidar_link"},
    "/molit/sensors/imu/data": {"frame": "imu_link"},
    "/molit/vehicle/twist": {"frame": "base_link"},
    "/molit/sensors/lidar/points": {"frame": "lidar_link"},
    "/molit/sensors/lidar/status": {"frame": "not_applicable"},
    "/molit/events/collision": {"frame": "pending_competition_packet_spec"},
    "/molit/interface/status": {"frame": "not_applicable"},
    "/molit/map/hd_map": {"frame": "map"},
    "/molit/map/status": {"frame": "not_applicable"},
    "/molit/perception/camera/front/observations": {
        "frame": "camera_front_optical_frame"
    },
    "/molit/perception/camera/left/observations": {
        "frame": "camera_left_optical_frame"
    },
    "/molit/perception/camera/right/observations": {
        "frame": "camera_right_optical_frame"
    },
    "/molit/perception/camera/status": {"frame": "not_applicable"},
    "/molit/perception/lidar/observations": {"frame": "lidar_link"},
    "/molit/perception/lidar/status": {"frame": "not_applicable"},
    "/molit/localization/local/odometry": {
        "frame": "odom",
        "child_frame": "base_link",
    },
    "/molit/localization/ego_state": {
        "frame": "map",
        "motion_frame": "base_link",
    },
    "/molit/localization/status": {"frame": "not_applicable"},
    "/molit/route/global_path": {"frame": "map"},
    "/molit/route/context": {"frame": "map"},
    "/molit/route/status": {"frame": "not_applicable"},
    "/molit/world_model/scene": {"frame": "map"},
    "/molit/world_model/status": {"frame": "not_applicable"},
    "/molit/planning/trajectory": {"frame": "odom"},
    "/molit/planning/status": {"frame": "not_applicable"},
    "/molit/control/nominal_command": {"frame": "base_link"},
    "/molit/control/status": {"frame": "not_applicable"},
    "/molit/system/readiness": {"frame": "not_applicable"},
    "/molit/safety/final_command": {"frame": "base_link"},
    "/molit/safety/state": {"frame": "not_applicable"},
    "/molit/evaluation/metrics": {"frame": "not_applicable"},
}
EXPECTED_TF_ROOT_FRAME = "map"
EXPECTED_DYNAMIC_TF_CHAIN = (("map", "odom"), ("odom", "base_link"))
EXPECTED_EXTERNAL_ADAPTERS = {
    "ingress": (
        "morai_camera_front",
        "morai_camera_left",
        "morai_camera_right",
        "morai_gps_bridge",
        "morai_imu_bridge",
        "morai_vehicle_status_bridge",
        "morai_velodyne_driver",
        "morai_collision_bridge",
    ),
    "egress": ("morai_control_sender",),
}
EXPECTED_ARCHITECTURE_VIEW_OUTPUTS = {
    "nominal_data_control": "system_nominal_flow",
    "health_readiness_safety_evaluation": "system_health_safety_flow",
}
GENERATED_HEADER = (
    "%% AUTO-GENERATED by ros_architecture_pkg/scripts/"
    "generate_interface_diagrams.py. DO NOT EDIT.\n"
    "%% Source of truth: ros_architecture_pkg/config/interface_contract.yaml\n"
)

NON_LIVE_STATUS_MARKERS = (
    "disabled",
    "not_implemented",
    "not_runtime_verified",
    "pending",
    "planned",
    "prohibited",
    "reserved",
    "skeleton",
    "unverified",
)
LIVE_STATUS_MARKERS = ("active", "implemented", "live", "runtime_verified")


class ContractError(ValueError):
    """Raised when the contract cannot form an unambiguous ROS graph."""


def load_contract(path):
    """Load a YAML mapping from *path*."""
    with path.open("r", encoding="utf-8") as stream:
        contract = yaml.safe_load(stream)
    if not isinstance(contract, dict):
        raise ContractError("interface contract root must be a YAML mapping")
    return contract


def _duplicates(values):
    seen = set()
    duplicates = set()
    for value in values:
        if value in seen:
            duplicates.add(value)
        seen.add(value)
    return sorted(duplicates, key=lambda item: str(item))


def _is_internal_topic(topic):
    name = str(topic.get("name", ""))
    return (
        topic.get("visibility") in ("internal", "package_internal")
        or topic.get("public") is False
        or "/internal/" in name
    )


def _as_string_list(value, field_path, errors):
    if not isinstance(value, list):
        errors.append("{} must be a list".format(field_path))
        return []
    invalid = [item for item in value if not isinstance(item, str) or not item]
    if invalid:
        errors.append("{} must contain only non-empty strings".format(field_path))
    return [item for item in value if isinstance(item, str) and item]


def _validate_korean_diagram_text(value, field_path, errors, max_length=60):
    """Validate concise, single-line Korean copy used only in package diagrams."""
    if not isinstance(value, str) or not value:
        errors.append("{} must be a non-empty string".format(field_path))
        return
    if value != value.strip():
        errors.append("{} must not have leading or trailing whitespace".format(field_path))
    if "\n" in value or "\r" in value:
        errors.append("{} must be a single line".format(field_path))
    if len(value) > max_length:
        errors.append("{} must be at most {} characters".format(field_path, max_length))
    if re.search(r"[가-힣]", value) is None:
        errors.append("{} must include Korean text".format(field_path))


def validate_contract(contract, repository_root):
    """Return graph/schema errors that would make generated views misleading."""
    errors = []
    if tuple(contract.get("required_interface_fields", ())) != (
        EXPECTED_REQUIRED_INTERFACE_FIELDS
    ):
        errors.append(
            "required_interface_fields must match the complete v1 field set"
        )
    nodes = contract.get("nodes")
    topics = contract.get("topics")
    boundaries = contract.get("package_boundaries")
    if not isinstance(nodes, list):
        errors.append("nodes must be a list")
        nodes = []
    if not isinstance(topics, list):
        errors.append("topics must be a list")
        topics = []
    if not isinstance(boundaries, dict):
        errors.append("package_boundaries must be a mapping")
        boundaries = {}

    runtime_readiness = contract.get("runtime_readiness_policy")
    if not isinstance(runtime_readiness, dict):
        errors.append("runtime_readiness_policy must be a mapping")
    else:
        if runtime_readiness.get("status") != EXPECTED_RUNTIME_READINESS_STATUS:
            errors.append(
                "runtime_readiness_policy.status must remain {!r}".format(
                    EXPECTED_RUNTIME_READINESS_STATUS
                )
            )
        readiness_rules = _as_string_list(
            runtime_readiness.get("rules"),
            "runtime_readiness_policy.rules",
            errors,
        )
        for required_rule in EXPECTED_RUNTIME_READINESS_RULES:
            if required_rule not in readiness_rules:
                errors.append(
                    "runtime_readiness_policy must include required v1 rule: {}".format(
                        required_rule
                    )
                )

        gps_blackout = runtime_readiness.get("gps_blackout")
        if not isinstance(gps_blackout, dict):
            errors.append("runtime_readiness_policy.gps_blackout must be a mapping")
        else:
            for field, expected_value in EXPECTED_GPS_BLACKOUT_POLICY.items():
                if gps_blackout.get(field) is not expected_value:
                    errors.append(
                        "runtime_readiness_policy.gps_blackout.{} must remain {!r}".format(
                            field, expected_value
                        )
                    )

    source_root = repository_root / "src"
    repository_packages = {
        path.parent.name for path in source_root.glob("*/package.xml")
    }
    missing_boundaries = sorted(repository_packages - set(boundaries))
    for package_name in missing_boundaries:
        errors.append(
            "repository package has no package boundary: {}".format(package_name)
        )

    node_names = [entry.get("name") for entry in nodes if isinstance(entry, dict)]
    topic_names = [entry.get("name") for entry in topics if isinstance(entry, dict)]
    for duplicate in _duplicates(node_names):
        errors.append("duplicate node name: {}".format(duplicate))
    for duplicate in _duplicates(topic_names):
        errors.append("duplicate topic name: {}".format(duplicate))
    if any(not isinstance(name, str) or not name for name in node_names):
        errors.append("every node must have a non-empty string name")
    if any(not isinstance(name, str) or not name for name in topic_names):
        errors.append("every topic must have a non-empty string name")

    node_by_name = {
        entry["name"]: entry
        for entry in nodes
        if isinstance(entry, dict)
        and isinstance(entry.get("name"), str)
        and entry.get("name")
    }
    topic_by_name = {
        entry["name"]: entry
        for entry in topics
        if isinstance(entry, dict)
        and isinstance(entry.get("name"), str)
        and entry.get("name")
    }

    config_root = repository_root / "src" / "ros_architecture_pkg" / "config"
    required_module_paths = {
        "package_registry": ("path",),
        "tf": ("frame_contract", "sensor_extrinsics"),
        "timestamp": ("timestamp_contract",),
        "morai_interface": ("udp_ros_bridge",),
    }
    loaded_modules = {}
    contract_modules = contract.get("contract_modules")
    if not isinstance(contract_modules, dict):
        errors.append("contract_modules must be a mapping")
        contract_modules = {}
    for module_name, path_fields in required_module_paths.items():
        module_reference = contract_modules.get(module_name)
        if not isinstance(module_reference, dict):
            errors.append(
                "contract_modules.{} must be a mapping".format(module_name)
            )
            continue
        required_version = module_reference.get("required_contract_version")
        if not isinstance(required_version, str) or not required_version:
            errors.append(
                "contract_modules.{} needs required_contract_version".format(
                    module_name
                )
            )
        for path_field in path_fields:
            relative_path = module_reference.get(path_field)
            if not isinstance(relative_path, str) or not relative_path:
                errors.append(
                    "contract_modules.{}.{} is missing".format(
                        module_name, path_field
                    )
                )
                continue
            module_path = config_root / relative_path
            try:
                module_contract = load_contract(module_path)
            except (ContractError, OSError, yaml.YAMLError) as error:
                errors.append(
                    "cannot load contract module {}.{}: {}".format(
                        module_name, path_field, error
                    )
                )
                continue
            loaded_modules[(module_name, path_field)] = module_contract
            if module_contract.get("contract_version") != required_version:
                errors.append(
                    "contract module {}.{} version must match required_contract_version {}".format(
                        module_name, path_field, required_version
                    )
                )

    planning_policy = contract.get("planning_policy")
    if not isinstance(planning_policy, dict):
        errors.append("planning_policy must be a mapping")
    else:
        expected_input_groups = {
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
        expected_trajectory_output = "/molit/planning/trajectory"
        expected_status_output = "/molit/planning/status"
        planner_name = "path_planner_node"

        exact_policy_values = {
            "planner_class": "modular_behavior_and_motion_planner",
            "owner_package": "path_planning_pkg",
            "public_boundary_node": planner_name,
            "implementation_status": "reserved_not_implemented",
            "direct_actuator_output_allowed": False,
        }
        for field, expected_value in exact_policy_values.items():
            if planning_policy.get(field) != expected_value:
                errors.append(
                    "planning_policy.{} must remain {!r}".format(
                        field, expected_value
                    )
                )
        if planning_policy.get("input_groups") != expected_input_groups:
            errors.append(
                "planning_policy.input_groups must match the exact modular planner inputs"
            )

        planner_input_types = {
            "/molit/localization/local/odometry": "nav_msgs/Odometry",
            "/molit/localization/ego_state": "common_msgs_pkg/EgoState",
            "/molit/localization/status": "common_msgs_pkg/LocalizationStatus",
            "/molit/route/context": "common_msgs_pkg/RouteContext",
            "/molit/route/status": "common_msgs_pkg/ComponentStatus",
            "/molit/world_model/scene": "common_msgs_pkg/WorldModel",
            "/molit/world_model/status": "common_msgs_pkg/ComponentStatus",
        }
        for topic_name, expected_type in planner_input_types.items():
            topic = topic_by_name.get(topic_name)
            if topic is not None and topic.get("data_type") != expected_type:
                errors.append(
                    "planning_policy input {} type must remain {}".format(
                        topic_name, expected_type
                    )
                )

        approved_outputs = planning_policy.get("approved_public_outputs")
        if not isinstance(approved_outputs, dict):
            errors.append("planning_policy.approved_public_outputs must be a mapping")
        else:
            expected_outputs = {
                "trajectory": {
                    "topic": expected_trajectory_output,
                    "frame": "odom",
                },
                "status": {
                    "topic": expected_status_output,
                    "frame": "not_applicable",
                },
            }
            if approved_outputs != expected_outputs:
                errors.append(
                    "planning_policy.approved_public_outputs must match trajectory and status"
                )
            for output in approved_outputs.values():
                if not isinstance(output, dict):
                    continue
                topic = topic_by_name.get(output.get("topic"))
                if topic is not None and output.get("frame") != topic.get("frame"):
                    errors.append(
                        "planning_policy output frame must match topic {}".format(
                            topic.get("name")
                        )
                    )

        declared_planner_inputs = []
        input_groups = planning_policy.get("input_groups")
        if isinstance(input_groups, dict):
            for values in input_groups.values():
                if isinstance(values, list):
                    declared_planner_inputs.extend(values)
        if _duplicates(declared_planner_inputs):
            errors.append("planning_policy inputs must not contain duplicates")

        planner_public_inputs = sorted(
            topic_name
            for topic_name, topic in topic_by_name.items()
            if not _is_internal_topic(topic)
            and planner_name in topic.get("consumers", [])
        )
        if sorted(declared_planner_inputs) != planner_public_inputs:
            errors.append(
                "planning_policy inputs must exactly match path_planner_node topic consumers"
            )
        planner_boundary = boundaries.get("path_planning_pkg")
        if isinstance(planner_boundary, dict):
            if sorted(planner_boundary.get("inputs", [])) != sorted(
                declared_planner_inputs
            ):
                errors.append(
                    "planning_policy inputs must exactly match path_planning_pkg boundary inputs"
                )
            expected_planner_outputs = [
                expected_trajectory_output,
                expected_status_output,
            ]
            if planner_boundary.get("outputs") != expected_planner_outputs:
                errors.append(
                    "path_planning_pkg outputs must remain trajectory then planning status"
                )
        planner_outputs = sorted(
            topic_name
            for topic_name, topic in topic_by_name.items()
            if planner_name in topic.get("producers", [])
        )
        if planner_outputs != sorted(
            [expected_trajectory_output, expected_status_output]
        ):
            errors.append(
                "path_planner_node outputs must be trajectory and planning status only"
            )

    udp_contract = loaded_modules.get(("morai_interface", "udp_ros_bridge"))
    if isinstance(udp_contract, dict):
        udp_channels = udp_contract.get("channels")
        if not isinstance(udp_channels, dict):
            errors.append("MORAI UDP contract channels must be a mapping")
            udp_channels = {}
        external_stub_contracts = {
            "collision": {
                "node_name": "morai_collision_bridge",
                "topic_field": "topic",
                "topic_name": "/molit/events/collision",
                "endpoint_field": "producers",
            },
            "control": {
                "node_name": "morai_control_sender",
                "topic_field": "input_topic",
                "topic_name": "/molit/safety/final_command",
                "endpoint_field": "consumers",
            },
        }
        for channel_name, expected in external_stub_contracts.items():
            channel = udp_channels.get(channel_name)
            if not isinstance(channel, dict):
                errors.append(
                    "MORAI UDP contract needs a disabled {} channel stub".format(
                        channel_name
                    )
                )
                continue
            if channel.get("runtime_activation_allowed") is not False:
                errors.append(
                    "MORAI UDP {} channel must remain runtime-disabled pending verification".format(
                        channel_name
                    )
                )
            if channel.get("node_name") != expected["node_name"]:
                errors.append(
                    "MORAI UDP {} node_name must be {}".format(
                        channel_name, expected["node_name"]
                    )
                )
            if channel.get(expected["topic_field"]) != expected["topic_name"]:
                errors.append(
                    "MORAI UDP {} {} must be {}".format(
                        channel_name,
                        expected["topic_field"],
                        expected["topic_name"],
                    )
                )
            top_level_topic = topic_by_name.get(expected["topic_name"])
            if top_level_topic is None:
                errors.append(
                    "MORAI UDP {} stub references a missing top-level topic".format(
                        channel_name
                    )
                )
                continue
            if expected["node_name"] not in top_level_topic.get(
                expected["endpoint_field"], []
            ):
                errors.append(
                    "MORAI UDP {} node is not the top-level topic {}".format(
                        channel_name,
                        expected["endpoint_field"].rstrip("s"),
                    )
                )
            for channel_field, topic_field in (
                ("message_type", "data_type"),
                ("frame_id", "frame"),
                ("timestamp_source", "timestamp_source"),
            ):
                if channel.get(channel_field) != top_level_topic.get(topic_field):
                    errors.append(
                        "MORAI UDP {} {} must match top-level {}".format(
                            channel_name, channel_field, expected["topic_name"]
                        )
                    )

    expected_topic_names = set(EXPECTED_TOPIC_FRAME_CONTRACT)
    actual_topic_names = set(topic_by_name)
    for topic_name in sorted(expected_topic_names - actual_topic_names):
        errors.append("missing v1 topic frame contract entry: {}".format(topic_name))
    for topic_name in sorted(actual_topic_names - expected_topic_names):
        errors.append("topic {} has no approved v1 frame contract".format(topic_name))
    for topic_name in sorted(expected_topic_names & actual_topic_names):
        topic = topic_by_name[topic_name]
        actual_frames = {
            field: topic[field]
            for field in ("frame", "child_frame", "motion_frame")
            if field in topic
        }
        expected_frames = EXPECTED_TOPIC_FRAME_CONTRACT[topic_name]
        if actual_frames != expected_frames:
            errors.append(
                "topic {} frame contract must remain {!r}".format(
                    topic_name, expected_frames
                )
            )

    tf_reference = contract_modules.get("tf")
    frame_module = (
        tf_reference.get("frame_contract")
        if isinstance(tf_reference, dict)
        else None
    )
    if not isinstance(frame_module, str) or not frame_module:
        errors.append("contract_modules.tf.frame_contract is missing")
    else:
        frame_path = (
            repository_root
            / "src"
            / "ros_architecture_pkg"
            / "config"
            / frame_module
        )
        try:
            frame_contract = load_contract(frame_path)
        except (ContractError, OSError, yaml.YAMLError) as error:
            errors.append("cannot load frame contract: {}".format(error))
            frame_contract = {}
        if frame_contract.get("root_frame") != EXPECTED_TF_ROOT_FRAME:
            errors.append(
                "TF root frame must remain {} for contract v1".format(
                    EXPECTED_TF_ROOT_FRAME
                )
            )
        dynamic_chain = tuple(
            (transform.get("parent"), transform.get("child"))
            for transform in frame_contract.get("transforms", [])
            if isinstance(transform, dict) and transform.get("type") == "dynamic"
        )
        if dynamic_chain != EXPECTED_DYNAMIC_TF_CHAIN:
            errors.append(
                "dynamic TF chain must remain map -> odom -> base_link for contract v1"
            )

    interface_policy = contract.get("public_interface_policy", {})
    for policy_name, expected_value in EXPECTED_PUBLIC_INTERFACE_POLICY.items():
        if interface_policy.get(policy_name) != expected_value:
            errors.append(
                "public interface policy {} must remain {!r} for contract v1".format(
                    policy_name, expected_value
                )
            )
    topic_namespace = interface_policy.get("topic_namespace")
    if not isinstance(topic_namespace, str) or not topic_namespace.startswith("/"):
        errors.append("public_interface_policy.topic_namespace must be absolute")
        topic_namespace = ""
    elif topic_namespace != APPROVED_TOPIC_NAMESPACE:
        errors.append(
            "public_interface_policy.topic_namespace must remain {} for contract v1".format(
                APPROVED_TOPIC_NAMESPACE
            )
        )
    internal_prefix = "{}/internal/".format(topic_namespace.rstrip("/"))
    public_prefix = "{}/".format(topic_namespace.rstrip("/"))
    for node_name in node_by_name:
        if interface_policy.get("node_entries_are_basenames_under_integrated_namespace"):
            if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", node_name):
                errors.append(
                    "node {} must be an exact ROS basename under the integrated namespace".format(
                        node_name
                    )
                )
    for topic_name, topic in topic_by_name.items():
        if (
            "//" in topic_name
            or topic_name.endswith("/")
            or not re.fullmatch(r"/[A-Za-z][A-Za-z0-9_/]*", topic_name)
        ):
            errors.append("topic {} is not a canonical absolute ROS name".format(topic_name))
        declared_internal = (
            topic.get("visibility") in ("internal", "package_internal")
            or topic.get("public") is False
        )
        name_is_internal = "/internal/" in topic_name
        if declared_internal:
            owner = str(topic.get("owner_package", ""))
            owner_segment = owner[:-4] if owner.endswith("_pkg") else owner
            owner_prefix = "{}{}/".format(internal_prefix, owner_segment)
            if topic_namespace and not topic_name.startswith(owner_prefix):
                errors.append(
                    "internal topic {} must use namespace {}".format(
                        topic_name, owner_prefix
                    )
                )
            for endpoint in topic.get("producers", []) + topic.get("consumers", []):
                endpoint_node = node_by_name.get(endpoint)
                if endpoint_node and endpoint_node.get("owner_package") != owner:
                    errors.append(
                        "internal topic {} crosses package boundary through {}".format(
                            topic_name, endpoint
                        )
                    )
        else:
            if name_is_internal:
                errors.append(
                    "public topic {} cannot use the internal namespace".format(topic_name)
                )
            elif topic_namespace and not topic_name.startswith(public_prefix):
                errors.append(
                    "public topic {} must use namespace {}".format(
                        topic_name, public_prefix
                    )
                )

    timestamp_reference = contract_modules.get("timestamp")
    timestamp_module = (
        timestamp_reference.get("timestamp_contract")
        if isinstance(timestamp_reference, dict)
        else None
    )
    timestamp_registry = {}
    if not isinstance(timestamp_module, str) or not timestamp_module:
        errors.append("contract_modules.timestamp.timestamp_contract is missing")
    else:
        timestamp_path = (
            repository_root
            / "src"
            / "ros_architecture_pkg"
            / "config"
            / timestamp_module
        )
        try:
            timestamp_contract = load_contract(timestamp_path)
        except (ContractError, OSError, yaml.YAMLError) as error:
            errors.append("cannot load timestamp contract: {}".format(error))
            timestamp_contract = {}
        timestamp_registry = timestamp_contract.get("timestamp_source_registry", {})
        if not isinstance(timestamp_registry, dict):
            errors.append("timestamp_source_registry must be a mapping")
            timestamp_registry = {}
        for source_name, source_policy in timestamp_registry.items():
            if not isinstance(source_policy, dict):
                errors.append(
                    "timestamp source {} policy must be a mapping".format(source_name)
                )
                continue
            allowed_topics = _as_string_list(
                source_policy.get("allowed_topics"),
                "timestamp_source_registry.{}.allowed_topics".format(source_name),
                errors,
            )
            if not allowed_topics:
                errors.append(
                    "timestamp source {} must scope at least one allowed topic".format(
                        source_name
                    )
                )
            for duplicate in _duplicates(allowed_topics):
                errors.append(
                    "timestamp source {} repeats allowed topic {}".format(
                        source_name, duplicate
                    )
                )
            for allowed_topic in allowed_topics:
                if allowed_topic not in topic_by_name:
                    errors.append(
                        "timestamp source {} allows unknown topic {}".format(
                            source_name, allowed_topic
                        )
                    )
                elif topic_by_name[allowed_topic].get("timestamp_source") != source_name:
                    errors.append(
                        "timestamp source {} requires topic {} to use it exactly".format(
                            source_name, allowed_topic
                        )
                    )
            if source_policy.get("kind") == "verified_external_raw" and not source_policy.get(
                "verification_scope"
            ):
                errors.append(
                    "verified external timestamp source {} needs verification_scope".format(
                        source_name
                    )
                )
        for derived_name, derived_policy in timestamp_contract.get(
            "derived_message_contract", {}
        ).items():
            stamp = derived_policy.get("stamp") if isinstance(derived_policy, dict) else None
            if stamp not in timestamp_registry:
                errors.append(
                    "derived timestamp {} uses unregistered source {}".format(
                        derived_name, stamp
                    )
                )

    for topic_name, topic in topic_by_name.items():
        source_name = topic.get("timestamp_source")
        source_policy = timestamp_registry.get(source_name)
        if source_policy is None:
            errors.append(
                "topic {} uses unregistered timestamp source {}".format(
                    topic_name, source_name
                )
            )
            continue
        allowed_topics = source_policy.get("allowed_topics")
        if allowed_topics is not None and topic_name not in allowed_topics:
            errors.append(
                "timestamp source {} is not allowed on topic {}".format(
                    source_name, topic_name
                )
            )
        allowed_owners = source_policy.get("allowed_owner_packages")
        if allowed_owners is not None and topic.get("owner_package") not in allowed_owners:
            errors.append(
                "timestamp source {} is not allowed for owner {}".format(
                    source_name, topic.get("owner_package")
                )
            )
        if source_policy.get("message_stamp_carried") is False and allowed_topics is None:
            errors.append(
                "headerless timestamp source {} must explicitly scope allowed_topics".format(
                    source_name
                )
            )

    message_entries = contract.get("messages")
    if not isinstance(message_entries, list):
        errors.append("messages must be a list")
        message_entries = []
    message_types = [
        entry.get("type") for entry in message_entries if isinstance(entry, dict)
    ]
    for duplicate in _duplicates(message_types):
        errors.append("duplicate message type: {}".format(duplicate))
    if any(not isinstance(name, str) or not name for name in message_types):
        errors.append("every message must have a non-empty string type")
    registered_message_types = set(message_types)
    for entry in message_entries:
        if isinstance(entry, dict) and not entry.get("status"):
            errors.append(
                "message type {} is missing status".format(entry.get("type"))
            )

    for node_name, node in node_by_name.items():
        if not node.get("owner_package"):
            errors.append("node {} is missing owner_package".format(node_name))
        if not node.get("status"):
            errors.append("node {} is missing status".format(node_name))
        if not node.get("visibility"):
            errors.append("node {} is missing visibility".format(node_name))
        elif node.get("visibility") not in ("public_boundary", "package_internal"):
            errors.append(
                "node {} has invalid visibility {}".format(
                    node_name, node.get("visibility")
                )
            )
        if node.get("visibility") == "public_boundary":
            _validate_korean_diagram_text(
                node.get("diagram_description_ko"),
                "node {}.diagram_description_ko".format(node_name),
                errors,
            )
        if node.get("external_transport_role") not in (
            None,
            "ingress_adapter",
            "egress_adapter",
        ):
            errors.append(
                "node {} has invalid external_transport_role {}".format(
                    node_name, node.get("external_transport_role")
                )
            )

    for topic_name, topic in topic_by_name.items():
        for field in contract.get("required_interface_fields", []):
            if field not in topic or topic[field] is None:
                errors.append(
                    "topic {} is missing required field {}".format(topic_name, field)
                )
        if not topic.get("owner_package"):
            errors.append("topic {} is missing owner_package".format(topic_name))
        if topic.get("visibility") not in ("public", "internal", "package_internal"):
            errors.append(
                "topic {} has invalid visibility {}".format(
                    topic_name, topic.get("visibility")
                )
            )
        if not _is_internal_topic(topic):
            _validate_korean_diagram_text(
                topic.get("diagram_description_ko"),
                "topic {}.diagram_description_ko".format(topic_name),
                errors,
            )
        if not topic.get("data_type"):
            errors.append("topic {} is missing data_type".format(topic_name))
        elif topic.get("data_type") not in registered_message_types:
            errors.append(
                "topic {} uses unregistered message type {}".format(
                    topic_name, topic.get("data_type")
                )
            )
        producers = _as_string_list(
            topic.get("producers"), "topic {}.producers".format(topic_name), errors
        )
        consumers = _as_string_list(
            topic.get("consumers"), "topic {}.consumers".format(topic_name), errors
        )
        if not producers:
            errors.append("topic {} has no producer node".format(topic_name))
        if (
            contract.get("public_interface_policy", {}).get(
                "single_producer_per_topic"
            )
            and len(producers) != 1
        ):
            errors.append(
                "topic {} violates single_producer_per_topic".format(topic_name)
            )
        for endpoint in producers + consumers:
            if endpoint not in node_by_name:
                errors.append(
                    "topic {} references unknown node {}".format(topic_name, endpoint)
                )
        for producer in producers:
            producer_entry = node_by_name.get(producer)
            if producer_entry and producer_entry.get("owner_package") != topic.get(
                "owner_package"
            ):
                errors.append(
                    "topic {} owner {} differs from producer {} owner {}".format(
                        topic_name,
                        topic.get("owner_package"),
                        producer,
                        producer_entry.get("owner_package"),
                    )
                )
        if _runtime_kind(topic.get("runtime_status", topic.get("status"))) == "live":
            non_live_producers = [
                producer
                for producer in producers
                if producer in node_by_name
                and _node_kind(node_by_name[producer]) != "live"
            ]
            if non_live_producers:
                errors.append(
                    "live topic {} has non-live producer {}".format(
                        topic_name, ", ".join(non_live_producers)
                    )
                )

    boundary_node_owner = {}
    for package_name, boundary in boundaries.items():
        if not isinstance(package_name, str) or not package_name:
            errors.append("package_boundaries keys must be non-empty strings")
            continue
        package_path = repository_root / "src" / package_name
        if not package_path.is_dir():
            errors.append("package directory does not exist: {}".format(package_name))
        if not isinstance(boundary, dict):
            errors.append("package boundary {} must be a mapping".format(package_name))
            continue
        if not boundary.get("runtime_status"):
            errors.append(
                "package boundary {} is missing runtime_status".format(package_name)
            )
        _validate_korean_diagram_text(
            boundary.get("diagram_summary_ko"),
            "package_boundaries.{}.diagram_summary_ko".format(package_name),
            errors,
        )
        public_nodes = _as_string_list(
            boundary.get("public_nodes"),
            "package_boundaries.{}.public_nodes".format(package_name),
            errors,
        )
        inputs = _as_string_list(
            boundary.get("inputs"),
            "package_boundaries.{}.inputs".format(package_name),
            errors,
        )
        outputs = _as_string_list(
            boundary.get("outputs"),
            "package_boundaries.{}.outputs".format(package_name),
            errors,
        )
        for duplicate in _duplicates(public_nodes):
            errors.append(
                "package {} repeats public node {}".format(package_name, duplicate)
            )
        for duplicate in _duplicates(inputs):
            errors.append("package {} repeats input {}".format(package_name, duplicate))
        for duplicate in _duplicates(outputs):
            errors.append(
                "package {} repeats output {}".format(package_name, duplicate)
            )

        for node_name in public_nodes:
            node = node_by_name.get(node_name)
            if node is None:
                errors.append(
                    "package {} references unknown public node {}".format(
                        package_name, node_name
                    )
                )
                continue
            if node.get("owner_package") != package_name:
                errors.append(
                    "package {} claims node {} owned by {}".format(
                        package_name, node_name, node.get("owner_package")
                    )
                )
            if node.get("visibility") != "public_boundary":
                errors.append(
                    "package {} exposes non-public node {} with visibility {}".format(
                        package_name, node_name, node.get("visibility")
                    )
                )
            previous_owner = boundary_node_owner.setdefault(node_name, package_name)
            if previous_owner != package_name:
                errors.append(
                    "public node {} appears in both {} and {}".format(
                        node_name, previous_owner, package_name
                    )
                )

        for direction, names in (("input", inputs), ("output", outputs)):
            for topic_name in names:
                topic = topic_by_name.get(topic_name)
                if topic is None:
                    errors.append(
                        "package {} references unknown {} topic {}".format(
                            package_name, direction, topic_name
                        )
                    )
                    continue
                if _is_internal_topic(topic):
                    errors.append(
                        "package {} exposes internal topic {} as {}".format(
                            package_name, topic_name, direction
                        )
                    )
                    continue
                endpoints = (
                    topic.get("consumers", [])
                    if direction == "input"
                    else topic.get("producers", [])
                )
                matching_nodes = [name for name in public_nodes if name in endpoints]
                if not matching_nodes:
                    errors.append(
                        "package {} {} {} has no matching public node endpoint".format(
                            package_name, direction, topic_name
                        )
                    )

        for topic_name in sorted(set(inputs) & set(outputs)):
            errors.append(
                "package {} lists {} as both input and output".format(
                    package_name, topic_name
                )
            )

    for node_name, node in node_by_name.items():
        owner_package = node.get("owner_package")
        if owner_package not in boundaries:
            errors.append(
                "node {} owner {} has no package boundary".format(
                    node_name, owner_package
                )
            )
        if (
            node.get("visibility") == "public_boundary"
            and boundary_node_owner.get(node_name) != owner_package
        ):
            errors.append(
                "public node {} is missing from {} public_nodes".format(
                    node_name, owner_package
                )
            )

    for topic_name, topic in topic_by_name.items():
        if _is_internal_topic(topic):
            continue
        owner_package = topic.get("owner_package")
        owner_boundary = boundaries.get(owner_package)
        if not isinstance(owner_boundary, dict):
            errors.append(
                "public topic {} owner {} has no package boundary".format(
                    topic_name, owner_package
                )
            )
        elif topic_name not in owner_boundary.get("outputs", []):
            errors.append(
                "public topic {} is missing from {} outputs".format(
                    topic_name, owner_package
                )
            )
        for consumer_name in topic.get("consumers", []):
            consumer = node_by_name.get(consumer_name)
            if consumer is None:
                continue
            consumer_package = consumer.get("owner_package")
            if consumer_package == owner_package:
                continue
            consumer_boundary = boundaries.get(consumer_package)
            if not isinstance(consumer_boundary, dict):
                errors.append(
                    "topic {} consumer package {} has no package boundary".format(
                        topic_name, consumer_package
                    )
                )
            elif topic_name not in consumer_boundary.get("inputs", []):
                errors.append(
                    "public topic {} is missing from {} inputs".format(
                        topic_name, consumer_package
                    )
                )

    # Contract v1 deliberately approves no service or action names.  Keep this
    # check in the validator rather than relying on an empty generated diagram:
    # otherwise a new RPC interface could bypass every topic/package-boundary
    # invariant above while ``--check`` still succeeds.
    for interface_kind in ("services", "actions"):
        if contract.get(interface_kind) != []:
            errors.append(
                "{} must remain an empty list until a v1 interface is approved".format(
                    interface_kind
                )
            )

    final_command_name = "/molit/safety/final_command"
    sender_name = "morai_control_sender"
    sender_public_inputs = [
        topic_name
        for topic_name, topic in topic_by_name.items()
        if not _is_internal_topic(topic) and sender_name in topic.get("consumers", [])
    ]
    if sender_public_inputs != [final_command_name]:
        errors.append(
            "morai_control_sender public inputs must be exactly {}".format(
                final_command_name
            )
        )
    morai_boundary = boundaries.get("morai_interface_pkg")
    if isinstance(morai_boundary, dict) and morai_boundary.get("inputs") != [
        final_command_name
    ]:
        errors.append(
            "morai_interface_pkg public inputs must be exactly {}".format(
                final_command_name
            )
        )

    evaluator_name = "runtime_evaluator_node"
    metrics_name = "/molit/evaluation/metrics"
    evaluator_outputs = [
        topic_name
        for topic_name, topic in topic_by_name.items()
        if evaluator_name in topic.get("producers", [])
    ]
    if evaluator_outputs != [metrics_name]:
        errors.append(
            "runtime_evaluator_node outputs must be exactly {}".format(metrics_name)
        )
    metrics = topic_by_name.get(metrics_name)
    if metrics is None:
        errors.append("missing required read-only runtime metrics topic")
    elif metrics.get("consumers") != []:
        errors.append(
            "runtime evaluation metrics must not feed any ROS node"
        )

    external_transport = contract.get("external_transport")
    if not isinstance(external_transport, dict):
        errors.append("external_transport must be a mapping")
    else:
        allowed_channels = _as_string_list(
            external_transport.get("allowed_logical_channels"),
            "external_transport.allowed_logical_channels",
            errors,
        )
        bindings = external_transport.get("diagram_bindings")
        if not isinstance(bindings, dict):
            errors.append("external_transport.diagram_bindings must be a mapping")
        else:
            bound_channels = []
            external_owner = external_transport.get("owner_package")
            expected_adapters = {
                direction: sorted(names)
                for direction, names in EXPECTED_EXTERNAL_ADAPTERS.items()
            }
            for direction in ("ingress", "egress"):
                role_adapters = sorted(
                    name
                    for name, node in node_by_name.items()
                    if node.get("external_transport_role")
                    == "{}_adapter".format(direction)
                )
                if role_adapters != expected_adapters[direction]:
                    errors.append(
                        "v1 external {} node roles must remain {}".format(
                            direction, ", ".join(expected_adapters[direction])
                        )
                    )
            for direction in ("ingress", "egress"):
                binding = bindings.get(direction)
                if not isinstance(binding, dict):
                    errors.append(
                        "external_transport.diagram_bindings.{} must be a mapping".format(
                            direction
                        )
                    )
                    continue
                if not binding.get("label"):
                    errors.append(
                        "external_transport.diagram_bindings.{} is missing label".format(
                            direction
                        )
                    )
                channels = _as_string_list(
                    binding.get("logical_channels"),
                    "external_transport.diagram_bindings.{}.logical_channels".format(
                        direction
                    ),
                    errors,
                )
                adapters = _as_string_list(
                    binding.get("adapter_nodes"),
                    "external_transport.diagram_bindings.{}.adapter_nodes".format(
                        direction
                    ),
                    errors,
                )
                if sorted(adapters) != expected_adapters[direction]:
                    errors.append(
                        "external {} adapters must exactly match node transport roles".format(
                            direction
                        )
                    )
                bound_channels.extend(channels)
                for channel in channels:
                    if channel not in allowed_channels:
                        errors.append(
                            "external {} uses non-allowlisted channel {}".format(
                                direction, channel
                            )
                        )
                for adapter_name in adapters:
                    adapter = node_by_name.get(adapter_name)
                    if adapter is None:
                        errors.append(
                            "external {} references unknown adapter node {}".format(
                                direction, adapter_name
                            )
                        )
                    elif adapter.get("owner_package") != external_owner:
                        errors.append(
                            "external {} adapter {} is not owned by {}".format(
                                direction, adapter_name, external_owner
                            )
                        )
            if sorted(bound_channels) != sorted(allowed_channels):
                errors.append(
                    "external diagram bindings must cover every allowed logical channel exactly once"
                )
            expected_egress_channels = ["Ego Ctrl Cmd"]
            if bindings.get("egress", {}).get("logical_channels") != expected_egress_channels:
                errors.append("external egress channel must be exactly Ego Ctrl Cmd")
            if sorted(bindings.get("ingress", {}).get("logical_channels", [])) != sorted(
                set(allowed_channels) - set(expected_egress_channels)
            ):
                errors.append(
                    "external ingress channels must be every allowlisted non-control channel"
                )

        final_command = topic_by_name.get("/molit/safety/final_command")
        if final_command is None:
            errors.append("missing required final safety command topic")
        else:
            if final_command.get("producers") != ["safety_supervisor_node"]:
                errors.append(
                    "final command producer must be exactly safety_supervisor_node"
                )
            watchdog_owner = final_command.get("consumer_watchdog_owner")
            if watchdog_owner != "morai_control_sender":
                errors.append(
                    "final command consumer watchdog owner must be morai_control_sender"
                )
            if watchdog_owner not in final_command.get("consumers", []):
                errors.append("final command watchdog owner must consume the final command")
            if not final_command.get("consumer_watchdog_timeout_sec"):
                errors.append("final command is missing consumer_watchdog_timeout_sec")
            timeout_policy = str(final_command.get("consumer_timeout_policy", ""))
            if timeout_policy != EXPECTED_FINAL_COMMAND_TIMEOUT_POLICY:
                errors.append(
                    "final command timeout policy must match the complete v1 fail-closed policy"
                )

        control_constraints = external_transport.get("control_constraints", {})
        consumer_gate = control_constraints.get("final_command_consumer_gate")
        if not isinstance(consumer_gate, dict):
            errors.append("external control constraints need final_command_consumer_gate")
        else:
            if consumer_gate.get("owner_node") != "morai_control_sender":
                errors.append(
                    "external final command consumer gate owner must be morai_control_sender"
                )
            required_checks = _as_string_list(
                consumer_gate.get("required_checks"),
                "external_transport.control_constraints.final_command_consumer_gate.required_checks",
                errors,
            )
            expected_checks = [
                "reject_zero_regressing_future_or_expired_command_stamp",
                "reject_nan_and_out_of_range_fields",
                "never_repeat_the_last_nonzero_command_after_input_loss",
                "serialize_only_a_packet_verified_fail_closed_stop_on_timeout",
            ]
            if required_checks != expected_checks:
                errors.append(
                    "external final command consumer gate required_checks must match the v1 safety set"
                )
            if not consumer_gate.get("enable_status"):
                errors.append(
                    "external final command consumer gate is missing enable_status"
                )

        sender = node_by_name.get("morai_control_sender")
        if sender is None:
            errors.append("missing morai_control_sender node")
        elif final_command is not None and isinstance(consumer_gate, dict):
            sender_is_live = _node_kind(sender) == "live"
            gate_status = str(consumer_gate.get("enable_status", ""))
            watchdog_timeout = final_command.get("consumer_watchdog_timeout_sec")
            if isinstance(watchdog_timeout, bool) or (
                isinstance(watchdog_timeout, (int, float))
                and (not math.isfinite(watchdog_timeout) or watchdog_timeout <= 0)
            ):
                errors.append(
                    "final command consumer watchdog timeout must be positive when numeric"
                )
            if sender_is_live:
                required_gate_markers = (
                    "enabled",
                    "packet_verified",
                    "watchdog_timeout_verified",
                )
                if not all(marker in gate_status for marker in required_gate_markers):
                    errors.append(
                        "live morai_control_sender needs an enabled packet-verified watchdog-verified gate"
                    )
                if not isinstance(watchdog_timeout, (int, float)) or isinstance(
                    watchdog_timeout, bool
                ) or watchdog_timeout <= 0:
                    errors.append(
                        "live morai_control_sender needs a positive numeric watchdog timeout"
                    )
                if _topic_kind(final_command, node_by_name) != "live":
                    errors.append(
                        "live morai_control_sender requires a live final command topic"
                    )
                executable = sender.get("executable")
                package_path = repository_root / "src" / sender["owner_package"]
                executable_candidates = (
                    package_path / "scripts" / str(executable),
                    package_path / "src" / str(executable),
                    package_path / "src" / "{}.py".format(executable),
                    package_path / "src" / "{}.cpp".format(executable),
                )
                if not executable or not any(path.is_file() for path in executable_candidates):
                    errors.append(
                        "live morai_control_sender executable is missing from its package"
                    )
            else:
                if gate_status != (
                    "prohibited_until_control_packet_and_watchdog_timeout_are_verified"
                ):
                    errors.append(
                        "non-live morai_control_sender gate must remain prohibited pending verification"
                    )
                if watchdog_timeout != "pending_safety_loop_jitter_measurement":
                    errors.append(
                        "non-live morai_control_sender watchdog timeout must remain pending measurement"
                    )

    architecture_views = contract.get("architecture_views")
    if not isinstance(architecture_views, dict):
        errors.append("architecture_views must be a mapping")
    else:
        if set(architecture_views) != set(EXPECTED_ARCHITECTURE_VIEW_OUTPUTS):
            errors.append(
                "architecture_views must contain exactly {}".format(
                    ", ".join(EXPECTED_ARCHITECTURE_VIEW_OUTPUTS)
                )
            )
        external_bindings = (
            external_transport.get("diagram_bindings", {})
            if isinstance(external_transport, dict)
            else {}
        )
        for view_name, expected_output in EXPECTED_ARCHITECTURE_VIEW_OUTPUTS.items():
            view = architecture_views.get(view_name)
            if not isinstance(view, dict):
                continue
            if not view.get("title"):
                errors.append("architecture view {} is missing title".format(view_name))
            if view.get("output_basename") != expected_output:
                errors.append(
                    "architecture view {} output_basename must remain {}".format(
                        view_name, expected_output
                    )
                )
            if view.get("direction") != "TB":
                errors.append(
                    "architecture view {} direction must remain TB for README readability".format(
                        view_name
                    )
                )
            view_nodes = _as_string_list(
                view.get("nodes"),
                "architecture_views.{}.nodes".format(view_name),
                errors,
            )
            view_topics = _as_string_list(
                view.get("topics"),
                "architecture_views.{}.topics".format(view_name),
                errors,
            )
            for duplicate in _duplicates(view_nodes):
                errors.append(
                    "architecture view {} repeats node {}".format(
                        view_name, duplicate
                    )
                )
            for duplicate in _duplicates(view_topics):
                errors.append(
                    "architecture view {} repeats topic {}".format(
                        view_name, duplicate
                    )
                )
            unknown_nodes = sorted(set(view_nodes) - set(node_by_name))
            unknown_topics = sorted(set(view_topics) - set(topic_by_name))
            for node_name in unknown_nodes:
                errors.append(
                    "architecture view {} references unknown node {}".format(
                        view_name, node_name
                    )
                )
            for topic_name in unknown_topics:
                errors.append(
                    "architecture view {} references unknown topic {}".format(
                        view_name, topic_name
                    )
                )

            external_channels = view.get("external_channels")
            if not isinstance(external_channels, dict):
                errors.append(
                    "architecture view {} external_channels must be a mapping".format(
                        view_name
                    )
                )
                external_channels = {}
            selected_node_set = set(view_nodes)
            for direction in ("ingress", "egress"):
                channels = _as_string_list(
                    external_channels.get(direction),
                    "architecture_views.{}.external_channels.{}".format(
                        view_name, direction
                    ),
                    errors,
                )
                binding = external_bindings.get(direction, {})
                allowed_for_direction = set(binding.get("logical_channels", []))
                for channel in channels:
                    if channel not in allowed_for_direction:
                        errors.append(
                            "architecture view {} uses {} channel {} outside the central binding".format(
                                view_name, direction, channel
                            )
                        )
                selected_adapters = [
                    node_name
                    for node_name in binding.get("adapter_nodes", [])
                    if node_name in selected_node_set
                ]
                if channels and not selected_adapters:
                    errors.append(
                        "architecture view {} has {} channels without an adapter node".format(
                            view_name, direction
                        )
                    )
                if selected_adapters and not channels:
                    errors.append(
                        "architecture view {} has {} adapter nodes without channels".format(
                            view_name, direction
                        )
                    )

            selected_topic_set = set(view_topics)
            layout_lanes = view.get("layout_lanes")
            if not isinstance(layout_lanes, list) or not layout_lanes:
                errors.append(
                    "architecture view {} layout_lanes must be a non-empty list".format(
                        view_name
                    )
                )
                layout_lanes = []
            layout_references = []
            allowed_layout_references = selected_node_set | selected_topic_set
            for lane_index, lane in enumerate(layout_lanes):
                lane_references = _as_string_list(
                    lane,
                    "architecture_views.{}.layout_lanes[{}]".format(
                        view_name, lane_index
                    ),
                    errors,
                )
                if len(lane_references) < 2:
                    errors.append(
                        "architecture view {} layout lane {} needs at least two entries".format(
                            view_name, lane_index
                        )
                    )
                layout_references.extend(lane_references)
                for reference in lane_references:
                    if reference not in allowed_layout_references:
                        errors.append(
                            "architecture view {} layout lane references unselected node/topic {}".format(
                                view_name, reference
                            )
                        )
            for duplicate in _duplicates(layout_references):
                errors.append(
                    "architecture view {} repeats layout reference {}".format(
                        view_name, duplicate
                    )
                )

            for topic_name in view_topics:
                topic = topic_by_name.get(topic_name)
                if topic is None:
                    continue
                if not any(
                    producer in selected_node_set
                    for producer in topic.get("producers", [])
                ):
                    errors.append(
                        "architecture view {} topic {} has no selected producer".format(
                            view_name, topic_name
                        )
                    )
            for node_name in view_nodes:
                node = node_by_name.get(node_name)
                if node is None:
                    continue
                topic_incident = any(
                    topic_name in selected_topic_set
                    and node_name
                    in topic_by_name[topic_name].get("producers", [])
                    + topic_by_name[topic_name].get("consumers", [])
                    for topic_name in view_topics
                    if topic_name in topic_by_name
                )
                external_incident = node.get("external_transport_role") in (
                    "ingress_adapter",
                    "egress_adapter",
                )
                if not topic_incident and not external_incident:
                    errors.append(
                        "architecture view {} node {} has no selected edge".format(
                            view_name, node_name
                        )
                    )

    return errors


def _escape_label(value):
    return (
        str(value)
        .replace("&", "&amp;")
        .replace('"', "&quot;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace("\n", " ")
    )


def _runtime_kind(status):
    normalized = str(status or "").lower()
    if any(marker in normalized for marker in NON_LIVE_STATUS_MARKERS):
        return "reserved"
    if any(marker in normalized for marker in LIVE_STATUS_MARKERS):
        return "live"
    return "reserved"


def _node_kind(node):
    return _runtime_kind(node.get("status"))


def _topic_kind(topic, node_by_name):
    explicit_status = topic.get("runtime_status", topic.get("status"))
    if explicit_status:
        return _runtime_kind(explicit_status)
    producers = [
        node_by_name[name]
        for name in topic.get("producers", [])
        if name in node_by_name
    ]
    if producers and all(_node_kind(node) == "live" for node in producers):
        return "live"
    return "reserved"


def _node_label(node):
    status = str(node.get("status", "status_missing"))
    executable = node.get("executable") or node.get("implementation_package")
    parts = [_escape_label(node["name"])]
    if executable:
        parts.append("exec: {}".format(_escape_label(executable)))
    parts.append("state: {} ({})".format(_node_kind(node).upper(), _escape_label(status)))
    return "<br/>".join(parts)


def _topic_label(topic, node_by_name):
    return "<br/>".join(
        (
            _escape_label(topic["name"]),
            "type: {}".format(_escape_label(topic["data_type"])),
            "state: {}".format(_topic_kind(topic, node_by_name).upper()),
        )
    )


def _package_status_label_ko(status):
    normalized = str(status or "").lower()
    if _runtime_kind(status) == "live":
        return "개발 환경 동작 확인"
    if "prohibited" in normalized:
        return "공식 규격 확인 전 사용 금지"
    if "disabled" in normalized:
        return "검증 전 비활성"
    return "설계 예약·미구현"


def _package_node_label(node):
    executable = node.get("executable") or node.get("implementation_package")
    parts = [
        _escape_label(node["name"]),
        "역할: {}".format(_escape_label(node["diagram_description_ko"])),
    ]
    if executable:
        parts.append("실행: {}".format(_escape_label(executable)))
    parts.append(
        "상태: {}".format(_package_status_label_ko(node.get("status")))
    )
    return "<br/>".join(parts)


def _package_topic_label(topic):
    return "<br/>".join(
        (
            _escape_label(topic["name"]),
            "설명: {}".format(_escape_label(topic["diagram_description_ko"])),
            "메시지: {}".format(_escape_label(topic["data_type"])),
            "상태: {}".format(
                _package_status_label_ko(
                    topic.get("runtime_status", topic.get("status"))
                )
            ),
        )
    )


def _style_definitions():
    return [
        "    classDef liveNode fill:#dcfce7,color:#14532d,stroke:#16a34a,stroke-width:2px;",
        "    classDef reservedNode fill:#f3f4f6,color:#374151,stroke:#9ca3af,stroke-width:2px,stroke-dasharray: 5 5;",
        "    classDef liveTopic fill:#dbeafe,color:#172554,stroke:#2563eb,stroke-width:2px;",
        "    classDef reservedTopic fill:#fff7ed,color:#7c2d12,stroke:#f97316,stroke-width:2px,stroke-dasharray: 5 5;",
        "    classDef emptyPort fill:#ffffff,color:#6b7280,stroke:#d1d5db,stroke-dasharray: 3 3;",
        "    classDef externalEndpoint fill:#f3e8ff,color:#581c87,stroke:#9333ea,stroke-width:3px;",
    ]


def render_system_architecture(contract):
    """Render every canonical node/topic edge, grouped by owning package."""
    nodes = contract["nodes"]
    topics = contract["topics"]
    boundaries = contract["package_boundaries"]
    node_by_name = {node["name"]: node for node in nodes}
    node_id = {node["name"]: "node_{:03d}".format(index) for index, node in enumerate(nodes)}
    topic_id = {
        topic["name"]: "topic_{:03d}".format(index)
        for index, topic in enumerate(topics)
    }

    lines = [GENERATED_HEADER.rstrip(), "flowchart LR"]
    lines.extend(_style_definitions())
    lines.append("")
    external_bindings = contract["external_transport"]["diagram_bindings"]
    ingress = external_bindings["ingress"]
    egress = external_bindings["egress"]
    lines.append('    subgraph external_transport["External MORAI transport boundary"]')
    lines.append(
        '        external_ingress["{}<br/>channels: {}"]'.format(
            _escape_label(ingress["label"]),
            _escape_label(", ".join(ingress["logical_channels"])),
        )
    )
    lines.append(
        '        external_egress["{}<br/>channels: {}"]'.format(
            _escape_label(egress["label"]),
            _escape_label(", ".join(egress["logical_channels"])),
        )
    )
    lines.append("    end")
    lines.append("    class external_ingress,external_egress externalEndpoint;")
    lines.append("")
    for package_index, (package_name, boundary) in enumerate(boundaries.items()):
        package_nodes = [
            node for node in nodes if node.get("owner_package") == package_name
        ]
        status = boundary.get("runtime_status", "status_missing")
        lines.append(
            '    subgraph pkg_{:03d}["{}<br/>runtime: {}"]'.format(
                package_index, _escape_label(package_name), _escape_label(status)
            )
        )
        if package_nodes:
            for node in package_nodes:
                lines.append(
                    '        {}["{}"]'.format(node_id[node["name"]], _node_label(node))
                )
        else:
            lines.append(
                '        pkg_empty_{:03d}["no runtime ROS node"]'.format(package_index)
            )
            lines.append(
                "        class pkg_empty_{:03d} emptyPort;".format(package_index)
            )
        lines.append("    end")
    lines.append("")

    for topic in topics:
        lines.append(
            '    {}["{}"]'.format(
                topic_id[topic["name"]], _topic_label(topic, node_by_name)
            )
        )
    lines.append("")
    for topic in topics:
        current_topic_id = topic_id[topic["name"]]
        for producer in topic.get("producers", []):
            lines.append("    {} --> {}".format(node_id[producer], current_topic_id))
        for consumer in topic.get("consumers", []):
            lines.append("    {} --> {}".format(current_topic_id, node_id[consumer]))
    for adapter_name in ingress["adapter_nodes"]:
        lines.append("    external_ingress --> {}".format(node_id[adapter_name]))
    for adapter_name in egress["adapter_nodes"]:
        lines.append("    {} --> external_egress".format(node_id[adapter_name]))
    lines.append("")
    for node in nodes:
        lines.append(
            "    class {} {}Node;".format(node_id[node["name"]], _node_kind(node))
        )
    for topic in topics:
        lines.append(
            "    class {} {}Topic;".format(
                topic_id[topic["name"]], _topic_kind(topic, node_by_name)
            )
        )
    lines.append("")
    return "\n".join(lines)


def _architecture_view_node_label(node):
    return "<br/>".join(
        (
            "node: {}".format(_escape_label(node["name"])),
            "package: {}".format(_escape_label(node["owner_package"])),
            "state: {}".format(_node_kind(node).upper()),
        )
    )


def render_architecture_view(view_name, view, contract):
    """Render one contract-curated, README-readable system projection."""
    node_by_name = {node["name"]: node for node in contract["nodes"]}
    topic_by_name = {topic["name"]: topic for topic in contract["topics"]}
    view_nodes = [node_by_name[name] for name in view["nodes"]]
    view_topics = [topic_by_name[name] for name in view["topics"]]
    selected_node_names = {node["name"] for node in view_nodes}
    node_id = {
        node["name"]: "view_node_{:03d}".format(index)
        for index, node in enumerate(view_nodes)
    }
    topic_id = {
        topic["name"]: "view_topic_{:03d}".format(index)
        for index, topic in enumerate(view_topics)
    }

    lines = [
        GENERATED_HEADER.rstrip(),
        "%% Curated architecture view: {}".format(_escape_label(view_name)),
        "flowchart {}".format(view["direction"]),
    ]
    lines.extend(_style_definitions())
    lines.append("")

    external_bindings = contract["external_transport"]["diagram_bindings"]
    external_channels = view["external_channels"]
    for direction in ("ingress", "egress"):
        channels = external_channels[direction]
        if not channels:
            continue
        binding = external_bindings[direction]
        lines.append(
            '    subgraph external_{}_boundary["External MORAI {} boundary"]'.format(
                direction, direction
            )
        )
        lines.append(
            '        external_{}["{}<br/>channels: {}"]'.format(
                direction,
                _escape_label(binding["label"]),
                _escape_label(", ".join(channels)),
            )
        )
        lines.append("    end")
        lines.append("    class external_{} externalEndpoint;".format(direction))
    lines.append("")

    for node in view_nodes:
        lines.append(
            '    {}["{}"]'.format(
                node_id[node["name"]], _architecture_view_node_label(node)
            )
        )
    lines.append("")
    for topic in view_topics:
        lines.append(
            '    {}["{}"]'.format(
                topic_id[topic["name"]], _topic_label(topic, node_by_name)
            )
        )
    lines.append("")

    for topic in view_topics:
        current_topic_id = topic_id[topic["name"]]
        for producer in topic.get("producers", []):
            if producer in selected_node_names:
                lines.append("    {} --> {}".format(node_id[producer], current_topic_id))
        for consumer in topic.get("consumers", []):
            if consumer in selected_node_names:
                lines.append("    {} --> {}".format(current_topic_id, node_id[consumer]))
    for direction in ("ingress", "egress"):
        if not external_channels[direction]:
            continue
        for adapter_name in external_bindings[direction]["adapter_nodes"]:
            if adapter_name not in selected_node_names:
                continue
            if direction == "ingress":
                lines.append(
                    "    external_ingress --> {}".format(node_id[adapter_name])
                )
            else:
                lines.append(
                    "    {} --> external_egress".format(node_id[adapter_name])
                )
    lines.append("    %% Invisible constraints keep README views in narrow lanes.")
    reference_id = dict(node_id)
    reference_id.update(topic_id)
    direct_edges = set()
    for topic in view_topics:
        for producer in topic.get("producers", []):
            if producer in selected_node_names:
                direct_edges.add((producer, topic["name"]))
        for consumer in topic.get("consumers", []):
            if consumer in selected_node_names:
                direct_edges.add((topic["name"], consumer))
    for lane in view["layout_lanes"]:
        for source, target in zip(lane, lane[1:]):
            if (source, target) not in direct_edges:
                lines.append(
                    "    {} ~~~ {}".format(
                        reference_id[source], reference_id[target]
                    )
                )
    lines.append("")

    for node in view_nodes:
        lines.append(
            "    class {} {}Node;".format(
                node_id[node["name"]], _node_kind(node)
            )
        )
    for topic in view_topics:
        lines.append(
            "    class {} {}Topic;".format(
                topic_id[topic["name"]], _topic_kind(topic, node_by_name)
            )
        )
    lines.append("")
    return "\n".join(lines)


def render_package_interface(package_name, boundary, contract):
    """Render only one package's declared public inputs and outputs."""
    node_by_name = {node["name"]: node for node in contract["nodes"]}
    topic_by_name = {topic["name"]: topic for topic in contract["topics"]}
    public_nodes = [node_by_name[name] for name in boundary["public_nodes"]]
    input_topics = [topic_by_name[name] for name in boundary["inputs"]]
    output_topics = [topic_by_name[name] for name in boundary["outputs"]]
    local_node_ids = {
        node["name"]: "local_node_{:03d}".format(index)
        for index, node in enumerate(public_nodes)
    }
    input_ids = {
        topic["name"]: "input_{:03d}".format(index)
        for index, topic in enumerate(input_topics)
    }
    output_ids = {
        topic["name"]: "output_{:03d}".format(index)
        for index, topic in enumerate(output_topics)
    }

    if not public_nodes:
        lines = [GENERATED_HEADER.rstrip(), "flowchart LR"]
        lines.extend(_style_definitions())
        lines.extend(
            [
                "",
                '    no_inputs["입력 (구독)<br/>런타임 ROS 입력 없음"]',
                '    package["패키지 처리<br/>{}<br/>역할: {}<br/>런타임 ROS 노드·토픽 없음"]'.format(
                    _escape_label(package_name),
                    _escape_label(boundary["diagram_summary_ko"]),
                ),
                '    no_outputs["출력 (발행)<br/>런타임 ROS 출력 없음"]',
                "    no_inputs -.-> package -.-> no_outputs",
                "    class no_inputs,package,no_outputs emptyPort;",
                "",
            ]
        )
        return "\n".join(lines)

    lines = [GENERATED_HEADER.rstrip(), "flowchart LR"]
    lines.extend(_style_definitions())
    lines.extend(
        [
            "",
            '    subgraph INPUTS["입력 (구독)"]',
        ]
    )
    if input_topics:
        for topic in input_topics:
            lines.append(
                '        {}["{}"]'.format(
                    input_ids[topic["name"]], _package_topic_label(topic)
                )
            )
    else:
        lines.append('        no_inputs["런타임 ROS 입력 없음"]')
        lines.append("        class no_inputs emptyPort;")
    lines.append("    end")
    lines.append(
        '    subgraph PACKAGE["패키지 처리<br/>{}<br/>{}"]'.format(
            _escape_label(package_name),
            _escape_label(boundary["diagram_summary_ko"]),
        )
    )
    if public_nodes:
        for node in public_nodes:
            lines.append(
                '        {}["{}"]'.format(
                    local_node_ids[node["name"]], _package_node_label(node)
                )
            )
    else:
        lines.append('        no_nodes["no runtime ROS node"]')
        lines.append("        class no_nodes emptyPort;")
    lines.append("    end")
    lines.append('    subgraph OUTPUTS["출력 (발행)"]')
    if output_topics:
        for topic in output_topics:
            lines.append(
                '        {}["{}"]'.format(
                    output_ids[topic["name"]], _package_topic_label(topic)
                )
            )
    else:
        lines.append('        no_outputs["런타임 ROS 출력 없음"]')
        lines.append("        class no_outputs emptyPort;")
    lines.append("    end")
    lines.append("")

    for topic in input_topics:
        for consumer in topic.get("consumers", []):
            if consumer in local_node_ids:
                lines.append(
                    "    {} --> {}".format(
                        input_ids[topic["name"]], local_node_ids[consumer]
                    )
                )
    for topic in output_topics:
        for producer in topic.get("producers", []):
            if producer in local_node_ids:
                lines.append(
                    "    {} --> {}".format(
                        local_node_ids[producer], output_ids[topic["name"]]
                    )
                )
    lines.append("")
    for node in public_nodes:
        lines.append(
            "    class {} {}Node;".format(
                local_node_ids[node["name"]], _node_kind(node)
            )
        )
    for topic in input_topics:
        lines.append(
            "    class {} {}Topic;".format(
                input_ids[topic["name"]], _topic_kind(topic, node_by_name)
            )
        )
    for topic in output_topics:
        lines.append(
            "    class {} {}Topic;".format(
                output_ids[topic["name"]], _topic_kind(topic, node_by_name)
            )
        )
    lines.append("")
    return "\n".join(lines)


def build_documents(contract, repository_root):
    """Return deterministic output-path/content pairs for every Mermaid view."""
    documents = {
        repository_root
        / "src"
        / "ros_architecture_pkg"
        / "docs"
        / "system_architecture.mmd": render_system_architecture(contract)
    }
    for view_name, view in contract["architecture_views"].items():
        documents[
            repository_root
            / "src"
            / "ros_architecture_pkg"
            / "docs"
            / "{}.mmd".format(view["output_basename"])
        ] = render_architecture_view(view_name, view, contract)
    for package_name, boundary in contract["package_boundaries"].items():
        documents[
            repository_root / "src" / package_name / "docs" / "interface_io.mmd"
        ] = render_package_interface(package_name, boundary, contract)
    return documents


def _check_documents(documents, repository_root):
    mismatches = []
    for path, expected in documents.items():
        display_path = path.relative_to(repository_root)
        if not path.is_file():
            mismatches.append("missing generated file: {}".format(display_path))
            continue
        actual = path.read_text(encoding="utf-8")
        if actual != expected:
            mismatch = ["outdated generated file: {}".format(display_path)]
            mismatch.extend(
                difflib.unified_diff(
                    actual.splitlines(),
                    expected.splitlines(),
                    fromfile=str(display_path),
                    tofile="{} (expected)".format(display_path),
                    lineterm="",
                    n=2,
                )
            )
            mismatches.append("\n".join(mismatch))
    return mismatches


def _write_documents(documents, repository_root):
    for path, content in documents.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        print("wrote {}".format(path.relative_to(repository_root)))


def _file_sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_render_manifest(documents, repository_root):
    """Hash each generated source and rendered artifact as one reviewable set."""
    contract_path = (
        repository_root
        / "src"
        / "ros_architecture_pkg"
        / "config"
        / "interface_contract.yaml"
    )
    renderer_config_path = (
        repository_root
        / "src"
        / "ros_architecture_pkg"
        / "config"
        / "mermaid_renderer.json"
    )
    for required_path in (contract_path, renderer_config_path):
        if not required_path.is_file():
            raise ContractError(
                "missing render input: {}".format(
                    required_path.relative_to(repository_root)
                )
            )
    artifacts = []
    for source_path in sorted(documents, key=lambda path: str(path)):
        svg_path = source_path.with_suffix(".svg")
        png_path = source_path.with_suffix(".png")
        for artifact_path in (source_path, svg_path, png_path):
            if not artifact_path.is_file():
                raise ContractError(
                    "missing render artifact: {}".format(
                        artifact_path.relative_to(repository_root)
                    )
                )
        artifacts.append(
            {
                "source": str(source_path.relative_to(repository_root)),
                "source_sha256": _file_sha256(source_path),
                "svg": str(svg_path.relative_to(repository_root)),
                "svg_sha256": _file_sha256(svg_path),
                "png": str(png_path.relative_to(repository_root)),
                "png_sha256": _file_sha256(png_path),
            }
        )
    return {
        "schema_version": 1,
        "renderer": "@mermaid-js/mermaid-cli@{}".format(MERMAID_CLI_VERSION),
        "source_of_truth": str(contract_path.relative_to(repository_root)),
        "source_of_truth_sha256": _file_sha256(contract_path),
        "renderer_config": str(renderer_config_path.relative_to(repository_root)),
        "renderer_config_sha256": _file_sha256(renderer_config_path),
        "artifacts": artifacts,
    }


def _manifest_text(manifest):
    return json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n"


def _check_render_manifest(documents, repository_root):
    try:
        expected = build_render_manifest(documents, repository_root)
    except (ContractError, OSError) as error:
        return [str(error)]
    manifest_path = (
        repository_root
        / "src"
        / "ros_architecture_pkg"
        / "docs"
        / "interface_diagram_manifest.json"
    )
    if not manifest_path.is_file():
        return [
            "missing render manifest: {}".format(
                manifest_path.relative_to(repository_root)
            )
        ]
    try:
        actual = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return ["invalid render manifest: {}".format(error)]
    if actual != expected:
        return [
            "rendered SVG/PNG hashes do not match the Mermaid render manifest"
        ]
    return []


def _write_render_manifest(documents, repository_root):
    manifest = build_render_manifest(documents, repository_root)
    manifest_path = (
        repository_root
        / "src"
        / "ros_architecture_pkg"
        / "docs"
        / "interface_diagram_manifest.json"
    )
    manifest_path.write_text(_manifest_text(manifest), encoding="utf-8")
    print("wrote {}".format(manifest_path.relative_to(repository_root)))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Generate Mermaid diagrams from the central ROS interface contract."
    )
    parser.add_argument(
        "--contract",
        type=Path,
        default=DEFAULT_CONTRACT_PATH,
        help="central interface contract YAML",
    )
    parser.add_argument(
        "--repository-root",
        type=Path,
        default=DEFAULT_REPOSITORY_ROOT,
        help="repository root containing src/<package>",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail when Mermaid or rendered image hashes are out of date",
    )
    parser.add_argument(
        "--write-render-manifest",
        action="store_true",
        help="record hashes after the pinned renderer has produced SVG and PNG files",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    contract_path = args.contract.resolve()
    repository_root = args.repository_root.resolve()
    try:
        contract = load_contract(contract_path)
        errors = validate_contract(contract, repository_root)
        if errors:
            raise ContractError("\n".join("- {}".format(error) for error in errors))
        documents = build_documents(contract, repository_root)
    except (ContractError, OSError, yaml.YAMLError) as error:
        print("interface diagram generation failed:\n{}".format(error), file=sys.stderr)
        return 2

    if args.check:
        mismatches = _check_documents(documents, repository_root)
        if not mismatches:
            mismatches.extend(_check_render_manifest(documents, repository_root))
        if mismatches:
            print("\n\n".join(mismatches), file=sys.stderr)
            print(
                "regenerate Mermaid files, then run render_interface_diagrams.sh to refresh images",
                file=sys.stderr,
            )
            return 1
        print("interface Mermaid and rendered image hashes match the central contract")
        return 0


    if args.write_render_manifest:
        mismatches = _check_documents(documents, repository_root)
        if mismatches:
            print("\n\n".join(mismatches), file=sys.stderr)
            return 1
        try:
            _write_render_manifest(documents, repository_root)
        except (ContractError, OSError) as error:
            print("render manifest generation failed:\n{}".format(error), file=sys.stderr)
            return 2
        return 0

    _write_documents(documents, repository_root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
