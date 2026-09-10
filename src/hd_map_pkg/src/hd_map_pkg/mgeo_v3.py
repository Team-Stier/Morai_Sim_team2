"""Strict reader for the JSON-based MORAI MGeo 3.0 map bundle."""

import ast
import hashlib
import json
import math
import re
from collections import defaultdict
from pathlib import Path


DATASET_FILES = (
    "crosswalk_set.json",
    "global_info.json",
    "intersection_controller_data.json",
    "intersection_controller_set.json",
    "junction_group_set.json",
    "junction_set.json",
    "lane_boundary_set.json",
    "lane_node_set.json",
    "link_set.json",
    "node_set.json",
    "object_set.json",
    "parking_space_set.json",
    "road_polygon_set.json",
    "road_set.json",
    "singlecrosswalk_set.json",
    "surface_marking_set.json",
    "synced_traffic_light_set.json",
    "traffic_light_set.json",
    "traffic_sign_set.json",
)

REQUIRED_RECORD_FIELDS = {
    "node_set": ("idx", "point"),
    "link_set": ("idx", "from_node_idx", "to_node_idx", "points"),
    "lane_node_set": ("idx", "point"),
    "lane_boundary_set": ("idx", "from_node_idx", "to_node_idx", "points",
                          "lane_type", "lane_shape", "lane_color"),
    "traffic_light_set": ("idx", "point", "type"),
    "singlecrosswalk_set": ("idx", "points", "sign_type"),
    "crosswalk_set": ("idx", "single_crosswalk_list", "ref_traffic_light_list"),
    "junction_set": ("idx", "road_id_list"),
    "road_set": ("idx", "links"),
    "intersection_controller_set": ("idx", "TL"),
    "intersection_controller_data": ("idx", "synced_light", "phase"),
    "synced_traffic_light_set": ("idx", "intersection_controller_id",
                                 "signal_id_list"),
}


class MGeoImportError(ValueError):
    """Raised when a source bundle does not satisfy the MGeo 3.0 contract."""


def _index(items, label):
    if not isinstance(items, list):
        raise MGeoImportError("{} root must be a list".format(label))
    result = {}
    duplicates = []
    for item in items:
        if not isinstance(item, dict):
            raise MGeoImportError("{} contains a non-object record".format(label))
        raw_identifier = item.get("idx")
        if raw_identifier is None or not str(raw_identifier).strip():
            raise MGeoImportError("{} contains an item without idx".format(label))
        identifier = str(raw_identifier)
        if identifier in result:
            duplicates.append(identifier)
        result[identifier] = item
    if duplicates:
        raise MGeoImportError("{} contains duplicate IDs: {}".format(label, duplicates[:10]))
    return result


_SUFFIX = re.compile(r"^(?P<base>.+)_(?P<number>[0-9]+)$")


def _copy_on_write_normalize(value, aliases, parent_key=""):
    """Normalize verified ID aliases while sharing large immutable point arrays."""
    if isinstance(value, str):
        return aliases.get(value, value)
    if isinstance(value, list):
        if parent_key in ("points", "point", "center_point", "line_coeff"):
            return value
        normalized = [_copy_on_write_normalize(item, aliases, parent_key) for item in value]
        return value if all(first is second for first, second in zip(value, normalized)) else normalized
    if isinstance(value, dict):
        changed = False
        normalized = {}
        for key, item in value.items():
            normalized_key = aliases.get(key, key) if isinstance(key, str) else key
            normalized_item = _copy_on_write_normalize(item, aliases, str(key))
            changed = changed or normalized_key != key or normalized_item is not item
            normalized[normalized_key] = normalized_item
        return normalized if changed else value
    return value


def _verified_suffix_aliases(data):
    """Find only suffix IDs whose recursively normalized record equals its base."""
    indices = {}
    potential_targets = defaultdict(set)
    for collection, items in data.items():
        if not isinstance(items, list) or not items or "idx" not in items[0]:
            continue
        index = {str(item["idx"]): item for item in items}
        indices[collection] = index
        for identifier in index:
            match = _SUFFIX.match(identifier)
            if match and match.group("base") in index:
                potential_targets[identifier].add(match.group("base"))

    ambiguous = {alias for alias, targets in potential_targets.items() if len(targets) != 1}
    potential = {alias: next(iter(targets)) for alias, targets in potential_targets.items()
                 if alias not in ambiguous}

    verified = dict(potential)
    changed = True
    while changed:
        changed = False
        for collection, index in indices.items():
            for alias in list(verified):
                base = verified[alias]
                if alias not in index or base not in index:
                    continue
                first = _copy_on_write_normalize(index[alias], verified)
                second = _copy_on_write_normalize(index[base], verified)
                if first != second:
                    del verified[alias]
                    changed = True
    collisions = sorted((set(potential) - set(verified)) | ambiguous)
    return verified, collisions


class MGeoV3Dataset(object):
    """Read-only-by-convention view of one on-disk MGeo 3.0 snapshot."""

    def __init__(self, root, expected_major=3, deduplicate_verified_suffix_clones=True):
        self.root = Path(root).resolve()
        missing = [name for name in DATASET_FILES if not (self.root / name).is_file()]
        if missing:
            raise MGeoImportError("missing MGeo files: {}".format(", ".join(missing)))
        self.raw_bytes = {}
        raw_data = {}
        for filename in DATASET_FILES:
            payload = (self.root / filename).read_bytes()
            self.raw_bytes[filename] = payload
            try:
                raw_data[filename[:-5]] = json.loads(payload.decode("utf-8-sig"))
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                raise MGeoImportError("cannot parse {}: {}".format(filename, error))
        self.raw_data = raw_data
        if not isinstance(raw_data.get("global_info"), dict):
            raise MGeoImportError("global_info root must be an object")
        for collection, value in raw_data.items():
            if collection == "global_info":
                continue
            if not isinstance(value, list):
                raise MGeoImportError("{} root must be a list".format(collection))
            required = REQUIRED_RECORD_FIELDS.get(collection, ("idx",))
            for position, item in enumerate(value):
                if not isinstance(item, dict):
                    raise MGeoImportError(
                        "{}[{}] must be an object".format(collection, position))
                missing_fields = [field for field in required if field not in item]
                if missing_fields:
                    raise MGeoImportError("{}[{}] missing fields: {}".format(
                        collection, position, ", ".join(missing_fields)))
                if item.get("idx") is None or not str(item.get("idx")).strip():
                    raise MGeoImportError("{}[{}] has an invalid idx".format(
                        collection, position))
        self.verified_aliases, self.duplicate_collisions = _verified_suffix_aliases(raw_data)
        if not deduplicate_verified_suffix_clones:
            self.verified_aliases = {}
        self.aliases_by_canonical_id = defaultdict(list)
        for alias, canonical in sorted(self.verified_aliases.items()):
            self.aliases_by_canonical_id[canonical].append(alias)
        self.data = {}
        for collection, value in raw_data.items():
            if isinstance(value, list) and value and "idx" in value[0]:
                self.data[collection] = [
                    _copy_on_write_normalize(item, self.verified_aliases)
                    for item in value if str(item["idx"]) not in self.verified_aliases
                ]
            else:
                self.data[collection] = value
        self.global_info = self.data["global_info"]
        if int(self.global_info.get("maj_ver", -1)) != int(expected_major):
            raise MGeoImportError(
                "expected MGeo major {}, got {}".format(expected_major, self.global_info.get("maj_ver")))

        self.nodes = _index(self.data["node_set"], "node_set")
        self.links = _index(self.data["link_set"], "link_set")
        self.lane_nodes = _index(self.data["lane_node_set"], "lane_node_set")
        self.lane_boundaries = _index(self.data["lane_boundary_set"], "lane_boundary_set")
        self.traffic_lights = _index(self.data["traffic_light_set"], "traffic_light_set")
        self.traffic_signs = _index(self.data["traffic_sign_set"], "traffic_sign_set")
        self.single_crosswalks = _index(self.data["singlecrosswalk_set"], "singlecrosswalk_set")
        self.surface_markings = _index(self.data["surface_marking_set"], "surface_marking_set")
        self.crosswalks = _index(self.data["crosswalk_set"], "crosswalk_set")
        self.junctions = _index(self.data["junction_set"], "junction_set")
        self.roads = _index(self.data["road_set"], "road_set")
        self.controllers = _index(self.data["intersection_controller_set"], "intersection_controller_set")
        self.controller_data = _index(
            self.data["intersection_controller_data"], "intersection_controller_data")
        self.synced_lights = _index(
            self.data["synced_traffic_light_set"], "synced_traffic_light_set")

        self.successors = defaultdict(list)
        self.predecessors = defaultdict(list)
        links_from_node = defaultdict(list)
        links_to_node = defaultdict(list)
        for link_id, link in self.links.items():
            links_from_node[str(link["from_node_idx"])].append(link_id)
            links_to_node[str(link["to_node_idx"])].append(link_id)
        for link_id, link in self.links.items():
            self.successors[link_id] = sorted(links_from_node[str(link["to_node_idx"])])
            self.predecessors[link_id] = sorted(links_to_node[str(link["from_node_idx"])])

        self.links_by_road = defaultdict(list)
        for link_id, link in self.links.items():
            self.links_by_road[str(link.get("road_id", ""))].append(link_id)
        for value in self.links_by_road.values():
            value.sort()

    @property
    def local_origin_utm(self):
        return tuple(float(value) for value in self.global_info["local_origin_in_global"])

    def canonical_id(self, identifier):
        return self.verified_aliases.get(str(identifier), str(identifier))

    def aliases_for(self, identifier):
        return list(self.aliases_by_canonical_id.get(str(identifier), []))

    def source_hashes(self):
        """Return raw and CRLF-canonical SHA-256 hashes without touching source files."""
        result = {}
        for filename, payload in sorted(self.raw_bytes.items()):
            lf_payload = payload.replace(b"\r\n", b"\n")
            crlf_payload = lf_payload.replace(b"\n", b"\r\n")
            result[filename] = {
                "sha256_raw": hashlib.sha256(payload).hexdigest(),
                "sha256_crlf": hashlib.sha256(crlf_payload).hexdigest(),
                "bytes": len(payload),
            }
        return result

    def declared_hashes(self):
        value = self.global_info.get("mgeo_file_hash", "{}")
        if isinstance(value, dict):
            return dict(value)
        try:
            parsed = ast.literal_eval(value)
        except (SyntaxError, ValueError) as error:
            raise MGeoImportError("invalid global_info mgeo_file_hash: {}".format(error))
        if not isinstance(parsed, dict):
            raise MGeoImportError("global_info mgeo_file_hash is not a mapping")
        return parsed

    def traffic_light_link_ids(self):
        """Resolve signal-group control to incoming vehicle approach links.

        Most KATRI signal heads have no direct link list.  MGeo instead binds a
        synchronized group to signal-bearing stop nodes.  Incoming links at any
        such node are the controlled approaches; direct group links are connector
        links, whose predecessors are also approaches.
        """
        result = {identifier: set() for identifier in self.traffic_lights}
        links_to_node = defaultdict(list)
        for link_id, link in self.links.items():
            links_to_node[str(link["to_node_idx"])].append(link_id)
        nodes_by_signal = defaultdict(list)
        for node_id, node in self.nodes.items():
            signal_id = node.get("traffic_light_id")
            if signal_id in self.traffic_lights:
                nodes_by_signal[str(signal_id)].append(node_id)

        for group in self.synced_lights.values():
            signal_ids = [str(value) for value in group.get("signal_id_list") or []
                          if str(value) in self.traffic_lights]
            approaches = set()
            for signal_id in signal_ids:
                for node_id in nodes_by_signal.get(signal_id, []):
                    approaches.update(links_to_node[node_id])
            direct_connectors = {str(value) for value in (group.get("link_id_list") or [])
                                 if str(value) in self.links}
            for connector_id in direct_connectors:
                approaches.update(self.predecessors.get(connector_id, []))
            if not approaches:
                approaches.update(direct_connectors)
            for signal_id in signal_ids:
                if signal_id in result:
                    signal = self.traffic_lights[signal_id]
                    if str(signal.get("type", "")).lower() in ("car", "bus"):
                        heading = float(signal.get("heading") or 0.0) % 360.0
                        facing = set()
                        for link_id in approaches:
                            points = self.links[link_id].get("points") or []
                            if len(points) < 2:
                                continue
                            tangent = math.degrees(math.atan2(
                                points[-1][1] - points[-2][1],
                                points[-1][0] - points[-2][0])) % 360.0
                            difference = abs((heading - tangent + 180.0) % 360.0 - 180.0)
                            if abs(180.0 - difference) <= 60.0:
                                facing.add(link_id)
                        result[signal_id].update(facing)
                    else:
                        result[signal_id].update(approaches)

        # Keep direct links as a fallback for data not covered by a synced group,
        # translating connector links to their incoming predecessors where possible.
        for signal_id, signal in self.traffic_lights.items():
            for direct_id in signal.get("link_id_list") or []:
                if direct_id not in self.links:
                    continue
                predecessors = self.predecessors.get(str(direct_id), [])
                result[signal_id].update(predecessors or [str(direct_id)])
        return {key: sorted(value for value in values if value in self.links)
                for key, values in result.items()}

    def traffic_light_crosswalk_ids(self):
        """Return canonical single-crosswalk IDs controlled by each signal head."""
        group_members = {group_id: [str(value) for value in group.get("single_crosswalk_list") or []]
                         for group_id, group in self.crosswalks.items()}
        result = {signal_id: set() for signal_id in self.traffic_lights}
        for signal_id, signal in self.traffic_lights.items():
            group_id = str(signal.get("ref_crosswalk_id") or "")
            result[signal_id].update(group_members.get(group_id, []))
        for group_id, group in self.crosswalks.items():
            members = group_members.get(group_id, [])
            for signal_id in group.get("ref_traffic_light_list") or []:
                if signal_id in result:
                    result[signal_id].update(members)
        return {signal_id: sorted(value for value in values if value in self.single_crosswalks)
                for signal_id, values in result.items()}

    def reference_errors(self):
        """Check all references used by the conversion, returning stable diagnostics."""
        errors = ["non-identical suffix collision {}".format(identifier)
                  for identifier in self.duplicate_collisions]

        def canonical(identifier):
            return self.canonical_id(identifier)

        def nested_identifiers(value):
            """Yield IDs from the scalar/list/nested-list fields used by MGeo."""
            if isinstance(value, list):
                for item in value:
                    for identifier in nested_identifiers(item):
                        yield identifier
            else:
                yield value

        def missing(identifier, target, optional=False):
            if identifier is None or not str(identifier).strip():
                return not optional
            return canonical(identifier) not in target

        for link_id, link in sorted(self.links.items()):
            for field in ("from_node_idx", "to_node_idx"):
                if missing(link.get(field), self.nodes):
                    errors.append("link {}: missing {} {}".format(link_id, field, link.get(field)))
            for field in ("lane_mark_left", "lane_mark_right"):
                for boundary_id in link.get(field) or []:
                    if missing(boundary_id, self.lane_boundaries):
                        errors.append("link {}: missing {} {}".format(link_id, field, boundary_id))
            for field in ("left_lane_change_dst_link_idx", "right_lane_change_dst_link_idx"):
                value = link.get(field)
                if missing(value, self.links, optional=True):
                    errors.append("link {}: missing {} {}".format(link_id, field, value))
            road_id = link.get("road_id")
            if missing(road_id, self.roads):
                errors.append("link {}: missing road {}".format(link_id, road_id))
        for boundary_id, boundary in sorted(self.lane_boundaries.items()):
            for field in ("from_node_idx", "to_node_idx"):
                if missing(boundary.get(field), self.lane_nodes):
                    errors.append("boundary {}: missing {} {}".format(
                        boundary_id, field, boundary.get(field)))
        for node_id, node in sorted(self.nodes.items()):
            signal_id = node.get("traffic_light_id")
            if missing(signal_id, self.traffic_lights, optional=True):
                errors.append("node {}: missing traffic_light {}".format(
                    node_id, signal_id))
        for signal_id, signal in sorted(self.traffic_lights.items()):
            for link_id in signal.get("link_id_list") or []:
                if missing(link_id, self.links):
                    errors.append("traffic light {}: missing link {}".format(
                        signal_id, link_id))
            crosswalk_id = signal.get("ref_crosswalk_id")
            if missing(crosswalk_id, self.crosswalks, optional=True):
                errors.append("traffic light {}: missing ref_crosswalk {}".format(
                    signal_id, crosswalk_id))
        for crossing_id, crossing in sorted(self.single_crosswalks.items()):
            for link_id in crossing.get("link_id_list") or []:
                # Some MGeo bundles use an empty string as an explicit no-link value.
                if missing(link_id, self.links, optional=True):
                    errors.append("single crosswalk {}: missing link {}".format(
                        crossing_id, link_id))
        for crossing_id, crossing in sorted(self.crosswalks.items()):
            for single_id in crossing.get("single_crosswalk_list") or []:
                if missing(single_id, self.single_crosswalks):
                    errors.append("crosswalk {}: missing single_crosswalk {}".format(
                        crossing_id, single_id))
            for signal_id in crossing.get("ref_traffic_light_list") or []:
                if missing(signal_id, self.traffic_lights):
                    errors.append("crosswalk {}: missing traffic_light {}".format(
                        crossing_id, signal_id))
        for marking_id, marking in sorted(self.surface_markings.items()):
            for link_id in marking.get("link_id_list") or []:
                if missing(link_id, self.links, optional=True):
                    errors.append("surface marking {}: missing link {}".format(
                        marking_id, link_id))
        for group_id, group in sorted(self.synced_lights.items()):
            for link_id in group.get("link_id_list") or []:
                if missing(link_id, self.links):
                    errors.append("synced traffic light {}: missing link {}".format(
                        group_id, link_id))
            for signal_id in group.get("signal_id_list") or []:
                if missing(signal_id, self.traffic_lights):
                    errors.append("synced traffic light {}: missing traffic_light {}".format(
                        group_id, signal_id))
            controller_id = group.get("intersection_controller_id")
            if missing(controller_id, self.controllers):
                errors.append("synced traffic light {}: missing controller {}".format(
                    group_id, controller_id))
        for controller_id, controller in sorted(self.controllers.items()):
            for signal_id in nested_identifiers(controller.get("TL") or []):
                if missing(signal_id, self.traffic_lights):
                    errors.append("controller {}: missing traffic_light {}".format(
                        controller_id, signal_id))
        for controller_id, data in sorted(self.controller_data.items()):
            if missing(controller_id, self.controllers):
                errors.append("controller data {}: missing controller {}".format(
                    controller_id, controller_id))
            for signal_id in nested_identifiers(data.get("synced_light") or []):
                if missing(signal_id, self.traffic_lights):
                    errors.append("controller data {}: missing synced traffic_light {}".format(
                        controller_id, signal_id))
            for phase_index, phase in enumerate(data.get("phase") or []):
                state = phase.get("state") if isinstance(phase, dict) else None
                if not isinstance(state, dict):
                    errors.append("controller data {}: phase {} has invalid state".format(
                        controller_id, phase_index))
                    continue
                for signal_id in sorted(state):
                    if missing(signal_id, self.traffic_lights):
                        errors.append("controller data {}: phase {} missing traffic_light {}".format(
                            controller_id, phase_index, signal_id))
        for junction_id, junction in sorted(self.junctions.items()):
            for road_id in junction.get("road_id_list") or []:
                if missing(road_id, self.roads):
                    errors.append("junction {}: missing road {}".format(
                        junction_id, road_id))
        for road_id, road in sorted(self.roads.items()):
            for field, singular in (("links", "link"), ("ref_lines", "ref_line")):
                for link_id in road.get(field) or []:
                    if missing(link_id, self.links):
                        errors.append("road {}: missing {} {}".format(
                            road_id, singular, link_id))
        return errors

    def counts(self):
        return {
            "nodes": len(self.nodes),
            "links": len(self.links),
            "lane_nodes": len(self.lane_nodes),
            "lane_boundaries": len(self.lane_boundaries),
            "traffic_lights": len(self.traffic_lights),
            "traffic_signs": len(self.traffic_signs),
            "single_crosswalks": len(self.single_crosswalks),
            "surface_markings": len(self.surface_markings),
            "crosswalk_groups": len(self.crosswalks),
            "junctions": len(self.junctions),
            "intersection_controllers": len(self.controllers),
        }

    def raw_counts(self):
        keys = {
            "nodes": "node_set", "links": "link_set", "lane_nodes": "lane_node_set",
            "lane_boundaries": "lane_boundary_set", "traffic_lights": "traffic_light_set",
            "traffic_signs": "traffic_sign_set", "single_crosswalks": "singlecrosswalk_set",
            "surface_markings": "surface_marking_set",
            "crosswalk_groups": "crosswalk_set", "junctions": "junction_set",
            "intersection_controllers": "intersection_controller_set",
        }
        return {label: len(self.raw_data[name]) for label, name in keys.items()}

    def deduplication_counts(self):
        return {
            "verified_suffix_clones_removed": len(self.verified_aliases),
            "nonidentical_suffix_collisions": len(self.duplicate_collisions),
        }
