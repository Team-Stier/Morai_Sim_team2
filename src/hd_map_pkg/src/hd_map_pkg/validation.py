"""Source, Lanelet2 structure, semantic coverage, and routing validation."""

import hashlib
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

from .geometry import point_segment_distance_2d, polyline_length


def _check(name, status, summary, metrics=None, samples=None):
    value = {"name": name, "status": status, "summary": summary}
    if metrics:
        value["metrics"] = metrics
    if samples:
        value["samples"] = list(samples)[:20]
    return value


def _file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _csv_values(value):
    """Return non-blank, whitespace-normalized values from a CSV tag."""
    return [item.strip() for item in str(value or "").split(",")
            if item.strip()]


def _blank_link_references(dataset):
    """Report explicit blank IDs without treating an absent association as one."""
    samples = []
    affected = set()
    collections = (
        ("single_crosswalk", getattr(dataset, "single_crosswalks", {})),
        ("surface_marking", getattr(dataset, "surface_markings", {})),
    )
    for label, items in collections:
        for source_id, item in sorted(items.items()):
            for position, link_id in enumerate(item.get("link_id_list") or []):
                if link_id is None or not str(link_id).strip():
                    affected.add((label, source_id))
                    samples.append("{} {} link_id_list[{}] is blank".format(
                        label, source_id, position))
    return affected, samples


def validate_source(dataset, config=None):
    checks = []
    references = dataset.reference_errors()
    checks.append(_check(
        "mgeo_reference_integrity",
        "pass" if not references else "fail",
        "all conversion-critical MGeo references resolve" if not references
        else "one or more MGeo references do not resolve",
        {"error_count": len(references)}, references))

    declared = dataset.declared_hashes()
    actual = dataset.source_hashes()
    mismatches = []
    modes = defaultdict(int)
    for filename, expected in sorted(declared.items()):
        hashes = actual.get(filename)
        if hashes is None:
            mismatches.append("{} missing".format(filename))
        elif hashes["sha256_raw"] == expected:
            modes["raw"] += 1
        elif hashes["sha256_crlf"] == expected:
            modes["crlf"] += 1
        else:
            modes["mismatch"] += 1
            mismatches.append("{} expected={} raw={} crlf={}".format(
                filename, expected, hashes["sha256_raw"], hashes["sha256_crlf"]))
    status = "pass" if not mismatches else "warning"
    checks.append(_check(
        "mgeo_declared_hashes",
        status,
        "declared hashes match raw or original CRLF serialization" if not mismatches
        else "upstream global_info hash metadata has mismatches; Git commit remains immutable authority",
        {
            "declared_files": len(declared),
            "raw_matches": modes["raw"],
            "crlf_matches": modes["crlf"],
            "mismatches": modes["mismatch"],
        }, mismatches))
    checks.append(_check(
        "mgeo_schema_version",
        "pass",
        "source is MGeo {}.{}".format(
            dataset.global_info.get("maj_ver"), dataset.global_info.get("min_ver")),
        {"major": dataset.global_info.get("maj_ver"),
         "minor": dataset.global_info.get("min_ver")}))

    blank_records, blank_samples = _blank_link_references(dataset)
    checks.append(_check(
        "mgeo_blank_link_references",
        "warning" if blank_samples else "pass",
        "source contains explicit blank link references; blanks are ignored during export"
        if blank_samples else "surface-marking and crossing link references contain no blanks",
        {"blank_references": len(blank_samples),
         "affected_records": len(blank_records)}, blank_samples))
    if config is not None:
        source = config.get("source", {})
        expected_commit = str(source.get("commit", ""))
        expected_tree = str(source.get("tree", ""))
        try:
            git_root = subprocess.check_output(
                ["git", "-C", str(dataset.root), "rev-parse", "--show-toplevel"],
                stderr=subprocess.STDOUT, text=True).strip()
            actual_commit = subprocess.check_output(
                ["git", "-C", git_root, "rev-parse", "HEAD"],
                stderr=subprocess.STDOUT, text=True).strip()
            relative = str(dataset.root.relative_to(Path(git_root)))
            actual_tree = subprocess.check_output(
                ["git", "-C", git_root, "rev-parse", "HEAD:{}".format(relative)],
                stderr=subprocess.STDOUT, text=True).strip()
            dirty = subprocess.check_output(
                ["git", "-C", git_root, "status", "--porcelain", "--", relative],
                stderr=subprocess.STDOUT, text=True).strip()
            immutable = (actual_commit == expected_commit and actual_tree == expected_tree and not dirty)
            checks.append(_check(
                "immutable_source_checkout", "pass" if immutable else "fail",
                "source checkout exactly matches the configured commit and KATRI tree"
                if immutable else "source checkout differs from immutable configuration",
                {"expected_commit": expected_commit, "actual_commit": actual_commit,
                 "expected_tree": expected_tree, "actual_tree": actual_tree,
                 "source_dirty": bool(dirty)}))
        except (subprocess.CalledProcessError, OSError, ValueError) as error:
            checks.append(_check(
                "immutable_source_checkout", "fail",
                "could not prove immutable Git source: {}".format(error)))
    return checks


def _parse_osm(path):
    nodes = {}
    ways = {}
    relations = {}
    root_checked = False
    for event, child in ET.iterparse(str(path), events=("start", "end")):
        if not root_checked and event == "start":
            root_checked = True
            if child.tag != "osm" or child.attrib.get("version") != "0.6":
                raise ValueError("not an OSM 0.6 document")
            continue
        if event != "end" or child.tag not in ("node", "way", "relation"):
            continue
        identifier = int(child.attrib["id"])
        tags = {tag.attrib["k"]: tag.attrib["v"] for tag in child.findall("tag")}
        if child.tag == "node":
            nodes[identifier] = {
                "lat": float(child.attrib["lat"]),
                "lon": float(child.attrib["lon"]),
                "tags": tags,
            }
        elif child.tag == "way":
            ways[identifier] = {
                "nodes": [int(node.attrib["ref"]) for node in child.findall("nd")],
                "tags": tags,
            }
        elif child.tag == "relation":
            relations[identifier] = {
                "members": [(member.attrib["type"], int(member.attrib["ref"]),
                             member.attrib.get("role", ""))
                            for member in child.findall("member")],
                "tags": tags,
            }
        child.clear()
    return nodes, ways, relations


def _side_endpoints(relation, ways, nodes):
    members = {role: ref for member_type, ref, role in relation["members"]
               if member_type == "way" and role in ("left", "right", "centerline")}
    if not all(role in members for role in ("left", "right", "centerline")):
        return None
    center = ways.get(members["centerline"])
    if not center or len(center["nodes"]) < 2:
        return None
    result = {}
    for role in ("left", "right"):
        way = ways.get(members[role])
        if not way or len(way["nodes"]) < 2:
            return None
        first_id, last_id = way["nodes"][0], way["nodes"][-1]
        result[role] = (first_id, last_id)
    result["center"] = (center["nodes"][0], center["nodes"][-1])
    return result


def validate_osm(osm_path, dataset, routing_path=None, config=None):
    checks = []
    try:
        nodes, ways, relations = _parse_osm(osm_path)
    except (ET.ParseError, OSError, ValueError) as error:
        return [_check("lanelet2_osm_parse", "fail", str(error))]
    checks.append(_check(
        "lanelet2_osm_parse", "pass", "OSM 0.6 XML is well formed",
        {"nodes": len(nodes), "ways": len(ways), "relations": len(relations),
         "sha256": _file_sha256(osm_path)}))

    identifiers = list(nodes) + list(ways) + list(relations)
    global_id_errors = []
    if len(identifiers) != len(set(identifiers)):
        global_id_errors.append("node/way/relation IDs are not globally unique")
    for node_id, node in nodes.items():
        if not math.isfinite(node["lat"]) or not math.isfinite(node["lon"]):
            global_id_errors.append("node {} has non-finite coordinates".format(node_id))
    for way_id, way in ways.items():
        if len(way["nodes"]) < 2 or len(set(way["nodes"])) < 2:
            global_id_errors.append("way {} is degenerate".format(way_id))
    checks.append(_check(
        "osm_primitive_sanity", "pass" if not global_id_errors else "fail",
        "primitive IDs are globally unique and geometries are finite/non-degenerate"
        if not global_id_errors else "invalid OSM primitive geometry or ID",
        {"global_ids": len(identifiers)}, global_id_errors))

    missing = []
    for way_id, way in ways.items():
        for node_id in way["nodes"]:
            if node_id not in nodes:
                missing.append("way {} -> node {}".format(way_id, node_id))
    containers = {"node": nodes, "way": ways, "relation": relations}
    for relation_id, relation in relations.items():
        for member_type, ref, role in relation["members"]:
            if member_type not in containers or ref not in containers[member_type]:
                missing.append("relation {} -> {} {} ({})".format(
                    relation_id, member_type, ref, role))
    checks.append(_check(
        "osm_reference_integrity", "pass" if not missing else "fail",
        "all OSM primitive references resolve" if not missing else "dangling OSM references found",
        {"error_count": len(missing)}, missing))

    lanelets = {relation_id: relation for relation_id, relation in relations.items()
                if relation["tags"].get("type") == "lanelet"}
    malformed = []
    lanelets_by_mgeo = defaultdict(list)
    speed_count = 0
    turn_count = 0
    derived_count = 0
    for relation_id, relation in lanelets.items():
        roles = [role for member_type, _, role in relation["members"]
                 if member_type == "way"]
        for required in ("left", "right", "centerline"):
            if roles.count(required) != 1:
                malformed.append("lanelet {} has {} way members with role {}".format(
                    relation_id, roles.count(required), required))
        mgeo_id = relation["tags"].get("mgeo:id", "")
        if mgeo_id:
            lanelets_by_mgeo[mgeo_id].append((relation_id, relation))
        else:
            malformed.append("lanelet {} lacks mgeo:id".format(relation_id))
        speed_count += int("speed_limit" in relation["tags"])
        turn_count += int("turn_direction" in relation["tags"])
        derived_count += int(relation["tags"].get("mgeo:derived_boundary") == "yes")

    expected_link_ids = set(dataset.links)
    exported_link_ids = set(lanelets_by_mgeo)
    for link_id in sorted(expected_link_ids - exported_link_ids):
        malformed.append("source link {} has no lanelet segments".format(link_id))
    for link_id in sorted(exported_link_ids - expected_link_ids):
        malformed.append("lanelet mgeo:id {} is not a source link".format(link_id))

    segment_tolerance = float((config or {}).get("validation", {}).get(
        "segment_chainage_tolerance_m", 0.02))
    ordered_lanelets_by_mgeo = {}
    lanelet_ids_by_mgeo = {}
    segmented_links = 0
    for link_id in sorted(expected_link_ids & exported_link_ids):
        entries = lanelets_by_mgeo[link_id]
        parsed = []
        for relation_id, relation in entries:
            tags = relation["tags"]
            try:
                index = int(tags["mgeo:segment_index"])
                count = int(tags["mgeo:segment_count"])
                start = float(tags["mgeo:start_chainage_m"])
                end = float(tags["mgeo:end_chainage_m"])
            except (KeyError, TypeError, ValueError):
                malformed.append(
                    "lanelet {} has invalid or missing segment metadata".format(
                        relation_id))
                index, count, start, end = relation_id, -1, float("nan"), float("nan")
            parsed.append((index, relation_id, relation, count, start, end))
        parsed.sort(key=lambda value: (value[0], value[1]))
        ordered_lanelets_by_mgeo[link_id] = [value[2] for value in parsed]
        lanelet_ids_by_mgeo[link_id] = [value[1] for value in parsed]
        if len(parsed) > 1:
            segmented_links += 1
        indices = [value[0] for value in parsed]
        if indices != list(range(len(parsed))):
            malformed.append("link {} segment indices are {}, expected {}".format(
                link_id, indices, list(range(len(parsed)))))
        for index, relation_id, relation, count, start, end in parsed:
            if count != len(parsed):
                malformed.append("lanelet {} segment_count {} != {}".format(
                    relation_id, count, len(parsed)))
            expected_segment_id = "{}#{}".format(link_id, index)
            if relation["tags"].get("mgeo:segment_id") != expected_segment_id:
                malformed.append("lanelet {} has invalid mgeo:segment_id".format(
                    relation_id))
            if not math.isfinite(start) or not math.isfinite(end) or end < start:
                malformed.append("lanelet {} has invalid chainage interval".format(
                    relation_id))
        if parsed and all(math.isfinite(value[4]) and math.isfinite(value[5])
                          for value in parsed):
            if abs(parsed[0][4]) > segment_tolerance:
                malformed.append("link {} segment coverage does not start at zero".format(
                    link_id))
            for previous, following in zip(parsed, parsed[1:]):
                if abs(previous[5] - following[4]) > segment_tolerance:
                    malformed.append("link {} has a segment chainage gap {} -> {}".format(
                        link_id, previous[1], following[1]))
            source_length = polyline_length(dataset.links[link_id].get("points") or [])
            if abs(parsed[-1][5] - source_length) > segment_tolerance:
                malformed.append(
                    "link {} segment coverage ends at {:.3f}, source length is {:.3f}".format(
                        link_id, parsed[-1][5], source_length))
    checks.append(_check(
        "lanelet_semantics", "pass" if not malformed else "fail",
        "each MGeo link has complete, contiguous Lanelet2 segment coverage"
        if not malformed else "Lanelet2 relation structure or segment coverage is incomplete",
        {"lanelets": len(lanelets), "source_links": len(dataset.links),
         "exported_source_links": len(expected_link_ids & exported_link_ids),
         "segmented_source_links": segmented_links,
         "speed_limits": speed_count, "turn_directions": turn_count,
         "derived_boundary_lanelets": derived_count}, malformed))

    boundary_ids = {}
    incomplete_boundaries = []
    for way_id, way in ways.items():
        mgeo_id = way["tags"].get("mgeo:id")
        category = way["tags"].get("mgeo:boundary_category")
        if mgeo_id and category:
            boundary_ids[mgeo_id] = way_id
            for required in ("type", "subtype", "color", "mgeo:boundary_category"):
                if not way["tags"].get(required):
                    incomplete_boundaries.append("{} lacks {}".format(mgeo_id, required))
    degenerate_boundary_ids = []
    nondegenerate_boundary_ids = []
    for boundary_id, boundary in sorted(dataset.lane_boundaries.items()):
        points = boundary.get("points") or []
        unique_xy = {(round(float(point[0]), 6), round(float(point[1]), 6))
                     for point in points if len(point) >= 2}
        if len(points) < 2 or len(unique_xy) < 2:
            degenerate_boundary_ids.append(boundary_id)
        else:
            nondegenerate_boundary_ids.append(boundary_id)
    missing_boundary_ids = sorted(set(nondegenerate_boundary_ids) - set(boundary_ids))
    unexpected_boundary_ids = sorted(set(boundary_ids) - set(nondegenerate_boundary_ids))
    incomplete_boundaries.extend(
        "missing non-degenerate source boundary {}".format(value)
        for value in missing_boundary_ids)
    incomplete_boundaries.extend(
        "exported source boundary {} is absent or degenerate".format(value)
        for value in unexpected_boundary_ids)
    checks.append(_check(
        "boundary_attribute_coverage", "pass" if not incomplete_boundaries else "fail",
        "all non-degenerate lane boundaries retain line style, color, and functional category"
        if not incomplete_boundaries else "lane-boundary semantic coverage is incomplete",
        {"source_boundaries": len(dataset.lane_boundaries),
         "nondegenerate_source_boundaries": len(nondegenerate_boundary_ids),
         "exported_source_boundaries": len(boundary_ids),
         "missing_nondegenerate_boundaries": len(missing_boundary_ids)},
        incomplete_boundaries))
    checks.append(_check(
        "degenerate_source_boundaries",
        "warning" if degenerate_boundary_ids else "pass",
        "degenerate source boundaries cannot form valid OSM ways and were intentionally omitted"
        if degenerate_boundary_ids else "source contains no degenerate lane boundaries",
        {"degenerate_source_boundaries": len(degenerate_boundary_ids)},
        degenerate_boundary_ids))

    surface_errors = []
    surface_relations = defaultdict(list)
    surface_source_ids = set(getattr(dataset, "surface_markings", {}))
    for relation_id, relation in relations.items():
        tags = relation["tags"]
        source_id = tags.get("mgeo:id", "")
        if tags.get("type") == "multipolygon" and source_id in surface_source_ids:
            surface_relations[source_id].append((relation_id, relation))
        elif (tags.get("type") == "multipolygon" and
              tags.get("subtype") in ("arrow", "speed_bump") and source_id and
              source_id not in surface_source_ids):
            surface_errors.append(
                "unexpected surface-marking area {}".format(source_id))

    surface_claims = defaultdict(set)
    for link_id, entries in lanelets_by_mgeo.items():
        for _, lanelet in entries:
            for marking_id in _csv_values(
                    lanelet["tags"].get("mgeo:surface_markings", "")):
                surface_claims[marking_id].add(link_id)
                if marking_id not in surface_source_ids:
                    surface_errors.append(
                        "lanelet {} claims unknown surface marking {}".format(
                            link_id, marking_id))

    linked_surface_markings = 0
    for marking_id, marking in sorted(
            getattr(dataset, "surface_markings", {}).items()):
        candidates = surface_relations.get(marking_id, [])
        if len(candidates) != 1:
            surface_errors.append(
                "surface marking {} has {} area relations, expected 1".format(
                    marking_id, len(candidates)))
            continue
        relation_id, relation = candidates[0]
        tags = relation["tags"]
        expected_subtype = (
            "speed_bump" if str(marking.get("sub_type", "")).lower() == "speedbump"
            else "arrow")
        if tags.get("area") != "yes" or tags.get("subtype") != expected_subtype:
            surface_errors.append(
                "surface marking {} has invalid area/subtype semantics".format(
                    marking_id))
        outer_refs = [ref for member_type, ref, role in relation["members"]
                      if member_type == "way" and role == "outer"]
        if len(outer_refs) != 1 or outer_refs[0] not in ways:
            surface_errors.append(
                "surface marking {} must have exactly one valid outer way".format(
                    marking_id))
        else:
            outer_tags = ways[outer_refs[0]]["tags"]
            if (outer_tags.get("area") == "yes" or
                    outer_tags.get("type") != "road_marking" or
                    outer_tags.get("subtype") != expected_subtype or
                    outer_tags.get("mgeo:id") != marking_id):
                surface_errors.append(
                    "surface marking {} outer way semantics differ from its area".format(
                        marking_id))
        expected_links = {str(value).strip()
                          for value in marking.get("link_id_list") or []
                          if value is not None and str(value).strip()}
        relation_links = set(_csv_values(tags.get("mgeo:link_ids", "")))
        if relation_links != expected_links:
            surface_errors.append(
                "surface marking {} area link association mismatch".format(
                    marking_id))
        claimed_links = surface_claims.get(marking_id, set())
        if claimed_links != expected_links:
            missing_claims = sorted(expected_links - claimed_links)
            extra_claims = sorted(claimed_links - expected_links)
            surface_errors.append(
                "surface marking {} lanelet association mismatch missing={} extra={}".format(
                    marking_id, missing_claims, extra_claims))
        if expected_links and expected_links <= claimed_links:
            linked_surface_markings += 1
    checks.append(_check(
        "surface_marking_coverage",
        "pass" if not surface_errors else "fail",
        "every source surface marking has one area and at least one segment per linked source link"
        if not surface_errors else
        "surface-marking ID, area, or lanelet-link association coverage is incomplete",
        {"source_markings": len(surface_source_ids),
         "exported_marking_ids": len(surface_relations),
         "linked_markings": linked_surface_markings}, surface_errors))

    crossing_semantics = {
        "5321": ("crosswalk", "pedestrian", "bicycle", False),
        "533": ("crosswalk", "pedestrian", "bicycle", True),
        "534": ("bicycle_crossing", "bicycle", "pedestrian", False),
    }
    crossing_errors = []
    expected_crossing_ids = {
        crossing_id for crossing_id, crossing in dataset.single_crosswalks.items()
        if str(crossing.get("sign_type", "")) in crossing_semantics
    }
    crossing_relations = defaultdict(list)
    for relation_id, relation in relations.items():
        tags = relation["tags"]
        source_id = tags.get("mgeo:id", "")
        if tags.get("type") == "multipolygon" and source_id in expected_crossing_ids:
            crossing_relations[source_id].append((relation_id, relation))
        elif (tags.get("type") == "multipolygon" and
              tags.get("subtype") in {value[0] for value in crossing_semantics.values()} and
              source_id and source_id not in expected_crossing_ids):
            crossing_errors.append("unexpected crossing area {}".format(source_id))
    for crossing_id in sorted(expected_crossing_ids):
        crossing = dataset.single_crosswalks[crossing_id]
        sign_type = str(crossing.get("sign_type", ""))
        expected_subtype, expected_participant, excluded_participant, raised = (
            crossing_semantics[sign_type])
        candidates = crossing_relations.get(crossing_id, [])
        if len(candidates) != 1:
            crossing_errors.append(
                "crossing {} has {} area relations, expected 1".format(
                    crossing_id, len(candidates)))
            continue
        _, relation = candidates[0]
        tags = relation["tags"]
        if (tags.get("area") != "yes" or tags.get("subtype") != expected_subtype or
                tags.get("one_way") != "no"):
            crossing_errors.append(
                "crossing {} has invalid subtype/area/one_way semantics".format(
                    crossing_id))
        if tags.get("participant:{}".format(expected_participant)) != "yes":
            crossing_errors.append(
                "crossing {} sign_type {} lacks participant:{}=yes".format(
                    crossing_id, sign_type, expected_participant))
        if tags.get("participant:{}".format(excluded_participant)) == "yes":
            crossing_errors.append(
                "crossing {} sign_type {} incorrectly allows participant:{}".format(
                    crossing_id, sign_type, excluded_participant))
        if (tags.get("mgeo:raised") == "yes") != raised:
            crossing_errors.append(
                "crossing {} sign_type {} has invalid mgeo:raised semantics".format(
                    crossing_id, sign_type))
        expected_links = {str(value).strip()
                          for value in crossing.get("link_id_list") or []
                          if value is not None and str(value).strip()}
        if set(_csv_values(tags.get("mgeo:link_ids", ""))) != expected_links:
            crossing_errors.append(
                "crossing {} link association mismatch".format(crossing_id))
        outer_refs = [ref for member_type, ref, role in relation["members"]
                      if member_type == "way" and role == "outer"]
        if len(outer_refs) != 1 or outer_refs[0] not in ways:
            crossing_errors.append(
                "crossing {} must have exactly one valid outer way".format(crossing_id))
        else:
            outer_tags = ways[outer_refs[0]]["tags"]
            if (outer_tags.get("area") == "yes" or
                    outer_tags.get("subtype") != expected_subtype or
                    outer_tags.get("mgeo:id") != crossing_id):
                crossing_errors.append(
                    "crossing {} outer way semantics differ from its area".format(
                        crossing_id))
    checks.append(_check(
        "crosswalk_semantics",
        "pass" if not crossing_errors else "fail",
        "5321/533 crossings are pedestrian and 534 crossings are bicycle areas"
        if not crossing_errors else "crossing subtype or participant semantics are incorrect",
        {"source_crossings": len(expected_crossing_ids),
         "exported_crossing_ids": len(crossing_relations),
         "pedestrian_crossings": sum(
             str(dataset.single_crosswalks[value].get("sign_type")) in ("5321", "533")
             for value in expected_crossing_ids),
         "bicycle_crossings": sum(
             str(dataset.single_crosswalks[value].get("sign_type")) == "534"
             for value in expected_crossing_ids)}, crossing_errors))

    snap_values = []
    for way in ways.values():
        for field in ("mgeo:routing_start_snap_m", "mgeo:routing_end_snap_m"):
            if field in way["tags"]:
                snap_values.append(float(way["tags"][field]))
    threshold = float((config or {}).get("validation", {}).get(
        "max_successor_endpoint_gap_m", 3.0))
    excessive_snaps = [value for value in snap_values if value > threshold]
    checks.append(_check(
        "routing_endpoint_geometry",
        "pass" if not excessive_snaps else "fail",
        "topology endpoint unification stays within the configured geometry tolerance"
        if not excessive_snaps else
        "some source/synthetic bounds were snapped farther than the configured tolerance",
        {"endpoint_values": len(snap_values), "threshold_m": threshold,
         "max_snap_m": max(snap_values) if snap_values else 0.0,
         "over_tolerance": len(excessive_snaps)}))

    stop_lines = [way for way in ways.values() if way["tags"].get("type") == "stop_line"]
    crosswalks = [relation for relation in relations.values()
                  if relation["tags"].get("subtype") in
                  ("crosswalk", "raised_crosswalk", "bicycle_crossing")]
    signal_ways = [way for way in ways.values() if way["tags"].get("type") == "traffic_light"]
    signal_regs = [relation for relation in relations.values()
                   if relation["tags"].get("type") == "regulatory_element" and
                   relation["tags"].get("subtype") == "traffic_light"]
    signal_links = dataset.traffic_light_link_ids()
    signal_crosswalks = dataset.traffic_light_crosswalk_ids()
    associated_signal_count = sum(
        bool(signal_crosswalks.get(signal_id)) if
        str(dataset.traffic_lights[signal_id].get("type", "")) == "pedestrian"
        else bool(signal_links.get(signal_id))
        for signal_id in dataset.traffic_lights)
    expected_regulated_signals = {
        signal_id for signal_id, signal in dataset.traffic_lights.items()
        if (bool(signal_crosswalks.get(signal_id))
            if str(signal.get("type", "")) == "pedestrian"
            else bool(signal_links.get(signal_id)))
    }
    actual_regulated_signals = {
        relation["tags"].get("mgeo:id") for relation in signal_regs
    }
    regulatory_errors = []
    if len(signal_ways) != len(dataset.traffic_lights):
        regulatory_errors.append("traffic-light geometry count differs from source")
    if actual_regulated_signals != expected_regulated_signals:
        regulatory_errors.append(
            "traffic-light regulatory coverage differs from associated source signals")
    signal_way_by_mgeo = {way["tags"].get("mgeo:id"): way_id
                          for way_id, way in ways.items()
                          if way["tags"].get("type") == "traffic_light"}
    crosswalk_by_mgeo = {relation["tags"].get("mgeo:id"): relation_id
                         for relation_id, relation in relations.items()
                         if relation["tags"].get("subtype") in
                         ("crosswalk", "raised_crosswalk", "bicycle_crossing")}
    for relation_id, relation in relations.items():
        if relation not in signal_regs:
            continue
        signal_id = relation["tags"].get("mgeo:id", "")
        refers = [ref for member_type, ref, role in relation["members"]
                  if member_type == "way" and role == "refers"]
        if refers != [signal_way_by_mgeo.get(signal_id)]:
            regulatory_errors.append("signal regulation {} has invalid refers".format(signal_id))
        expected_stop_ids = [value for value in
                             relation["tags"].get("mgeo:stop_line_ids", "").split(",") if value]
        expected_stop_refs = {boundary_ids[value] for value in expected_stop_ids
                              if value in boundary_ids}
        actual_stop_refs = {ref for member_type, ref, role in relation["members"]
                            if member_type == "way" and role == "ref_line"}
        if len(actual_stop_refs) > 1:
            regulatory_errors.append(
                "signal regulation {} has more than one ref_line".format(signal_id))
        if actual_stop_refs != expected_stop_refs:
            regulatory_errors.append("signal regulation {} ref_line mismatch".format(signal_id))
        controlled = relation["tags"].get("mgeo:controlled_feature")
        if controlled == "vehicle_lane":
            for link_id in [value for value in
                            relation["tags"].get("mgeo:link_ids", "").split(",") if value]:
                expected_member = ("relation", relation_id, "regulatory_element")
                controlled_segments = [relations.get(lanelet_id, {})
                                       for lanelet_id in lanelet_ids_by_mgeo.get(link_id, [])]
                if not any(expected_member in lanelet.get("members", [])
                           for lanelet in controlled_segments):
                    regulatory_errors.append(
                        "no lanelet segment for {} references signal regulation {}".format(
                            link_id, signal_id))
        elif controlled == "pedestrian_crosswalk":
            for crosswalk_id in [value for value in
                                 relation["tags"].get("mgeo:crosswalk_ids", "").split(",")
                                 if value]:
                area_id = crosswalk_by_mgeo.get(crosswalk_id)
                area = relations.get(area_id, {})
                expected_member = ("relation", relation_id, "regulatory_element")
                if expected_member not in area.get("members", []):
                    regulatory_errors.append(
                        "crosswalk {} does not reference signal regulation {}".format(
                            crosswalk_id, signal_id))
        else:
            regulatory_errors.append(
                "signal regulation {} has unknown controlled feature".format(signal_id))
    actual_links_by_signal = defaultdict(set)
    actual_crosswalks_by_signal = defaultdict(set)
    for relation in signal_regs:
        signal_id = relation["tags"].get("mgeo:id", "")
        actual_links_by_signal[signal_id].update(
            value for value in relation["tags"].get("mgeo:link_ids", "").split(",")
            if value)
        actual_crosswalks_by_signal[signal_id].update(
            value for value in relation["tags"].get("mgeo:crosswalk_ids", "").split(",")
            if value)
    for signal_id in expected_regulated_signals:
        signal = dataset.traffic_lights[signal_id]
        if str(signal.get("type", "")) == "pedestrian":
            if actual_crosswalks_by_signal[signal_id] != set(
                    signal_crosswalks.get(signal_id, [])):
                regulatory_errors.append(
                    "signal {} crosswalk regulation coverage differs from source".format(
                        signal_id))
        elif actual_links_by_signal[signal_id] != set(signal_links.get(signal_id, [])):
            regulatory_errors.append(
                "signal {} lane regulation coverage differs from source".format(signal_id))
    rules_status = "fail" if regulatory_errors else (
        "warning" if len(dataset.traffic_lights) - associated_signal_count else "pass")
    checks.append(_check(
        "road_rule_features", rules_status,
        "stop lines, crosswalks, traffic-light IDs and lane associations are exported"
        if rules_status == "pass" else (
            "some source signal heads have no resolvable vehicle-lane or crosswalk association"
            if not regulatory_errors else "road-rule feature export is incomplete"),
        {"stop_lines": len(stop_lines), "crosswalk_areas": len(crosswalks),
         "traffic_light_ways": len(signal_ways),
         "traffic_light_regulatory_elements": len(signal_regs),
         "unassociated_traffic_lights": len(dataset.traffic_lights) - associated_signal_count},
        regulatory_errors))

    intersection_areas = [relation for relation in relations.values()
                          if relation["tags"].get("subtype") == "intersection_area"]
    checks.append(_check(
        "intersection_area_coverage",
        "pass" if len(intersection_areas) == len(dataset.junctions) else "warning",
        "junction records are represented as explicitly marked derived intersection areas",
        {"source_junctions": len(dataset.junctions),
         "derived_intersection_areas": len(intersection_areas)}))
    maximum_extent = float((config or {}).get("validation", {}).get(
        "max_derived_intersection_extent_m", 120.0))
    broad_intersections = []
    largest_extent = 0.0
    largest_area = 0.0
    for relation in intersection_areas:
        tags = relation["tags"]
        try:
            width = float(tags.get("mgeo:bbox_width_m", 0.0))
            height = float(tags.get("mgeo:bbox_height_m", 0.0))
            area = float(tags.get("mgeo:area_m2", 0.0))
        except ValueError:
            broad_intersections.append(
                "{} has invalid derived geometry metrics".format(tags.get("mgeo:id")))
            continue
        largest_extent = max(largest_extent, width, height)
        largest_area = max(largest_area, area)
        if max(width, height) > maximum_extent:
            broad_intersections.append(
                "{} bbox={:.1f}x{:.1f}m area={:.1f}m2".format(
                    tags.get("mgeo:id"), width, height, area))
    checks.append(_check(
        "derived_intersection_geometry",
        "warning" if broad_intersections else "pass",
        "some source junction memberships produce broad convex-hull approximations"
        if broad_intersections else "derived intersection extents are within the configured limit",
        {"extent_limit_m": maximum_extent,
         "over_limit": len(broad_intersections),
         "max_extent_m": round(largest_extent, 3),
         "max_area_m2": round(largest_area, 3)}, broad_intersections))

    endpoints = {relation_id: _side_endpoints(relation, ways, nodes)
                 for relation_id, relation in lanelets.items()}
    source_successor_edges = 0
    internal_segment_edges = 0
    native_edges = 0
    native_failures = []

    def follows(first_id, second_id):
        first = endpoints.get(first_id)
        second = endpoints.get(second_id)
        return bool(first and second and
                    first["left"][1] == second["left"][0] and
                    first["right"][1] == second["right"][0])

    for link_id in sorted(dataset.links):
        current_segments = lanelet_ids_by_mgeo.get(link_id, [])
        for current_id, following_id in zip(current_segments, current_segments[1:]):
            internal_segment_edges += 1
            if follows(current_id, following_id):
                native_edges += 1
            else:
                native_failures.append("{} internal {} -> {}".format(
                    link_id, current_id, following_id))
        for successor_id in dataset.successors.get(link_id, []):
            source_successor_edges += 1
            following_segments = lanelet_ids_by_mgeo.get(successor_id, [])
            if (current_segments and following_segments and
                    follows(current_segments[-1], following_segments[0])):
                native_edges += 1
            else:
                native_failures.append("{} -> {}".format(link_id, successor_id))
    total_edges = source_successor_edges + internal_segment_edges
    native_status = "pass" if native_edges == total_edges else "warning"
    checks.append(_check(
        "lanelet2_native_successor_topology", native_status,
        "source successors and internal segments share ordered left/right endpoint Point IDs"
        if native_status == "pass" else
        "some source successors or internal segments lack native Lanelet2 endpoint topology",
        {"mgeo_successor_edges": source_successor_edges,
         "internal_segment_edges": internal_segment_edges,
         "native_shared_endpoint_edges": native_edges,
         "coverage_percent": round(100.0 * native_edges / total_edges, 3) if total_edges else 100.0},
        native_failures))

    if routing_path is None:
        checks.append(_check("explicit_routing_graph", "fail", "routing graph path was not supplied"))
    else:
        try:
            graph = json.loads(Path(routing_path).read_text(encoding="utf-8"))
            graph_links = graph.get("links", {})
            graph_lanelets = graph.get("lanelets", {})
            route_errors = []
            extras = sorted(set(graph_links) - set(dataset.links))
            if extras:
                route_errors.append("unexpected links: {}".format(",".join(extras[:10])))
            for link_id in dataset.links:
                entry = graph_links.get(link_id)
                if entry is None:
                    route_errors.append("missing {}".format(link_id))
                    continue
                if entry.get("successors") != dataset.successors.get(link_id, []):
                    route_errors.append("successor mismatch {}".format(link_id))
                if entry.get("predecessors") != dataset.predecessors.get(link_id, []):
                    route_errors.append("predecessor mismatch {}".format(link_id))
                source_link = dataset.links[link_id]
                expected_values = {
                    "left_lane_change": source_link.get("left_lane_change_dst_link_idx"),
                    "right_lane_change": source_link.get("right_lane_change_dst_link_idx"),
                    "can_move_left": bool(source_link.get("can_move_left_lane")),
                    "can_move_right": bool(source_link.get("can_move_right_lane")),
                }
                for field, expected in expected_values.items():
                    if entry.get(field) != expected:
                        route_errors.append("{} mismatch {}".format(field, link_id))
                expected_relation_ids = lanelet_ids_by_mgeo.get(link_id, [])
                if entry.get("lanelet_relation_ids") != expected_relation_ids:
                    route_errors.append("lanelet_relation_ids mismatch {}".format(
                        link_id))

            expected_lanelet_ids = {
                str(relation_id) for values in lanelet_ids_by_mgeo.values()
                for relation_id in values
            }
            extra_lanelets = sorted(set(graph_lanelets) - expected_lanelet_ids)
            missing_lanelets = sorted(expected_lanelet_ids - set(graph_lanelets))
            if extra_lanelets:
                route_errors.append("unexpected expanded lanelets: {}".format(
                    ",".join(extra_lanelets[:10])))
            if missing_lanelets:
                route_errors.append("missing expanded lanelets: {}".format(
                    ",".join(missing_lanelets[:10])))
            for link_id in sorted(dataset.links):
                relation_ids = lanelet_ids_by_mgeo.get(link_id, [])
                for index, relation_id in enumerate(relation_ids):
                    entry = graph_lanelets.get(str(relation_id))
                    if entry is None:
                        continue
                    expected_predecessors = (
                        [relation_ids[index - 1]] if index else [
                            values[-1]
                            for predecessor in dataset.predecessors.get(link_id, [])
                            for values in [lanelet_ids_by_mgeo.get(predecessor, [])]
                            if values
                        ])
                    expected_successors = (
                        [relation_ids[index + 1]]
                        if index + 1 < len(relation_ids) else [
                            values[0]
                            for successor in dataset.successors.get(link_id, [])
                            for values in [lanelet_ids_by_mgeo.get(successor, [])]
                            if values
                        ])
                    expected = {
                        "source_link_id": link_id,
                        "segment_index": index,
                        "segment_count": len(relation_ids),
                        "predecessor_lanelet_relation_ids": expected_predecessors,
                        "successor_lanelet_relation_ids": expected_successors,
                    }
                    for field, value in expected.items():
                        if entry.get(field) != value:
                            route_errors.append(
                                "expanded {} mismatch {}#{}".format(
                                    field, link_id, index))
            checks.append(_check(
                "explicit_routing_graph", "pass" if not route_errors else "fail",
                "all source and expanded lanelet routing relations are preserved"
                if not route_errors else "explicit routing graph differs from MGeo",
                {"links": len(graph_links), "lanelets": len(graph_lanelets),
                 "successor_edges": total_edges}, route_errors))
        except (OSError, ValueError) as error:
            checks.append(_check("explicit_routing_graph", "fail", str(error)))
    return checks


def validate_reference_path(dataset, transformer, path):
    path = Path(path)
    if not path.is_file():
        return _check("simulator_path_alignment", "warning", "reference path not found: {}".format(path))
    grid_size = 1.0
    grid = defaultdict(list)
    for link in dataset.links.values():
        points = [transformer.mgeo_to_sim(point) for point in link.get("points") or []]
        for start, end in zip(points, points[1:]):
            min_cell_x = int(math.floor(min(start[0], end[0]) / grid_size))
            max_cell_x = int(math.floor(max(start[0], end[0]) / grid_size))
            min_cell_y = int(math.floor(min(start[1], end[1]) / grid_size))
            max_cell_y = int(math.floor(max(start[1], end[1]) / grid_size))
            for cell_x in range(min_cell_x, max_cell_x + 1):
                for cell_y in range(min_cell_y, max_cell_y + 1):
                    grid[(cell_x, cell_y)].append((start, end))
    distances = []
    point_count = 0
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            values = line.split()
            if len(values) < 2:
                continue
            point_count += 1
            x, y = float(values[0]), float(values[1])
            cell_x = int(math.floor(x / grid_size))
            cell_y = int(math.floor(y / grid_size))
            candidates = []
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    candidates.extend(grid.get((cell_x + dx, cell_y + dy), []))
            if not candidates:
                distances.append(float("inf"))
            else:
                distances.append(min(point_segment_distance_2d(
                    (x, y), segment[0], segment[1]) for segment in candidates))
    finite = [value for value in distances if math.isfinite(value)]
    maximum = max(finite) if finite else float("inf")
    sorted_values = sorted(finite)
    median = sorted_values[len(sorted_values) // 2] if sorted_values else float("inf")
    status = "pass" if len(finite) == point_count and maximum <= 0.10 else "fail"
    return _check(
        "simulator_path_alignment", status,
        "reference SIM path aligns with KATRI centerline after origin translation"
        if status == "pass" else "reference SIM path does not align with converted centerlines",
        {"points": point_count, "matched_points": len(finite),
         "median_error_m": round(median, 6), "max_error_m": round(maximum, 6)})


def build_report(dataset, transformer, config, osm_path, routing_path, reference_path=None,
                 exporter_statistics=None):
    checks = validate_source(dataset, config)
    checks.extend(validate_osm(osm_path, dataset, routing_path, config))
    if reference_path is not None:
        checks.append(validate_reference_path(dataset, transformer, reference_path))
    statuses = defaultdict(int)
    for check in checks:
        statuses[check["status"]] += 1
    report = {
        "format": "hd_map_pkg.validation_report.v1",
        "overall_status": "fail" if statuses["fail"] else (
            "warning" if statuses["warning"] else "pass"),
        "source": {
            "status": config.get("source", {}).get("status"),
            "repository": config.get("source", {}).get("repository"),
            "commit": config.get("source", {}).get("commit"),
            "tree": config.get("source", {}).get("tree"),
            "path": str(dataset.root),
            "counts": dataset.counts(),
            "raw_counts": dataset.raw_counts(),
            "deduplication": dataset.deduplication_counts(),
            "global_info": {
                "saved_utc_time": dataset.global_info.get("saved_utc_time"),
                "mgeo_version": "{}.{}".format(dataset.global_info.get("maj_ver"),
                                                  dataset.global_info.get("min_ver")),
                "crs": dataset.global_info.get("global_coordinate_system"),
                "local_origin_in_global": dataset.global_info.get("local_origin_in_global"),
                "license_metadata": dataset.global_info.get("license"),
                "license_status": config.get("source", {}).get("license_status"),
            },
        },
        "artifacts": {
            "lanelet2_osm": str(Path(osm_path).resolve()),
            "lanelet2_osm_sha256": _file_sha256(osm_path),
            "routing_graph": str(Path(routing_path).resolve()),
        },
        "exporter_statistics": exporter_statistics or {},
        "summary": dict(sorted(statuses.items())),
        "checks": checks,
    }
    return report


def write_report(report, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")
    return path
