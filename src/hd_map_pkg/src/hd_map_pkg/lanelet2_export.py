"""Deterministic MGeo 3.0 to Lanelet2 OSM exporter.

The exporter keeps native MGeo IDs as tags, reuses single source boundary ways so
adjacent lanelets share topology, and records every unavoidable derivation.
"""

import json
import math
from collections import defaultdict
from pathlib import Path
from xml.sax.saxutils import quoteattr

from .geometry import (
    cumulative_lengths,
    closest_polyline_distance,
    convex_hull,
    distance_2d,
    offset_polyline,
    orient_and_stitch,
    point_at_progress,
    polyline_length,
    progress_along_polyline,
    segment_distance_2d,
    simplify_rdp,
    slice_polyline,
)


class _UnionFind(object):
    def __init__(self):
        self.parent = {}
        self.members = {}

    def add(self, item):
        if item not in self.parent:
            self.parent[item] = item
            self.members[item] = {item}

    def find(self, item):
        self.add(item)
        parent = self.parent[item]
        if parent != item:
            self.parent[item] = self.find(parent)
        return self.parent[item]

    def union(self, first, second):
        first_root = self.find(first)
        second_root = self.find(second)
        if first_root == second_root:
            return True
        if repr(first_root) <= repr(second_root):
            self.parent[second_root] = first_root
            self.members[first_root].update(self.members.pop(second_root))
        else:
            self.parent[first_root] = second_root
            self.members[second_root].update(self.members.pop(first_root))
        return True

    def can_union_without_opposites(self, first, second, opposites):
        first_root = self.find(first)
        second_root = self.find(second)
        if first_root == second_root:
            return True
        second_members = self.members[second_root]
        return not any(opposites.get(item) in second_members
                       for item in self.members[first_root])

    def union_without_opposites(self, first, second, opposites):
        if not self.can_union_without_opposites(first, second, opposites):
            return False
        return self.union(first, second)


def _as_code(value):
    if isinstance(value, list):
        value = value[0] if value else ""
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _first(value, default=""):
    if isinstance(value, list):
        return value[0] if value else default
    return value if value is not None else default


def _shape_subtype(shape):
    normalized = str(_first(shape, "unknown")).strip().lower().replace("_", " ")
    return {
        "broken": "dashed",
        "dashed": "dashed",
        "solid": "solid",
        "solid solid": "solid_solid",
        "solid broken": "solid_dashed",
        "broken solid": "dashed_solid",
        "none": "unknown",
        "": "unknown",
    }.get(normalized, normalized.replace(" ", "_"))


def boundary_tags(boundary, mapping):
    """Translate NGII/MGeo line semantics into Lanelet2 line-string tags."""
    code = _as_code(boundary.get("lane_type"))
    subtype = _shape_subtype(boundary.get("lane_shape"))
    color = str(_first(boundary.get("lane_color"), "unknown")).lower()
    if code in set(mapping.get("stop_line_codes", [])):
        lanelet_type = "stop_line"
        subtype = "solid"
        category = "stop_line"
    elif code in set(mapping.get("centerline_codes", [])):
        lanelet_type = "line_thick"
        category = "centerline"
    elif code in set(mapping.get("thick_line_codes", [])):
        lanelet_type = "line_thick"
        category = "lane_boundary"
    elif code in set(mapping.get("road_border_codes", [])):
        lanelet_type = "road_border"
        category = "road_border"
    elif code in set(mapping.get("standalone_marking_codes", [])):
        lanelet_type = "road_marking"
        subtype = "bike_marking"
        category = "standalone_marking"
    else:
        lanelet_type = "line_thin"
        category = "lane_boundary"
    tags = {
        "type": lanelet_type,
        "subtype": subtype,
        "color": color,
        "mgeo:id": str(boundary["idx"]),
        "mgeo:lane_type": str(code if code is not None else ""),
        "mgeo:boundary_category": category,
        "mgeo:lane_shape": str(_first(boundary.get("lane_shape"), "")),
        "mgeo:lane_color": str(_first(boundary.get("lane_color"), "")),
    }
    if lanelet_type == "line_thin" and subtype == "dashed":
        tags["lane_change"] = "yes"
    elif lanelet_type in ("line_thick", "road_border", "stop_line") or "solid" in subtype:
        tags["lane_change"] = "no"
    return tags


SURFACE_MANEUVER = {
    "5371": "straight",
    "5372": "left",
    "5373": "right",
    "5374": "left_right",
    "5379": "straight_left_right",
    "5381": "straight_left",
    "5382": "straight_right",
    "5383": "straight_uturn",
    "5391": "uturn",
    "5392": "left_uturn",
    "5431": "merge_left",
}


def surface_marking_maneuver(marking):
    return SURFACE_MANEUVER.get(str(marking.get("sub_type", "")), "")


class OSMBuilder(object):
    """Minimal in-memory OSM primitive builder with coordinate de-duplication."""

    def __init__(self, transformer, deduplication_m=0.001):
        self.transformer = transformer
        self.resolution = float(deduplication_m)
        self._next_id = 1
        self._coordinate_nodes = {}
        self.nodes = []
        self.ways = []
        self.relations = []

    def new_id(self):
        result = self._next_id
        self._next_id += 1
        return result

    def node(self, mgeo_point, tags=None, identity=None):
        point = [float(value) for value in mgeo_point]
        while len(point) < 3:
            point.append(0.0)
        divisor = self.resolution if self.resolution > 0.0 else 1.0e-9
        coordinate_key = tuple(int(round(value / divisor)) for value in point)
        key = ("identity", str(identity)) if identity is not None else ("coordinate", coordinate_key)
        if key in self._coordinate_nodes:
            return self._coordinate_nodes[key]
        # An explicitly identified endpoint should also satisfy later geometry-only
        # lookups (notably composite bounds that end on the same MGeo lane node).
        coordinate_lookup = ("coordinate", coordinate_key)
        if identity is None and coordinate_lookup in self._coordinate_nodes:
            return self._coordinate_nodes[coordinate_lookup]
        utm = self.transformer.mgeo_to_utm(point)
        latitude, longitude = self.transformer.utm_to_wgs84(utm)
        identifier = self.new_id()
        node_tags = {"ele": "{:.3f}".format(utm[2])}
        if tags:
            node_tags.update(tags)
        self.nodes.append({
            "id": identifier,
            "lat": "{:.10f}".format(latitude),
            "lon": "{:.10f}".format(longitude),
            "tags": node_tags,
        })
        self._coordinate_nodes[key] = identifier
        self._coordinate_nodes.setdefault(coordinate_lookup, identifier)
        return identifier

    def way(self, points, tags, endpoint_identities=None):
        if len(points) < 2:
            raise ValueError("OSM ways require at least two points")
        node_ids = []
        for index, point in enumerate(points):
            identity = None
            if endpoint_identities:
                if index == 0:
                    identity = endpoint_identities[0]
                elif index == len(points) - 1:
                    identity = endpoint_identities[1]
            node_id = self.node(point, identity=identity)
            if not node_ids or node_ids[-1] != node_id:
                node_ids.append(node_id)
        if len(node_ids) < 2 or len(set(node_ids)) < 2:
            raise ValueError("OSM ways require at least two distinct nodes")
        identifier = self.new_id()
        self.ways.append({
            "id": identifier,
            "nodes": node_ids,
            "tags": dict(tags),
        })
        return identifier

    def relation(self, members, tags):
        identifier = self.new_id()
        value = {"id": identifier, "members": list(members), "tags": dict(tags)}
        self.relations.append(value)
        return value

    @staticmethod
    def _tags(lines, tags, indentation):
        for key, value in sorted(tags.items()):
            lines.append("{}<tag k={} v={}/>\n".format(
                indentation, quoteattr(str(key)), quoteattr(str(value))))

    def write(self, output_path, generator):
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8", newline="\n") as stream:
            stream.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
            stream.write("<osm version=\"0.6\" generator={}>\n".format(quoteattr(generator)))
            for node in self.nodes:
                stream.write("  <node id=\"{}\" visible=\"true\" lat=\"{}\" lon=\"{}\">\n".format(
                    node["id"], node["lat"], node["lon"]))
                tag_lines = []
                self._tags(tag_lines, node["tags"], "    ")
                stream.writelines(tag_lines)
                stream.write("  </node>\n")
            for way in self.ways:
                stream.write("  <way id=\"{}\" visible=\"true\">\n".format(way["id"]))
                for node_id in way["nodes"]:
                    stream.write("    <nd ref=\"{}\"/>\n".format(node_id))
                tag_lines = []
                self._tags(tag_lines, way["tags"], "    ")
                stream.writelines(tag_lines)
                stream.write("  </way>\n")
            for relation in self.relations:
                stream.write("  <relation id=\"{}\" visible=\"true\">\n".format(relation["id"]))
                for member_type, ref, role in relation["members"]:
                    stream.write("    <member type={} ref=\"{}\" role={}/>\n".format(
                        quoteattr(member_type), ref, quoteattr(role)))
                tag_lines = []
                self._tags(tag_lines, relation["tags"], "    ")
                stream.writelines(tag_lines)
                stream.write("  </relation>\n")
            stream.write("</osm>\n")
        return output_path


class Lanelet2Exporter(object):
    def __init__(self, dataset, transformer, config):
        self.dataset = dataset
        self.transformer = transformer
        self.config = config
        conversion = config.get("conversion", {})
        self.tolerance = float(conversion.get("geometry_simplification_m", 0.02))
        self.default_width = float(conversion.get("default_lane_width_m", 3.5))
        self.stop_line_radius = float(conversion.get("stop_line_search_radius_m", 35.0))
        self.stop_line_tolerance = float(
            conversion.get("stop_line_intersection_tolerance_m", 0.75))
        self.builder = OSMBuilder(
            transformer, float(conversion.get("node_deduplication_m", 0.001)))
        self.boundary_way_ids = {}
        self.boundary_geometries = {}
        self.centerline_way_ids = {}
        self.lanelet_relations = {}
        self.lanelet_relations_by_link = defaultdict(list)
        self.link_segment_ranges = defaultdict(list)
        self.stop_line_way_ids = {}
        self.signal_way_ids = {}
        self.signal_regulatory_ids = {}
        self._stop_line_match_by_link = {}
        self.crosswalk_area_ids = {}
        self.non_crosswalk_marking_area_ids = {}
        self.surface_marking_area_ids = {}
        self.intersection_area_ids = {}
        self.intersection_geometries = {}
        self.lane_side_way_ids = {}
        self.lane_side_metadata = {}
        self.synthetic_side_count = 0
        self.composite_side_count = 0
        self.completed_side_count = 0
        self.lateral_shared_edges = 0
        self.lateral_unshared_edges = 0
        self.max_routing_endpoint_snap_m = 0.0
        self.routing_endpoint_classes_over_tolerance = 0
        self.unresolved_successor_edges = []
        self.unresolved_internal_segment_edges = []
        self.shared_lane_side_ways = 0
        self.piecewise_source_lane_sides = 0
        self.omitted_degenerate_boundaries = []

    def _simplify(self, points):
        return simplify_rdp(points, self.tolerance)

    def _add_source_boundaries(self):
        mapping = self.config.get("lane_boundary", {})
        for boundary_id, boundary in sorted(self.dataset.lane_boundaries.items()):
            points = self._simplify(boundary.get("points") or [])
            unique_xy = {(round(float(point[0]), 6), round(float(point[1]), 6))
                         for point in points}
            if len(points) < 2 or len(unique_xy) < 2:
                self.omitted_degenerate_boundaries.append(boundary_id)
                continue
            tags = boundary_tags(boundary, mapping)
            aliases = self.dataset.aliases_for(boundary_id)
            if aliases:
                tags["mgeo:aliases"] = ",".join(aliases)
            # These are immutable source-feature copies, not the clipped ways used
            # as lanelet bounds.  Keep their coordinates independent so a reused
            # MGeo lane-node ID can never move or collapse source geometry.
            way_id = self.builder.way(points, tags)
            self.boundary_way_ids[boundary_id] = way_id
            self.boundary_geometries[boundary_id] = points
            if tags["type"] == "stop_line":
                self.stop_line_way_ids[boundary_id] = way_id

    @staticmethod
    def _semantic_signature(tags):
        return tags.get("type"), tags.get("subtype"), tags.get("color")

    def _project_side_fragments(self, link, side, center_points, center_length):
        """Project only fragment endpoints, avoiding the former O(Pb*Pc) scan."""
        fragments = []
        mapping = self.config.get("lane_boundary", {})
        for raw_id in link.get("lane_mark_{}".format(side)) or []:
            source_id = str(raw_id)
            geometry = self.boundary_geometries.get(source_id)
            if not geometry or len(geometry) < 2:
                continue
            start, start_distance = progress_along_polyline(geometry[0], center_points)
            end, end_distance = progress_along_polyline(geometry[-1], center_points)
            points = [list(point) for point in geometry]
            tags = boundary_tags(self.dataset.lane_boundaries[source_id], mapping)
            if end < start:
                start, end = end, start
                start_distance, end_distance = end_distance, start_distance
                points.reverse()
                if tags["subtype"] == "solid_dashed":
                    tags["subtype"] = "dashed_solid"
                elif tags["subtype"] == "dashed_solid":
                    tags["subtype"] = "solid_dashed"
                tags["mgeo:reversed_for_lane_direction"] = "yes"
            start = max(0.0, min(center_length, start))
            end = max(0.0, min(center_length, end))
            if end - start <= 0.01:
                continue
            fragments.append({
                "id": source_id,
                "ids": [source_id],
                "points": points,
                "length": polyline_length(points),
                "start": start,
                "end": end,
                "endpoint_lateral_m": math.sqrt(max(start_distance, end_distance)),
                "tags": tags,
            })
        fragments.sort(key=lambda value: (value["start"], value["end"], value["id"]))
        merged = []
        event_tolerance = float(self.config.get("conversion", {}).get(
            "boundary_event_merge_tolerance_m", 0.10))
        stitch_tolerance = float(self.config.get("conversion", {}).get(
            "boundary_stitch_tolerance_m", 0.50))
        for fragment in fragments:
            if merged:
                previous = merged[-1]
                compatible = (
                    self._semantic_signature(previous["tags"]) ==
                    self._semantic_signature(fragment["tags"]) and
                    fragment["start"] <= previous["end"] + event_tolerance and
                    distance_2d(previous["points"][-1], fragment["points"][0]) <=
                    stitch_tolerance)
                if compatible:
                    if distance_2d(
                            previous["points"][-1], fragment["points"][0]) <= 0.001:
                        previous["points"].extend(fragment["points"][1:])
                    else:
                        previous["points"].extend(fragment["points"])
                    previous["ids"].extend(fragment["ids"])
                    previous["id"] = ",".join(previous["ids"])
                    previous["end"] = max(previous["end"], fragment["end"])
                    previous["length"] = polyline_length(previous["points"])
                    previous["endpoint_lateral_m"] = max(
                        previous["endpoint_lateral_m"],
                        fragment["endpoint_lateral_m"])
                    continue
            merged.append(fragment)
        fragments = merged
        signatures = {self._semantic_signature(value["tags"]) for value in fragments}
        if len(signatures) > 1:
            self.piecewise_source_lane_sides += 1
        if len(fragments) > 1:
            self.composite_side_count += 1
        return fragments

    def _split_positions(self, center_length, fragment_sets):
        merge_tolerance = float(self.config.get("conversion", {}).get(
            "boundary_event_merge_tolerance_m", 0.10))
        minimum_length = float(self.config.get("conversion", {}).get(
            "minimum_lanelet_segment_length_m", 0.10))
        values = [0.0, center_length]
        for fragments in fragment_sets:
            for fragment in fragments:
                values.extend((fragment["start"], fragment["end"]))
        values.sort()
        clusters = []
        for value in values:
            if not clusters or value - clusters[-1][-1] > merge_tolerance:
                clusters.append([value])
            else:
                clusters[-1].append(value)
        result = []
        for cluster in clusters:
            if cluster[0] <= merge_tolerance:
                result.append(0.0)
            elif center_length - cluster[-1] <= merge_tolerance:
                result.append(center_length)
            else:
                result.append(sum(cluster) / len(cluster))
        result = sorted(set(round(value, 6) for value in result))
        changed = True
        while changed and len(result) > 2:
            changed = False
            for index in range(1, len(result)):
                if result[index] - result[index - 1] >= minimum_length:
                    continue
                remove = index if index < len(result) - 1 else index - 1
                del result[remove]
                changed = True
                break
        return result

    def _lane_width(self, link, progress, total_length):
        start = float(link.get("width_start") or link.get("width_end") or self.default_width)
        end = float(link.get("width_end") or link.get("width_start") or self.default_width)
        ratio = 0.0 if total_length <= 1.0e-9 else progress / total_length
        return start + max(0.0, min(1.0, ratio)) * (end - start)

    def _materialize_bound_segment(self, link, side, center_points, center_length,
                                   start, end, fragments):
        midpoint = 0.5 * (start + end)
        center_segment = slice_polyline(center_points, start, end)
        width = self._lane_width(link, midpoint, center_length)
        expected = offset_polyline(
            center_segment, 0.5 * width * (1.0 if side == "left" else -1.0))

        def synthetic_result():
            self.synthetic_side_count += 1
            return {
                "points": expected,
                "tags": {
                    "type": "virtual",
                    "subtype": "synthetic_boundary",
                    "color": "unknown",
                    "mgeo:boundary_category": "virtual",
                    "mgeo:synthetic": "yes",
                    "mgeo:derived": "centerline_offset",
                    "mgeo:source_boundaries": "",
                },
                "source_ids": [],
                "synthetic": True,
            }

        # The former +/-0.10 m midpoint tolerance could select a fragment that
        # did not actually cover this segment.  Both clip ratios then clamped to
        # one endpoint and produced a zero-length Lanelet bound.
        candidates = [value for value in fragments
                      if value["start"] <= midpoint <= value["end"]]
        fragment = None
        if candidates:
            expected_midpoint = point_at_progress(expected, 0.5 * polyline_length(expected))

            def score(value):
                ratio = ((midpoint - value["start"]) /
                         max(value["end"] - value["start"], 1.0e-9))
                point = point_at_progress(
                    value["points"], ratio * value["length"])
                return distance_2d(point, expected_midpoint), value["id"]

            fragment = min(candidates, key=score)
            if score(fragment)[0] > float(self.config.get("validation", {}).get(
                    "max_boundary_to_center_distance_m", 30.0)):
                fragment = None
        if fragment is None:
            return synthetic_result()
        start_ratio = max(0.0, min(
            1.0, (start - fragment["start"]) /
            max(fragment["end"] - fragment["start"], 1.0e-9)))
        end_ratio = max(0.0, min(
            1.0, (end - fragment["start"]) /
            max(fragment["end"] - fragment["start"], 1.0e-9)))
        points = slice_polyline(
            fragment["points"], start_ratio * fragment["length"],
            end_ratio * fragment["length"])
        minimum_geometry = float(self.config.get("conversion", {}).get(
            "node_deduplication_m", 0.001))
        if polyline_length(points) <= minimum_geometry:
            return synthetic_result()
        tags = dict(fragment["tags"])
        tags.pop("mgeo:id", None)
        tags["mgeo:source_boundaries"] = ",".join(fragment["ids"])
        tags["mgeo:derived"] = "source_boundary_chainage_clip"
        return {
            "points": points,
            "tags": tags,
            "source_ids": list(fragment["ids"]),
            "synthetic": False,
        }

    def _plan_lanelet_segments(self):
        plans = {}
        for link_id, link in sorted(self.dataset.links.items()):
            center_points = self._simplify(link.get("points") or [])
            center_length = polyline_length(center_points)
            if len(center_points) < 2 or center_length <= 0.01:
                continue
            left_fragments = self._project_side_fragments(
                link, "left", center_points, center_length)
            right_fragments = self._project_side_fragments(
                link, "right", center_points, center_length)
            positions = self._split_positions(
                center_length, (left_fragments, right_fragments))
            segments = []
            for index, (start, end) in enumerate(zip(positions, positions[1:])):
                center_segment = slice_polyline(center_points, start, end)
                segments.append({
                    "index": index,
                    "count": len(positions) - 1,
                    "start": start,
                    "end": end,
                    "center": center_segment,
                    "left": self._materialize_bound_segment(
                        link, "left", center_points, center_length,
                        start, end, left_fragments),
                    "right": self._materialize_bound_segment(
                        link, "right", center_points, center_length,
                        start, end, right_fragments),
                })
            plans[link_id] = segments
        return plans

    def build_lanelet_segment_geometry(self):
        """Materialize validated lane-side geometry without writing OSM.

        Runtime consumers such as a rolling local planner need the same source
        boundary clipping, synthetic-gap policy, and routing-endpoint treatment
        as the authoritative Lanelet2 export.  This public entry point prevents
        those consumers from reimplementing a subtly different map converter.

        The returned mapping is read-only by convention and belongs to this
        exporter instance.  Create a fresh exporter if a subsequent full export
        is required, because source-boundary OSM primitives are initialized as
        part of this operation.
        """
        if self.boundary_geometries:
            raise RuntimeError("lanelet segment geometry was already initialized")
        self._add_source_boundaries()
        plans = self._plan_lanelet_segments()
        self._resolve_lanelet_endpoints(plans)
        return plans

    @staticmethod
    def _endpoint_token(link_id, segment_index, side, position):
        return link_id, int(segment_index), side, position

    def _apply_endpoint_anchors(self, points, start_anchor, end_anchor):
        """Blend endpoint unification into a bound instead of creating a kink."""
        original = [list(point) for point in points]
        lengths = cumulative_lengths(original)
        total = lengths[-1] if lengths else 0.0
        if len(original) < 2 or total <= 1.0e-9:
            return original
        dimension = max(
            3, max(len(point) for point in original),
            len(start_anchor), len(end_anchor))

        def padded(point):
            return [float(point[index]) if index < len(point) else 0.0
                    for index in range(dimension)]

        first = padded(original[0])
        last = padded(original[-1])
        padded_start_anchor = padded(start_anchor)
        padded_end_anchor = padded(end_anchor)
        start_delta = [padded_start_anchor[index] - first[index]
                       for index in range(dimension)]
        end_delta = [padded_end_anchor[index] - last[index]
                     for index in range(dimension)]
        taper = max(0.01, float(self.config.get("conversion", {}).get(
            "endpoint_snap_taper_m", 10.0)))

        def smoothstep(value):
            value = max(0.0, min(1.0, value))
            return value * value * (3.0 - 2.0 * value)

        adjusted = []
        short = total < 2.0 * taper
        for progress, raw_point in zip(lengths, original):
            point = padded(raw_point)
            if short:
                ratio = smoothstep(progress / total)
                start_weight = 1.0 - ratio
                end_weight = ratio
            else:
                start_weight = 1.0 - smoothstep(progress / taper)
                end_weight = 1.0 - smoothstep((total - progress) / taper)
            adjusted.append([
                point[index] + start_weight * start_delta[index] +
                end_weight * end_delta[index]
                for index in range(dimension)
            ])
        # Avoid floating-point drift at topology-defining endpoints.
        adjusted[0] = padded_start_anchor
        adjusted[-1] = padded_end_anchor
        return adjusted

    def _resolve_lanelet_endpoints(self, plans):
        endpoints = _UnionFind()
        candidates = {}
        synthetic_tokens = {}
        opposites = {}
        threshold = float(self.config.get("validation", {}).get(
            "max_successor_endpoint_gap_m", 5.0))
        # Routing connectivity does not authorize averaging physically
        # different lane boundaries.  In particular, a fork can place two
        # successor sides several metres apart while both remain below the
        # broader validation threshold.  Use the boundary stitching tolerance
        # for geometry unification and retain the wider threshold only for
        # reporting source-map topology gaps.
        merge_threshold = float(self.config.get("conversion", {}).get(
            "routing_endpoint_merge_tolerance_m",
            self.config.get("conversion", {}).get(
                "boundary_stitch_tolerance_m", 0.50)))
        if merge_threshold < 0.0 or merge_threshold > threshold:
            raise ValueError(
                "routing endpoint merge tolerance must be within the "
                "successor validation tolerance")
        for link_id, segments in plans.items():
            for segment in segments:
                for side in ("left", "right"):
                    for position, point in (("start", segment[side]["points"][0]),
                                            ("end", segment[side]["points"][-1])):
                        token = self._endpoint_token(
                            link_id, segment["index"], side, position)
                        endpoints.add(token)
                        candidates[token] = point
                        synthetic_tokens[token] = segment[side]["synthetic"]
                    start_token = self._endpoint_token(
                        link_id, segment["index"], side, "start")
                    end_token = self._endpoint_token(
                        link_id, segment["index"], side, "end")
                    opposites[start_token] = end_token
                    opposites[end_token] = start_token
            for first, second in zip(segments, segments[1:]):
                for side in ("left", "right"):
                    first_token = self._endpoint_token(
                        link_id, first["index"], side, "end")
                    second_token = self._endpoint_token(
                        link_id, second["index"], side, "start")
                    if not endpoints.union_without_opposites(
                            first_token, second_token, opposites):
                        self.unresolved_internal_segment_edges.append({
                            "link": link_id,
                            "from_segment": first["index"],
                            "to_segment": second["index"],
                            "side": side,
                        })

        # Reuse a lateral bound only when the source fragment, semantics and the
        # complete clipped geometry are identical.  This deliberately rejects the
        # KATRI partial-overlap and disjoint cases instead of guessing from nearby
        # centerline endpoints.
        shared_geometry = {}
        for link_id, segments in sorted(plans.items()):
            for segment in segments:
                for side in ("left", "right"):
                    side_plan = segment[side]
                    if not side_plan["source_ids"]:
                        continue
                    geometry_key = tuple(
                        tuple(int(round(float(value) * 1000.0)) for value in point[:3])
                        for point in side_plan["points"])
                    key = (tuple(side_plan["source_ids"]),
                           self._semantic_signature(side_plan["tags"]), geometry_key)
                    tokens = (
                        self._endpoint_token(
                            link_id, segment["index"], side, "start"),
                        self._endpoint_token(
                            link_id, segment["index"], side, "end"),
                    )
                    if key in shared_geometry:
                        first_start, first_end = shared_geometry[key]
                        if (endpoints.can_union_without_opposites(
                                first_start, tokens[0], opposites) and
                                endpoints.can_union_without_opposites(
                                    first_end, tokens[1], opposites)):
                            endpoints.union(first_start, tokens[0])
                            endpoints.union(first_end, tokens[1])
                            self.lateral_shared_edges += 1
                    else:
                        shared_geometry[key] = tokens

        for link_id, segments in sorted(plans.items()):
            if not segments:
                continue
            for successor_id in self.dataset.successors.get(link_id, []):
                following = plans.get(successor_id) or []
                if not following:
                    continue
                gaps = {}
                for side in ("left", "right"):
                    current_token = self._endpoint_token(
                        link_id, segments[-1]["index"], side, "end")
                    following_token = self._endpoint_token(
                        successor_id, following[0]["index"], side, "start")
                    gaps[side] = distance_2d(
                        candidates[current_token], candidates[following_token])
                pairs = [(
                    self._endpoint_token(
                        link_id, segments[-1]["index"], side, "end"),
                    self._endpoint_token(
                        successor_id, following[0]["index"], side, "start"))
                    for side in ("left", "right")]
                safe = all(endpoints.can_union_without_opposites(
                    first, second, opposites) for first, second in pairs)
                if max(gaps.values()) <= merge_threshold and safe:
                    for first, second in pairs:
                        endpoints.union(first, second)
                else:
                    self.unresolved_successor_edges.append({
                        "from": link_id,
                        "to": successor_id,
                        "left_gap_m": round(gaps["left"], 6),
                        "right_gap_m": round(gaps["right"], 6),
                        "reason": (
                            "endpoint_gap"
                            if max(gaps.values()) > threshold
                            else "routing_boundary_misalignment"
                            if max(gaps.values()) > merge_threshold
                            else "would_collapse_lane_bound"
                        ),
                    })

        members = defaultdict(list)
        for token, point in candidates.items():
            members[endpoints.find(token)].append((token, point))
        anchors = {}
        snaps = {}
        adjustments = {}
        classes_over = 0
        for root, values in members.items():
            # Virtual bounds are allowed to bridge missing source geometry.  They
            # conform to real boundary endpoints; immutable source-derived points
            # must never be averaged toward an arbitrary offset line.
            physical = [(token, point) for token, point in values
                        if not synthetic_tokens[token]]
            anchors_from = physical or values
            dimension = max(max(len(point) for _, point in anchors_from), 3)
            anchor = [sum(float(point[index]) if index < len(point) else 0.0
                          for _, point in anchors_from) / len(anchors_from)
                      for index in range(dimension)]
            maximum = 0.0
            for token, point in values:
                adjustment = distance_2d(point, anchor)
                adjustments[token] = adjustment
                snap = 0.0 if synthetic_tokens[token] else adjustment
                snaps[token] = snap
                maximum = max(maximum, snap)
            if maximum > threshold:
                classes_over += 1
            self.max_routing_endpoint_snap_m = max(
                self.max_routing_endpoint_snap_m, maximum)
            anchors[root] = anchor
        self.routing_endpoint_classes_over_tolerance = classes_over

        for link_id, segments in plans.items():
            for segment in segments:
                for side in ("left", "right"):
                    start_token = self._endpoint_token(
                        link_id, segment["index"], side, "start")
                    end_token = self._endpoint_token(
                        link_id, segment["index"], side, "end")
                    segment[side]["points"] = self._apply_endpoint_anchors(
                        segment[side]["points"],
                        anchors[endpoints.find(start_token)],
                        anchors[endpoints.find(end_token)])
                    segment[side]["start_identity"] = (
                        "routing_endpoint:{}".format(repr(endpoints.find(start_token))))
                    segment[side]["end_identity"] = (
                        "routing_endpoint:{}".format(repr(endpoints.find(end_token))))
                    segment[side]["start_snap_m"] = snaps[start_token]
                    segment[side]["end_snap_m"] = snaps[end_token]
                    segment[side]["start_adjustment_m"] = adjustments[start_token]
                    segment[side]["end_adjustment_m"] = adjustments[end_token]

    def _add_lanelets(self):
        plans = self._plan_lanelet_segments()
        self._resolve_lanelet_endpoints(plans)
        side_way_cache = {}
        for link_id, segments in sorted(plans.items()):
            link = self.dataset.links[link_id]
            for segment in segments:
                segment_id = "{}#{}".format(link_id, segment["index"])
                center_tags = {
                    "type": "virtual",
                    "subtype": "centerline",
                    "mgeo:id": link_id,
                    "mgeo:segment_id": segment_id,
                }
                center_way = self.builder.way(segment["center"], center_tags)
                self.centerline_way_ids[segment_id] = center_way
                side_way_ids = {}
                for side in ("left", "right"):
                    side_plan = segment[side]
                    tags = dict(side_plan["tags"])
                    tags["mgeo:routing_start_snap_m"] = "{:.3f}".format(
                        side_plan["start_snap_m"])
                    tags["mgeo:routing_end_snap_m"] = "{:.3f}".format(
                        side_plan["end_snap_m"])
                    if side_plan["synthetic"]:
                        tags["mgeo:virtual_start_adjustment_m"] = "{:.3f}".format(
                            side_plan["start_adjustment_m"])
                        tags["mgeo:virtual_end_adjustment_m"] = "{:.3f}".format(
                            side_plan["end_adjustment_m"])
                    geometry_key = tuple(
                        tuple(int(round(float(value) * 1000.0)) for value in point[:3])
                        for point in side_plan["points"])
                    cache_key = None
                    if side_plan["source_ids"]:
                        cache_key = (
                            tuple(side_plan["source_ids"]),
                            self._semantic_signature(tags), geometry_key,
                            side_plan["start_identity"], side_plan["end_identity"],
                            tags.get("mgeo:routing_start_snap_m"),
                            tags.get("mgeo:routing_end_snap_m"),
                        )
                    if cache_key is not None and cache_key in side_way_cache:
                        side_way_ids[side] = side_way_cache[cache_key]
                        self.shared_lane_side_ways += 1
                    else:
                        side_way_ids[side] = self.builder.way(
                            side_plan["points"], tags,
                            endpoint_identities=(
                                side_plan["start_identity"], side_plan["end_identity"]),
                        )
                        if cache_key is not None:
                            side_way_cache[cache_key] = side_way_ids[side]
                    self.lane_side_way_ids[(link_id, segment["index"], side)] = (
                        side_way_ids[side])
                    self.lane_side_metadata[(link_id, segment["index"], side)] = side_plan

                predecessor_ids = self.dataset.predecessors.get(link_id, [])
                successor_ids = self.dataset.successors.get(link_id, [])
                tags = {
                    "type": "lanelet",
                    "subtype": "road",
                    "location": "urban",
                    "one_way": "yes",
                    "mgeo:id": link_id,
                    "mgeo:segment_id": segment_id,
                    "mgeo:segment_index": str(segment["index"]),
                    "mgeo:segment_count": str(segment["count"]),
                    "mgeo:start_chainage_m": "{:.3f}".format(segment["start"]),
                    "mgeo:end_chainage_m": "{:.3f}".format(segment["end"]),
                    "mgeo:road_id": str(link.get("road_id", "")),
                    "mgeo:from_node": str(link.get("from_node_idx", "")),
                    "mgeo:to_node": str(link.get("to_node_idx", "")),
                    "mgeo:predecessors": ",".join(predecessor_ids),
                    "mgeo:successors": ",".join(successor_ids),
                    "mgeo:left_boundary_ids": ",".join(
                        segment["left"]["source_ids"]),
                    "mgeo:right_boundary_ids": ",".join(
                        segment["right"]["source_ids"]),
                    "mgeo:derived_boundary": "yes",
                    "mgeo:synthetic_boundary": "yes" if (
                        segment["left"]["synthetic"] or
                        segment["right"]["synthetic"]) else "no",
                    "mgeo:ego_lane": str(link.get("ego_lane", "")),
                    "mgeo:related_signal": str(link.get("related_signal") or ""),
                    "mgeo:speed_unit_assumption": "km/h",
                }
                aliases = self.dataset.aliases_for(link_id)
                if aliases:
                    tags["mgeo:aliases"] = ",".join(aliases)
                speed = link.get("max_speed")
                if speed is not None and float(speed) > 0.0:
                    tags["speed_limit"] = "{} km/h".format(int(float(speed)))
                turn_direction = self._turn_direction(link.get("related_signal"))
                if turn_direction:
                    tags["turn_direction"] = turn_direction
                elif str(link.get("related_signal") or "").lower().startswith("uturn"):
                    tags["mgeo:maneuver"] = "uturn"
                if link.get("left_lane_change_dst_link_idx"):
                    tags["mgeo:lane_change_left"] = str(
                        link["left_lane_change_dst_link_idx"])
                if link.get("right_lane_change_dst_link_idx"):
                    tags["mgeo:lane_change_right"] = str(
                        link["right_lane_change_dst_link_idx"])
                members = [
                    ("way", side_way_ids["left"], "left"),
                    ("way", side_way_ids["right"], "right"),
                    ("way", center_way, "centerline"),
                ]
                relation = self.builder.relation(members, tags)
                self.lanelet_relations[segment_id] = relation
                self.lanelet_relations_by_link[link_id].append(relation)
                self.link_segment_ranges[link_id].append(
                    (segment["start"], segment["end"], relation))

    @staticmethod
    def _turn_direction(related_signal):
        value = str(related_signal or "").lower()
        if value.startswith("left"):
            return "left"
        if value.startswith("right"):
            return "right"
        if value.startswith("straight"):
            return "straight"
        return ""

    def _add_crosswalks(self):
        group_for_single = {}
        for group_id, group in self.dataset.crosswalks.items():
            for single_id in group.get("single_crosswalk_list") or []:
                group_for_single[str(single_id)] = group_id
        for crossing_id, crossing in sorted(self.dataset.single_crosswalks.items()):
            points = self._simplify(crossing.get("points") or [])
            if len(points) < 3:
                continue
            if points[0][:2] != points[-1][:2]:
                points.append(list(points[0]))
            sign_type = str(crossing.get("sign_type", ""))
            is_crossing = sign_type in ("5321", "533", "534")
            crossing_subtype = "bicycle_crossing" if sign_type == "534" else "crosswalk"
            outer_way = self.builder.way(points, {
                "type": "pedestrian_marking" if is_crossing else "road_marking",
                "subtype": crossing_subtype if is_crossing else "uphill_slope_marking",
                "mgeo:id": crossing_id,
                "mgeo:sign_type": str(crossing.get("sign_type", "")),
            })
            tags = {
                "type": "multipolygon",
                "subtype": crossing_subtype if is_crossing else "road_marking",
                "location": "urban",
                "area": "yes",
                "mgeo:id": crossing_id,
                "mgeo:group_id": str(group_for_single.get(crossing_id, "")),
                "mgeo:link_ids": ",".join(str(value) for value in crossing.get("link_id_list") or [] if value),
            }
            if is_crossing:
                if sign_type == "534":
                    tags["participant:bicycle"] = "yes"
                else:
                    tags["participant:pedestrian"] = "yes"
                if sign_type == "533":
                    tags["mgeo:raised"] = "yes"
                tags["one_way"] = "no"
            aliases = self.dataset.aliases_for(crossing_id)
            if aliases:
                tags["mgeo:aliases"] = ",".join(aliases)
            relation = self.builder.relation([("way", outer_way, "outer")], tags)
            if is_crossing:
                self.crosswalk_area_ids[crossing_id] = relation["id"]
            else:
                self.non_crosswalk_marking_area_ids[crossing_id] = relation["id"]

    def _add_surface_markings(self):
        markings_by_relation = defaultdict(list)
        for marking_id, marking in sorted(self.dataset.surface_markings.items()):
            points = self._simplify(marking.get("points") or [])
            if len(points) < 3:
                continue
            if points[0][:2] != points[-1][:2]:
                points.append(list(points[0]))
            maneuver = surface_marking_maneuver(marking)
            subtype = "speed_bump" if str(marking.get("sub_type")) == "speedbump" else "arrow"
            outer_way = self.builder.way(points, {
                "type": "road_marking",
                "subtype": subtype,
                "mgeo:id": marking_id,
                "mgeo:sub_type": str(marking.get("sub_type", "")),
                "mgeo:maneuver": maneuver,
            })
            relation = self.builder.relation([("way", outer_way, "outer")], {
                "type": "multipolygon",
                "subtype": subtype,
                "location": "urban",
                "area": "yes",
                "mgeo:id": marking_id,
                "mgeo:maneuver": maneuver,
                "mgeo:link_ids": ",".join(
                    str(value) for value in marking.get("link_id_list") or [] if value),
            })
            aliases = self.dataset.aliases_for(marking_id)
            if aliases:
                relation["tags"]["mgeo:aliases"] = ",".join(aliases)
            self.surface_marking_area_ids[marking_id] = relation["id"]
            for link_id in marking.get("link_id_list") or []:
                link_id = str(link_id)
                ranges = self.link_segment_ranges.get(link_id, [])
                if not ranges:
                    continue
                centroid = [
                    sum(float(point[axis]) for point in points[:-1]) /
                    max(len(points) - 1, 1)
                    for axis in range(2)
                ]
                progress, _ = progress_along_polyline(
                    centroid, self.dataset.links[link_id].get("points") or [])
                containing = [value for value in ranges
                              if value[0] - 0.01 <= progress <= value[1] + 0.01]
                target = min(
                    containing or ranges,
                    key=lambda value: abs(0.5 * (value[0] + value[1]) - progress))
                markings_by_relation[target[2]["id"]].append(
                    (marking_id, maneuver, subtype))
        relations_by_id = {value["id"]: value for value in self.builder.relations}
        for relation_id, values in markings_by_relation.items():
            lanelet = relations_by_id[relation_id]
            lanelet["tags"]["mgeo:surface_markings"] = ",".join(
                value[0] for value in values)
            maneuvers = sorted({value[1] for value in values if value[1]})
            if maneuvers:
                lanelet["tags"]["mgeo:lane_arrows"] = ",".join(maneuvers)
                if "turn_direction" not in lanelet["tags"] and len(maneuvers) == 1 and (
                        maneuvers[0] in ("straight", "left", "right")):
                    lanelet["tags"]["turn_direction"] = maneuvers[0]

    def _junction_points(self, junction):
        road_ids = {str(value) for value in junction.get("road_id_list") or []}
        link_ids = [link_id for road_id in road_ids
                    for link_id in self.dataset.links_by_road.get(road_id, [])]
        if not link_ids:
            return []
        all_points = [point for link_id in link_ids
                      for point in self.dataset.links[link_id].get("points") or []]
        if len(all_points) < 3:
            return []
        min_x = min(point[0] for point in all_points)
        max_x = max(point[0] for point in all_points)
        min_y = min(point[1] for point in all_points)
        max_y = max(point[1] for point in all_points)
        if math.hypot(max_x - min_x, max_y - min_y) <= 120.0:
            return all_points

        incident = defaultdict(list)
        for link_id, link in self.dataset.links.items():
            incident[str(link["from_node_idx"])].append(link_id)
            incident[str(link["to_node_idx"])].append(link_id)
        anchors = set()
        for link_id in link_ids:
            link = self.dataset.links[link_id]
            for node_id in (str(link["from_node_idx"]), str(link["to_node_idx"])):
                incident_roads = {str(self.dataset.links[value].get("road_id", ""))
                                  for value in incident[node_id]}
                if len(incident[node_id]) >= 3 or len(incident_roads & road_ids) >= 2:
                    anchors.add(node_id)
        result = []
        for link_id in link_ids:
            link = self.dataset.links[link_id]
            points = link.get("points") or []
            if str(link["from_node_idx"]) in anchors:
                anchor = points[0]
                result.extend(point for point in points
                              if math.hypot(point[0] - anchor[0], point[1] - anchor[1]) <= 18.0)
            if str(link["to_node_idx"]) in anchors:
                anchor = points[-1]
                result.extend(point for point in points
                              if math.hypot(point[0] - anchor[0], point[1] - anchor[1]) <= 18.0)
        return result or all_points

    def _add_intersections(self):
        for junction_id, junction in sorted(self.dataset.junctions.items()):
            hull = convex_hull(self._junction_points(junction))
            if len(hull) < 4:
                continue
            width = max(point[0] for point in hull) - min(point[0] for point in hull)
            height = max(point[1] for point in hull) - min(point[1] for point in hull)
            area = 0.5 * abs(sum(
                first[0] * second[1] - second[0] * first[1]
                for first, second in zip(hull, hull[1:])))
            outer_way = self.builder.way(self._simplify(hull), {
                "type": "intersection_area",
                "mgeo:id": junction_id,
                "mgeo:derived": "yes",
            })
            relation = self.builder.relation([("way", outer_way, "outer")], {
                "type": "multipolygon",
                "subtype": "intersection_area",
                "location": "urban",
                "area": "yes",
                "mgeo:id": junction_id,
                "mgeo:derived_from": "junction road_id_list convex hull",
                "mgeo:road_ids": ",".join(str(value) for value in junction.get("road_id_list") or []),
                "mgeo:bbox_width_m": "{:.3f}".format(width),
                "mgeo:bbox_height_m": "{:.3f}".format(height),
                "mgeo:area_m2": "{:.3f}".format(area),
            })
            aliases = self.dataset.aliases_for(junction_id)
            if aliases:
                relation["tags"]["mgeo:aliases"] = ",".join(aliases)
            self.intersection_area_ids[junction_id] = relation["id"]
            self.intersection_geometries[junction_id] = hull
            road_ids = {str(value) for value in junction.get("road_id_list") or []}
            for road_id in road_ids:
                for link_id in self.dataset.links_by_road.get(road_id, []):
                    for lanelet in self.lanelet_relations_by_link.get(link_id, []):
                        lanelet["tags"]["intersection_area"] = str(relation["id"])

    def _traffic_light_geometry(self, signal):
        point = [float(value) for value in signal.get("point") or [0.0, 0.0, 0.0]]
        while len(point) < 3:
            point.append(0.0)
        width = float(signal.get("width") or
                      self.config.get("conversion", {}).get("traffic_light_bar_width_m", 0.6))
        heading = math.radians(float(signal.get("heading") or 0.0) + 90.0)
        dx = 0.5 * width * math.cos(heading)
        dy = 0.5 * width * math.sin(heading)
        return [
            [point[0] - dx, point[1] - dy, point[2]],
            [point[0] + dx, point[1] + dy, point[2]],
        ]

    def _stop_lines_for_links(self, link_ids):
        """Match stop bars crossing the downstream portion of each approach."""
        matches = {}
        for link_id in link_ids:
            if link_id in self._stop_line_match_by_link:
                cached = self._stop_line_match_by_link[link_id]
                if cached is not None:
                    boundary_id, lateral_distance, upstream_distance = cached
                    existing = matches.get(boundary_id)
                    value = (lateral_distance, upstream_distance)
                    if existing is None or value < existing:
                        matches[boundary_id] = value
                continue
            link = self.dataset.links.get(link_id)
            points = link.get("points") if link else None
            if not points:
                continue
            downstream_segments = []
            distance_from_end = 0.0
            for upstream, downstream in reversed(list(zip(points, points[1:]))):
                segment_length = math.hypot(
                    downstream[0] - upstream[0], downstream[1] - upstream[1])
                if distance_from_end > self.stop_line_radius:
                    break
                downstream_segments.append((upstream, downstream, distance_from_end))
                distance_from_end += segment_length
            best = None
            for boundary_id in sorted(self.stop_line_way_ids):
                geometry = self.boundary_geometries[boundary_id]
                closest = min(
                    (min(segment_distance_2d(upstream, downstream, start, end)
                         for start, end in zip(geometry, geometry[1:])), progress)
                    for upstream, downstream, progress in downstream_segments)
                if closest[0] <= self.stop_line_tolerance:
                    score = (closest[1], closest[0], boundary_id)
                    if best is None or score < best[0]:
                        best = (score, boundary_id, closest[0], closest[1])
            if best is not None:
                _, boundary_id, lateral_distance, upstream_distance = best
                self._stop_line_match_by_link[link_id] = (
                    boundary_id, lateral_distance, upstream_distance)
                existing = matches.get(boundary_id)
                value = (lateral_distance, upstream_distance)
                if existing is None or value < existing:
                    matches[boundary_id] = value
            else:
                self._stop_line_match_by_link[link_id] = None
        return [(boundary_id, values[0], values[1])
                for boundary_id, values in sorted(matches.items())]

    def _add_traffic_lights(self):
        associations = self.dataset.traffic_light_link_ids()
        crosswalk_associations = self.dataset.traffic_light_crosswalk_ids()
        controller_for_signal = {}
        for controller_id, controller in self.dataset.controllers.items():
            for group in controller.get("TL") or []:
                for signal_id in group:
                    controller_for_signal[str(signal_id)] = controller_id

        for signal_id, signal in sorted(self.dataset.traffic_lights.items()):
            source_type = str(signal.get("type", "unknown"))
            signal_tags = {
                "type": "traffic_light",
                "subtype": "red_green" if source_type == "pedestrian" else "red_yellow_green",
                "height": str(signal.get("height", "")),
                "mgeo:id": signal_id,
                "mgeo:bulbs": json.dumps(signal.get("sub_type") or [], separators=(",", ":")),
                "mgeo:orientation": str(signal.get("orientation", "")),
                "mgeo:signal_type": source_type,
                "mgeo:z_offset": str(signal.get("z_offset", "")),
                "mgeo:controller_id": str(controller_for_signal.get(signal_id, "")),
            }
            aliases = self.dataset.aliases_for(signal_id)
            if aliases:
                signal_tags["mgeo:aliases"] = ",".join(aliases)
            signal_way = self.builder.way(self._traffic_light_geometry(signal), signal_tags)
            self.signal_way_ids[signal_id] = signal_way
            link_ids = associations.get(signal_id, [])
            crosswalk_ids = crosswalk_associations.get(signal_id, [])
            is_pedestrian = str(signal.get("type", "")).lower() == "pedestrian"
            controlled_ids = crosswalk_ids if is_pedestrian else link_ids
            if not controlled_ids:
                continue
            # Lanelet2's TrafficLight regulatory element accepts at most one
            # stop line.  One MGeo head can reference approaches with distinct
            # stop bars, so split that source record into one regulatory element
            # per stop-line group while reusing the immutable signal geometry.
            if is_pedestrian:
                groups = [(list(crosswalk_ids), None, [])]
            else:
                grouped_links = defaultdict(list)
                grouped_matches = defaultdict(list)
                for link_id in link_ids:
                    matches = self._stop_lines_for_links([link_id])
                    stop_id = matches[0][0] if matches else None
                    grouped_links[stop_id].append(link_id)
                    if matches:
                        grouped_matches[stop_id].append(matches[0])
                groups = [
                    (grouped_links[stop_id], stop_id, grouped_matches[stop_id])
                    for stop_id in sorted(
                        grouped_links, key=lambda value: "" if value is None else value)
                ]

            self.signal_regulatory_ids[signal_id] = []
            for group_index, (group_ids, stop_id, stop_matches) in enumerate(groups):
                members = [("way", signal_way, "refers")]
                if stop_id is not None:
                    members.append((
                        "way", self.stop_line_way_ids[stop_id], "ref_line"))
                relation_links = [] if is_pedestrian else group_ids
                relation_crosswalks = group_ids if is_pedestrian else []
                relation = self.builder.relation(members, {
                    "type": "regulatory_element",
                    "subtype": "traffic_light",
                    "mgeo:id": signal_id,
                    "mgeo:regulatory_instance": "{}#{}".format(
                        signal_id, group_index),
                    "mgeo:link_ids": ",".join(relation_links),
                    "mgeo:crosswalk_ids": ",".join(relation_crosswalks),
                    "mgeo:controlled_feature": (
                        "pedestrian_crosswalk" if is_pedestrian else "vehicle_lane"),
                    "mgeo:stop_line_ids": stop_id or "",
                    "mgeo:stop_line_max_lateral_error_m": (
                        "{:.3f}".format(max(value[1] for value in stop_matches))
                        if stop_matches else ""),
                })
                self.signal_regulatory_ids[signal_id].append(relation["id"])
                if is_pedestrian:
                    for crosswalk_id in group_ids:
                        area_id = self.crosswalk_area_ids.get(crosswalk_id)
                        if area_id is not None:
                            area = next(value for value in self.builder.relations
                                        if value["id"] == area_id)
                            area["members"].append(
                                ("relation", relation["id"], "regulatory_element"))
                else:
                    for link_id in group_ids:
                        controlled = self.lanelet_relations_by_link.get(link_id, [])
                        # A traffic light governs the downstream approach segment,
                        # not every upstream segment created by marking changes.
                        for lanelet in controlled[-1:]:
                            lanelet["members"].append(
                                ("relation", relation["id"], "regulatory_element"))

    def _add_metadata(self):
        source = self.config.get("source", {})
        coordinates = self.config.get("coordinates", {})
        self.builder.relation([], {
            "type": "map_metadata",
            "format": "Lanelet2 OSM",
            "mgeo:version": "{}.{}".format(
                self.dataset.global_info.get("maj_ver"), self.dataset.global_info.get("min_ver")),
            "source:status": str(source.get("status", "immutable_candidate")),
            "source:repository": str(source.get("repository", "")),
            "source:commit": str(source.get("commit", "")),
            "source:tree": str(source.get("tree", "")),
            "source:license_status": str(source.get("license_status", "")),
            "mgeo:origin_utm": ",".join(str(value) for value in self.dataset.local_origin_utm),
            "simulator:scene": str(coordinates.get("simulator_scene", "")),
            "simulator:origin_utm": ",".join(
                str(value) for value in coordinates.get("simulator_scene_origin_utm", [])),
        })

    def export(self, output_path, routing_path=None, id_map_path=None):
        self._add_source_boundaries()
        self._add_lanelets()
        self._add_surface_markings()
        self._add_crosswalks()
        self._add_intersections()
        self._add_traffic_lights()
        self._add_metadata()
        generator = self.config.get("conversion", {}).get(
            "osm_generator", "hd_map_pkg/mgeo3_lanelet2")
        result = self.builder.write(output_path, generator)
        if routing_path is not None:
            self.write_routing_graph(routing_path)
        if id_map_path is not None:
            self.write_id_map(id_map_path)
        return result

    def write_id_map(self, output_path):
        value = {
            "format": "hd_map_pkg.mgeo_to_lanelet2_id_map.v2",
            "lanelet_relations": {
                key: [item["id"] for item in values]
                for key, values in sorted(self.lanelet_relations_by_link.items())
            },
            "lanelet_segments": {key: item["id"]
                                  for key, item in sorted(self.lanelet_relations.items())},
            "centerline_ways": dict(sorted(self.centerline_way_ids.items())),
            "source_boundary_ways": dict(sorted(self.boundary_way_ids.items())),
            "traffic_lights": {
                key: {"way": self.signal_way_ids[key],
                      "regulatory_elements": self.signal_regulatory_ids.get(key, [])}
                for key in sorted(self.signal_way_ids)},
            "crosswalk_areas": dict(sorted(self.crosswalk_area_ids.items())),
            "surface_marking_areas": dict(sorted(self.surface_marking_area_ids.items())),
            "intersection_areas": dict(sorted(self.intersection_area_ids.items())),
            "verified_suffix_aliases": dict(sorted(self.dataset.verified_aliases.items())),
        }
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                          sort_keys=True) + "\n", encoding="utf-8")
        return output_path

    def write_routing_graph(self, output_path):
        graph = {
            "format": "hd_map_pkg.explicit_mgeo_routing_graph.v2",
            "coordinate_frame": "MGeo local ENU",
            "links": {},
            "lanelets": {},
            "unresolved_internal_segment_edges": list(
                self.unresolved_internal_segment_edges),
            "unresolved_native_successor_edges": list(self.unresolved_successor_edges),
        }
        for link_id, link in sorted(self.dataset.links.items()):
            lanelets = self.lanelet_relations_by_link.get(link_id, [])
            graph["links"][link_id] = {
                "lanelet_relation_id": lanelets[0]["id"] if len(lanelets) == 1 else None,
                "lanelet_relation_ids": [value["id"] for value in lanelets],
                "predecessors": self.dataset.predecessors.get(link_id, []),
                "successors": self.dataset.successors.get(link_id, []),
                "left_lane_change": link.get("left_lane_change_dst_link_idx"),
                "right_lane_change": link.get("right_lane_change_dst_link_idx"),
                "can_move_left": bool(link.get("can_move_left_lane")),
                "can_move_right": bool(link.get("can_move_right_lane")),
            }
            for index, lanelet in enumerate(lanelets):
                predecessors = ([lanelets[index - 1]["id"]] if index else [
                    values[-1]["id"]
                    for predecessor in self.dataset.predecessors.get(link_id, [])
                    for values in [self.lanelet_relations_by_link.get(predecessor, [])]
                    if values
                ])
                successors = ([lanelets[index + 1]["id"]]
                              if index + 1 < len(lanelets) else [
                                  values[0]["id"]
                                  for successor in self.dataset.successors.get(link_id, [])
                                  for values in [self.lanelet_relations_by_link.get(successor, [])]
                                  if values
                              ])
                graph["lanelets"][str(lanelet["id"])] = {
                    "source_link_id": link_id,
                    "segment_index": index,
                    "segment_count": len(lanelets),
                    "predecessor_lanelet_relation_ids": predecessors,
                    "successor_lanelet_relation_ids": successors,
                }
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(graph, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                               encoding="utf-8")
        return output_path

    def statistics(self):
        return {
            "osm_nodes": len(self.builder.nodes),
            "osm_ways": len(self.builder.ways),
            "osm_relations": len(self.builder.relations),
            "lanelets": len(self.lanelet_relations),
            "source_links": len(self.lanelet_relations_by_link),
            "source_boundaries": len(self.boundary_way_ids),
            "centerlines": len(self.centerline_way_ids),
            "stop_lines": len(self.stop_line_way_ids),
            "crosswalk_areas": len(self.crosswalk_area_ids),
            "non_crosswalk_marking_areas": len(self.non_crosswalk_marking_area_ids),
            "surface_marking_areas": len(self.surface_marking_area_ids),
            "traffic_lights": len(self.signal_way_ids),
            "intersection_areas": len(self.intersection_area_ids),
            "synthetic_lane_sides": self.synthetic_side_count,
            "composite_lane_sides": self.composite_side_count,
            "completed_lane_sides": self.completed_side_count,
            "lateral_shared_edges": self.lateral_shared_edges,
            "lateral_unshared_edges": self.lateral_unshared_edges,
            "max_routing_endpoint_snap_m": round(self.max_routing_endpoint_snap_m, 6),
            "routing_endpoint_classes_over_tolerance": (
                self.routing_endpoint_classes_over_tolerance),
            "unresolved_internal_segment_edges": len(
                self.unresolved_internal_segment_edges),
            "unresolved_native_successor_edges": len(self.unresolved_successor_edges),
            "shared_lane_side_ways": self.shared_lane_side_ways,
            "piecewise_source_lane_sides": self.piecewise_source_lane_sides,
            "omitted_degenerate_source_boundaries": len(
                self.omitted_degenerate_boundaries),
        }
