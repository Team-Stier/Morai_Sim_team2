"""CLI orchestration for the reproducible, offline HD-map build."""

import argparse
import gc
import json
import os
import sys
from pathlib import Path

import yaml

from .coordinates import CoordinateTransformer
from .lanelet2_export import Lanelet2Exporter
from .mgeo_v3 import MGeoImportError, MGeoV3Dataset
from .validation import build_report, write_report
from .viewer import build_viewer_data, open_viewer, write_viewer


def _find_package_root():
    source_candidate = Path(__file__).resolve().parents[2]
    if (source_candidate / "config" / "map_conversion.yaml").is_file():
        return source_candidate
    try:
        import rospkg
    except ImportError:
        return source_candidate
    try:
        return Path(rospkg.RosPack().get_path("hd_map_pkg")).resolve()
    except rospkg.ResourceNotFound:
        return source_candidate


PACKAGE_ROOT = _find_package_root()
DEFAULT_CONFIG = PACKAGE_ROOT / "config" / "map_conversion.yaml"
DEFAULT_OUTPUT = PACKAGE_ROOT / "data" / "derived"


def _configuration(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("configuration root must be a mapping")
    return value


def _resolve(package_root, value):
    path = Path(value)
    return path if path.is_absolute() else (package_root / path).resolve()


def _context(arguments):
    config_path = Path(arguments.config).resolve()
    package_root = config_path.parent.parent
    config = _configuration(config_path)
    source_path = (Path(arguments.source).resolve() if arguments.source else
                   _resolve(package_root, config["source"]["relative_path"]))
    output_dir = Path(arguments.output_dir).resolve()
    dataset = MGeoV3Dataset(
        source_path,
        expected_major=int(config["source"].get("expected_mgeo_major", 3)),
        deduplicate_verified_suffix_clones=bool(
            config.get("conversion", {}).get("deduplicate_verified_suffix_clones", True)),
    )
    coordinates = config.get("coordinates", {})
    transformer = CoordinateTransformer(
        dataset.local_origin_utm,
        coordinates["simulator_scene_origin_utm"],
        utm_zone=int(coordinates.get("utm_zone", 52)),
        northern=bool(coordinates.get("northern_hemisphere", True)),
    )
    return package_root, config, dataset, transformer, output_dir


def _artifact_paths(output_dir):
    return {
        "osm": output_dir / "KATRI_lanelet2.osm",
        "routing": output_dir / "KATRI_routing_graph.json",
        "viewer": output_dir / "KATRI_hd_map_preview.html",
        "report": output_dir / "KATRI_validation_report.json",
        "manifest": output_dir / "KATRI_source_manifest.json",
        "id_map": output_dir / "KATRI_lanelet2_id_map.json",
    }


def _write_manifest(dataset, config, path):
    declared = dataset.declared_hashes()
    files = dataset.source_hashes()
    for filename, values in files.items():
        values["sha256_declared"] = declared.get(filename)
        if values["sha256_raw"] == values["sha256_declared"]:
            values["declared_match_mode"] = "raw"
        elif values["sha256_crlf"] == values["sha256_declared"]:
            values["declared_match_mode"] = "crlf"
        elif values["sha256_declared"] is not None:
            values["declared_match_mode"] = "mismatch"
        else:
            values["declared_match_mode"] = "not_declared"
    manifest = {
        "format": "hd_map_pkg.immutable_source_manifest.v1",
        "status": config.get("source", {}).get("status"),
        "repository": config.get("source", {}).get("repository"),
        "commit": config.get("source", {}).get("commit"),
        "tree": config.get("source", {}).get("tree"),
        "license_status": config.get("source", {}).get("license_status"),
        "verified_suffix_aliases": dict(sorted(dataset.verified_aliases.items())),
        "files": files,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")
    return path


def _convert(dataset, transformer, config, paths):
    paths["osm"].parent.mkdir(parents=True, exist_ok=True)
    temporary_osm = paths["osm"].with_suffix(".osm.tmp")
    temporary_routing = paths["routing"].with_suffix(".json.tmp")
    temporary_id_map = paths["id_map"].with_suffix(".json.tmp")
    exporter = Lanelet2Exporter(dataset, transformer, config)
    exporter.export(temporary_osm, temporary_routing, temporary_id_map)
    os.replace(str(temporary_osm), str(paths["osm"]))
    os.replace(str(temporary_routing), str(paths["routing"]))
    os.replace(str(temporary_id_map), str(paths["id_map"]))
    return exporter


def _reference_path(package_root, config):
    value = config.get("references", {}).get("simulator_global_path")
    return _resolve(package_root, value) if value else None


def command_build(arguments):
    package_root, config, dataset, transformer, output_dir = _context(arguments)
    paths = _artifact_paths(output_dir)
    reference_path = _reference_path(package_root, config)
    print("[1/4] Importing immutable MGeo 3.0 source: {}".format(dataset.root))
    _write_manifest(dataset, config, paths["manifest"])
    print("[2/4] Exporting complete Lanelet2 OSM and explicit routing graph")
    exporter = _convert(dataset, transformer, config, paths)
    print("[3/4] Building standalone interactive preview")
    viewer_data = build_viewer_data(
        dataset, transformer, config, exporter,
        reference_path=reference_path)
    write_viewer(viewer_data, paths["viewer"])
    exporter_statistics = exporter.statistics()
    del viewer_data
    del exporter
    gc.collect()
    print("[4/4] Validating source, OSM semantics, rules, topology and SIM alignment")
    report = build_report(
        dataset, transformer, config, paths["osm"], paths["routing"],
        reference_path=reference_path,
        exporter_statistics=exporter_statistics)
    write_report(report, paths["report"])
    for name, path in paths.items():
        print("  {:9s} {}".format(name + ":", path))
    print("Validation: {} ({})".format(report["overall_status"], report["summary"]))
    if arguments.open:
        opened = open_viewer(paths["viewer"])
        print("Viewer open request: {}".format("sent" if opened else "not handled"))
    return 2 if report["overall_status"] == "fail" else 0


def command_convert(arguments):
    _, config, dataset, transformer, output_dir = _context(arguments)
    paths = _artifact_paths(output_dir)
    _write_manifest(dataset, config, paths["manifest"])
    exporter = _convert(dataset, transformer, config, paths)
    print(json.dumps(exporter.statistics(), indent=2, sort_keys=True))
    print(paths["osm"])
    return 0


def command_validate(arguments):
    package_root, config, dataset, transformer, output_dir = _context(arguments)
    paths = _artifact_paths(output_dir)
    report = build_report(
        dataset, transformer, config, paths["osm"], paths["routing"],
        reference_path=_reference_path(package_root, config))
    write_report(report, paths["report"])
    print(json.dumps({"overall_status": report["overall_status"],
                      "summary": report["summary"]}, indent=2, sort_keys=True))
    print(paths["report"])
    return 2 if report["overall_status"] == "fail" else 0


def command_view(arguments):
    package_root, config, dataset, transformer, output_dir = _context(arguments)
    path = _artifact_paths(output_dir)["viewer"]
    write_viewer(build_viewer_data(
        dataset, transformer, config,
        reference_path=_reference_path(package_root, config)), path)
    print(path)
    if arguments.open:
        open_viewer(path)
    return 0


def command_inspect(arguments):
    _, config, dataset, transformer, _ = _context(arguments)
    origin = transformer.mgeo_to_sim([0.0, 0.0, 0.0])
    latitude, longitude = transformer.utm_to_wgs84(dataset.local_origin_utm)
    print(json.dumps({
        "source": str(dataset.root),
        "status": config.get("source", {}).get("status"),
        "mgeo_version": "{}.{}".format(dataset.global_info.get("maj_ver"),
                                         dataset.global_info.get("min_ver")),
        "counts": dataset.counts(),
        "raw_counts": dataset.raw_counts(),
        "deduplication": dataset.deduplication_counts(),
        "mgeo_origin_in_sim_enu": origin,
        "mgeo_origin_wgs84": {"latitude": latitude, "longitude": longitude},
        "reference_errors": dataset.reference_errors(),
    }, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


def parser():
    value = argparse.ArgumentParser(
        description="Offline MGeo 3.0 → Lanelet2 build, validation and preview")
    value.add_argument("--config", default=str(DEFAULT_CONFIG),
                       help="conversion YAML (default: %(default)s)")
    value.add_argument("--source", help="override immutable MGeo directory")
    value.add_argument("--output-dir", default=str(DEFAULT_OUTPUT),
                       help="derived artifact directory (default: %(default)s)")
    subparsers = value.add_subparsers(dest="command", required=True)
    build = subparsers.add_parser("build-all", help="convert, validate and render preview")
    build.add_argument("--open", action="store_true", help="open preview in the desktop browser")
    build.set_defaults(function=command_build)
    convert = subparsers.add_parser("convert", help="write OSM and routing graph")
    convert.set_defaults(function=command_convert)
    validate = subparsers.add_parser("validate", help="validate existing artifacts")
    validate.set_defaults(function=command_validate)
    view = subparsers.add_parser("view", help="write standalone HTML preview")
    view.add_argument("--open", action="store_true", help="open preview in the desktop browser")
    view.set_defaults(function=command_view)
    inspect = subparsers.add_parser("inspect-source", help="inspect MGeo schema and coordinates")
    inspect.set_defaults(function=command_inspect)
    return value


def main(argv=None):
    arguments = parser().parse_args(argv)
    try:
        result = arguments.function(arguments)
    except (MGeoImportError, OSError, ValueError, KeyError, yaml.YAMLError) as error:
        print("hd_map_tool: {}".format(error), file=sys.stderr)
        result = 2
    raise SystemExit(result)
