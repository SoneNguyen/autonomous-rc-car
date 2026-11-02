#!/usr/bin/env python3
"""Validate simulator-to-RC constants and scenario setup sheets."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml

REPO = Path(__file__).resolve().parents[1]
STATUSES = {"scaled", "measured", "placeholder"}
SCENARIO_IDS = ("S0", "S1", "S2", "S3", "S3b", "S4", "S5", "S6-lidar-drop", "S6-camera-drop", "S7")


def numeric_constants(value, prefix=""):
    found = {}
    if isinstance(value, dict):
        if "value" in value and "unit" in value and "provenance" in value:
            item = value["value"]
            numeric = isinstance(item, (int, float)) and not isinstance(item, bool)
            numeric_list = isinstance(item, list) and item and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in item)
            if numeric or numeric_list:
                found[f"{prefix}.value".strip(".")] = value
        for key, child in value.items():
            if key != "value":
                found.update(numeric_constants(child, f"{prefix}.{key}".strip(".")))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.update(numeric_constants(child, f"{prefix}[{index}]"))
    return found


def validate_mapping(mapping_path: Path, profile_path: Path, scenario_dir: Path | list | None, sheets_dir: Path) -> dict:
    mapping = yaml.safe_load(Path(mapping_path).read_text(encoding="utf-8"))
    profile = yaml.safe_load(Path(profile_path).read_text(encoding="utf-8"))
    constants = numeric_constants(profile)
    by_path = {}
    for entry in mapping.get("entries", []):
        path = entry.get("constant_path")
        if not path or path in by_path:
            raise ValueError(f"mapping has missing or duplicate constant path: {path!r}")
        by_path[path] = entry
    missing = sorted(set(constants) - set(by_path))
    extra = sorted(set(by_path) - set(constants))
    if missing:
        raise ValueError(f"unmapped simulator constants: {missing}")
    if extra:
        raise ValueError(f"mapping references unknown constants: {extra}")
    for path, source in constants.items():
        entry = by_path[path]
        for field in ("unit", "provenance", "rc_provenance", "rc_status"):
            if not isinstance(entry.get(field), str) or not entry[field].strip():
                raise ValueError(f"mapping {path} requires {field}")
        if entry["unit"] != source["unit"]:
            raise ValueError(f"mapping {path} unit mismatch")
        if entry["simulation_value"] != source["value"]:
            raise ValueError(f"mapping {path} simulator value mismatch")
        if entry["rc_status"] not in STATUSES:
            raise ValueError(f"mapping {path} rc_status must be scaled, measured, or placeholder")
        if entry["rc_status"] == "placeholder" and entry.get("rc_value") is not None:
            raise ValueError(f"placeholder mapping {path} must not claim a physical RC value")
        if entry["rc_status"] != "placeholder" and entry.get("rc_value") is None:
            raise ValueError(f"mapped RC value required for {entry['rc_status']} constant {path}")
    scenario_ids = []
    if scenario_dir:
        scenario_paths = sorted(Path(scenario_dir).glob("*.yaml"))
        scenarios = [yaml.safe_load(path.read_text(encoding="utf-8")) for path in scenario_paths]
        scenarios = [item for item in scenarios if item.get("scenario_id") in SCENARIO_IDS]
        scenario_ids = [item["scenario_id"] for item in scenarios]
        target = Path(sheets_dir)
        if target.is_file():
            document_path = target.resolve()
            document_rel = document_path.relative_to(REPO).as_posix()
            document = document_path.read_text(encoding="utf-8")
            missing_sheets = []
            for scenario in scenarios:
                configured = str(scenario.get("setup_sheet", ""))
                linked_path, separator, anchor = configured.partition("#")
                expected_anchor = "setup-" + scenario["scenario_id"].lower()
                if (
                    not separator or linked_path != document_rel
                    or anchor != expected_anchor
                    or f'<a id="{expected_anchor}"></a>' not in document
                ):
                    missing_sheets.append(scenario["scenario_id"])
        else:
            missing_sheets = sorted(
                scenario_id for scenario_id in scenario_ids
                if not (target / f"{scenario_id}.md").is_file()
            )
        if missing_sheets:
            raise ValueError(f"missing setup sheet or document anchor for scenarios: {missing_sheets}")
    return {"constant_count": len(constants), "scenario_sheet_count": len(scenario_ids), "status": "PASS"}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=Path("ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml"))
    parser.add_argument("--scenarios", type=Path)
    parser.add_argument("--sheets", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = validate_mapping(args.mapping, args.profile, args.scenarios, args.sheets)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f"validate_real_car_mapping: {exc}", file=sys.stderr)
        return 2
    print(f"REAL-CAR MAPPING PASS ({result['constant_count']} constants; {result['scenario_sheet_count']} setup sheets)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
