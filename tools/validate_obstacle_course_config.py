#!/usr/bin/env python3
"""Validate obstacle-course contracts, scenarios, and vehicle provenance."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

from jsonschema import Draft202012Validator, FormatChecker
from referencing import Registry, Resource
import yaml


REPO = Path(__file__).resolve().parents[1]
CONTRACTS = Path("ros2_ws/src/car_bringup/config/obstacle_course/contracts")
CONFIG = Path("ros2_ws/src/car_bringup/config/obstacle_course")
SCHEMA_NAMES = (
    "decision-config.schema.json",
    "trigger-matrix.schema.json",
    "component-evidence.schema.json",
    "decision-event.schema.json",
    "fault-event.schema.json",
    "frozen-inputs.schema.json",
    "planner-intent.schema.json",
    "scenario.schema.json",
    "run-report.schema.json",
)
class ContractValidationError(ValueError):
    """An obstacle-course config or report violates its contract."""


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def _load_yaml(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ContractValidationError(f"{path}: expected a YAML mapping")
    return document


def _schema(name: str) -> dict:
    if name not in SCHEMA_NAMES:
        raise ContractValidationError(f"unknown obstacle-course contract schema: {name}")
    path = REPO / CONTRACTS / name
    try:
        schema = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ContractValidationError(f"cannot load {path}: {exc}") from exc
    Draft202012Validator.check_schema(schema)
    return schema


def _contract_registry() -> Registry:
    """Build a local-only registry for relative contract references."""
    registry = Registry()
    for name in SCHEMA_NAMES:
        path = (REPO / CONTRACTS / name).resolve()
        schema = json.loads(path.read_text(encoding="utf-8"))
        # Absolute file identifiers keep resolution local and make network
        # retrieval impossible for this repository's contract set.
        schema["$id"] = path.as_uri()
        registry = registry.with_resource(path.as_uri(), Resource.from_contents(schema))
    return registry


def validate_instance(name: str, instance: dict) -> None:
    """Validate one instance, resolving local schema references without network access."""
    schema = _schema(name)
    schema["$id"] = (REPO / CONTRACTS / name).resolve().as_uri()
    validator = Draft202012Validator(
        schema, registry=_contract_registry(), format_checker=FormatChecker()
    )
    errors = sorted(validator.iter_errors(instance), key=lambda error: list(map(str, error.absolute_path)))
    if errors:
        message = "; ".join(
            f"/{'/'.join(map(str, error.absolute_path))}: {error.message}" for error in errors[:8]
        )
        raise ContractValidationError(f"{name} validation failed: {message}")


def _value(measurement, label: str) -> float:
    if not isinstance(measurement, dict) or "value" not in measurement:
        raise ContractValidationError(f"{label} must be a unit-bearing measurement")
    value = float(measurement["value"])
    if not math.isfinite(value):
        raise ContractValidationError(f"{label} must be finite")
    return value


def validate_scenario_semantics(scenario: dict) -> None:
    """Check cross-field scenario invariants that JSON Schema cannot express."""
    validate_instance("scenario.schema.json", scenario)
    scenario_id = scenario["scenario_id"]
    top_speed = _value(scenario["entry_speed_mps"], "entry_speed_mps")
    route_speed = _value(scenario["route"]["sizing"]["entry_speed_mps"], "route.sizing.entry_speed_mps")
    if not math.isclose(top_speed, route_speed, rel_tol=0.0, abs_tol=1e-9):
        raise ContractValidationError(f"{scenario_id}: route sizing speed differs from entry speed")

    faults = scenario.get("fault_injections", [])
    reveal_faults = []
    for index, fault in enumerate(faults):
        start = _value(fault["start_sim_time"], f"fault_injections[{index}].start_sim_time")
        end = _value(fault["end_sim_time"], f"fault_injections[{index}].end_sim_time")
        if end <= start:
            raise ContractValidationError(
                f"{scenario_id}: fault {index} end_sim_time must be later than start_sim_time"
            )
        if fault["action"] == "reveal_obstacle":
            reveal_faults.append(fault)
            matching = [
                obstacle for obstacle in scenario["obstacles"]
                if obstacle["entity_id"] == fault["entity_id"] and "initial_pose" in obstacle
            ]
            if len(matching) != 1:
                raise ContractValidationError(
                    f"{scenario_id}: reveal fault entity {fault['entity_id']!r} must match exactly one obstacle with initial_pose"
                )
            obstacle = matching[0]
            initial = obstacle["initial_pose"]
            target = obstacle["pose"]
            initial_xy = (_value(initial["x"], "initial_pose.x"), _value(initial["y"], "initial_pose.y"))
            target_xy = (_value(target["x"], "pose.x"), _value(target["y"], "pose.y"))
            if initial_xy == target_xy:
                raise ContractValidationError(f"{scenario_id}: revealed obstacle starts at its target pose")
    if scenario_id == "S5" and len(reveal_faults) != 1:
        raise ContractValidationError("S5 must define exactly one tracked late-reveal fault")
    if scenario_id != "S5" and reveal_faults:
        raise ContractValidationError("only S5 may contain the late-reveal fault")
    has_contact_trigger = "contact" in scenario["expected_trigger_components"]
    contact_branch = scenario["contact_rule"] == "contact_expected_and_latched"
    if has_contact_trigger != contact_branch:
        raise ContractValidationError(
            f"{scenario_id}: contact trigger must be present exactly for the unavoidable-contact branch"
        )


def validate_report_consistency(report: dict) -> None:
    """Check repeat and fault-event identities shared by a run report."""
    validate_instance("run-report.schema.json", report)
    for repeat in report["repeats"]:
        for event in repeat["fault_events"]:
            expected = {
                "run_id": repeat["run_id"],
                "scenario_id": report["scenario_id"],
                "seed": repeat["seed"],
                "run_purpose": report["run_purpose"],
            }
            for key, value in expected.items():
                if event[key] != value:
                    raise ContractValidationError(
                        f"fault event {event.get('event_id')} has {key}={event[key]!r}, expected {value!r}"
                    )
            report_root = report.get("frozen_inputs_root_sha256")
            event_root = event.get("frozen_inputs_root_sha256")
            if report_root is not None and event_root != report_root:
                raise ContractValidationError("fault-event frozen-input root differs from its parent report")


def validate_config_directory() -> dict:
    config_root = REPO / CONFIG
    profile_path = config_root / "rc_vehicle_reference.yaml"
    geometry_path = config_root / "vehicle_geometry.yaml"
    decision_path = config_root / "decision.yaml"
    matrix_path = config_root / "trigger_matrix.yaml"
    profile = _load_yaml(profile_path)
    geometry = _load_yaml(geometry_path)
    decision = _load_yaml(decision_path)
    matrix = _load_yaml(matrix_path)
    measured = profile["dynamics"]
    for key in ("braking_deceleration_mps2", "request_to_deceleration_latency_s"):
        parameter = measured[key]
        if parameter.get("status") != "measured_sim" or float(parameter.get("value", 0)) <= 0:
            raise ContractValidationError(f"vehicle profile {key} must be positive measured_sim data")
    profile_sha = sha256_file(profile_path)
    for label, ref in (("geometry", geometry["profile_reference"]), ("decision", decision["vehicle_profile"])):
        if ref["path"] != profile_path.relative_to(REPO).as_posix() or ref["sha256"] != profile_sha:
            raise ContractValidationError(f"{label} profile reference hash/path is stale")
    validate_instance("decision-config.schema.json", decision)
    validate_instance("trigger-matrix.schema.json", matrix)

    scenarios_dir = config_root / "scenarios"
    scenario_paths = sorted(scenarios_dir.glob("*.yaml")) if scenarios_dir.exists() else []
    scenarios = []
    for path in scenario_paths:
        if path.name == "registry.yaml":
            continue
        scenario = _load_yaml(path)
        validate_scenario_semantics(scenario)
        if path.stem != scenario["scenario_id"]:
            raise ContractValidationError(f"scenario file/name mismatch: {path.name} has id {scenario['scenario_id']}")
        scenarios.append(scenario)

    scenario_ids = [scenario["scenario_id"] for scenario in scenarios]
    if len(scenario_ids) != len(set(scenario_ids)):
        raise ContractValidationError("production scenario IDs must be unique")
    return {
        "profile_sha256": profile_sha,
        "scenario_count": len(scenarios),
        "scenario_ids": scenario_ids,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--all", action="store_true", help="validate all obstacle-course configs")
    args = parser.parse_args(argv)
    if not args.all:
        parser.error("select --all")
    try:
        for name in SCHEMA_NAMES:
            _schema(name)
        result = validate_config_directory()
        print(json.dumps(result, indent=2, sort_keys=True))
        print(f"OBSTACLE COURSE CONFIG VALIDATION PASS ({result['scenario_count']} production scenarios)")
        return 0
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError, yaml.YAMLError) as exc:
        print(f"validate_obstacle_course_config: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
