#!/usr/bin/env python3
"""Validate Feature 007 contracts, tracked scenario inputs, provenance, and split isolation."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

from jsonschema import Draft202012Validator, FormatChecker
from referencing import Registry, Resource
import yaml


REPO = Path(__file__).resolve().parents[1]
FEATURE = Path("docs/features/007-multi-sensor-obstacle-course")
CONTRACTS = FEATURE / "contracts"
CONFIG = Path("ros2_ws/src/car_bringup/config/obstacle_course")
BASELINE = FEATURE / "protected-inputs.json"
OVERLAP_SNAPSHOT = Path("tools/vendor/scenario_overlap/feature002_overlap.py")
OVERLAP_SHA256 = "d01ede3dbdfc73606cc52cd8e03c3f86da07d80c690ed48c4d860837b7849dd3"
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
FEATURE001_COMMIT = "952aa962857ff91e5e1c48bc27a156a4fd751b95"
FEATURE002_COMMIT = "3a0a835179c7366f2921d181436a019b28ba7dca"


class ContractValidationError(ValueError):
    """A Feature 007 config or report violates its contract."""


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
        raise ContractValidationError(f"unknown Feature 007 contract schema: {name}")
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


def project_feature007_scenario(scenario: dict) -> dict:
    """Map a typed Feature 007 scenario into Feature 002's frozen projection shape."""
    start = scenario["start_pose"]
    obstacles = []
    for obstacle in scenario["obstacles"]:
        pose = obstacle["pose"]
        obstacles.append({
            "entity": obstacle["entity_id"],
            "pose": {
                "x_m": _value(pose["x"], f"{obstacle['entity_id']}.pose.x"),
                "y_m": _value(pose["y"], f"{obstacle['entity_id']}.pose.y"),
                "z_m": 0.0,
            },
        })
    return {
        "scenario_id": scenario["scenario_id"],
        "seed": scenario["seed"],
        "start_pose": {
            "x_m": _value(start["x"], "start_pose.x"),
            "y_m": _value(start["y"], "start_pose.y"),
            "yaw_rad": _value(start["yaw"], "start_pose.yaw"),
        },
        "obstacles": obstacles,
        "target_speed_mps": _value(scenario["entry_speed_mps"], "entry_speed_mps"),
    }


def _git_blob(commit: str, path: str) -> bytes:
    result = subprocess.run(
        ["git", "show", f"{commit}:{path}"], cwd=REPO,
        capture_output=True, check=False,
    )
    if result.returncode != 0:
        raise ContractValidationError(
            f"required frozen source blob is unavailable locally: {commit}:{path}; "
            "restore the pinned Git history before using the overlap check"
        )
    return result.stdout


def _historical_source_hash(baseline: dict, path: str, feature: str) -> str:
    entry = next((item for item in baseline["entries"] if item["path"] == path), None)
    if not entry:
        raise ContractValidationError(f"protected baseline does not list historical input {path}")
    source = next((item for item in entry.get("historical_sources", []) if item["feature"] == feature), None)
    if not source:
        raise ContractValidationError(f"protected baseline has no {feature} source hash for {path}")
    return source["sha256"]


def load_frozen_scenarios(baseline_path: Path = REPO / BASELINE) -> tuple[list[dict], dict, float]:
    """Read and hash-check all frozen Feature 001/002 scenario blobs from local Git history."""
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    if FEATURE001_COMMIT not in {source["commit"] for source in baseline["historical_sources"]}:
        raise ContractValidationError("protected baseline is missing the pinned Feature 001 commit")
    if FEATURE002_COMMIT not in {source["commit"] for source in baseline["historical_sources"]}:
        raise ContractValidationError("protected baseline is missing the pinned Feature 002 commit")

    manifest_path = "ros2_ws/src/car_bringup/config/rc_scenario_sets/manifest.json"
    manifest_raw = _git_blob(FEATURE002_COMMIT, manifest_path)
    if sha256_bytes(manifest_raw) != _historical_source_hash(baseline, manifest_path, "feature002"):
        raise ContractValidationError("Feature 002 manifest blob differs from the protected-baseline hash")
    manifest = json.loads(manifest_raw)
    inventory = baseline["feature001_002_manifest_registry_inventory"]
    if manifest.get("evaluation_manifest_sha256") != inventory["feature002_manifest_evaluation_set_sha256"]:
        raise ContractValidationError("Feature 002 manifest evaluation hash differs from the protected baseline")
    registry_path = inventory["feature002_registry_path"]
    registry_raw = _git_blob(FEATURE002_COMMIT, registry_path)
    if sha256_bytes(registry_raw) != _historical_source_hash(baseline, registry_path, "feature002"):
        raise ContractValidationError("Feature 002 registry blob differs from the protected-baseline hash")
    registry = yaml.safe_load(registry_raw)
    registered = registry.get("evaluation_sets", [])
    if not any(item.get("manifest_sha256") == manifest["evaluation_manifest_sha256"] for item in registered):
        raise ContractValidationError("Feature 002 frozen registry does not name the pinned evaluation manifest")

    historical: list[dict] = []
    scenario_root = "ros2_ws/src/car_bringup/config/rc_scenario_sets"
    for split, descriptor in manifest["sets"].items():
        for item in descriptor["scenarios"]:
            path = f"{scenario_root}/{item['path']}"
            raw = _git_blob(FEATURE002_COMMIT, path)
            if sha256_bytes(raw) != item["sha256"] or item["sha256"] != _historical_source_hash(baseline, path, "feature002"):
                raise ContractValidationError(f"Feature 002 frozen scenario hash mismatch: {path}")
            document = yaml.safe_load(raw)
            if document.get("scenario_id") != item["scenario_id"] or document.get("seed") != item["seed"]:
                raise ContractValidationError(f"Feature 002 scenario identity mismatch: {path}")
            historical.append(document)

    for path in inventory["feature001_evaluation_scenario_paths"]:
        raw = _git_blob(FEATURE001_COMMIT, path)
        if sha256_bytes(raw) != _historical_source_hash(baseline, path, "feature001"):
            raise ContractValidationError(f"Feature 001 frozen scenario hash mismatch: {path}")
        historical.append(yaml.safe_load(raw))

    thresholds = manifest["thresholds"]
    taus = {key: float(value) for key, value in thresholds["tau_g"].items()}
    footprint_radius = float(thresholds["footprint_radius_m"])
    return historical, taus, footprint_radius


def validate_overlap(scenarios: list[dict], baseline_path: Path = REPO / BASELINE) -> list[dict]:
    """Reject candidates that overlap any Feature 001/002 frozen set."""
    snapshot_path = REPO / OVERLAP_SNAPSHOT
    if sha256_file(snapshot_path) != OVERLAP_SHA256:
        raise ContractValidationError("pinned Feature 002 overlap snapshot hash mismatch")
    import importlib.util
    spec = importlib.util.spec_from_file_location("feature002_overlap_snapshot", snapshot_path)
    if not spec or not spec.loader:
        raise ContractValidationError("cannot load pinned Feature 002 overlap snapshot")
    overlap = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(overlap)
    historical, taus, footprint_radius = load_frozen_scenarios(baseline_path)
    results = []
    for scenario in scenarios:
        candidate = project_feature007_scenario(scenario)
        result = overlap.nearest_training_distance(candidate, historical, [], taus, footprint_radius)
        if float(result["nearest_train_distance"]) < 1.0:
            raise ContractValidationError(
                f"scenario {scenario['scenario_id']} overlaps frozen Feature 001/002 set "
                f"{result['nearest_scenario_id']} at normalized distance "
                f"{result['nearest_train_distance']:.6f} ({result['limiting_group']})"
            )
        results.append({
            "scenario_id": scenario["scenario_id"],
            "nearest_scenario_id": result["nearest_scenario_id"],
            "normalized_distance": result["nearest_train_distance"],
            "limiting_group": result["limiting_group"],
        })
    return results


def _is_tracked(path: Path) -> bool:
    try:
        relative = path.resolve().relative_to(REPO).as_posix()
    except ValueError:
        return False
    result = subprocess.run(
        ["git", "ls-files", "--error-unmatch", relative], cwd=REPO,
        capture_output=True, text=True, check=False,
    )
    return result.returncode == 0


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
        if not _is_tracked(path):
            raise ContractValidationError(f"scenario must be committed before use: {path.relative_to(REPO)}")
        scenario = _load_yaml(path)
        validate_scenario_semantics(scenario)
        if path.stem != scenario["scenario_id"]:
            raise ContractValidationError(f"scenario file/name mismatch: {path.name} has id {scenario['scenario_id']}")
        scenarios.append(scenario)

    overlap_results = validate_overlap(scenarios) if scenarios else []
    protected = REPO / Path("tools/check_feature_007_protected_paths.py")
    if not protected.is_file():
        raise ContractValidationError("protected-path checker is missing; complete foundational T016 before validation")
    result = subprocess.run(
        [sys.executable, str(protected), "--baseline", str(REPO / BASELINE)],
        cwd=REPO, capture_output=True, text=True, check=False,
    )
    if result.returncode:
        raise ContractValidationError(f"protected-path check failed:\n{result.stdout}{result.stderr}")
    return {
        "profile_sha256": profile_sha,
        "scenario_count": len(scenarios),
        "overlap_checks": overlap_results,
        "protected_path_result": (result.stdout + result.stderr).strip(),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--all", action="store_true", help="validate all currently authored Feature 007 configs")
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
