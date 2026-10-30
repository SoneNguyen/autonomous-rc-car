#!/usr/bin/env python3
"""Run isolated, fixed-seed RC sensor and actuation checks.

Reports are append-only under ``artifacts/rc-car-modules``. Scenario separation
is checked against the current RC module matrix before simulation starts.
"""
from __future__ import annotations

import argparse
from datetime import UTC, datetime
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import uuid
from typing import Any, Iterable

import jsonschema
from referencing import Registry, Resource
from referencing.jsonschema import DRAFT202012
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
CONFIG_ROOT = REPO_ROOT / "ros2_ws/src/car_bringup/config/rc_module_matrix"
SCHEMA_PATH = REPO_ROOT / "docs/contracts/rc-car-modules/module-scenario.schema.json"
REPORT_SCHEMA_PATH = REPO_ROOT / "docs/contracts/rc-car-modules/module-report.schema.json"
REPORT_ROOT = REPO_ROOT / "artifacts/rc-car-modules"
DETECTOR_PATH = REPO_ROOT / "models/rc_module_matrix/detector.onnx"
DETECTOR_METADATA_PATH = DETECTOR_PATH.with_name("metadata.json")
CORE_MODULES = ("lidar", "camera", "imu_odometry", "contact_tf")
STRETCH_MODULE = "command_path"
MODULES = (*CORE_MODULES, STRETCH_MODULE)
SIMULATION_ONLY_STATEMENT = (
    "These noise and drift tolerances characterize the configured simulated "
    "sensor-noise model only; they do not claim real-IMU or real-vehicle accuracy."
)
SCENARIO_SOURCE_PATHS: dict[str, Path] = {}


class MatrixError(ValueError):
    """Invalid selection, frozen input, or matrix runtime contract."""


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()


def scenario_sha256(scenario: dict[str, Any]) -> str:
    """Hash exact checked-in YAML bytes, falling back to canonical test data."""
    source = SCENARIO_SOURCE_PATHS.get(scenario["scenario_id"])
    return sha256_file(source) if source is not None else hashlib.sha256(_canonical(scenario)).hexdigest()


def _load_yaml(path: Path) -> Any:
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise MatrixError(f"cannot read YAML {path}: {exc}") from exc


def _finite_tree(value: Any, where: str = "$root") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise MatrixError(f"{where}: values must be finite")
    if isinstance(value, dict):
        for key, child in value.items():
            _finite_tree(child, f"{where}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _finite_tree(child, f"{where}[{index}]")


def _pointer_value(document: Any, pointer: str) -> Any:
    if pointer in {"", "/"}:
        return document
    if not pointer.startswith("/"):
        raise MatrixError(f"source reference pointer must be a JSON pointer: {pointer!r}")
    current = document
    for raw in pointer.lstrip("/").split("/"):
        token = raw.replace("~1", "/").replace("~0", "~")
        try:
            current = current[int(token)] if isinstance(current, list) else current[token]
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise MatrixError(f"source reference pointer does not resolve: {pointer}") from exc
    return current


def _validate_source_reference(reference: dict[str, Any], *, label: str) -> None:
    path = (REPO_ROOT / reference["path"]).resolve()
    if not path.is_relative_to(REPO_ROOT.resolve()) or not path.is_file():
        raise MatrixError(f"{label} source path is missing or outside the checkout: {reference['path']}")
    if "source_sha256" in reference and sha256_file(path) != reference["source_sha256"]:
        raise MatrixError(f"{label} source SHA-256 does not match: {reference['path']}")
    if "expected_value" in reference:
        try:
            document = json.loads(path.read_text(encoding="utf-8")) if path.suffix == ".json" else _load_yaml(path)
        except (OSError, json.JSONDecodeError, yaml.YAMLError) as exc:
            raise MatrixError(f"{label} source cannot be parsed: {reference['path']}") from exc
        actual = _pointer_value(document, reference["pointer"])
        if actual != reference["expected_value"]:
            raise MatrixError(f"{label} expected value differs at {reference['path']}#{reference['pointer']}")


def validate_scenario(scenario: Any, *, path: Path | None = None) -> dict[str, Any]:
    """Validate the matrix scenario schema plus cross-field invariants."""
    if not isinstance(scenario, dict):
        raise MatrixError("module scenario must be a YAML object")
    _finite_tree(scenario)
    if scenario.get("run_purpose") == "tolerance_pilot" and scenario.get("quick_case"):
        raise MatrixError("tolerance pilots cannot be marked quick_case")
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    errors = sorted(
        jsonschema.Draft202012Validator(schema).iter_errors(scenario),
        key=lambda error: tuple(str(part) for part in error.absolute_path),
    )
    if errors:
        detail = "; ".join(
            f"{'.'.join(str(part) for part in error.absolute_path) or '$'}: {error.message}"
            for error in errors
        )
        raise MatrixError(f"invalid module scenario {path or scenario.get('scenario_id')}: {detail}")
    world_path = (REPO_ROOT / scenario["world_path"]).resolve()
    if not world_path.is_relative_to(REPO_ROOT.resolve()) or not world_path.is_file():
        raise MatrixError(f"scenario world is missing or outside the checkout: {scenario['world_path']}")
    _validate_source_reference(scenario["source_refs"]["vehicle_profile"], label="vehicle_profile")
    if scenario["module_id"] == "camera":
        detector_id, detector_hash, threshold = _detector_identity()
        declared = scenario["source_refs"]
        if (
            declared.get("detector_id") != detector_id
            or declared.get("detector_sha256") != detector_hash
            or float(declared.get("threshold", -1)) != threshold
        ):
            raise MatrixError("camera scenario must use the matrix-locked detector and threshold")
    for index, reference in enumerate(scenario["source_refs"].get("controller_bounds", [])):
        _validate_source_reference(reference, label=f"controller_bounds[{index}]")
    measurement_ids = [item["measurement_id"] for item in scenario["measurements"]]
    if len(measurement_ids) != len(set(measurement_ids)):
        raise MatrixError("measurement_id values must be unique within a scenario")
    tolerance_ids = [item["tolerance_id"] for item in scenario["tolerances"]]
    if len(tolerance_ids) != len(set(tolerance_ids)):
        raise MatrixError("tolerance_id values must be unique within a scenario")
    unknown_tolerances = {
        item.get("tolerance_id") for item in scenario["ground_truth"].get("expected_observations", [])
        if item.get("tolerance_id") and item["tolerance_id"] not in tolerance_ids
    }
    if unknown_tolerances:
        raise MatrixError(f"ground truth refers to unknown tolerances: {sorted(unknown_tolerances)}")
    for tolerance in scenario["tolerances"]:
        if tolerance["measurement_id"] not in measurement_ids:
            raise MatrixError(f"tolerance {tolerance['tolerance_id']} refers to an unknown measurement")
        if tolerance.get("source_ref"):
            _validate_source_reference(tolerance["source_ref"], label=f"tolerance {tolerance['tolerance_id']}")
    if not _ground_truth_ref(scenario):
        raise MatrixError("every scenario requires a non-empty ground_truth_ref")
    profile = scenario["vehicle_motion"]["speed_profile"]
    times = [float(point["sim_time_s"]) for point in profile]
    if times[0] != 0.0 or any(right <= left for left, right in zip(times, times[1:])):
        raise MatrixError("vehicle_motion.speed_profile must start at zero and increase strictly")
    for obstacle in scenario["obstacle_placements"]:
        obstacle_profile = obstacle.get("speed_profile", [])
        obstacle_times = [float(point["sim_time_s"]) for point in obstacle_profile]
        if obstacle_times and (
            obstacle_times[0] != 0.0
            or any(right <= left for left, right in zip(obstacle_times, obstacle_times[1:]))
        ):
            raise MatrixError(f"obstacle {obstacle['entity_id']} speed profile must start at zero and increase")
    if scenario["run_purpose"] == "tolerance_pilot":
        if scenario.get("quick_case"):
            raise MatrixError("tolerance pilots cannot be marked quick_case")
        if scenario["tolerances"]:
            raise MatrixError("tolerance pilots must have an empty tolerances list")
        if any("tolerance_id" in item for item in scenario["ground_truth"].get("expected_observations", [])):
            raise MatrixError("tolerance pilot expected observations cannot name tolerance IDs")
    elif not scenario["tolerances"]:
        raise MatrixError("qualification scenarios must declare at least one tolerance")
    if path is not None:
        try:
            path.resolve().relative_to(CONFIG_ROOT.resolve())
        except ValueError as exc:
            raise MatrixError(f"scenario path must remain inside {CONFIG_ROOT}") from exc
    return scenario


def load_case_file(path: Path) -> list[dict[str, Any]]:
    document = _load_yaml(path)
    if not isinstance(document, dict) or not isinstance(document.get("scenarios"), list):
        raise MatrixError(f"{path}: expected a scenarios list")
    scenarios = [validate_scenario(item, path=path) for item in document["scenarios"]]
    ids = [item["scenario_id"] for item in scenarios]
    if len(ids) != len(set(ids)):
        raise MatrixError(f"{path}: scenario IDs must be unique")
    for item in scenarios:
        SCENARIO_SOURCE_PATHS[item["scenario_id"]] = path
    return scenarios


def discover_scenarios(module: str, *, pilot: bool = False) -> list[dict[str, Any]]:
    if module not in MODULES:
        raise MatrixError(f"unknown module {module!r}")
    if pilot:
        path = CONFIG_ROOT / "pilots" / f"{module}.yaml"
        if not path.is_file():
            # Camera and IMU have stable descriptive filenames in the plan.
            aliases = {
                "camera": "camera-label-probe.yaml",
                "imu_odometry": "imu-rest-baseline.yaml",
            }
            path = CONFIG_ROOT / "pilots" / aliases.get(module, f"{module}.yaml")
            if not path.is_file():
                raise MatrixError(f"no checked-in tolerance pilot for module {module}")
        scenarios = [validate_scenario(_load_yaml(path), path=path)]
        SCENARIO_SOURCE_PATHS[scenarios[0]["scenario_id"]] = path
    else:
        path = CONFIG_ROOT / f"{module}.yaml"
        if not path.is_file():
            raise MatrixError(f"module configuration is missing: {path.relative_to(REPO_ROOT)}")
        scenarios = load_case_file(path)
    if any(item["module_id"] != module for item in scenarios):
        raise MatrixError(f"{module} configuration contains a scenario for another module")
    return scenarios


def select_scenarios(
    *, module: str | None = None, case: str | None = None, all_modules: bool = False,
    quick: bool = False, pilot: bool = False, include_stretch: bool = False,
) -> tuple[str, dict[str, list[dict[str, Any]]]]:
    """Resolve a CLI selection, enforcing one quick case per module."""
    if case and pilot:
        raise MatrixError("choose only one of --case or --pilot")
    if all_modules and module:
        raise MatrixError("choose either --module or --all")
    if (case or pilot) and (all_modules or quick):
        raise MatrixError("--case and --pilot cannot be combined with --all or --quick")
    if pilot and not module:
        raise MatrixError("--pilot requires --module")
    if case and not module:
        raise MatrixError("--case requires --module")
    if module and module not in MODULES:
        raise MatrixError(f"unknown module {module!r}")
    if module == STRETCH_MODULE and not include_stretch:
        raise MatrixError("command_path is stretch work; pass --include-stretch")
    if all_modules or (quick and not module):
        selected_modules = list(CORE_MODULES)
        if include_stretch:
            selected_modules.append(STRETCH_MODULE)
    else:
        selected_modules = [module] if module else []
    if not selected_modules:
        raise MatrixError("select --list, --module, --case, --pilot, --quick, or --all")
    selected: dict[str, list[dict[str, Any]]] = {}
    for current in selected_modules:
        config_path = CONFIG_ROOT / f"{current}.yaml"
        if pilot:
            pilot_path = CONFIG_ROOT / "pilots" / f"{current}.yaml"
            pilot_alias = {
                "camera": "camera-label-probe.yaml",
                "imu_odometry": "imu-rest-baseline.yaml",
            }.get(current)
            if not pilot_path.is_file() and (pilot_alias is None or not (CONFIG_ROOT / "pilots" / pilot_alias).is_file()):
                selected[current] = []
                continue
        if not pilot and not config_path.is_file():
            if case:
                raise MatrixError(f"module {current} has no scenario set; cannot select case {case!r}")
            selected[current] = []
            continue
        scenarios = discover_scenarios(current, pilot=pilot)
        if case:
            scenarios = [item for item in scenarios if item["scenario_id"] == case]
            if not scenarios:
                raise MatrixError(f"case {case!r} is not in module {current}")
        if quick:
            quick_cases = [item for item in scenarios if item.get("quick_case") is True]
            if len(quick_cases) != 1:
                raise MatrixError(f"{current} must have exactly one quick_case; found {len(quick_cases)}")
            scenarios = quick_cases
        selected[current] = scenarios
    mode = "pilot" if pilot else "quick" if quick else "case" if case else "full_matrix" if all_modules and include_stretch else "core_matrix" if all_modules else "module"
    return mode, selected


def run_overlap_preflight(scenarios: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    """Reject duplicate physical setups among current qualification and pilot cases."""
    candidates = list(scenarios)
    if not candidates:
        raise MatrixError("overlap preflight requires at least one candidate scenario")
    candidate_hash = hashlib.sha256(_canonical([scenario_sha256(item) for item in candidates])).hexdigest()
    now = datetime.now(UTC).isoformat().replace("+00:00", "Z")
    declared: list[dict[str, Any]] = []
    for module in MODULES:
        module_path = CONFIG_ROOT / f"{module}.yaml"
        if module_path.is_file():
            declared.extend(load_case_file(module_path))
        pilot_aliases = {
            "camera": "camera-label-probe.yaml",
            "imu_odometry": "imu-rest-baseline.yaml",
        }
        pilot_path = CONFIG_ROOT / "pilots" / pilot_aliases.get(module, f"{module}.yaml")
        if pilot_path.is_file():
            pilot = validate_scenario(_load_yaml(pilot_path), path=pilot_path)
            SCENARIO_SOURCE_PATHS[pilot["scenario_id"]] = pilot_path
            declared.append(pilot)

    all_scenarios = {item["scenario_id"]: item for item in declared}
    for candidate in candidates:
        existing = all_scenarios.get(candidate["scenario_id"])
        if existing is not None and scenario_sha256(existing) != scenario_sha256(candidate):
            return [{
                "protocol_id": "matrix-unique-physical-set-v1",
                "checker_path": "tools/rc_module_matrix.py::run_overlap_preflight",
                "passed": False, "checked_at_utc": now,
                "candidate_set_sha256": candidate_hash,
                "reference_manifest_hashes": {}, "threshold_source": "matrix scenario contracts",
                "comparisons": [], "error": f"scenario ID is reused with different content: {candidate['scenario_id']}",
            }]
        all_scenarios[candidate["scenario_id"]] = candidate

    signatures: dict[str, str] = {}
    for item in all_scenarios.values():
        speed = [
            (float(point["sim_time_s"]), float(point["speed_mps"]))
            for point in item["vehicle_motion"]["speed_profile"]
        ]
        active_obstacles = [
            row for row in item["obstacle_placements"]
            if -14.0 <= float(row["pose"]["x_m"]) <= 14.0
            and -2.5 <= float(row["pose"]["y_m"]) <= 2.5
        ]
        setup = item.get("setup", {})
        condition_keys = {
            "lighting_preset", "quality_preset", "floor_texture",
            "expect_detector_stale", "suppress_detector", "condition_tags",
        }
        condition_signature = {key: setup[key] for key in sorted(condition_keys) if key in setup}
        signature = _canonical({
            "start_pose": item["start_pose"],
            "obstacles": sorted(active_obstacles, key=lambda row: row["entity_id"]),
            "speed_profile": speed,
            "conditions": condition_signature,
            "world_id": item["world_id"],
        }).decode("utf-8")
        previous = signatures.get(signature)
        if previous is not None and previous != item["scenario_id"]:
            error = f"duplicate physical setup: {previous} and {item['scenario_id']}"
            return [{
                "protocol_id": "matrix-unique-physical-set-v1",
                "checker_path": "tools/rc_module_matrix.py::run_overlap_preflight",
                "passed": False, "checked_at_utc": now,
                "candidate_set_sha256": candidate_hash,
                "reference_manifest_hashes": {}, "threshold_source": "matrix scenario contracts",
                "comparisons": [], "error": error,
            }]
        signatures[signature] = item["scenario_id"]

    source_hashes = {
        item["scenario_id"]: scenario_sha256(item)
        for item in all_scenarios.values() if item["scenario_id"] not in {c["scenario_id"] for c in candidates}
    }
    return [{
        "protocol_id": "matrix-unique-physical-set-v1",
        "checker_path": "tools/rc_module_matrix.py::run_overlap_preflight",
        "passed": True, "checked_at_utc": now,
        "candidate_set_sha256": candidate_hash,
        "reference_manifest_hashes": source_hashes,
        "threshold_source": "matrix scenario contracts",
        "comparisons": [], "error": None,
    }]


def _source_identity(scenarios: list[dict[str, Any]]) -> dict[str, Any]:
    paths = {
        "vehicle_profile": REPO_ROOT / "ros2_ws/src/car_bringup/config/rc_vehicle.yaml",
        "research_assets": REPO_ROOT / "ros2_ws/src/car_bringup/config/rc_module_matrix/research_assets.lock.yaml",
        "research_vehicle": REPO_ROOT / "ros2_ws/src/car_bringup/config/rc_module_matrix/research_vehicle.lock.yaml",
        "lighting_profile": REPO_ROOT / "ros2_ws/src/car_bringup/config/rc_module_matrix/lighting.yaml",
        "quality_profile": REPO_ROOT / "ros2_ws/src/car_bringup/config/rc_module_matrix/quality.yaml",
        "frozen_detector": DETECTOR_PATH,
        "frozen_detector_metadata": DETECTOR_METADATA_PATH,
    }
    scenario_hashes = {
        item["scenario_id"]: scenario_sha256(item) for item in scenarios
    }
    for module in sorted({item["module_id"] for item in scenarios}):
        path = CONFIG_ROOT / f"{module}.yaml"
        if path.is_file():
            paths[f"{module}_matrix_config"] = path
    versions = {
        "ubuntu": _os_release(), "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        "gazebo_version": _command_version(["gz", "sim", "--version"]),
    }
    detector_id, detector_hash, detector_threshold = _detector_identity(required=False)
    return {
        "ubuntu": versions["ubuntu"], "ros_distro": versions["ros_distro"],
        "gazebo_version": versions["gazebo_version"], "ros_gz_version": None,
        "world_id": next(iter({item["world_id"] for item in scenarios})) if len({item["world_id"] for item in scenarios}) == 1 else None,
        "detector_id": detector_id, "detector_sha256": detector_hash,
        "detector_threshold": detector_threshold,
        "config_hashes": {key: sha256_file(path) for key, path in paths.items() if path.is_file()},
        "source_hashes": scenario_hashes,
    }


def _detector_identity(*, required: bool = True) -> tuple[str | None, str | None, float | None]:
    """Return the locked inference identity and fail if its bytes drift."""
    if not DETECTOR_PATH.is_file() or not DETECTOR_METADATA_PATH.is_file():
        if required:
            raise MatrixError("matrix detector or metadata is missing")
        return None, None, None
    metadata = json.loads(DETECTOR_METADATA_PATH.read_text(encoding="utf-8"))
    digest = sha256_file(DETECTOR_PATH)
    if metadata.get("model_sha256") != digest:
        raise MatrixError("matrix detector bytes do not match their metadata hash")
    threshold = metadata.get("confidence_threshold")
    if not isinstance(threshold, (int, float)) or not 0.0 < float(threshold) <= 1.0:
        raise MatrixError("matrix detector metadata has no valid confidence threshold")
    detector_id = metadata.get("model_id")
    if not isinstance(detector_id, str) or not detector_id:
        raise MatrixError("matrix detector metadata has no model identity")
    return detector_id, digest, float(threshold)


def _os_release() -> str:
    try:
        for line in Path("/etc/os-release").read_text().splitlines():
            if line.startswith("PRETTY_NAME="):
                return line.split("=", 1)[1].strip().strip('"')
    except OSError:
        pass
    return "unknown"


def _command_version(command: list[str]) -> str:
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=3, check=False)
        return (result.stdout or result.stderr).strip().splitlines()[0][:160]
    except (OSError, subprocess.TimeoutExpired, IndexError):
        return "unavailable"


def _ground_truth_ref(scenario: dict[str, Any]) -> str:
    references = scenario.get("ground_truth", {}).get("references", [])
    if not references:
        raise MatrixError(f"{scenario['scenario_id']} has no ground truth reference")
    for ref in references:
        if isinstance(ref, dict) and ref.get("entity_id"):
            return f"{ref.get('path', 'scenario-ground-truth')}#entity={ref['entity_id']}"
        if isinstance(ref, dict) and ref.get("ref"):
            return str(ref["ref"])
    ref = references[0]
    if isinstance(ref, str):
        return ref
    if isinstance(ref, dict):
        return str(ref.get("ref", ref.get("path", ref.get("entity_id", ""))))
    return ""


def _report_path(report_id: str) -> Path:
    root = REPORT_ROOT.resolve()
    path = (root / report_id).resolve()
    if not path.is_relative_to(root):
        raise MatrixError("report path escaped matrix artifact root")
    return path


def validate_report(report: dict[str, Any]) -> None:
    """Validate one report against the tracked module-matrix schemas."""
    schema = json.loads(REPORT_SCHEMA_PATH.read_text(encoding="utf-8"))
    schema_uri = REPORT_SCHEMA_PATH.resolve().as_uri()
    schema["$id"] = schema_uri
    stop_schema_path = REPO_ROOT / "docs/contracts/rc-car-modules/stop-reasons.json"
    registry = Registry().with_resource(
        schema_uri, Resource.from_contents(schema, default_specification=DRAFT202012)
    ).with_resource(
        stop_schema_path.resolve().as_uri(),
        Resource.from_contents(json.loads(stop_schema_path.read_text(encoding="utf-8")), default_specification=DRAFT202012),
    )
    validator = jsonschema.Draft202012Validator(
        schema, registry=registry, format_checker=jsonschema.FormatChecker()
    )
    errors = sorted(validator.iter_errors(report), key=lambda e: tuple(str(p) for p in e.absolute_path))
    if errors:
        detail = "; ".join(f"{list(e.absolute_path)}: {e.message}" for e in errors)
        raise MatrixError(f"internal report does not satisfy tracked schema: {detail}")


def write_report(report: dict[str, Any], *, output_root: Path = REPORT_ROOT) -> Path:
    """Validate and atomically create one immutable report directory."""
    resolved_root = output_root.resolve()
    if resolved_root != REPORT_ROOT.resolve():
        raise MatrixError("matrix reports must remain under artifacts/rc-car-modules")
    validate_report(report)
    report_id = report["report_id"]
    target_dir = _report_path(report_id)
    report_file = target_dir / f"{report['module_id']}.json"
    if target_dir.exists():
        if report_file.exists() or (target_dir / ".report.json.tmp").exists():
            raise MatrixError(f"refusing to overwrite append-only report {report_file}")
    else:
        target_dir.mkdir(parents=True, exist_ok=False)
    temp = target_dir / ".report.json.tmp"
    temp.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temp.replace(report_file)
    return report_file


def _blocked_result(scenario: dict[str, Any], reason: str) -> dict[str, Any]:
    return {
        "scenario_id": scenario["scenario_id"],
        "scenario_sha256": scenario_sha256(scenario),
        "seed": scenario["seed"], "world_id": scenario["world_id"],
        "status": "blocked", "ground_truth_ref": _ground_truth_ref(scenario),
        "observations": [], "evidence_refs": [], "reason": reason,
    }


def derive_report_status(
    *, mode: str, required_case_ids: list[str], scenario_results: list[dict[str, Any]],
) -> tuple[str, bool]:
    """Keep smoke, partial, and pilot observations from qualifying a module."""
    statuses = [item.get("status") for item in scenario_results]
    if any(status == "blocked" for status in statuses):
        return "blocked", False
    if any(status == "fail" for status in statuses):
        return "fail", False
    if mode == "pilot":
        return "partial", False
    if mode == "quick":
        return ("smoke" if statuses and all(status == "pass" for status in statuses) else "partial"), False
    if mode == "case":
        return "partial", False
    observed = {item.get("scenario_id") for item in scenario_results}
    if observed != set(required_case_ids):
        return "partial", False
    if statuses and all(status == "pass" for status in statuses):
        return "pass", True
    return "partial", False


def _make_report(
    *, module: str, mode: str, scenarios: list[dict[str, Any]], required_case_ids: list[str],
    overlap: list[dict[str, Any]], reason: str | None, run_id: str,
    scenario_results: list[dict[str, Any]] | None = None,
    implementation_state: str = "capture_registered",
) -> dict[str, Any]:
    results = scenario_results if scenario_results is not None else [_blocked_result(item, reason) for item in scenarios]
    status, qualified = derive_report_status(
        mode=mode, required_case_ids=required_case_ids, scenario_results=results
    )
    report: dict[str, Any] = {
        "schema_version": "1.0",
        "report_id": run_id,
        "created_utc": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "module_id": module,
        "mode": mode,
        "status": status,
        "qualified": qualified,
        "required_case_ids": required_case_ids,
        "scenario_results": results,
        "overlap_checks": overlap,
        "source_identity": _source_identity(scenarios),
        "diagnosis": None,
        "stop_reason": None,
        "protected_metrics_written": False,
        "implementation_state": implementation_state,
        "reason": reason,
    }
    if module == "imu_odometry":
        report["simulation_noise_statement"] = SIMULATION_ONLY_STATEMENT
    return report


def _list_cases(include_stretch: bool) -> int:
    modules = (*CORE_MODULES, *((STRETCH_MODULE,) if include_stretch else ()))
    for module in modules:
        try:
            scenarios = discover_scenarios(module)
        except MatrixError as exc:
            print(f"{module}: qualification set unavailable ({exc})")
            try:
                pilots = discover_scenarios(module, pilot=True)
            except MatrixError:
                continue
            print("  unscored pilots:")
            for pilot in pilots:
                print(f"    {pilot['scenario_id']}")
            continue
        print(f"{module}:")
        for item in scenarios:
            print(f"  {item['scenario_id']}{' [quick]' if item.get('quick_case') else ''}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--module", choices=MODULES)
    parser.add_argument("--case")
    parser.add_argument("--all", action="store_true", help="run all core module sets")
    parser.add_argument("--quick", action="store_true", help="one explicitly marked smoke case per selected module")
    parser.add_argument("--pilot", action="store_true", help="run the selected module's unscored tolerance pilot")
    parser.add_argument("--include-stretch", action="store_true")
    parser.add_argument("--world", choices=("rc_module_matrix",))
    parser.add_argument("--output-root", type=Path, default=REPORT_ROOT)
    args = parser.parse_args(argv)
    try:
        if args.list:
            if any((args.module, args.case, args.all, args.quick, args.pilot)):
                raise MatrixError("--list cannot be combined with a run selection")
            return _list_cases(args.include_stretch)
        mode, selection = select_scenarios(
            module=args.module, case=args.case, all_modules=args.all, quick=args.quick,
            pilot=args.pilot, include_stretch=args.include_stretch,
        )
        scenarios = [scenario for group in selection.values() for scenario in group]
        if scenarios:
            from tools.checks.runtime_capture import package_config_root
            package_config_root()
        if args.output_root.resolve() != REPORT_ROOT.resolve():
            raise MatrixError("report output is fixed at artifacts/rc-car-modules")
        if args.world and any(item["world_id"] != args.world for item in scenarios):
            raise MatrixError("--world must match the checked-in research world; scenario files are immutable")
        overlap = run_overlap_preflight(scenarios) if scenarios else []
        failed_preflights = [record for record in overlap if not record["passed"]]
        if failed_preflights:
            failures = "; ".join(record["error"] or "overlap failed" for record in failed_preflights)
            reason = f"overlap preflight failed: {failures}"
        else:
            reason = None
        report_id = str(uuid.uuid4())
        required_by_module = {
            current: ([item["scenario_id"] for item in discover_scenarios(current)]
                      if (CONFIG_ROOT / f"{current}.yaml").is_file() else [])
            for current in selection
        }
        missing_modules = [current for current, cases in selection.items() if not cases]
        report_paths = []
        module_reports = []
        module_reasons: dict[str, str | None] = {}
        needs_aggregate_report = len(selection) > 1
        for current, cases in selection.items():
            submode = "pilot" if mode == "pilot" else "quick" if mode == "quick" else "case" if mode == "case" else "module"
            module_report_id = str(uuid.uuid4())
            module_reason = reason
            implementation_state = "capture_registered"
            if not cases:
                module_reason = f"scenario set is not implemented for {current}; no simulator was started"
                implementation_state = "scenario_set_missing"
                case_results = []
            elif failed_preflights:
                case_results = [_blocked_result(item, reason) for item in cases]
            elif current == "lidar" and not args.pilot:
                from tools.checks.lidar_matrix import run_lidar_case
                case_results = []
                for case in cases:
                    evidence_dir = _report_path(module_report_id) / "evidence" / case["scenario_id"].replace("/", "_")
                    captured = run_lidar_case(case, output_dir=evidence_dir)
                    case_results.append({
                        "scenario_id": case["scenario_id"],
                        "scenario_sha256": scenario_sha256(case),
                        "seed": case["seed"], "world_id": case["world_id"],
                        "status": captured["status"], "ground_truth_ref": _ground_truth_ref(case),
                        "observations": captured["observations"],
                        "evidence_refs": captured["evidence_refs"],
                        "reason": captured.get("reason"),
                    })
            elif current == "camera":
                from tools.checks.camera_matrix import run_camera_case, run_camera_pilot
                case_results = []
                for case in cases:
                    evidence_dir = _report_path(module_report_id) / "evidence" / case["scenario_id"].replace("/", "_")
                    captured = (
                        run_camera_pilot(case, output_dir=evidence_dir)
                        if args.pilot else run_camera_case(case, output_dir=evidence_dir)
                    )
                    case_results.append({
                        "scenario_id": case["scenario_id"],
                        "scenario_sha256": scenario_sha256(case),
                        "seed": case["seed"], "world_id": case["world_id"],
                        "status": captured["status"], "ground_truth_ref": _ground_truth_ref(case),
                        "observations": captured["observations"],
                        "evidence_refs": captured["evidence_refs"],
                        "reason": captured.get("reason"),
                    })
            elif current == "imu_odometry" and args.pilot:
                from tools.checks.imu_odom_matrix import run_imu_odom_pilot
                captured = run_imu_odom_pilot(cases[0], output_dir=_report_path(module_report_id) / "evidence" / "imu-rest-baseline")
                case = cases[0]
                case_results = [{
                    "scenario_id": case["scenario_id"],
                    "scenario_sha256": scenario_sha256(case),
                    "seed": case["seed"], "world_id": case["world_id"],
                    "status": captured["status"], "ground_truth_ref": _ground_truth_ref(case),
                    "observations": captured["observations"],
                    "evidence_refs": captured["evidence_refs"],
                    "reason": captured.get("reason"),
                }]
            elif current == "imu_odometry":
                from tools.checks.imu_odom_matrix import run_imu_odom_case
                case_results = []
                for case in cases:
                    evidence_dir = _report_path(module_report_id) / "evidence" / case["scenario_id"].replace("/", "_")
                    captured = run_imu_odom_case(case, output_dir=evidence_dir)
                    case_results.append({
                        "scenario_id": case["scenario_id"],
                        "scenario_sha256": scenario_sha256(case),
                        "seed": case["seed"], "world_id": case["world_id"],
                        "status": captured["status"], "ground_truth_ref": _ground_truth_ref(case),
                        "observations": captured["observations"],
                        "evidence_refs": captured["evidence_refs"],
                        "reason": captured.get("reason"),
                    })
            elif current == "contact_tf":
                from tools.checks.contact_tf_matrix import run_contact_tf_case
                case_results = []
                for case in cases:
                    evidence_dir = _report_path(module_report_id) / "evidence" / case["scenario_id"].replace("/", "_")
                    captured = run_contact_tf_case(case, output_dir=evidence_dir)
                    case_results.append({
                        "scenario_id": case["scenario_id"],
                        "scenario_sha256": scenario_sha256(case),
                        "seed": case["seed"], "world_id": case["world_id"],
                        "status": captured["status"], "ground_truth_ref": _ground_truth_ref(case),
                        "observations": captured["observations"],
                        "evidence_refs": captured["evidence_refs"],
                        "reason": captured.get("reason"),
                    })
            elif current == "command_path":
                from tools.checks.command_path_matrix import run_command_path_case
                case_results = []
                for case in cases:
                    evidence_dir = _report_path(module_report_id) / "evidence" / case["scenario_id"].replace("/", "_")
                    captured = run_command_path_case(case, output_dir=evidence_dir)
                    case_results.append({
                        "scenario_id": case["scenario_id"],
                        "scenario_sha256": scenario_sha256(case),
                        "seed": case["seed"], "world_id": case["world_id"],
                        "status": captured["status"], "ground_truth_ref": _ground_truth_ref(case),
                        "observations": captured["observations"],
                        "evidence_refs": captured["evidence_refs"],
                        "reason": captured.get("reason"),
                    })
            else:
                unavailable = "no matrix runtime capture backend is registered for this selection"
                case_results = [_blocked_result(item, unavailable) for item in cases]
                module_reason = unavailable
                implementation_state = "capture_unavailable"
            if cases and not failed_preflights and submode != "pilot":
                failed_cases = [item for item in case_results if item.get("status") != "pass"]
                module_reason = (
                    "; ".join(
                        f"{item['scenario_id']}: {item.get('reason') or item.get('status')}"
                        for item in failed_cases
                    ) or None
                )
            report = _make_report(
                module=current, mode=submode, scenarios=cases,
                required_case_ids=required_by_module[current], overlap=overlap,
                reason=module_reason, run_id=module_report_id, scenario_results=case_results,
                implementation_state=implementation_state,
            )
            module_path = write_report(report, output_root=args.output_root)
            report_paths.append(module_path)
            module_reasons[current] = report["reason"]
            module_reports.append({"module_id": current, "status": report["status"],
                                   "qualified": report["qualified"], "report_ref": str(module_path.relative_to(REPO_ROOT))})
        if needs_aggregate_report:
            aggregate_report_results = []
            for report_path in report_paths:
                module_report = json.loads(report_path.read_text(encoding="utf-8"))
                aggregate_report_results.extend(module_report["scenario_results"])
            aggregate = _make_report(
                module="matrix", mode=mode, scenarios=scenarios,
                required_case_ids=[item["scenario_id"] for item in scenarios],
                overlap=overlap,
                reason=("; ".join(f"{current}: {module_reasons[current]}" for current in selection
                                  if module_reasons.get(current)) or None),
                run_id=report_id,
                scenario_results=aggregate_report_results,
            )
            aggregate["aggregate_module_reports"] = module_reports
            aggregate["missing_modules"] = missing_modules
            if missing_modules:
                aggregate["status"] = "partial"
                aggregate["qualified"] = False
                aggregate["implementation_state"] = "module_scenarios_missing"
                aggregate["reason"] = "scenario sets are missing for: " + ", ".join(missing_modules)
            report_paths.append(write_report(aggregate, output_root=args.output_root))
        for path in report_paths:
            print(f"report: {path.relative_to(REPO_ROOT)}")
        if failed_preflights:
            print(f"matrix: blocked ({reason})")
            return 2
        for entry in module_reports:
            print(f"{entry['module_id']}: {entry['status']} (qualified={str(entry['qualified']).lower()})")
        if needs_aggregate_report:
            aggregate_doc = json.loads(report_paths[-1].read_text(encoding="utf-8"))
            print(f"matrix: {aggregate_doc['status']} (qualified={str(aggregate_doc['qualified']).lower()})")
            return 0 if aggregate_doc["status"] in {"pass", "smoke", "partial"} else 2
        return 0 if module_reports[0]["status"] in {"pass", "smoke", "partial"} else 2
    except (MatrixError, OSError, RuntimeError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"rc_module_matrix: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
