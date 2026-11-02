#!/usr/bin/env python3
"""Hash the exact tracked sources and verified research assets used by P4."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import sys
import uuid

import yaml


REPO = Path(__file__).resolve().parents[1]
CONFIG = Path("ros2_ws/src/car_bringup/config/obstacle_course")
CONTRACTS = Path("ros2_ws/src/car_bringup/config/obstacle_course/contracts")
SCENARIO_IDS = ("S0", "S1", "S2", "S3", "S3b", "S4", "S5", "S6-lidar-drop", "S6-camera-drop", "S7")
SCHEMA_NAMES = (
    "decision-config.schema.json", "trigger-matrix.schema.json", "component-evidence.schema.json",
    "decision-event.schema.json", "fault-event.schema.json", "frozen-inputs.schema.json",
    "planner-intent.schema.json", "scenario.schema.json", "run-report.schema.json",
)
RUNTIME_PATHS = (
    Path("tools/fetch_obstacle_course_assets.py"), Path("tools/generate_obstacle_course_world.py"),
    Path("tools/build_obstacle_course_profile.py"), Path("tools/validate_obstacle_course_config.py"),
    Path("tools/check_obstacle_course_sizing.py"),
    Path("tools/freeze_obstacle_course_inputs.py"), Path("tools/verify_obstacle_course_inputs.py"),
    Path("tools/run_obstacle_course.py"),
    Path("ros2_ws/src/car_decision/setup.py"), Path("ros2_ws/src/car_decision/package.xml"),
    Path("ros2_ws/src/car_bringup/launch/obstacle_course.launch.py"),
    Path("ros2_ws/src/car_autonomy_nodes/car_autonomy_nodes/safe_drive_controller.py"),
)
NODE_PATHS = tuple(
    Path("ros2_ws/src/car_decision/car_decision") / name for name in (
        "evidence_common.py", "evidence_node_base.py", "lidar_evidence_node.py",
        "camera_evidence_node.py", "imu_evidence_node.py", "odometry_evidence_node.py",
        "contact_evidence_node.py", "fault_gate_node.py", "decision_engine.py",
        "decision_node.py", "gate_corridor.py", "planner_intent_adapter.py",
        "controller_request_adapter.py", "run_logger.py",
    )
)
NODE_PATHS += (Path("ros2_ws/src/car_decision/car_decision/vehicle_visual_overlay.py"),)


def root_hash_for_inputs(inputs: list[dict[str, str]]) -> str:
    normalized: dict[str, str] = {}
    for item in inputs:
        path = str(item["path"])
        digest = str(item["sha256"])
        if not path or path.startswith("/") or ".." in Path(path).parts:
            raise ValueError(f"invalid relative input path: {path!r}")
        if path in normalized:
            raise ValueError(f"duplicate frozen input path: {path}")
        if not re.fullmatch(r"[a-f0-9]{64}", digest):
            raise ValueError(f"invalid SHA-256 for {path}")
        normalized[path] = digest
    payload = "".join(f"{path}\t{normalized[path]}\n" for path in sorted(normalized))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _locked_asset_paths(lock_path: Path, cache_root: Path) -> list[Path]:
    lock = yaml.safe_load((REPO / lock_path).read_text(encoding="utf-8"))
    result: list[Path] = []
    for asset in lock.get("assets", []):
        if not asset.get("file_name", "").lower().endswith(".glb"):
            continue
        result.append(cache_root / "models/research" / asset["class_id"] / asset["asset_id"] / asset["file_name"])
    return result


def _locked_asset_hashes() -> dict[str, str]:
    result = {}
    for lock, cache in (
        (Path("ros2_ws/src/car_bringup/config/obstacle_course/research_assets.lock.yaml"), Path("artifacts/obstacle-course/assets")),
        (Path("ros2_ws/src/car_bringup/config/obstacle_course/research_vehicle.lock.yaml"), Path("artifacts/obstacle-course/assets")),
        (Path("ros2_ws/src/car_bringup/config/obstacle_course/assets.lock.yaml"), Path("artifacts/obstacle-course/assets")),
    ):
        document = yaml.safe_load((REPO / lock).read_text(encoding="utf-8"))
        for asset in document.get("assets", []):
            if asset.get("file_name", "").lower().endswith(".glb"):
                key = (cache / "models/research" / asset["class_id"] / asset["asset_id"] / asset["file_name"]).as_posix()
                result[key] = asset["file_sha256"]
    return result


def collect_input_paths() -> list[Path]:
    paths: set[Path] = {
        Path("ros2_ws/src/car_bringup/config/rc_vehicle.yaml"),
        Path("ros2_ws/src/car_bringup/config/obstacle_course/research_assets.lock.yaml"),
        Path("ros2_ws/src/car_bringup/config/obstacle_course/research_vehicle.lock.yaml"),
        Path("simulation/models/rc_car/model.sdf"), Path("simulation/models/rc_car/model.config"),
        Path("simulation/worlds/obstacle_condensed.sdf"),
        Path("ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml"),
        Path("artifacts/obstacle-course/dynamics/rc-vehicle-reference.json"),
        Path("ros2_ws/src/car_bringup/config/obstacle_course/assets.lock.yaml"),
        Path("ros2_ws/src/car_bringup/config/obstacle_course/bridge.yaml"),
        Path("ros2_ws/src/car_bringup/config/obstacle_course/vehicle_geometry.yaml"),
        Path("ros2_ws/src/car_bringup/config/obstacle_course/decision.yaml"),
        Path("ros2_ws/src/car_bringup/config/obstacle_course/trigger_matrix.yaml"),
        Path("ros2_ws/src/car_bringup/config/obstacle_course/scenarios/registry.yaml"),
        *[CONFIG / "scenarios" / f"{scenario_id}.yaml" for scenario_id in SCENARIO_IDS],
        *[Path("simulation/worlds/obstacle_course/fragments") / name for name in (
            "barrier_red.sdf", "post_orange.sdf", "gate_blue.sdf", "gate_blocked.sdf", "generic_obstacle.sdf",
        )],
        *[CONTRACTS / name for name in SCHEMA_NAMES],
        *RUNTIME_PATHS, *NODE_PATHS,
    }
    for lock, cache in (
        (Path("ros2_ws/src/car_bringup/config/obstacle_course/research_assets.lock.yaml"), Path("artifacts/obstacle-course/assets")),
        (Path("ros2_ws/src/car_bringup/config/obstacle_course/research_vehicle.lock.yaml"), Path("artifacts/obstacle-course/assets")),
        (Path("ros2_ws/src/car_bringup/config/obstacle_course/assets.lock.yaml"), Path("artifacts/obstacle-course/assets")),
    ):
        asset_paths = _locked_asset_paths(lock, cache)
        paths.update(asset_paths)
        if lock.name == "research_assets.lock.yaml":
            document = yaml.safe_load((REPO / lock).read_text(encoding="utf-8"))
            for asset in document.get("assets", []):
                if not asset.get("file_name", "").lower().endswith(".glb"):
                    continue
                model_root = Path("simulation/models/research") / asset["class_id"] / asset["asset_id"]
                for name in ("model.sdf", "model.config"):
                    candidate = REPO / model_root / name
                    if candidate.is_file():
                        paths.add(model_root / name)
    return sorted(paths, key=lambda item: item.as_posix())


def create_manifest(paths: list[Path] | None = None) -> dict:
    entries = []
    locked_assets = _locked_asset_hashes()
    for relative in (paths if paths is not None else collect_input_paths()):
        relative = Path(relative)
        source = (REPO / relative).resolve()
        try:
            normalized = source.relative_to(REPO).as_posix()
        except ValueError as exc:
            raise ValueError(f"frozen input is outside repository: {relative}") from exc
        if not source.is_file():
            raise FileNotFoundError(f"required frozen input is missing: {normalized}")
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        if normalized in locked_assets and digest != locked_assets[normalized]:
            raise ValueError(f"verified runtime asset does not match its source lock: {normalized}")
        entries.append({"path": normalized, "sha256": digest})
    entries.sort(key=lambda item: item["path"])
    root_hash = root_hash_for_inputs(entries)
    document = {
        "schema_version": "1.0", "manifest_id": str(uuid.uuid4()),
        "created_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "inputs": entries, "root_sha256": root_hash,
    }
    schema_path = REPO / CONTRACTS / "frozen-inputs.schema.json"
    import jsonschema
    jsonschema.Draft202012Validator.check_schema(json.loads(schema_path.read_text(encoding="utf-8")))
    jsonschema.validate(document, json.loads(schema_path.read_text(encoding="utf-8")), format_checker=jsonschema.FormatChecker())
    return document


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        document = create_manifest()
        output = args.output if args.output.is_absolute() else REPO / args.output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f"freeze_obstacle_course_inputs: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"input_count": len(document["inputs"]), "root_sha256": document["root_sha256"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
