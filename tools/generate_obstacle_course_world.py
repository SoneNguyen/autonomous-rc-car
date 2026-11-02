#!/usr/bin/env python3
"""Generate a deterministic scenario world from tracked Obstacle course inputs."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml

REPO = Path(__file__).resolve().parents[1]
CONFIG_ROOT = Path("ros2_ws/src/car_bringup/config/obstacle_course")
SCENARIO_ROOT = CONFIG_ROOT / "scenarios"
DEVELOPMENT_SCENARIO_ROOT = CONFIG_ROOT / "development/scenarios"
BASE_WORLD = Path("simulation/worlds/obstacle_condensed.sdf")
FRAGMENT_ROOT = Path("simulation/worlds/obstacle_course/fragments")
ALLOWED_PURPOSES = {"demonstration", "development", "diagnostic"}


class WorldGenerationError(ValueError):
    """A Obstacle course scenario or world input is invalid."""


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _is_tracked(path: Path) -> bool:
    try:
        relative = path.resolve().relative_to(REPO).as_posix()
    except ValueError:
        return False
    result = subprocess.run(
        ["git", "ls-files", "--error-unmatch", relative],
        cwd=REPO, capture_output=True, text=True, check=False,
    )
    return result.returncode == 0


def _measurement(record: dict, label: str) -> float:
    if not isinstance(record, dict) or "value" not in record:
        raise WorldGenerationError(f"{label} must be a unit-bearing measurement")
    return float(record["value"])


def _pose(scenario_pose: dict, *, z: float = 0.0) -> tuple[float, float, float, float]:
    return (
        _measurement(scenario_pose["x"], "pose.x"),
        _measurement(scenario_pose["y"], "pose.y"),
        z,
        _measurement(scenario_pose["yaw"], "pose.yaw"),
    )


def load_tracked_scenario(scenario_path: Path) -> dict:
    """Load a fixed-seed obstacle-course scenario file."""
    source = scenario_path if scenario_path.is_absolute() else REPO / scenario_path
    source = source.resolve()
    production_root = (REPO / SCENARIO_ROOT).resolve()
    development_root = (REPO / DEVELOPMENT_SCENARIO_ROOT).resolve()
    is_development = source.is_relative_to(development_root)
    if (not source.is_relative_to(production_root) and not is_development) or source.suffix != ".yaml":
        raise WorldGenerationError(
            f"scenario must be under {SCENARIO_ROOT} or {DEVELOPMENT_SCENARIO_ROOT}"
        )
    if not _is_tracked(source):
        raise WorldGenerationError(f"scenario must be tracked before generation: {source.relative_to(REPO)}")
    try:
        scenario = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise WorldGenerationError(f"cannot load scenario {source}: {exc}") from exc
    if not isinstance(scenario, dict) or not isinstance(scenario.get("seed"), int) or scenario["seed"] <= 0:
        raise WorldGenerationError("scenario must contain a positive fixed integer seed")
    if scenario.get("run_purpose") not in ALLOWED_PURPOSES:
        raise WorldGenerationError("scenario must define a recognized run_purpose")
    if is_development and scenario.get("run_purpose") != "development":
        raise WorldGenerationError("development scenarios must declare run_purpose: development")
    if source.stem != scenario.get("scenario_id"):
        raise WorldGenerationError("scenario file name must match scenario_id")
    if scenario.get("world_path") != BASE_WORLD.as_posix():
        raise WorldGenerationError(f"scenario world_path must be {BASE_WORLD.as_posix()}")
    from validate_obstacle_course_config import validate_scenario_semantics
    try:
        validate_scenario_semantics(scenario)
    except (ValueError, KeyError, TypeError) as exc:
        raise WorldGenerationError(str(exc)) from exc
    return scenario


def _fragment_model(fragment_path: Path) -> ET.Element:
    try:
        fragment_root = ET.parse(fragment_path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise WorldGenerationError(f"cannot parse fragment {fragment_path}: {exc}") from exc
    model = fragment_root.find("model")
    if model is None:
        raise WorldGenerationError(f"fragment must contain one model: {fragment_path}")
    return copy.deepcopy(model)


def _set_child_text(parent: ET.Element, tag: str, text: str) -> None:
    child = parent.find(tag)
    if child is None:
        child = ET.Element(tag)
        parent.insert(0, child)
    child.text = text


def _f(value: float) -> str:
    return format(float(value), ".12g")


def build_world(
    scenario_path: Path,
    output_path: Path | None = None,
    *,
    run_purpose: str | None = None,
) -> dict:
    scenario_source = scenario_path if scenario_path.is_absolute() else REPO / scenario_path
    scenario_source = scenario_source.resolve()
    scenario = load_tracked_scenario(scenario_source)
    if run_purpose is not None:
        if run_purpose not in ALLOWED_PURPOSES:
            raise WorldGenerationError(f"unrecognized run_purpose: {run_purpose}")
        scenario = {**scenario, "run_purpose": run_purpose}
    base_path = REPO / BASE_WORLD
    if not base_path.is_file():
        raise WorldGenerationError(f"base world is missing: {BASE_WORLD}")
    try:
        root = ET.parse(base_path).getroot()
    except ET.ParseError as exc:
        raise WorldGenerationError(f"base world is invalid: {exc}") from exc
    world = root.find("world")
    if world is None or world.get("name") != "obstacle_course":
        raise WorldGenerationError("base world must contain world name obstacle_course")

    vehicle_include = next((
        include for include in world.findall("include")
        if include.findtext("name") == "gazebo_truck"
    ), None)
    if vehicle_include is None:
        raise WorldGenerationError("base world is missing the gazebo_truck model include")
    start = _pose(scenario["start_pose"], z=0.0675)
    _set_child_text(vehicle_include, "pose", " ".join(_f(value) for value in (*start[:3], 0.0, 0.0, start[3])))
    overlay = world.find("model[@name='gazebo_truck_visual']")
    if overlay is None:
        raise WorldGenerationError("base world is missing the collisionless research vehicle overlay")
    _set_child_text(overlay, "pose", f"{_f(start[0])} {_f(start[1])} 0.0675 0 0 {_f(start[3])}")

    fragment_ids = []
    entity_ids = ["gazebo_truck", "gazebo_truck_visual"]
    for obstacle in scenario.get("obstacles", []):
        fragment_id = obstacle["mesh_or_fragment"]
        if Path(fragment_id).name != fragment_id:
            raise WorldGenerationError(f"invalid fragment id: {fragment_id}")
        fragment_path = REPO / FRAGMENT_ROOT / f"{fragment_id}.sdf"
        if not fragment_path.is_file():
            raise WorldGenerationError(f"unknown obstacle fragment {fragment_id!r}")
        model = _fragment_model(fragment_path)
        model.set("name", obstacle["entity_id"])
        selected_pose = obstacle.get("initial_pose", obstacle["pose"])
        x, y, z, yaw = _pose(selected_pose)
        _set_child_text(model, "pose", " ".join(_f(value) for value in (x, y, z, 0.0, 0.0, yaw)))
        world.append(model)
        fragment_ids.append(fragment_id)
        entity_ids.append(obstacle["entity_id"])

    ET.indent(root, space="  ")
    payload = ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"
    if output_path is None:
        output_path = REPO / "artifacts/obstacle-course/worlds" / f"{scenario['scenario_id']}.sdf"
    elif not output_path.is_absolute():
        output_path = REPO / output_path
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(payload)

    source_paths = [BASE_WORLD, scenario_source.relative_to(REPO)]
    source_paths.extend(FRAGMENT_ROOT / f"{fragment_id}.sdf" for fragment_id in sorted(set(fragment_ids)))
    source_paths.extend([
        Path("ros2_ws/src/car_bringup/config/obstacle_course/research_assets.lock.yaml"),
        Path("ros2_ws/src/car_bringup/config/obstacle_course/research_vehicle.lock.yaml"),
        Path("ros2_ws/src/car_bringup/config/obstacle_course/assets.lock.yaml"),
        Path("simulation/models/rc_car/model.sdf"),
    ])
    return {
        "scenario_id": scenario["scenario_id"],
        "seed": scenario["seed"],
        "run_purpose": scenario["run_purpose"],
        "world_path": output_path.relative_to(REPO).as_posix() if output_path.is_relative_to(REPO) else str(output_path),
        "world_sha256": hashlib.sha256(payload).hexdigest(),
        "scenario_sha256": _sha256(scenario_source),
        "entity_ids": entity_ids,
        "fragment_ids": fragment_ids,
        "source_sha256": {
            path.as_posix(): _sha256(REPO / path)
            for path in source_paths
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-config", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--run-purpose", choices=sorted(ALLOWED_PURPOSES))
    args = parser.parse_args(argv)
    try:
        metadata = build_world(args.scenario_config, args.output, run_purpose=args.run_purpose)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f"generate_obstacle_course_world: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(metadata, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
