#!/usr/bin/env python3
"""Build a deterministic runtime profile from tracked Obstacle course inputs."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import yaml

from generate_obstacle_course_world import WorldGenerationError, load_tracked_scenario

REPO = Path(__file__).resolve().parents[1]
CONFIG = Path("ros2_ws/src/car_bringup/config/obstacle_course")


def build_profile(
    scenario_path: Path,
    profile_path: Path,
    output_path: Path,
    run_purpose: str | None = None,
    decision_config_path: Path | None = None,
) -> dict:
    scenario_source = scenario_path if scenario_path.is_absolute() else REPO / scenario_path
    vehicle_profile_path = profile_path if profile_path.is_absolute() else REPO / profile_path
    decision_source = decision_config_path or CONFIG / "decision.yaml"
    if not decision_source.is_absolute():
        decision_source = REPO / decision_source
    scenario = load_tracked_scenario(scenario_source)
    if run_purpose is not None:
        if run_purpose not in {"diagnostic", "development", "demonstration"}:
            raise ValueError("run-purpose must be diagnostic, development, or demonstration")
        scenario = {**scenario, "run_purpose": run_purpose}
    vehicle_profile = yaml.safe_load(vehicle_profile_path.read_text(encoding="utf-8"))
    geometry = yaml.safe_load((REPO / CONFIG / "vehicle_geometry.yaml").read_text(encoding="utf-8"))
    decision = yaml.safe_load(decision_source.read_text(encoding="utf-8"))
    matrix = yaml.safe_load((REPO / CONFIG / "trigger_matrix.yaml").read_text(encoding="utf-8"))
    sources = {
        "scenario": {"path": scenario_source.relative_to(REPO).as_posix(), "sha256": hashlib.sha256(scenario_source.read_bytes()).hexdigest()},
        "vehicle_profile": {"path": vehicle_profile_path.relative_to(REPO).as_posix(), "sha256": hashlib.sha256(vehicle_profile_path.read_bytes()).hexdigest()},
    }
    for name, path in (
        ("vehicle_geometry.yaml", REPO / CONFIG / "vehicle_geometry.yaml"),
        ("decision_config", decision_source),
        ("trigger_matrix.yaml", REPO / CONFIG / "trigger_matrix.yaml"),
    ):
        sources[name] = {"path": path.relative_to(REPO).as_posix(), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    body = {
        "schema_version": "1.0",
        "scenario_id": scenario["scenario_id"],
        "seed": scenario["seed"],
        "run_purpose": scenario["run_purpose"],
        "scenario": scenario,
        "vehicle_profile": vehicle_profile,
        "vehicle_geometry": geometry,
        "decision": decision,
        "trigger_matrix": matrix,
        "source_hashes": sources,
    }
    canonical = json.dumps(body, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    document = {**body, "profile_sha256": hashlib.sha256(canonical).hexdigest()}
    output = output_path if output_path.is_absolute() else REPO / output_path
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return document


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-config", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=CONFIG / "rc_vehicle_reference.yaml")
    parser.add_argument("--decision-config", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--run-purpose", choices=["diagnostic", "development", "demonstration"])
    args = parser.parse_args(argv)
    if args.output is None:
        scenario = load_tracked_scenario(args.scenario_config)
        args.output = Path("artifacts/obstacle-course/config") / f"{scenario['scenario_id']}.json"
    try:
        result = build_profile(
            args.scenario_config, args.profile, args.output, args.run_purpose,
            args.decision_config,
        )
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError, WorldGenerationError) as exc:
        print(f"build_obstacle_course_profile: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"scenario_id": result["scenario_id"], "profile_sha256": result["profile_sha256"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
