#!/usr/bin/env python3
"""Validate the scenario-disjoint data splits and freeze evaluation configs.

Camera collection and training are intentionally not implemented in P3. This
module establishes the immutable split contract that those later steps must use.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import yaml

from car_bringup.scenario import (
    ScenarioValidationError,
    load_scenario,
    validate_scenario_obstacle,
)


DEFAULT_SCENARIO_DIR = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "car_bringup" / "config" / "scenarios"
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = Path(__file__).resolve().parents[1] / "artifacts" / "scenario-splits" / "split-manifest.json"
SCHEMA_VERSION = "1.0"
GENERATOR_VERSION = "p3-split-validator-1"
EXPECTED_SPLITS = {"training", "validation", "evaluation"}
EXPECTED_EVALUATION_COUNT = 10


class SplitValidationError(ValueError):
    """A scenario set is not safe to train/evaluate against."""


def _number(value: Any) -> str:
    return f"{float(value):.6f}"


def canonical_tuple(scenario: dict[str, Any]) -> tuple[Any, ...]:
    """Canonical full split-isolation tuple; seed and filename are excluded."""
    pose = scenario["start_pose"]
    obstacle = scenario["obstacle"]
    obstacle_pose = obstacle["pose"]
    return (
        *(_number(pose[key]) for key in ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")),
        obstacle["entity"],
        obstacle["collision"],
        *(_number(obstacle_pose[key]) for key in ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")),
        _number(scenario["commanded_speed_mps"]),
    )


def obstacle_placement(scenario: dict[str, Any]) -> tuple[Any, ...]:
    obstacle = scenario["obstacle"]
    pose = obstacle["pose"]
    return (
        obstacle["entity"],
        obstacle["collision"],
        *(_number(pose[key]) for key in ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")),
    )


def _config_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_files(scenario_dir: Path) -> list[tuple[Path, dict[str, Any]]]:
    files = sorted(scenario_dir.rglob("*.yaml"))
    if not files:
        raise SplitValidationError(f"no scenario YAML files found under {scenario_dir}")
    loaded = []
    ids = set()
    for path in files:
        try:
            scenario = load_scenario(path)
            validate_scenario_obstacle(scenario, REPO_ROOT / scenario["world"])
        except ScenarioValidationError as exc:
            raise SplitValidationError(f"{path}: {exc}") from exc
        relative = path.relative_to(scenario_dir).with_suffix("").as_posix()
        if relative != scenario["scenario_id"]:
            raise SplitValidationError(
                f"{path}: scenario_id {scenario['scenario_id']!r} must match path {relative!r}"
            )
        if scenario["split"] != relative.split("/", maxsplit=1)[0]:
            raise SplitValidationError(f"{path}: split does not match its directory")
        if scenario["scenario_id"] in ids:
            raise SplitValidationError(f"duplicate scenario_id: {scenario['scenario_id']}")
        ids.add(scenario["scenario_id"])
        loaded.append((path, scenario))
    return loaded


def _manifest_payload(
    scenario_dir: Path, loaded: list[tuple[Path, dict[str, Any]]]
) -> tuple[dict[str, Any], str]:
    entries = {}
    tuple_owner: dict[tuple[Any, ...], str] = {}
    split_obstacles: dict[str, set[tuple[Any, ...]]] = {name: set() for name in EXPECTED_SPLITS}
    split_counts = {name: 0 for name in EXPECTED_SPLITS}
    for path, scenario in loaded:
        split = scenario["split"]
        if split not in EXPECTED_SPLITS:
            raise SplitValidationError(f"unexpected split {split!r} in {path}")
        full_tuple = canonical_tuple(scenario)
        if full_tuple in tuple_owner:
            raise SplitValidationError(
                "duplicate full scenario tuple (seed or filename cannot make it distinct): "
                f"{tuple_owner[full_tuple]} and {scenario['scenario_id']}"
            )
        tuple_owner[full_tuple] = scenario["scenario_id"]
        split_obstacles[split].add(obstacle_placement(scenario))
        split_counts[split] += 1
        relative = path.relative_to(scenario_dir).as_posix()
        entries[relative] = {
            "scenario_id": scenario["scenario_id"],
            "split": split,
            "seed": scenario["seed"],
            "full_tuple": list(full_tuple),
            "obstacle_placement": list(obstacle_placement(scenario)),
            "sha256": _config_sha256(path),
        }

    missing = sorted(name for name, count in split_counts.items() if not count)
    if missing:
        raise SplitValidationError(f"scenario split(s) are empty: {', '.join(missing)}")
    if split_counts["evaluation"] != EXPECTED_EVALUATION_COUNT:
        raise SplitValidationError(
            f"evaluation must contain exactly {EXPECTED_EVALUATION_COUNT} configs; "
            f"found {split_counts['evaluation']}"
        )

    training_obstacles = split_obstacles["training"]
    for path, scenario in loaded:
        expected_tags = (
            ["unseen_obstacle_placement"]
            if scenario["split"] == "evaluation" and obstacle_placement(scenario) not in training_obstacles
            else []
        )
        if scenario["unseen_condition_tags"] != expected_tags:
            raise SplitValidationError(
                f"{path}: unseen_condition_tags must be {expected_tags!r} derived from training placements"
            )

    evaluation_entries = {
        name: value for name, value in sorted(entries.items()) if value["split"] == "evaluation"
    }
    eval_hash = hashlib.sha256(
        json.dumps(evaluation_entries, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    payload = {
        "schema_version": SCHEMA_VERSION,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "generator_version": GENERATOR_VERSION,
        "scenario_files": dict(sorted(entries.items())),
        "evaluation_frozen": False,
        "evaluation_manifest_sha256": eval_hash,
        "split_counts": split_counts,
    }
    return payload, eval_hash


def validate_splits(
    scenario_dir: Path = DEFAULT_SCENARIO_DIR,
    manifest_path: Path = DEFAULT_MANIFEST,
    *,
    freeze_evaluation: bool = False,
    require_frozen: bool = False,
) -> dict[str, Any]:
    """Validate all YAMLs and optionally write/check the immutable split manifest."""
    scenario_dir = Path(scenario_dir).resolve()
    manifest_path = Path(manifest_path).resolve()
    loaded = _load_files(scenario_dir)
    payload, current_eval_hash = _manifest_payload(scenario_dir, loaded)

    if require_frozen or freeze_evaluation:
        if manifest_path.is_file():
            try:
                frozen = json.loads(manifest_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise SplitValidationError(f"cannot read frozen manifest {manifest_path}: {exc}") from exc
            if not frozen.get("evaluation_frozen"):
                raise SplitValidationError("existing split manifest is not frozen")
            if frozen.get("evaluation_manifest_sha256") != current_eval_hash:
                raise SplitValidationError(
                    "evaluation config changed after freeze; do not overwrite the frozen set"
                )
            if require_frozen:
                return frozen
            # Preserve the original freeze timestamp and file bytes on an idempotent call.
            return frozen
        if require_frozen:
            raise SplitValidationError(f"frozen evaluation manifest is missing: {manifest_path}")
        payload["evaluation_frozen"] = True
        payload["frozen_evaluation_ids"] = sorted(
            value["scenario_id"]
            for value in payload["scenario_files"].values()
            if value["split"] == "evaluation"
        )
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        manifest_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return payload

    return payload


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["validate-splits"])
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--freeze-evaluation", action="store_true")
    mode.add_argument("--require-frozen", action="store_true")
    parser.add_argument("--scenario-dir", type=Path, default=DEFAULT_SCENARIO_DIR)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    args = parser.parse_args(argv)
    try:
        result = validate_splits(
            args.scenario_dir,
            args.manifest,
            freeze_evaluation=args.freeze_evaluation,
            require_frozen=args.require_frozen,
        )
    except SplitValidationError as exc:
        print(f"split validation failed: {exc}", file=sys.stderr)
        return 2
    print(
        "Scenario split validation passed: "
        + ", ".join(f"{name}={count}" for name, count in result["split_counts"].items())
    )
    if result["evaluation_frozen"]:
        print(f"Frozen evaluation SHA-256: {result['evaluation_manifest_sha256']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
