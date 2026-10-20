"""Scenario-level isolation and immutable evaluation-freeze tests."""

import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ml"))

from collect_dataset import SplitValidationError, validate_splits  # noqa: E402


def _scenario(split, name, *, x, speed, seed, obstacle="post_01", tags=None):
    obstacle_poses = {
        "post_01": (-42.0, -2.0, 1.5),
        "post_02": (5.0, 7.0, 1.5),
    }
    ox, oy, oz = obstacle_poses[obstacle]
    return {
        "scenario_id": f"{split}/{name}",
        "split": split,
        "world": "simulation/worlds/obstacle.sdf",
        "seed": seed,
        "lighting_id": "obstacle_world_default",
        "unseen_condition_tags": tags or [],
        "start_pose": {
            "x_m": x, "y_m": oy, "z_m": 1.96,
            "roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": 0.0,
        },
        "obstacle": {
            "entity": "course_obstacles",
            "collision": f"{obstacle}_collision",
            "pose": {
                "x_m": ox, "y_m": oy, "z_m": oz,
                "roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": 0.0,
            },
        },
        "commanded_speed_mps": speed,
        "stop_distance_m": 5.0,
        "corridor_half_width_m": 5.5,
        "ground_z_min_m": 0.2,
        "run_timeout_sim_s": 60.0,
    }


def _write(root, scenario):
    path = root / Path(scenario["scenario_id"]).with_suffix(".yaml")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(scenario, sort_keys=False), encoding="utf-8")
    return path


def _valid_set(root):
    _write(root, _scenario("training", "train_01", x=-55.0, speed=0.75, seed=1))
    _write(root, _scenario("validation", "val_01", x=-56.0, speed=0.8, seed=2))
    for index in range(10):
        _write(
            root,
            _scenario(
                "evaluation", f"eval_{index + 1:02d}",
                x=-10.0 - index, speed=0.5 + index * 0.1, seed=100 + index,
                obstacle="post_02", tags=["unseen_obstacle_placement"],
            ),
        )


def test_validates_three_disjoint_sets_and_freezes_exactly_ten_eval_configs(tmp_path):
    scenarios = tmp_path / "scenarios"
    _valid_set(scenarios)
    manifest_path = tmp_path / "artifacts" / "split-manifest.json"
    frozen = validate_splits(scenarios, manifest_path, freeze_evaluation=True)
    assert frozen["evaluation_frozen"] is True
    assert frozen["split_counts"] == {"training": 1, "validation": 1, "evaluation": 10}
    assert len(frozen["frozen_evaluation_ids"]) == 10
    assert validate_splits(scenarios, manifest_path, require_frozen=True) == frozen


def test_seed_or_filename_alone_cannot_make_full_tuple_disjoint(tmp_path):
    scenarios = tmp_path / "scenarios"
    _valid_set(scenarios)
    duplicate = _scenario("validation", "val_01", x=-55.0, speed=0.75, seed=999)
    _write(scenarios, duplicate)
    with pytest.raises(SplitValidationError, match="duplicate full scenario tuple"):
        validate_splits(scenarios, tmp_path / "manifest.json")


def test_eval_placement_tags_are_derived_not_user_claimed(tmp_path):
    scenarios = tmp_path / "scenarios"
    _valid_set(scenarios)
    eval_path = scenarios / "evaluation" / "eval_01.yaml"
    data = yaml.safe_load(eval_path.read_text(encoding="utf-8"))
    data["unseen_condition_tags"] = []
    eval_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    with pytest.raises(SplitValidationError, match="unseen_condition_tags"):
        validate_splits(scenarios, tmp_path / "manifest.json")


def test_frozen_eval_config_change_is_rejected_without_overwriting_manifest(tmp_path):
    scenarios = tmp_path / "scenarios"
    _valid_set(scenarios)
    manifest_path = tmp_path / "artifacts" / "manifest.json"
    original = validate_splits(scenarios, manifest_path, freeze_evaluation=True)
    eval_path = scenarios / "evaluation" / "eval_01.yaml"
    data = yaml.safe_load(eval_path.read_text(encoding="utf-8"))
    data["seed"] += 500
    eval_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    with pytest.raises(SplitValidationError, match="changed after freeze"):
        validate_splits(scenarios, manifest_path, require_frozen=True)
    assert yaml.safe_load(manifest_path.read_text(encoding="utf-8")) == original


def test_evaluation_must_contain_exactly_ten_scenarios(tmp_path):
    scenarios = tmp_path / "scenarios"
    _valid_set(scenarios)
    (scenarios / "evaluation" / "eval_10.yaml").unlink()
    with pytest.raises(SplitValidationError, match="exactly 10"):
        validate_splits(scenarios, tmp_path / "manifest.json")
