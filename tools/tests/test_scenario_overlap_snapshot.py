"""Pin and exercise the read-only Feature 002 overlap implementation."""

from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path


SNAPSHOT = Path(__file__).resolve().parents[1] / "vendor/scenario_overlap/feature002_overlap.py"
EXPECTED_SHA256 = "d01ede3dbdfc73606cc52cd8e03c3f86da07d80c690ed48c4d860837b7849dd3"
SPEC = importlib.util.spec_from_file_location("feature002_overlap", SNAPSHOT)
assert SPEC and SPEC.loader
overlap = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(overlap)


def test_overlap_snapshot_is_pinned_byte_for_byte():
    assert hashlib.sha256(SNAPSHOT.read_bytes()).hexdigest() == EXPECTED_SHA256


def test_feature007_projection_uses_historical_duplicate_projection():
    candidate = {
        "scenario_id": "S1",
        "seed": 7,
        "start_pose": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        "obstacles": [{"entity_id": "barrier", "pose": {"x_m": 1.0, "y_m": 0.0, "z_m": 0.0}}],
        "target_speed_mps": 0.5,
    }
    copy = {**candidate, "scenario_id": "other", "seed": 8}
    assert overlap.exact_duplicate(candidate, copy, 0.22)
    distances = overlap.group_distances(candidate, copy, 0.22)
    assert distances == {"start_pose": 0.0, "obstacle_placement": 0.0, "speed": 0.0}


def test_overlap_snapshot_detects_duplicated_or_near_historical_scenarios():
    historical = {
        "scenario_id": "legacy-eval",
        "start_pose": {"x_m": 10.0, "y_m": 1.0, "yaw_rad": 0.0},
        "obstacles": [],
        "target_speed_mps": 0.5,
    }
    candidate = {**historical, "scenario_id": "S0"}
    result = overlap.normalized_distance(
        candidate,
        historical,
        {"start_pose": 0.25, "obstacle_placement": 0.44, "speed": 0.1},
        0.22,
    )
    assert result["distance"] == 0.0
    assert overlap.exact_duplicate(candidate, historical, 0.22)
