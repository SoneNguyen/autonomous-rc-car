"""Unit tests for baseline inventory and canonical-path enforcement."""

from __future__ import annotations

import importlib.util
from pathlib import Path


TOOL = Path(__file__).resolve().parents[1] / "check_feature_007_protected_paths.py"
SPEC = importlib.util.spec_from_file_location("check_feature_007_protected_paths", TOOL)
assert SPEC and SPEC.loader
protected = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(protected)


def _baseline():
    return {
        "entries": [
            {"path": "simulation/worlds/existing.sdf", "status": "present", "sha256": "a"},
            {"path": "simulation/models/car/model.sdf", "status": "absent", "sha256": None},
        ],
        "protected_roots": ["simulation/worlds/", "simulation/models/car/"],
        "allowed_feature007_world_additions": ["simulation/worlds/obstacle_condensed.sdf"],
    }


def _current(**overrides):
    values = {
        "base_paths": {"simulation/worlds/existing.sdf"},
        "current_tracked": {"simulation/worlds/existing.sdf"},
        "current_untracked": set(),
        "current_files": {"simulation/worlds/existing.sdf"},
        "current_hashes": {"simulation/worlds/existing.sdf": "a"},
        "canonical_paths": {"tools/check.py"},
        "base_hashes": {"simulation/worlds/existing.sdf": "a"},
    }
    values.update(overrides)
    return values


def test_unchanged_baseline_and_explicit_world_addition_pass():
    current = _current(
        current_tracked={"simulation/worlds/existing.sdf", "simulation/worlds/obstacle_condensed.sdf"},
        current_files={"simulation/worlds/existing.sdf", "simulation/worlds/obstacle_condensed.sdf"},
        canonical_paths={"tools/check.py", "simulation/worlds/obstacle_condensed.sdf"},
    )
    assert protected.compare_inventory(baseline=_baseline(), **current) == []


def test_modification_deletion_and_reintroduction_are_rejected():
    changed = _current(current_hashes={"simulation/worlds/existing.sdf": "b"})
    assert any("content changed" in error for error in protected.compare_inventory(baseline=_baseline(), **changed))

    deleted = _current(current_tracked=set(), current_files=set())
    errors = protected.compare_inventory(baseline=_baseline(), **deleted)
    assert any("deleted" in error for error in errors)

    restored = _current(
        current_files={"simulation/worlds/existing.sdf", "simulation/models/car/model.sdf"},
        current_untracked={"simulation/models/car/model.sdf"},
    )
    assert any("reintroduced" in error or "protected root" in error for error in protected.compare_inventory(baseline=_baseline(), **restored))


def test_unlisted_world_and_noncanonical_additions_are_rejected():
    current = _current(
        current_tracked={"simulation/worlds/existing.sdf", "simulation/worlds/other.sdf", "tools/unlisted.py"},
        current_untracked={"simulation/worlds/other.sdf", "tools/unlisted.py"},
        current_files={"simulation/worlds/existing.sdf", "simulation/worlds/other.sdf", "tools/unlisted.py"},
    )
    errors = protected.compare_inventory(baseline=_baseline(), **current)
    assert any("not explicitly allowed" in error for error in errors)
    assert any("not listed in canonical-paths.md" in error for error in errors)


def test_canonical_directory_entries_allow_only_their_subtree():
    allowed = protected.canonical_allowlist("- `tools/check.py`\n- `ros2_ws/src/car_decision/`\n")
    assert protected.is_canonical("ros2_ws/src/car_decision/node.py", allowed)
    assert not protected.is_canonical("ros2_ws/src/car_navigation/node.py", allowed)


def test_absent_historical_directory_is_rejected_when_ignored_file_reappears():
    baseline = _baseline()
    baseline["entries"].append({"path": "artifacts/rc-car/", "status": "absent", "sha256": None})
    current = _current(
        current_files={"simulation/worlds/existing.sdf", "artifacts/rc-car/report.json"},
    )
    errors = protected.compare_inventory(baseline=baseline, **current)
    assert any("historical directory was reintroduced: artifacts/rc-car/" in error for error in errors)
