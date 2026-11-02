"""Frozen-input manifest tests for Obstacle course."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import uuid

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from freeze_obstacle_course_inputs import root_hash_for_inputs  # noqa: E402
from verify_obstacle_course_inputs import verify_manifest  # noqa: E402


def test_root_hash_uses_sorted_path_tab_sha256_lines():
    inputs = [
        {"path": "z/config.yaml", "sha256": "b" * 64},
        {"path": "a/config.yaml", "sha256": "a" * 64},
    ]
    expected_bytes = f"a/config.yaml\t{'a' * 64}\nz/config.yaml\t{'b' * 64}\n".encode()
    assert root_hash_for_inputs(inputs) == hashlib.sha256(expected_bytes).hexdigest()


def test_manifest_rejects_duplicate_paths():
    with pytest.raises(ValueError, match="duplicate"):
        root_hash_for_inputs([
            {"path": "same", "sha256": "a" * 64},
            {"path": "same", "sha256": "b" * 64},
        ])


def test_verify_reports_changed_and_missing_inputs(tmp_path):
    source = tmp_path / "source.yaml"
    source.write_text("before\n")
    inputs = [{"path": source.name, "sha256": hashlib.sha256(source.read_bytes()).hexdigest()}]
    manifest = {
        "schema_version": "1.0",
        "manifest_id": str(uuid.uuid4()),
        "created_utc": "2026-01-01T00:00:00Z",
        "inputs": inputs,
        "root_sha256": root_hash_for_inputs(inputs),
    }
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(manifest))
    assert verify_manifest(path, root=tmp_path)["valid"] is True
    source.write_text("after\n")
    result = verify_manifest(path, root=tmp_path)
    assert result["valid"] is False
    assert result["changed"] == [source.name]
    source.unlink()
    result = verify_manifest(path, root=tmp_path)
    assert result["missing"] == [source.name]
