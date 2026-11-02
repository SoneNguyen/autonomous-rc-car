"""Completeness and vehicle-relative setup-sheet tests for Obstacle course."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from generate_setup_sheets import generate_sheets, update_combined_setup_document  # noqa: E402
from validate_real_car_mapping import validate_mapping  # noqa: E402


def test_mapping_requires_every_numeric_profile_constant_and_real_provenance(tmp_path):
    profile = ROOT / "ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml"
    mapping = tmp_path / "mapping.yaml"
    mapping.write_text(yaml.safe_dump({"schema_version": "1.0", "entries": []}))
    with pytest.raises(ValueError, match="unmapped"):
        validate_mapping(mapping, profile, [], tmp_path)


def test_setup_sheet_generation_is_vehicle_relative_and_complete(tmp_path):
    mapping = ROOT / "ros2_ws/src/car_bringup/config/obstacle_course/real_car_mapping.yaml"
    sheets = generate_sheets(mapping, tmp_path)
    assert len(sheets) == 10
    assert {path.stem for path in sheets} >= {"S0", "S1", "S3b", "S6-lidar-drop", "S6-camera-drop"}
    content = (tmp_path / "S1.md").read_text()
    assert "vehicle-relative" in content
    assert "0.75 m" in content
    assert "single-plane lidar" in content.lower()


def test_validator_rejects_missing_scenario_sheet(tmp_path):
    mapping = ROOT / "ros2_ws/src/car_bringup/config/obstacle_course/real_car_mapping.yaml"
    scenarios = ROOT / "ros2_ws/src/car_bringup/config/obstacle_course/scenarios"
    sheets = tmp_path / "sheets"
    sheets.mkdir()
    with pytest.raises(ValueError, match="missing setup sheet"):
        validate_mapping(mapping, ROOT / "ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml", scenarios, sheets)


def test_all_setup_sections_live_in_the_single_implementation_document(tmp_path):
    mapping = ROOT / "ros2_ws/src/car_bringup/config/obstacle_course/real_car_mapping.yaml"
    source = (ROOT / "docs/implementation.md").read_text(encoding="utf-8")
    document = tmp_path / "implementation.md"
    document.write_text(source, encoding="utf-8")
    scenarios = ROOT / "ros2_ws/src/car_bringup/config/obstacle_course/scenarios"
    assert update_combined_setup_document(document, scenarios) == [
        "S0", "S1", "S2", "S3", "S3b", "S4", "S5",
        "S6-lidar-drop", "S6-camera-drop", "S7",
    ]
    text = document.read_text(encoding="utf-8")
    assert '<a id="setup-s3b"></a>' in text
    assert "opening_blocker" in text
    result = validate_mapping(
        mapping,
        ROOT / "ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml",
        scenarios,
        ROOT / "docs/implementation.md",
    )
    assert result["scenario_sheet_count"] == 10
