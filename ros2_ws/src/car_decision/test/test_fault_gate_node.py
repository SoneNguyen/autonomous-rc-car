"""Tracked fault schedules and event contract tests for Obstacle course."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest
import yaml
from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(PACKAGE_ROOT))
from car_decision.evidence_common import fault_active  # noqa: E402


CONFIGS = REPO / "ros2_ws/src/car_bringup/config/obstacle_course/scenarios"
SCHEMA_PATH = REPO / "ros2_ws/src/car_bringup/config/obstacle_course/contracts/fault-event.schema.json"


def _yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _event(scenario_id, event_type, component, entity_id, progress, obstacle_range):
    scenario = _yaml(CONFIGS / f"{scenario_id}.yaml")
    event = {
        "schema_version": "1.0",
        "event_id": "8a516856-d07d-49d7-82e0-c75142a50588",
        "run_id": "1de8fb69-2874-49d2-bf46-c485617e6f62",
        "run_purpose": "diagnostic",
        "scenario_id": scenario_id,
        "event_type": event_type,
        "seed": scenario["seed"],
        "sim_time_ns": 2_000_000_000,
        "component": component,
        "entity_id": entity_id,
        "route_progress_m": progress,
        "obstacle_range_m": obstacle_range,
        "config_sha256": "a" * 64,
    }
    return event


def test_s6_fault_schedule_is_fixed_seed_and_suppresses_only_selected_component():
    lidar = _yaml(CONFIGS / "S6-lidar-drop.yaml")
    camera = _yaml(CONFIGS / "S6-camera-drop.yaml")
    assert lidar["seed"] != camera["seed"]
    assert fault_active(lidar, "lidar", 1_500_000_000)
    assert not fault_active(lidar, "camera", 1_500_000_000)
    assert fault_active(camera, "camera", 1_500_000_000)
    assert not fault_active(camera, "lidar", 1_500_000_000)


def test_fault_events_validate_and_demonstrations_need_the_frozen_root():
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    validator = Draft202012Validator(schema)
    validator.validate(_event("S6-lidar-drop", "sensor_dropout_started", "lidar", None, None, None))
    validator.validate(_event("S5", "obstacle_revealed", None, "late_barrier", 0.7, 0.15))
    demonstration = _event("S5", "obstacle_revealed", None, "late_barrier", 0.7, 0.15)
    demonstration["run_purpose"] = "demonstration"
    demonstration["frozen_inputs_root_sha256"] = "b" * 64
    validator.validate(demonstration)
    del demonstration["frozen_inputs_root_sha256"]
    with pytest.raises(Exception):
        validator.validate(demonstration)


def test_s5_reveal_has_hidden_initial_pose_and_does_not_add_a_command_publisher():
    scenario = _yaml(CONFIGS / "S5.yaml")
    obstacle = next(item for item in scenario["obstacles"] if item["entity_id"] == "late_barrier")
    assert obstacle["initial_pose"]["y"]["value"] != obstacle["pose"]["y"]["value"]
    source = (PACKAGE_ROOT / "car_decision/fault_gate_node.py").read_text(encoding="utf-8")
    assert "AckermannDriveStamped" not in source
    assert 'create_publisher(String, "/scenario/fault_event"' in source
    assert "SetEntityPose" in source
