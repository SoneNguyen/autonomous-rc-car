"""Configuration matrix guardrails for Obstacle course decision inputs."""

from __future__ import annotations

from pathlib import Path
import copy

import yaml
from jsonschema import Draft202012Validator
import pytest

from tools.validate_obstacle_course_config import (
    ContractValidationError,
    SCHEMA_NAMES,
    REPO,
    validate_instance,
    validate_report_consistency,
    validate_scenario_semantics,
)

ROOT = Path(__file__).resolve().parents[4]
CONFIG = ROOT / "ros2_ws/src/car_bringup/config/obstacle_course"
COMPONENTS = {"lidar", "camera", "imu", "odometry", "contact"}
BEHAVIORS = {"FOLLOW_ROUTE", "CAUTION", "AVOID_LEFT", "AVOID_RIGHT", "STOP", "RESUME"}
ROLES = {"trigger", "support", "veto", "overruled"}


def _yaml(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_trigger_matrix_rows_use_known_components_behaviors_and_roles():
    matrix = _yaml(CONFIG / "trigger_matrix.yaml")
    assert set(matrix["components"]) == COMPONENTS
    assert set(matrix["behaviors"]) == BEHAVIORS
    for row in matrix["rows"]:
        assert row["component"] in COMPONENTS
        assert row["behavior"] in BEHAVIORS
        assert row["role"] in ROLES
        assert row["rule"]


def test_camera_cannot_trigger_avoid_or_stop_alone():
    matrix = _yaml(CONFIG / "trigger_matrix.yaml")
    forbidden = [
        row for row in matrix["rows"]
        if row["component"] == "camera"
        and row["behavior"] in {"AVOID_LEFT", "AVOID_RIGHT", "STOP"}
        and row["role"] == "trigger"
    ]
    assert forbidden == []
    assert any(row["component"] == "camera" and row["behavior"] == "CAUTION" and row["role"] == "trigger" for row in matrix["rows"])


def test_decision_thresholds_have_units_and_provenance():
    config = _yaml(CONFIG / "decision.yaml")
    assert set(config["camera"]["hsv_classes"]) == {"barrier", "post", "gate"}
    assert config["vehicle_profile"]["sha256"]
    clearance = config["thresholds"]["class_clearance_vehicle_width_multiple"]
    assert set(clearance) == {"barrier", "post", "gate", "conservative_lidar_only"}

    def visit(value):
        if isinstance(value, dict):
            if "value" in value:
                assert "unit" in value, value
                assert value.get("provenance"), value
            for child in value.values():
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)

    visit(config)


SHA = "a" * 64
RUN_ID = "456f8b5a-253e-4a27-8c0d-f3a0a1b8fca4"
EVENT_ID = "e9a6d788-8d99-4be0-a5c3-e89e4b0c373b"
ROOT_SHA = "b" * 64


def _measurement(value, unit, status="placeholder"):
    return {"value": value, "unit": unit, "status": status, "provenance": "contract test fixture"}


def _pose(x, y, yaw=0.0):
    return {"x": _measurement(x, "m"), "y": _measurement(y, "m"), "yaw": _measurement(yaw, "rad")}


def _scenario():
    start = _pose(0.0, 0.0)
    target = _pose(1.0, 0.0)
    return {
        "schema_version": "1.0",
        "scenario_id": "S5",
        "seed": 151007,
        "run_purpose": "diagnostic",
        "world_path": "simulation/worlds/obstacle_condensed.sdf",
        "lighting_profile": "world_default",
        "vehicle_preset": "rc_vehicle_reference",
        "entry_speed_mps": _measurement(0.5, "m/s"),
        "start_pose": start,
        "route": {
            "frame": "vehicle_relative",
            "waypoints": [start, target],
            "length_m": _measurement(1.0, "m", "derived"),
            "sizing": {
                "entry_speed_mps": _measurement(0.5, "m/s"),
                "approach_distance_m": _measurement(0.3, "m"),
                "lateral_shift_m": _measurement(0.0, "m"),
                "recovery_distance_m": _measurement(0.3, "m"),
            },
        },
        "obstacles": [{
            "entity_id": "late_barrier",
            "class": "barrier",
            "colour": "red",
            "pose": target,
            "initial_pose": _pose(-2.0, 0.0),
            "dimensions_m": {
                "length": _measurement(0.5, "m"),
                "width": _measurement(0.3, "m"),
                "height": _measurement(0.4, "m"),
            },
            "mesh_or_fragment": "barrier_red.sdf",
            "scan_plane_intersection": True,
        }],
        "fault_injections": [{
            "component": None,
            "start_sim_time": _measurement(0.0, "s"),
            "end_sim_time": _measurement(5.0, "s"),
            "action": "reveal_obstacle",
            "entity_id": "late_barrier",
            "trigger_pose_progress_m": _measurement(0.2, "m", "derived"),
        }],
        "expected_behavior_sequence": ["FOLLOW_ROUTE", "STOP"],
        "expected_trigger_components": ["lidar", "contact"],
        "contact_rule": "contact_expected_and_latched",
        "setup_sheet": "docs/implementation.md#setup-s5",
    }


def _fault_event():
    return {
        "schema_version": "1.0",
        "event_id": EVENT_ID,
        "run_id": RUN_ID,
        "run_purpose": "demonstration",
        "scenario_id": "S5",
        "event_type": "obstacle_revealed",
        "seed": 151007,
        "sim_time_ns": 1500000000,
        "component": None,
        "entity_id": "late_barrier",
        "route_progress_m": 0.2,
        "obstacle_range_m": 0.1,
        "config_sha256": SHA,
        "frozen_inputs_root_sha256": ROOT_SHA,
    }


def _run_report():
    return {
        "schema_version": "1.0",
        "report_id": "5b384a2d-4dd5-4a10-aef9-994e48e194a0",
        "created_utc": "2026-11-01T00:00:00Z",
        "scenario_id": "S5",
        "run_purpose": "demonstration",
        "mode": "single",
        "status": "pass",
        "qualified": False,
        "expected_behavior_sequence": ["FOLLOW_ROUTE", "STOP"],
        "expected_trigger_components": ["lidar", "contact"],
        "repeats": [{
            "repeat_index": 1,
            "run_id": RUN_ID,
            "seed": 151007,
            "status": "pass",
            "qualified": False,
            "observed_behavior_sequence": ["FOLLOW_ROUTE", "STOP"],
            "behavior_sequence_matches": True,
            "actual_trigger_components": ["lidar", "contact"],
            "trigger_set_matches": True,
            "contact_outcome": "permitted_inside_d_stop",
            "fault_events": [_fault_event()],
            "event_refs": [],
        }],
        "source_identity": {
            "ubuntu": "Ubuntu 26.04",
            "ros_distro": "lyrical",
            "gazebo_version": "10.5.0",
            "profile_sha256": SHA,
            "scenario_sha256": SHA,
            "decision_config_path": "ros2_ws/src/car_bringup/config/obstacle_course/decision.yaml",
            "decision_config_sha256": SHA,
            "trigger_matrix_sha256": SHA,
            "world_sha256": SHA,
        },
        "protected_metrics_written": False,
        "frozen_inputs_root_sha256": ROOT_SHA,
    }


def _valid_imu_evidence():
    return {
        "schema_version": "1.0",
        "component": "imu",
        "scenario_id": "S0",
        "run_id": RUN_ID,
        "run_purpose": "diagnostic",
        "sequence": 0,
        "source_sample_sequence": 0,
        "sim_time_ns": 100000000,
        "clock_epoch": 0,
        "freshness": {"state": "fresh", "age_s": 0.0, "timeout_s": 0.08},
        "valid": True,
        "source": {"topic": "/imu/data", "message_type": "sensor_msgs/msg/Imu", "adapter_id": "rc_imu_v1"},
        "config_sha256": SHA,
        "profile_sha256": SHA,
        "frame_id": "imu_link",
        "evidence": {"yaw_rate_rad_s": 0.0, "lateral_acceleration_mps2": 0.0, "longitudinal_acceleration_mps2": 0.0},
    }


def _decision_event():
    return {
        "schema_version": "1.0",
        "event_id": EVENT_ID,
        "run_id": RUN_ID,
        "run_purpose": "diagnostic",
        "scenario_id": "S1",
        "sequence": 1,
        "event_kind": "transition",
        "sim_time_ns": 100000000,
        "clock_epoch": 0,
        "published_monotonic_ns": 500000000,
        "previous_behavior": "FOLLOW_ROUTE",
        "behavior": "CAUTION",
        "candidate_behavior": None,
        "rule_id": "LIDAR_CAUTION",
        "reason": "barrier inside caution distance",
        "attribution": {
            "triggers": [{"component": "lidar", "evidence_sequence": 1, "value": 1.0, "unit": "m", "freshness": "fresh"}],
            "supporters": [], "vetoes": [], "overruled": [], "stale_components": [],
        },
        "decision_config_sha256": SHA,
        "trigger_matrix_sha256": SHA,
        "profile_sha256": SHA,
    }


def _planner_intent():
    return {
        "schema_version": "1.0",
        "scenario_id": "S1",
        "run_id": RUN_ID,
        "run_purpose": "demonstration",
        "decision_event_id": EVENT_ID,
        "sim_time_ns": 100000000,
        "clock_epoch": 0,
        "behavior": "AVOID_LEFT",
        "preferred_side": "left",
        "selected_class": "barrier",
        "class_margin_m": 0.1125,
        "speed_cap_mps": 0.35,
        "rule_id": "LIDAR_AVOID",
        "valid_until_ns": 300000000,
        "decision_config_sha256": SHA,
        "trigger_matrix_sha256": SHA,
        "profile_sha256": SHA,
        "frozen_inputs_root_sha256": ROOT_SHA,
    }


def test_all_nine_json_schemas_are_valid_and_have_examples():
    for name in SCHEMA_NAMES:
        schema_path = REPO / "ros2_ws/src/car_bringup/config/obstacle_course/contracts" / name
        schema = __import__("json").loads(schema_path.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)

    validate_instance("decision-config.schema.json", _yaml(CONFIG / "decision.yaml"))
    validate_instance("trigger-matrix.schema.json", _yaml(CONFIG / "trigger_matrix.yaml"))
    validate_instance("component-evidence.schema.json", _valid_imu_evidence())
    validate_instance("decision-event.schema.json", _decision_event())
    validate_instance("fault-event.schema.json", _fault_event())
    validate_instance("frozen-inputs.schema.json", {
        "schema_version": "1.0", "manifest_id": RUN_ID, "created_utc": "2026-11-01T00:00:00Z",
        "inputs": [{"path": "simulation/worlds/obstacle_condensed.sdf", "sha256": SHA}], "root_sha256": ROOT_SHA,
    })
    validate_instance("planner-intent.schema.json", _planner_intent())
    validate_scenario_semantics(_scenario())
    validate_report_consistency(_run_report())


def test_conditional_schema_rules_for_demonstration_roots():
    evidence = _valid_imu_evidence()
    evidence["run_purpose"] = "demonstration"
    with pytest.raises(ContractValidationError, match="frozen_inputs_root_sha256"):
        validate_instance("component-evidence.schema.json", evidence)

    intent = _planner_intent()
    intent.pop("frozen_inputs_root_sha256")
    with pytest.raises(ContractValidationError, match="frozen_inputs_root_sha256"):
        validate_instance("planner-intent.schema.json", intent)

def test_transition_and_evaluation_event_requirements_are_distinct():
    transition = _decision_event()
    transition["attribution"]["triggers"] = []
    with pytest.raises(ContractValidationError, match="too short"):
        validate_instance("decision-event.schema.json", transition)

    evaluation = _decision_event()
    evaluation["event_kind"] = "evaluation"
    evaluation["candidate_behavior"] = "AVOID_LEFT"
    evaluation["attribution"]["triggers"] = []
    evaluation["attribution"]["vetoes"] = [{"component": "camera", "evidence_sequence": 2, "value": "gate-clear", "unit": "rule", "freshness": "fresh"}]
    validate_instance("decision-event.schema.json", evaluation)


def test_scenario_requires_measurement_provenance_fault_order_and_matching_reveal_entity():
    scenario = _scenario()
    scenario["route"]["length_m"].pop("provenance")
    with pytest.raises(ContractValidationError, match="provenance"):
        validate_scenario_semantics(scenario)

    scenario = _scenario()
    scenario["fault_injections"][0]["end_sim_time"]["value"] = 0.0
    with pytest.raises(ContractValidationError, match="later than start_sim_time"):
        validate_scenario_semantics(scenario)

    scenario = _scenario()
    scenario["fault_injections"][0]["entity_id"] = "missing-obstacle"
    with pytest.raises(ContractValidationError, match="match exactly one obstacle"):
        validate_scenario_semantics(scenario)


def test_report_fault_events_use_standalone_schema_and_match_parent_identity():
    report = _run_report()
    report["repeats"][0]["fault_events"][0]["config_sha256"] = "invalid"
    with pytest.raises(ContractValidationError, match="fault_events/0/config_sha256"):
        validate_report_consistency(report)

    report = _run_report()
    report["repeats"][0]["fault_events"][0]["seed"] = 9
    with pytest.raises(ContractValidationError, match="seed"):
        validate_report_consistency(report)
