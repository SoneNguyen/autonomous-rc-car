"""Rule, gate, attribution, and hysteresis tests for Obstacle course."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import sys
import uuid

import pytest
import yaml
from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parents[4]
PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from car_decision.decision_engine import DecisionEngine  # noqa: E402


CONFIG = ROOT / "ros2_ws/src/car_bringup/config/obstacle_course"
SCHEMAS = ROOT / "ros2_ws/src/car_bringup/config/obstacle_course/contracts"
RUN_ID = str(uuid.UUID("456f8b5a-253e-4a27-8c0d-f3a0a1b8fca4"))
ROOT_SHA = "c" * 64
_UNSET = object()


def _yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _engine(scenario_id="S1", purpose="diagnostic", root_hash=""):
    scenario = _yaml(CONFIG / "scenarios" / f"{scenario_id}.yaml")
    profile = {
        "vehicle_profile": _yaml(CONFIG / "rc_vehicle_reference.yaml"),
        "vehicle_geometry": _yaml(CONFIG / "vehicle_geometry.yaml"),
    }
    return DecisionEngine(
        decision=_yaml(CONFIG / "decision.yaml"),
        profile=profile,
        scenario=scenario,
        trigger_matrix=_yaml(CONFIG / "trigger_matrix.yaml"),
        sensor_profile=_yaml(ROOT / "ros2_ws/src/car_bringup/config/rc_vehicle.yaml"),
        decision_sha256="a" * 64,
        matrix_sha256="b" * 64,
        profile_sha256="d" * 64,
        run_id=RUN_ID,
        run_purpose=purpose,
        frozen_root_sha256=root_hash,
    )


def _record(component, evidence, *, sequence=0, sample_sequence=0, state="fresh", valid=True, purpose="diagnostic", scenario="S1"):
    record = {
        "schema_version": "1.0", "component": component,
        "scenario_id": scenario, "run_id": RUN_ID, "run_purpose": purpose,
        "sequence": sequence, "source_sample_sequence": sample_sequence,
        "sim_time_ns": 1_000_000_000, "clock_epoch": 0,
        "freshness": {"state": state, "age_s": 0.0 if state == "fresh" else 1.0, "timeout_s": 0.5},
        "valid": valid, "evidence": evidence,
    }
    return record


def _records(*, distance=None, speed=0.35, camera_objects=None, contact=False, state_overrides=None, scenario="S1", purpose="diagnostic"):
    sectors = {name: None for name in ("right_m", "front_right_m", "front_m", "front_left_m", "left_m")}
    evidence = {
        "lidar": {
            "input_adapter": "laser_scan", "sectors": sectors,
            "route_corridor": {"occupied": distance is not None, "nearest_range_m": distance},
            "points_base_link": ([{"x_m": distance, "y_m": 0.0, "range_m": distance, "bearing_rad": 0.0}] if distance is not None else []),
            "ground_points_rejected": 0, "invalid_returns_rejected": 0, "usable_points": int(distance is not None),
        },
        "camera": {"classifier": "hsv", "objects": camera_objects or []},
        "imu": {"yaw_rate_rad_s": 0.0, "lateral_acceleration_mps2": 0.0, "longitudinal_acceleration_mps2": 0.0},
        "odometry": {
            "world_pose": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            "start_relative_pose": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            "speed_mps": speed, "route_progress_m": 0.1,
            "d_stop_m": speed * speed / (2 * 0.3999092578809597) + speed * 0.14,
            "profile_id": "obstacle-course-rc-car",
        },
        "contact": {
            "contacts": ([{"collision_a": "gazebo_truck::base_link::chassis_collision",
                            "collision_b": "barrier_01::obstacle_link::barrier_contact",
                            "ground": False, "allowlisted": True}] if contact else []),
            "ground_excluded": not contact, "non_ground_collision": contact,
        },
    }
    states = state_overrides or {}
    return {
        name: _record(name, payload, sequence=0, sample_sequence=0,
                      state=states.get(name, "fresh"), valid=states.get(name, "fresh") == "fresh",
                      scenario=scenario, purpose=purpose)
        for name, payload in evidence.items()
    }


def _advance(records, *, distance=_UNSET, seq=None, state=None):
    updated = copy.deepcopy(records)
    if distance is not _UNSET:
        updated["lidar"]["evidence"]["route_corridor"] = {
            "occupied": distance is not None, "nearest_range_m": distance,
        }
        updated["lidar"]["evidence"]["points_base_link"] = (
            [{"x_m": distance, "y_m": 0.0, "range_m": distance, "bearing_rad": 0.0}]
            if distance is not None else []
        )
    if seq is not None:
        updated["lidar"]["source_sample_sequence"] = seq
        updated["lidar"]["sequence"] = seq
    if state:
        for component, status in state.items():
            updated[component]["freshness"]["state"] = status
            updated[component]["valid"] = status == "fresh"
    return updated


def _schema_validate_event(event):
    schema = json.loads((SCHEMAS / "decision-event.schema.json").read_text(encoding="utf-8"))
    Draft202012Validator(schema).validate(event)


def test_engine_waits_for_all_streams_and_requires_one_coherent_run_identity():
    engine = _engine()
    records = _records()
    assert engine.evaluate({"lidar": records["lidar"]}, 1_000_000_000) is None
    records["camera"]["run_id"] = str(uuid.uuid4())
    assert engine.evaluate(records, 1_000_000_000) is None


def test_stale_lidar_or_odometry_transitions_to_stop_and_emits_schema_valid_trigger():
    for component in ("lidar", "odometry", "contact"):
        engine = _engine()
        records = _records()
        first = engine.evaluate(records, 1_000_000_000)
        assert first["state"]["behavior"] == "FOLLOW_ROUTE"
        records[component]["freshness"]["state"] = "stale"
        records[component]["valid"] = False
        result = engine.evaluate(records, 1_100_000_000)
        assert result["state"]["behavior"] == "STOP"
        assert result["events"][0]["attribution"]["stale_components"] == [component]
        _schema_validate_event(result["events"][0])


def test_near_lidar_return_causes_caution_and_close_feasible_return_causes_attributed_avoidance():
    engine = _engine()
    records = _records(distance=0.5, speed=0.35, camera_objects=[{"class": "barrier", "bearing_rad": 0.0, "apparent_size_px": 60}])
    first = engine.evaluate(records, 1_000_000_000)
    assert first["state"]["behavior"] == "FOLLOW_ROUTE"
    records = _advance(records, distance=0.4, seq=1)
    caution = engine.evaluate(records, 1_100_000_000)
    assert caution["state"]["behavior"] == "CAUTION"
    assert caution["events"][-1]["attribution"]["triggers"][0]["component"] == "lidar"
    _schema_validate_event(caution["events"][-1])
    result = engine.evaluate(_advance(records, distance=0.30, seq=2), 1_400_000_000)
    assert result["state"]["behavior"] == "AVOID_LEFT"
    event = result["events"][-1]
    assert [item["component"] for item in event["attribution"]["triggers"]] == ["lidar"]
    assert [item["component"] for item in event["attribution"]["supporters"]] == ["camera"]
    _schema_validate_event(event)


def test_same_obstacle_keeps_its_detour_side_across_caution_range_jitter():
    engine = _engine()
    records = _records(distance=0.4, speed=0.35)
    assert engine.evaluate(records, 1_000_000_000)["state"]["behavior"] == "CAUTION"

    first_avoid = engine.evaluate(_advance(records, distance=0.3, seq=1), 1_300_000_000)
    assert first_avoid["state"]["behavior"] == "AVOID_LEFT"

    range_jitter = engine.evaluate(_advance(records, distance=0.4, seq=2), 1_600_000_000)
    assert range_jitter["state"]["behavior"] == "AVOID_LEFT"
    assert range_jitter["intent"]["preferred_side"] == "left"

    same_obstacle = engine.evaluate(_advance(records, distance=0.3, seq=3), 1_900_000_000)
    assert same_obstacle["state"]["behavior"] == "AVOID_LEFT"
    assert same_obstacle["intent"]["preferred_side"] == "left"


def test_caution_hysteresis_does_not_clear_when_speed_reduction_shrinks_d_stop():
    engine = _engine()
    records = _records(distance=0.35, speed=0.35)
    first = engine.evaluate(records, 1_000_000_000)
    assert first["state"]["behavior"] == "CAUTION"

    slower = _advance(records, distance=0.35, seq=1)
    slower["odometry"]["evidence"]["speed_mps"] = 0.25
    slower["odometry"]["evidence"]["d_stop_m"] = 0.25**2 / (2 * 0.3999092578809597) + 0.25 * 0.14
    held = engine.evaluate(slower, 1_300_000_000)
    assert held["state"]["behavior"] == "CAUTION"

    clear = _advance(slower, distance=0.5, seq=2)
    recovered = engine.evaluate(clear, 1_600_000_000)
    assert recovered["state"]["behavior"] == "FOLLOW_ROUTE"


def test_avoidance_waits_until_object_is_ahead_clear_before_releasing_route_target():
    engine = _engine()
    records = _records(
        distance=0.4, speed=0.35,
        camera_objects=[{"class": "barrier", "bearing_rad": 0.0, "apparent_size_px": 60}],
    )
    assert engine.evaluate(records, 1_000_000_000)["state"]["behavior"] == "CAUTION"
    first_avoid = engine.evaluate(_advance(records, distance=0.3, seq=1), 1_300_000_000)
    assert first_avoid["state"]["behavior"] == "AVOID_LEFT"
    assert engine.active_avoid_entity_id == "barrier_01"

    farther_but_still_ahead = _advance(records, distance=0.6, seq=2)
    farther_but_still_ahead["odometry"]["evidence"]["start_relative_pose"] = {
        "x_m": 0.1, "y_m": 0.08, "yaw_rad": 0.2,
    }
    held = engine.evaluate(farther_but_still_ahead, 1_600_000_000)
    assert held["state"]["behavior"] == "AVOID_LEFT"
    assert held["intent"]["preferred_side"] == "left"

    beside_but_ahead = _advance(records, distance=None, seq=3)
    beside_but_ahead["odometry"]["evidence"]["start_relative_pose"] = {
        "x_m": 0.2, "y_m": 0.12, "yaw_rad": 0.43,
    }
    beside_but_ahead["lidar"]["evidence"]["points_base_link"] = [
        {"x_m": 1.2, "y_m": -0.3, "range_m": math.hypot(1.2, 0.3), "bearing_rad": -0.245},
    ]
    beside = engine.evaluate(beside_but_ahead, 1_700_000_000)
    assert beside["state"]["behavior"] == "AVOID_LEFT"
    assert beside["intent"]["preferred_side"] == "left"

    clear = _advance(beside_but_ahead, distance=None)
    clear["odometry"]["evidence"]["start_relative_pose"] = {
        "x_m": 1.3, "y_m": 0.35, "yaw_rad": 0.0,
    }
    clear["lidar"]["evidence"]["points_base_link"] = []
    result = beside
    for sample in range(4, 9):
        clear = _advance(clear, distance=None, seq=sample)
        clear["lidar"]["evidence"]["points_base_link"] = []
        clear["odometry"]["evidence"]["start_relative_pose"] = {
            "x_m": 1.3, "y_m": 0.35, "yaw_rad": 0.0,
        }
        result = engine.evaluate(clear, 1_600_000_000 + (sample - 2) * 100_000_000)
    assert result["state"]["behavior"] == "RESUME"
    assert result["intent"]["preferred_side"] == "none"


def test_tracked_obstacle_stays_active_beyond_the_caution_distance_until_rear_clear():
    engine = _engine()
    engine.scenario["route"]["waypoints"][-1]["x"]["value"] = 4.0
    engine.scenario["route"]["length_m"]["value"] = 4.0
    obstacle = engine.scenario["obstacles"][0]
    obstacle["pose"]["x"]["value"] = 2.4
    engine.active_avoid_side = "left"
    engine.active_avoid_entity_id = obstacle["entity_id"]
    odometry = {"start_relative_pose": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}}

    assert engine._active_object_ahead(
        lidar={}, selected_class=None, visual_range_m=None,
        caution_distance_m=0.1, odometry=odometry,
    )

    obstacle_half_extent = max(
        obstacle["dimensions_m"][axis]["value"] / 2.0 for axis in ("length", "width")
    )
    odometry["start_relative_pose"]["x_m"] = 2.4 + engine.length_m / 2.0 + obstacle_half_extent + 0.01
    assert not engine._active_object_ahead(
        lidar={}, selected_class=None, visual_range_m=None,
        caution_distance_m=0.1, odometry=odometry,
    )


def test_camera_alone_can_caution_but_never_avoid_or_stop():
    engine = _engine()
    records = _records(camera_objects=[{"class": "barrier", "bearing_rad": 0.0, "apparent_size_px": 1000}])
    result = engine.evaluate(records, 1_000_000_000)
    assert result["state"]["behavior"] == "CAUTION"
    assert result["events"][-1]["attribution"]["triggers"][0]["component"] == "camera"
    assert result["state"]["behavior"] not in {"AVOID_LEFT", "AVOID_RIGHT", "STOP"}

    for frame in range(1, 8):
        clear = _advance(records, distance=None, seq=frame)
        clear["camera"]["source_sample_sequence"] = frame
        clear["camera"]["sequence"] = frame
        clear["camera"]["evidence"]["objects"] = []
        clear["odometry"]["evidence"]["start_relative_pose"]["x_m"] = 3.0
        result = engine.evaluate(clear, 1_000_000_000 + frame * 100_000_000)
        assert result["state"]["behavior"] == "CAUTION"
    clear = _advance(records, distance=None, seq=8)
    clear["camera"]["source_sample_sequence"] = 8
    clear["camera"]["sequence"] = 8
    clear["camera"]["evidence"]["objects"] = []
    clear["odometry"]["evidence"]["start_relative_pose"]["x_m"] = 3.0
    result = engine.evaluate(clear, 1_800_000_000)
    assert result["state"]["behavior"] == "CAUTION"
    for frame in range(9, 13):
        clear = _advance(clear, distance=None, seq=frame)
        clear["camera"]["source_sample_sequence"] = frame
        clear["camera"]["sequence"] = frame
        result = engine.evaluate(clear, 1_000_000_000 + frame * 100_000_000)
        assert result["state"]["behavior"] == ("RESUME" if frame == 12 else "CAUTION")


def test_lidar_and_contact_take_priority_over_camera_only_caution():
    collision = _records(
        camera_objects=[{"class": "barrier", "bearing_rad": 0.0, "apparent_size_px": 1000}],
        contact=True,
    )
    assert _engine().evaluate(collision, 1_000_000_000)["state"]["behavior"] == "STOP"

    lidar_hazard = _records(
        distance=0.3,
        camera_objects=[{"class": "barrier", "bearing_rad": 0.0, "apparent_size_px": 1000}],
    )
    state = _engine().evaluate(lidar_hazard, 1_000_000_000)["state"]["behavior"]
    assert state in {"AVOID_LEFT", "AVOID_RIGHT", "STOP"}


def test_lidar_odometry_and_contact_can_start_run_while_camera_is_already_stale():
    engine = _engine()
    records = _records(distance=0.5, state_overrides={"camera": "stale"})
    result = engine.evaluate(records, 1_000_000_000)
    assert result["state"]["behavior"] == "FOLLOW_ROUTE"
    assert result["state"]["freshness"]["camera"] == "stale"


def test_no_feasible_detour_stops_instead_of_oscillating():
    engine = _engine()
    records = _records(distance=0.30, speed=0.35, camera_objects=[{"class": "post", "bearing_rad": 0.0, "apparent_size_px": 50}])
    records["lidar"]["evidence"]["sectors"]["left_m"] = 0.2
    records["lidar"]["evidence"]["sectors"]["right_m"] = 0.2
    result = engine.evaluate(records, 1_000_000_000)
    assert result["state"]["behavior"] == "STOP"
    assert result["events"][-1]["rule_id"] == "NO_FEASIBLE_DETOUR"


def test_passable_clear_gate_vetoes_avoidance_after_distinct_clear_scans():
    engine = _engine("S3")
    records = _records(scenario="S3", distance=0.5, camera_objects=[
        {"class": "gate", "bearing_rad": -0.12, "apparent_size_px": 80},
        {"class": "gate", "bearing_rad": 0.12, "apparent_size_px": 80},
    ])
    records["lidar"]["evidence"]["points_base_link"] = [
        {"x_m": 0.65, "y_m": -0.34, "range_m": 0.73, "bearing_rad": -0.48},
        {"x_m": 0.65, "y_m": 0.34, "range_m": 0.73, "bearing_rad": 0.48},
    ]
    for sample, now in ((0, 1_000_000_000), (1, 1_100_000_000), (2, 1_200_000_000)):
        current = _advance(records, distance=0.5, seq=sample)
        current["lidar"]["evidence"]["points_base_link"] = copy.deepcopy(records["lidar"]["evidence"]["points_base_link"])
        result = engine.evaluate(current, now)
    evaluation = [event for event in result["events"] if event["event_kind"] == "evaluation"]
    assert evaluation
    assert result["state"]["behavior"] == "FOLLOW_ROUTE"
    assert evaluation[0]["candidate_behavior"] in {"AVOID_LEFT", "AVOID_RIGHT"}
    assert evaluation[0]["attribution"]["vetoes"][0]["component"] == "camera"
    _schema_validate_event(evaluation[0])


def test_liDAR_inside_gate_opening_overrules_camera_and_uses_hazard_rules():
    engine = _engine("S3b")
    records = _records(scenario="S3b", distance=0.4, speed=0.35, camera_objects=[
        {"class": "gate", "bearing_rad": -0.12, "apparent_size_px": 80},
        {"class": "gate", "bearing_rad": 0.12, "apparent_size_px": 80},
    ])
    records["lidar"]["evidence"]["points_base_link"] = [
        {"x_m": 0.4, "y_m": 0.0, "range_m": 0.4, "bearing_rad": 0.0},
    ]
    result = engine.evaluate(records, 1_000_000_000)
    assert result["state"]["behavior"] == "CAUTION"
    assert result["events"][-1]["attribution"]["overruled"][0]["overruled_by"] == "lidar"
    records = _advance(records, distance=0.16, seq=1)
    records["lidar"]["evidence"]["points_base_link"] = [
        {"x_m": 0.16, "y_m": 0.0, "range_m": 0.16, "bearing_rad": 0.0},
    ]
    result = engine.evaluate(records, 1_050_000_000)
    assert result["state"]["behavior"] == "STOP"
    assert result["events"][-1]["attribution"]["overruled"][0]["component"] == "camera"
    _schema_validate_event(result["events"][-1])


def test_contact_latches_stop_and_stale_camera_uses_lidar_only_clearance():
    engine = _engine()
    records = _records(contact=True)
    result = engine.evaluate(records, 1_000_000_000)
    assert result["state"]["behavior"] == "STOP"
    assert result["events"][-1]["attribution"]["triggers"][0]["component"] == "contact"
    assert result["events"][-1]["rule_id"] == "CONTACT_LATCH"

    # The contact topic can go quiet during collision bounce. This run has no
    # explicit reset path, so later empty scans cannot resume the vehicle.
    records = _advance(records, distance=None, seq=1)
    records["contact"]["evidence"] = {
        "contacts": [], "ground_excluded": True, "non_ground_collision": False,
    }
    for index in range(1, 12):
        result = engine.evaluate(_advance(records, distance=None, seq=index + 1),
                                 1_000_000_000 + index * 250_000_000)
        assert result["state"]["behavior"] == "STOP"
        assert result["state"]["attribution"]["triggers"][0]["component"] == "contact"
    assert result["state"]["attribution"]["triggers"][0]["value"][0]["collision_b"] == "barrier_01::obstacle_link::barrier_contact"

    fallback = _engine()
    records = _records(distance=0.30, camera_objects=[{"class": "post", "bearing_rad": 0.0, "apparent_size_px": 60}])
    assert fallback.evaluate(records, 900_000_000)["state"]["behavior"] == "AVOID_LEFT"
    records = _advance(records, distance=0.30, seq=1)
    records["camera"]["freshness"]["state"] = "stale"
    records["camera"]["valid"] = False
    result = fallback.evaluate(records, 1_000_000_000)
    assert result["intent"]["class_margin_m"] == pytest.approx(0.225 * 0.75)


def test_stop_requires_clear_scan_dwell_then_resume_and_follow():
    engine = _engine()
    records = _records(distance=0.1, speed=0.35)
    assert engine.evaluate(records, 1_000_000_000)["state"]["behavior"] == "STOP"
    for index in range(1, 8):
        records = _advance(records, distance=None, seq=index)
        result = engine.evaluate(records, 1_000_000_000 + index * 250_000_000)
        if result["state"]["behavior"] == "RESUME":
            break
    assert result["state"]["behavior"] == "RESUME"
    result = engine.evaluate(records, 2_900_000_000)
    assert result["state"]["behavior"] == "FOLLOW_ROUTE"


def test_demonstration_records_propagate_frozen_root_and_fixed_inputs_replay_deterministically():
    engine = _engine(purpose="demonstration", root_hash=ROOT_SHA)
    records = _records(distance=0.4, purpose="demonstration")
    for record in records.values():
        record["frozen_inputs_root_sha256"] = ROOT_SHA
    result = engine.evaluate(records, 1_000_000_000)
    assert result["state"]["frozen_inputs_root_sha256"] == ROOT_SHA
    assert result["intent"]["frozen_inputs_root_sha256"] == ROOT_SHA
    assert result["intent"]["run_purpose"] == "demonstration"
    replay = _engine(purpose="demonstration", root_hash=ROOT_SHA).evaluate(records, 1_000_000_000)
    assert result["state"]["behavior"] == replay["state"]["behavior"]
    assert result["state"]["attribution"] == replay["state"]["attribution"]
