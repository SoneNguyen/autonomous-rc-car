"""Pure contract and normalization tests for the five Obstacle course evidence adapters."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml
from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(PACKAGE_ROOT))
from car_decision.evidence_common import (  # noqa: E402
    ClockEpochTracker,
    classify_hsv_image,
    fault_active,
    freshness,
    lidar_payload,
    route_progress,
)
from car_decision.contact_evidence_node import normalize_contacts  # noqa: E402
from car_decision.odometry_evidence_node import pose_components, relative_pose  # noqa: E402


CONFIG_ROOT = REPO / "ros2_ws/src/car_bringup/config/obstacle_course"
SCENARIOS = CONFIG_ROOT / "scenarios"
SCHEMA = REPO / "ros2_ws/src/car_bringup/config/obstacle_course/contracts/component-evidence.schema.json"


def _scenario(name):
    return yaml.safe_load((SCENARIOS / f"{name}.yaml").read_text(encoding="utf-8"))


def _record(component, payload, *, freshness_state="fresh", run_purpose="diagnostic"):
    record = {
        "schema_version": "1.0",
        "component": component,
        "scenario_id": "S1",
        "run_id": "1de8fb69-2874-49d2-bf46-c485617e6f62",
        "run_purpose": run_purpose,
        "sequence": 0,
        "source_sample_sequence": 0,
        "sim_time_ns": 1_000_000_000,
        "clock_epoch": 0,
        "freshness": {"state": freshness_state, "age_s": 0.0, "timeout_s": 1.0},
        "valid": freshness_state == "fresh",
        "invalid_reason": None if freshness_state == "fresh" else "test stale record",
        "source": {"topic": f"/{component}", "message_type": "std_msgs/msg/String", "adapter_id": f"test-{component}"},
        "config_sha256": "a" * 64,
        "profile_sha256": "b" * 64,
        "frame_id": "base_link",
        "evidence": payload,
    }
    if run_purpose == "demonstration":
        record["frozen_inputs_root_sha256"] = "c" * 64
    return record


def test_freshness_tracks_missing_old_and_current_samples_in_sim_time():
    assert freshness(None, 10_000_000_000, 0.25)["state"] == "unavailable"
    assert freshness(9_800_000_000, 10_000_000_000, 0.25)["state"] == "fresh"
    assert freshness(9_000_000_000, 10_000_000_000, 0.25)["state"] == "stale"
    with pytest.raises(ValueError, match="positive"):
        freshness(None, 0, 0.0)


def test_clock_epoch_increments_on_simulation_reset_and_not_on_equal_time():
    clock = ClockEpochTracker()
    assert not clock.observe(100)
    assert not clock.observe(100)
    assert not clock.observe(200)
    assert clock.observe(10)
    assert clock.epoch == 1


def test_fixed_seed_dropout_windows_are_half_open_and_sensor_specific():
    scenario = _scenario("S6-lidar-drop")
    assert not fault_active(scenario, "lidar", 999_999_999)
    assert fault_active(scenario, "lidar", 1_000_000_000)
    assert fault_active(scenario, "lidar", 3_999_999_999)
    assert not fault_active(scenario, "lidar", 4_000_000_000)
    assert not fault_active(scenario, "camera", 2_000_000_000)


def test_contact_ground_pairs_do_not_count_as_collisions_and_obstacles_are_identified():
    ground = SimpleNamespace(
        collision1=SimpleNamespace(name="gazebo_truck::wheel_collision"),
        collision2=SimpleNamespace(name="ground_plane::link::collision"),
    )
    obstacle = SimpleNamespace(
        collision1=SimpleNamespace(name="gazebo_truck::base_link::collision"),
        collision2=SimpleNamespace(name="barrier_01::obstacle_link::collision"),
    )
    ground_result = normalize_contacts([ground], {"barrier_01"})
    assert ground_result["ground_excluded"] is True
    assert ground_result["non_ground_collision"] is False
    obstacle_result = normalize_contacts([obstacle], {"barrier_01"})
    assert obstacle_result["ground_excluded"] is False
    assert obstacle_result["non_ground_collision"] is True
    assert obstacle_result["contacts"][0]["allowlisted"] is True


def test_route_progress_projects_to_nearest_segment_and_clamps_endpoints():
    route = {"waypoints": [{"x": {"value": 0}, "y": {"value": 0}}, {"x": {"value": 1}, "y": {"value": 0}}, {"x": {"value": 1}, "y": {"value": 1}}]}
    assert route_progress(route, 0.5, 0.1) == pytest.approx(0.5)
    assert route_progress(route, 1.0, 0.5) == pytest.approx(1.5)
    assert route_progress(route, 1.0, 2.0) == pytest.approx(2.0)


def test_pose_components_extracts_world_pose_from_quaternion():
    pose = SimpleNamespace(
        position=SimpleNamespace(x=2.0, y=-1.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.4), w=math.cos(0.4)),
    )
    assert pose_components(pose) == pytest.approx((2.0, -1.0, 0.8))


def test_ground_truth_pose_is_normalized_to_scenario_start_frame():
    assert relative_pose((7.0, 9.0, math.pi / 2), (7.0, 10.0, math.pi)) == pytest.approx(
        (1.0, 0.0, math.pi / 2)
    )


def test_lidar_transform_groups_returns_into_five_vehicle_relative_sectors():
    payload = lidar_payload(
        [1.0, 0.8, float("inf"), 2.0, 1.5, 1.3, 1.2],
        angle_min=-math.pi / 2,
        angle_increment=math.radians(31),
        range_min=0.05,
        range_max=8.0,
        lidar_pose=[0.0, 0.0, 0.055, 0, 0, 0],
        sector_boundaries=[-math.pi / 2, -math.pi / 3, -math.pi / 6, math.pi / 6, math.pi / 3, math.pi / 2],
        corridor_half_width_m=0.225,
        self_filter_radius_m=0.25,
    )
    assert payload["input_adapter"] == "laser_scan"
    assert payload["ground_points_rejected"] == 0
    assert payload["invalid_returns_rejected"] == 1
    assert payload["usable_points"] == 5
    assert len(payload["points_base_link"]) == 5
    assert payload["points_base_link"][0]["range_m"] == pytest.approx(1.0)
    assert payload["sectors"]["front_m"] == 2.0
    assert payload["route_corridor"]["occupied"] is True
    assert payload["route_corridor"]["nearest_range_m"] == 2.0


def test_hsv_classifier_separates_red_barrier_orange_post_and_blue_gate():
    decision = yaml.safe_load((CONFIG_ROOT / "decision.yaml").read_text(encoding="utf-8"))
    frame = np.zeros((100, 180, 3), dtype=np.uint8)
    frame[20:45, 10:40] = (0, 0, 255)
    frame[20:45, 70:100] = (0, 165, 255)
    frame[20:45, 130:160] = (255, 0, 0)
    objects = classify_hsv_image(frame, decision, math.radians(70))
    assert {item["class"] for item in objects} == {"barrier", "post", "gate"}
    assert all(item["confidence"] == pytest.approx(1.0) for item in objects)
    assert objects[0]["bearing_rad"] < 0
    assert objects[-1]["bearing_rad"] > 0


def test_hsv_classifier_merges_fragmented_same_color_mesh_but_keeps_separate_objects():
    decision = yaml.safe_load((CONFIG_ROOT / "decision.yaml").read_text(encoding="utf-8"))
    frame = np.zeros((70, 180, 3), dtype=np.uint8)
    frame[20:35, 15:25] = (0, 0, 255)
    frame[20:35, 31:41] = (0, 0, 255)
    frame[20:35, 105:115] = (0, 0, 255)
    objects = classify_hsv_image(frame, decision, math.radians(70))
    barriers = [item for item in objects if item["class"] == "barrier"]
    assert len(barriers) == 2
    assert sorted(item["blob_area_px"] for item in barriers) == [150, 300]


def test_hsv_classifier_deduplicates_overlapping_colors_on_one_mesh():
    decision = yaml.safe_load((CONFIG_ROOT / "decision.yaml").read_text(encoding="utf-8"))
    hsv = np.zeros((80, 180, 3), dtype=np.uint8)
    hsv[20:45, 20:34] = (0, 255, 255)   # red center
    hsv[20:45, 10:20] = (10, 255, 255)  # orange reflector at the left edge
    hsv[20:45, 34:44] = (10, 255, 255)  # orange reflector at the right edge
    hsv[20:45, 110:135] = (18, 255, 255)  # a separate orange post
    frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    objects = classify_hsv_image(frame, decision, math.radians(70))

    assert [item["class"] for item in objects].count("barrier") == 1
    assert [item["class"] for item in objects].count("post") == 1
    assert objects[0]["class"] == "post"
    assert objects[1]["class"] == "barrier"


@pytest.mark.parametrize("component,payload", [
    ("lidar", {"input_adapter": "laser_scan", "sectors": {name: None for name in ("right_m", "front_right_m", "front_m", "front_left_m", "left_m")}, "route_corridor": {"occupied": False, "nearest_range_m": None}, "points_base_link": [], "ground_points_rejected": 0, "invalid_returns_rejected": 0, "usable_points": 0}),
    ("camera", {"classifier": "hsv", "objects": []}),
    ("imu", {"yaw_rate_rad_s": 0.0, "lateral_acceleration_mps2": 0.0, "longitudinal_acceleration_mps2": 0.0}),
    ("odometry", {"world_pose": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}, "start_relative_pose": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}, "speed_mps": 0.0, "route_progress_m": 0.0, "d_stop_m": 0.0, "profile_id": "obstacle-course-rc-car", "pose_source": "gazebo_named_model_pose", "speed_source": "ackermann_odometry_twist", "wheel_odometry_pose_error_m": 0.0}),
    ("contact", {"contacts": [], "ground_excluded": True, "non_ground_collision": False}),
])
def test_component_records_validate_against_shared_schema(component, payload):
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    Draft202012Validator(schema).validate(_record(component, payload))


def test_demonstration_records_require_the_frozen_root():
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    record = _record("imu", {"yaw_rate_rad_s": 0.0, "lateral_acceleration_mps2": 0.0, "longitudinal_acceleration_mps2": 0.0}, run_purpose="demonstration")
    Draft202012Validator(schema).validate(record)
    del record["frozen_inputs_root_sha256"]
    with pytest.raises(Exception):
        Draft202012Validator(schema).validate(record)
