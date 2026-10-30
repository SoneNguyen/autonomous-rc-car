from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.checks.lidar_matrix import (  # noqa: E402
    beam_for_angle,
    endpoint_observation,
    evaluate_scan_case,
    evaluate_moving_case,
    moving_target_order,
    nearest_range_in_sector,
    sector_for_angle,
    stale_status,
)


BOUNDARIES = tuple(map(lambda value: value * 3.141592653589793 / 180, (-90, -30, -10, 10, 30, 90)))


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(-90, "right"), (-30, "front_right"), (-10, "front"), (0, "front"), (10, "front_left"), (30, "left"), (90, "left")],
)
def test_sector_boundaries_use_configured_bisect_right_convention(angle, expected):
    assert sector_for_angle(angle * 3.141592653589793 / 180, BOUNDARIES) == expected


def test_beam_quantization_selects_nearest_supported_sample():
    assert beam_for_angle(-1.0, -3.141592653589793 / 2, 3.141592653589793 / 360, 361) == 65
    with pytest.raises(ValueError, match="outside"):
        beam_for_angle(-2.0, -1.0, 0.5, 5)


def test_range_lookup_ignores_invalid_and_out_of_range_returns():
    scan = {"ranges": [float("nan"), 0.04, 2.0, 8.01, 4.5], "angle_min": -1.0, "angle_increment": 0.1,
            "range_min": 0.05, "range_max": 8.0}
    assert nearest_range_in_sector(scan, center_angle=-0.8, half_width_rad=0.05) == pytest.approx(2.0)


@pytest.mark.parametrize(("measured", "classification"), [(0.04, "below"), (0.05, "boundary"), (0.06, "inside")])
def test_minimum_range_endpoint_observation(measured, classification):
    observation = endpoint_observation(measured, endpoint_m=0.05, resolution_m=0.01)
    assert observation["classification"] == classification


def test_maximum_range_endpoint_observation():
    assert endpoint_observation(7.99, endpoint_m=8.0, resolution_m=0.01)["classification"] == "inside"
    assert endpoint_observation(8.0, endpoint_m=8.0, resolution_m=0.01)["classification"] == "boundary"
    assert endpoint_observation(8.01, endpoint_m=8.0, resolution_m=0.01)["classification"] == "beyond"


def test_stale_status_brackets_configured_age_and_maps_existing_stop_reason():
    assert stale_status(observed_age_s=0.49, max_age_s=0.5)["stale"] is False
    stale = stale_status(observed_age_s=0.51, max_age_s=0.5)
    assert stale["stale"] is True
    assert stale["stop_reason"] == "sensor_stale"


def test_ground_only_hit_accounting_does_not_count_nonfinite_returns():
    scan = {"ranges": [float("inf"), float("nan"), 0.0, 8.01], "range_min": 0.05, "range_max": 8.0,
            "angle_min": -1.0, "angle_increment": 0.1}
    assert nearest_range_in_sector(scan, center_angle=0.0, half_width_rad=3.14) is None


def test_moving_target_samples_are_ordered_by_simulation_stamp():
    ordered = moving_target_order([
        {"stamp_sim_s": 2.0, "range_m": 3.0},
        {"stamp_sim_s": 1.0, "range_m": 4.0},
    ])
    assert [item["stamp_sim_s"] for item in ordered] == [1.0, 2.0]
    assert ordered[1]["range_rate_mps"] == -1.0


def test_moving_lidar_case_requires_acknowledged_motion_and_measured_close_then_recede():
    scenario = {
        "scenario_id": "lidar/moving-obstacle",
        "setup": {"target_entity": "vehicle_01", "target_bearing_rad": 0.0},
        "obstacle_placements": [{"entity_id": "vehicle_01", "speed_profile": [
            {"sim_time_s": 0.0, "speed_mps": -0.2}, {"sim_time_s": 1.0, "speed_mps": 0.2},
        ]}],
        "tolerances": [{"tolerance_id": "moving-range", "measurement_id": "nearest_return"}],
    }
    scans = []
    for stamp_s, distance in ((0.2, 3.0), (0.4, 2.9), (0.6, 2.8),
                              (1.2, 2.7), (1.4, 2.8), (1.6, 2.9)):
        ranges = [float("inf")] * 361
        ranges[180] = distance
        scans.append({"stamp_sim_ns": int(stamp_s * 1e9), "angle_min": -1.5707963,
                      "angle_increment": 3.1415926 / 360, "range_min": 0.05,
                      "range_max": 8.0, "ranges": ranges})
    capture = {
        "clock_start_sim_ns": 0,
        "clock_last_sim_ns": 2_000_000_000,
        "scans": scans,
        "pose_commands": [{"service_success": True} for _ in range(8)],
    }
    status, observations, reason = evaluate_moving_case(scenario, capture)
    assert status == "pass" and reason is None
    assert observations[0]["value"]["range_decreased_before_turnaround"]
    assert observations[0]["value"]["range_increased_after_turnaround"]


def test_moving_lidar_case_fails_without_sensor_returns_even_when_pose_service_works():
    scenario = {
        "scenario_id": "lidar/moving-obstacle",
        "setup": {"target_entity": "vehicle_01", "target_bearing_rad": 0.0},
        "obstacle_placements": [{"entity_id": "vehicle_01", "speed_profile": [
            {"sim_time_s": 0.0, "speed_mps": -0.2}, {"sim_time_s": 1.0, "speed_mps": 0.2},
        ]}],
        "tolerances": [{"tolerance_id": "moving-range", "measurement_id": "nearest_return"}],
    }
    scan = {"stamp_sim_ns": 100_000_000, "angle_min": -1.5707963,
            "angle_increment": 3.1415926 / 360, "range_min": 0.05,
            "range_max": 8.0, "ranges": [float("inf")] * 361}
    status, observations, reason = evaluate_moving_case(
        scenario, {"clock_start_sim_ns": 0, "scans": [scan],
                   "pose_commands": [{"service_success": True} for _ in range(8)]},
    )
    assert status == "fail"
    assert reason == "the moving research target produced no finite in-range LiDAR return"


def test_case_evaluation_records_frame_stamp_and_expected_sector():
    scenario = {
        "scenario_id": "lidar/sector-front", "setup": {"target_bearing_rad": 0.0,
        "target_surface_range_m": 2.0, "expected_sector": "front"},
        "measurements": [{"measurement_id": "nearest_return"}],
        "tolerances": [{"tolerance_id": "front-range", "absolute_error": 0.02}],
    }
    scan = {"stamp_sim_ns": 10, "frame_id": "lidar_link", "angle_min": -0.1,
            "angle_increment": 0.1, "range_min": 0.05, "range_max": 8.0,
            "ranges": [2.1, 2.0, 2.1]}
    status, observations, reason = evaluate_scan_case(scenario, [scan])
    assert status == "pass" and reason is None
    assert observations[0]["stamp_sim_ns"] == 10
    assert observations[0]["value"]["sector"] == "front"


def test_minimum_range_endpoint_rounding_requires_a_real_boundary_return():
    scenario = {
        "scenario_id": "lidar/range-min-at",
        "setup": {"target_bearing_rad": 0.0, "target_surface_range_m": 0.05},
        "measurements": [{"measurement_id": "nearest_return"}],
        "tolerances": [{"tolerance_id": "minimum-range", "absolute_error": 0.02}],
    }
    scan = {"stamp_sim_ns": 10, "frame_id": "lidar_link", "angle_min": -0.1,
            "angle_increment": 0.1, "range_min": 0.050000000745,
            "range_max": 8.0, "ranges": [float("inf")] * 3}
    status, observations, reason = evaluate_scan_case(scenario, [scan])
    assert status == "fail"
    assert observations[0]["ground_truth"] == {"target_surface_range_m": 0.05, "beam_index": 1}
    assert reason == "the declared target produced no finite in-range return"


def test_case_evaluation_rejects_wrong_sensor_frame():
    scenario = {"scenario_id": "lidar/frame", "setup": {"target_bearing_rad": 0.0,
        "target_surface_range_m": 2.0}, "measurements": [{}],
        "tolerances": [{"tolerance_id": "range", "absolute_error": 0.02}]}
    scan = {"stamp_sim_ns": 10, "frame_id": "base_link", "angle_min": -0.1,
            "angle_increment": 0.1, "range_min": 0.05, "range_max": 8.0,
            "ranges": [2.0, 2.0, 2.0]}
    status, observations, reason = evaluate_scan_case(scenario, [scan])
    assert status == "fail" and observations[0]["result"] == "fail"
    assert "frame_id" in reason
