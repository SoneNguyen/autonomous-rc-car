"""Unit tests for Obstacle course simulated braking measurements."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


TOOL = Path(__file__).resolve().parents[1] / "measure_rc_vehicle_dynamics.py"
SPEC = importlib.util.spec_from_file_location("measure_rc_vehicle_dynamics", TOOL)
assert SPEC and SPEC.loader
measurement = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(measurement)


def _trial(index: int, *, source_hashes=None, simulator="gz-sim-10.5.0"):
    return {
        "trial_index": index,
        "accepted": True,
        "source_hashes": source_hashes or {"profile": "profile-a", "scenario": "scenario-a"},
        "simulator_identity": simulator,
        "brake_request_sim_time_s": 2.0,
        "speed_before_brake_mps": 0.5,
        "samples": [
            {"sim_time_s": 2.0, "speed_mps": 0.50, "distance_m": 0.00},
            {"sim_time_s": 2.1, "speed_mps": 0.45, "distance_m": 0.05},
            {"sim_time_s": 2.2, "speed_mps": 0.35, "distance_m": 0.09},
            {"sim_time_s": 2.3, "speed_mps": 0.25, "distance_m": 0.12},
            {"sim_time_s": 2.4, "speed_mps": 0.15, "distance_m": 0.14},
            {"sim_time_s": 2.5, "speed_mps": 0.05, "distance_m": 0.15},
            {"sim_time_s": 2.6, "speed_mps": 0.01, "distance_m": 0.151},
        ],
    }


def test_stopping_distance_uses_braking_and_latency_terms():
    assert measurement.stopping_distance_m(1.0, 0.5, 0.2) == pytest.approx(1.2)
    assert measurement.stopping_distance_m(0.0, 0.5, 0.2) == 0.0


@pytest.mark.parametrize("values", [(-1.0, 1.0, 0.1), (1.0, 0.0, 0.1), (1.0, 1.0, -0.1), (float("nan"), 1.0, 0.1)])
def test_stopping_distance_rejects_invalid_inputs(values):
    with pytest.raises(ValueError):
        measurement.stopping_distance_m(*values)


def test_aggregation_uses_conservative_repeat_bounds():
    first = _trial(1)
    second = _trial(2)
    second["samples"] = [
        {**sample, "speed_mps": sample["speed_mps"] * 0.95}
        for sample in second["samples"]
    ]
    second["speed_before_brake_mps"] = second["samples"][0]["speed_mps"]
    result = measurement.aggregate_trials(
        [first, second],
        expected_repeats=2,
        expected_source_hashes={"profile": "profile-a", "scenario": "scenario-a"},
        expected_simulator_identity="gz-sim-10.5.0",
        stop_speed_threshold_mps=0.02,
        onset_delta_mps=0.015,
        maximum_gap_s=0.25,
        minimum_samples=4,
    )
    assert result["accepted_trial_count"] == 2
    assert result["braking_deceleration_mps2"] > 0.0
    assert result["request_to_deceleration_latency_s"] == pytest.approx(0.1)
    assert result["conservative_braking_deceleration_mps2"] <= result["braking_deceleration_mps2"]
    assert result["conservative_request_to_deceleration_latency_s"] >= result["request_to_deceleration_latency_s"]


def test_aggregation_rejects_incomplete_or_stale_repeats():
    bad = _trial(1)
    for index, sample in enumerate(bad["samples"][3:], start=3):
        sample["sim_time_s"] = 3.0 + (index - 3) * 0.1
    with pytest.raises(ValueError, match="sample gap"):
        measurement.aggregate_trials(
            [bad], expected_repeats=1,
            expected_source_hashes={"profile": "profile-a", "scenario": "scenario-a"},
            expected_simulator_identity="gz-sim-10.5.0", stop_speed_threshold_mps=0.02,
            onset_delta_mps=0.015, maximum_gap_s=0.25, minimum_samples=4,
        )


@pytest.mark.parametrize("field,value", [("source_hashes", {"profile": "changed", "scenario": "scenario-a"}), ("simulator_identity", "gz-sim-other")])
def test_aggregation_rejects_profile_or_simulator_hash_mismatch(field, value):
    bad = _trial(1)
    bad[field] = value
    with pytest.raises(ValueError, match="mismatch"):
        measurement.aggregate_trials(
            [bad], expected_repeats=1,
            expected_source_hashes={"profile": "profile-a", "scenario": "scenario-a"},
            expected_simulator_identity="gz-sim-10.5.0", stop_speed_threshold_mps=0.02,
            onset_delta_mps=0.015, maximum_gap_s=0.25, minimum_samples=4,
        )


def test_aggregation_rejects_nonmonotonic_and_zero_deceleration_trials():
    bad = _trial(1)
    bad["samples"][2]["sim_time_s"] = bad["samples"][1]["sim_time_s"]
    with pytest.raises(ValueError, match="strictly increasing"):
        measurement.aggregate_trials(
            [bad], expected_repeats=1,
            expected_source_hashes={"profile": "profile-a", "scenario": "scenario-a"},
            expected_simulator_identity="gz-sim-10.5.0", stop_speed_threshold_mps=0.02,
            onset_delta_mps=0.015, maximum_gap_s=0.25, minimum_samples=4,
        )
