"""Boundary tests for course-length limits, independent of Gazebo/world files."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


TOOL = Path(__file__).resolve().parents[1] / "check_obstacle_course_sizing.py"
SPEC = importlib.util.spec_from_file_location("check_obstacle_course_sizing", TOOL)
assert SPEC and SPEC.loader
sizing = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sizing)


PROFILE = {
    "geometry": {"wheelbase_m": {"value": 0.27}},
    "steering": {"effective_center_wheel_limit_rad": {"value": 0.45}},
    "derived_geometry": {"minimum_turn_radius_m": {"value": 0.5589424875513274}},
    "dynamics": {
        "braking_deceleration_mps2": {"value": 0.3999, "status": "measured_sim"},
        "request_to_deceleration_latency_s": {"value": 0.14, "status": "measured_sim"},
    },
}
GEOMETRY = {
    "vehicle": {
        "length_m": {"value": 0.38, "unit": "m", "provenance": "model"},
        "width_m": {"value": 0.225, "unit": "m", "provenance": "wheel envelope"},
    }
}


def _scenario(route_length, *, entry_speed=0.5, approach=0.4, shift=0.3, recovery=0.4):
    return {
        "scenario_id": "test-route",
        "route": {
            "length_m": {"value": route_length, "unit": "m", "provenance": "test fixture"},
            "sizing": {
                "entry_speed_mps": {"value": entry_speed, "unit": "m/s", "provenance": "test fixture"},
                "approach_distance_m": {"value": approach, "unit": "m", "provenance": "test fixture"},
                "lateral_shift_m": {"value": shift, "unit": "m", "provenance": "test fixture"},
                "recovery_distance_m": {"value": recovery, "unit": "m", "provenance": "test fixture"},
            },
        },
    }


def test_route_at_derived_margin_limit_passes():
    baseline = _scenario(2.0)
    details = sizing.derive_scenario_sizing(baseline, PROFILE, GEOMETRY)
    at_bound = _scenario(details["maximum_route_length_m"])
    assert sizing.derive_scenario_sizing(at_bound, PROFILE, GEOMETRY)["route_length_m"] == pytest.approx(
        details["maximum_route_length_m"]
    )


def test_route_above_derived_margin_fails_before_world_creation():
    details = sizing.derive_scenario_sizing(_scenario(2.0), PROFILE, GEOMETRY)
    too_long = _scenario(details["maximum_route_length_m"] + 1e-6)
    with pytest.raises(ValueError, match="exceeds 1.25"):
        sizing.derive_scenario_sizing(too_long, PROFILE, GEOMETRY)


def test_route_immediately_below_strict_280m_cap_passes():
    route = _scenario(279.999, entry_speed=0.1, approach=224.9735, shift=0, recovery=0)
    result = sizing.derive_scenario_sizing(route, PROFILE, GEOMETRY)
    assert result["route_length_m"] < 280
    assert result["route_length_m"] <= result["maximum_route_length_m"]


def test_route_at_or_above_280m_fails_even_when_margin_allows_it():
    route = _scenario(280.0, entry_speed=0.1, approach=224.9735, shift=0, recovery=0)
    with pytest.raises(ValueError, match="strict 280m course cap"):
        sizing.derive_scenario_sizing(route, PROFILE, GEOMETRY)


def test_stopping_distance_is_part_of_the_minimum_route():
    route = _scenario(2.0)
    result = sizing.derive_scenario_sizing(route, PROFILE, GEOMETRY)
    assert result["minimum_feasible_route_length_m"] == pytest.approx(
        0.4 + sizing.stopping_distance_m(0.5, 0.3999, 0.14)
        + sizing.lane_change_minimum_lengths_m(0.3, 0.5589424875513274)["path_length_m"]
        + 0.4
    )
