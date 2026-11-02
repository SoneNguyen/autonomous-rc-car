"""Route projection, pure-pursuit, and command freshness tests."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from car_decision.controller_request_adapter import bounded_drive_target  # noqa: E402
from car_decision.planner_intent_adapter import project_route, route_point_at, target_drive  # noqa: E402


ROUTE = {
    "waypoints": [
        {"x": {"value": 0.0}, "y": {"value": 0.0}},
        {"x": {"value": 1.0}, "y": {"value": 0.0}},
        {"x": {"value": 1.0}, "y": {"value": 1.0}},
    ],
    "length_m": {"value": 2.0},
}


def _intent(behavior="FOLLOW_ROUTE", **overrides):
    return {
        "behavior": behavior,
        "preferred_side": "none",
        "class_margin_m": 0.1125,
        "speed_cap_mps": 0.35,
        "valid_until_ns": 5_000_000_000,
        **overrides,
    }


def test_polyline_projection_and_lookahead_follow_nonzero_route_origin():
    assert project_route(ROUTE["waypoints"], 0.5, 0.1) == pytest.approx((0.5, 0.0, 0.1))
    assert route_point_at(ROUTE["waypoints"], 1.5) == pytest.approx((1.0, 0.5))
    command = target_drive(
        pose={"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0}, intent=_intent(), route=ROUTE,
        vehicle_width_m=0.225, vehicle_length_m=0.38, wheelbase_m=0.27,
        entry_speed_mps=0.35, now_ns=1_000_000_000,
    )
    assert command["speed_mps"] == pytest.approx(0.35)
    assert command["steering_angle_rad"] > 0


def test_straight_route_holds_zero_trim_and_corrects_lateral_drift():
    straight = {
        "waypoints": [
            {"x": {"value": 0.0}, "y": {"value": 0.0}},
            {"x": {"value": 10.0}, "y": {"value": 0.0}},
        ],
        "length_m": {"value": 10.0},
    }
    kwargs = {
        "intent": _intent(), "route": straight, "vehicle_width_m": 0.225,
        "vehicle_length_m": 0.38, "wheelbase_m": 0.27,
        "entry_speed_mps": 0.35, "now_ns": 1_000_000_000,
    }
    centered = target_drive(pose={"x_m": 2.0, "y_m": 0.0, "yaw_rad": 0.0}, **kwargs)
    drifted = target_drive(pose={"x_m": 2.0, "y_m": 0.1, "yaw_rad": 0.0}, **kwargs)
    assert centered["steering_angle_rad"] == pytest.approx(0.0, abs=1e-9)
    assert drifted["steering_angle_rad"] < 0.0


def test_avoidance_intent_turns_to_configured_side_and_applies_speed_cap():
    pose = {"x_m": 0.3, "y_m": 0.0, "yaw_rad": 0.0}
    left = target_drive(
        pose=pose, intent=_intent("AVOID_LEFT", preferred_side="left", speed_cap_mps=0.2),
        route=ROUTE, vehicle_width_m=0.225, vehicle_length_m=0.38, wheelbase_m=0.27,
        entry_speed_mps=0.35, now_ns=1_000_000_000, target_offset_extra_m=0.1,
    )
    right = target_drive(
        pose=pose, intent=_intent("AVOID_RIGHT", preferred_side="right", speed_cap_mps=0.2),
        route=ROUTE, vehicle_width_m=0.225, vehicle_length_m=0.38, wheelbase_m=0.27,
        entry_speed_mps=0.35, now_ns=1_000_000_000, target_offset_extra_m=0.1,
    )
    caution_hold = target_drive(
        pose=pose, intent=_intent("CAUTION", preferred_side="left", speed_cap_mps=0.2),
        route=ROUTE, vehicle_width_m=0.225, vehicle_length_m=0.38, wheelbase_m=0.27,
        entry_speed_mps=0.35, now_ns=1_000_000_000, target_offset_extra_m=0.1,
    )
    assert left["steering_angle_rad"] > 0
    assert right["steering_angle_rad"] < 0
    assert caution_hold["steering_angle_rad"] > 0
    assert left["speed_mps"] == right["speed_mps"] == pytest.approx(0.2)


def test_avoidance_uses_short_lookahead_while_clear_route_tracking_stays_smooth():
    straight = {
        "waypoints": [
            {"x": {"value": 0.0}, "y": {"value": 0.0}},
            {"x": {"value": 10.0}, "y": {"value": 0.0}},
        ],
        "length_m": {"value": 10.0},
    }
    common = {
        "pose": {"x_m": 0.3, "y_m": 0.0, "yaw_rad": 0.0},
        "route": straight,
        "vehicle_width_m": 0.225,
        "vehicle_length_m": 0.38,
        "wheelbase_m": 0.27,
        "entry_speed_mps": 0.35,
        "now_ns": 1_000_000_000,
        "target_offset_extra_m": 0.1,
    }
    maneuver = target_drive(
        intent=_intent("AVOID_LEFT", preferred_side="left"), **common,
    )
    long_horizon_maneuver = target_drive(
        intent=_intent("AVOID_LEFT", preferred_side="left"),
        lookahead_min_multiple_L=2.5, **common,
    )
    route_tracking = target_drive(intent=_intent(), **common)

    assert maneuver["steering_angle_rad"] > long_horizon_maneuver["steering_angle_rad"]
    assert maneuver["steering_angle_rad"] > 0.35
    assert route_tracking["steering_angle_rad"] == pytest.approx(0.0, abs=1e-9)


def test_scenario_lateral_shift_is_a_minimum_clearance_target():
    straight = {
        "waypoints": [
            {"x": {"value": 0.0}, "y": {"value": 0.0}},
            {"x": {"value": 10.0}, "y": {"value": 0.0}},
        ],
        "length_m": {"value": 10.0},
    }
    common = {
        "pose": {"x_m": 0.3, "y_m": 0.0, "yaw_rad": 0.0},
        "intent": _intent("AVOID_LEFT", preferred_side="left", speed_cap_mps=0.2),
        "route": straight, "vehicle_width_m": 0.225, "vehicle_length_m": 0.38,
        "wheelbase_m": 0.27, "entry_speed_mps": 0.35, "now_ns": 1_000_000_000,
    }
    geometric = target_drive(**common)
    configured = target_drive(**common, target_offset_min_m=0.55)

    assert configured["steering_angle_rad"] > geometric["steering_angle_rad"]


def test_avoidance_returns_to_route_instead_of_holding_a_steering_trim():
    straight = {
        "waypoints": [
            {"x": {"value": 0.0}, "y": {"value": 0.0}},
            {"x": {"value": 10.0}, "y": {"value": 0.0}},
        ],
        "length_m": {"value": 10.0},
    }
    common = {
        "route": straight, "vehicle_width_m": 0.225, "vehicle_length_m": 0.38,
        "wheelbase_m": 0.27, "entry_speed_mps": 0.35,
        "now_ns": 1_000_000_000,
    }
    detour = target_drive(
        pose={"x_m": 3.0, "y_m": 0.0, "yaw_rad": 0.0},
        intent=_intent("AVOID_LEFT", preferred_side="left"),
        target_offset_extra_m=0.1, **common,
    )
    return_to_route = target_drive(
        pose={"x_m": 4.0, "y_m": 0.3, "yaw_rad": 0.2},
        intent=_intent("FOLLOW_ROUTE"), **common,
    )
    centered = target_drive(
        pose={"x_m": 5.0, "y_m": 0.0, "yaw_rad": 0.0},
        intent=_intent("FOLLOW_ROUTE"), **common,
    )

    assert detour["steering_angle_rad"] > 0.0  # turn left only while avoidance is requested
    assert return_to_route["steering_angle_rad"] < 0.0  # counter-steer toward the route
    assert centered["steering_angle_rad"] == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("behavior", ["STOP"])
def test_stop_or_expired_intent_fails_closed(behavior):
    command = target_drive(
        pose={"x_m": 0.2, "y_m": 0.0, "yaw_rad": 0.0},
        intent=_intent(behavior), route=ROUTE, vehicle_width_m=0.225,
        vehicle_length_m=0.38, wheelbase_m=0.27, entry_speed_mps=0.35,
        now_ns=1_000_000_000,
    )
    assert command == {"speed_mps": 0.0, "steering_angle_rad": 0.0}
    expired = target_drive(
        pose={"x_m": 0.2, "y_m": 0.0, "yaw_rad": 0.0},
        intent=_intent(valid_until_ns=999), route=ROUTE, vehicle_width_m=0.225,
        vehicle_length_m=0.38, wheelbase_m=0.27, entry_speed_mps=0.35,
        now_ns=1_000,
    )
    assert expired["speed_mps"] == 0.0


def test_route_completion_commands_zero_speed_and_target_bounds_are_enforced():
    completed = target_drive(
        pose={"x_m": 2.0, "y_m": 1.0, "yaw_rad": math.pi / 2}, intent=_intent(),
        route=ROUTE, vehicle_width_m=0.225, vehicle_length_m=0.38,
        wheelbase_m=0.27, entry_speed_mps=0.35, now_ns=1_000_000_000,
    )
    assert completed["speed_mps"] == 0.0
    assert bounded_drive_target(
        {"speed_mps": 5.0, "steering_angle_rad": 3.0, "valid_until_ns": 10},
        max_speed_mps=1.0, max_steering_angle_rad=0.45, now_ns=9,
    ) == pytest.approx((1.0, 0.45))
    assert bounded_drive_target(
        {"speed_mps": 5.0, "steering_angle_rad": 3.0, "valid_until_ns": 8},
        max_speed_mps=1.0, max_steering_angle_rad=0.45, now_ns=9,
    ) == (0.0, 0.0)
    with pytest.raises(ValueError, match="finite"):
        bounded_drive_target(
            {"speed_mps": float("nan"), "steering_angle_rad": 0.0, "valid_until_ns": 10},
            max_speed_mps=1.0, max_steering_angle_rad=0.45, now_ns=9,
        )
