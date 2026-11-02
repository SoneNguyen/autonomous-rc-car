#!/usr/bin/env python3
"""Recompute the Obstacle course route-length bound from measured RC geometry."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

import yaml


REPO = Path(__file__).resolve().parents[1]
COURSE_CAP_M = 280.0
LAYOUT_MARGIN_RATIO = 0.25


def _positive(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be a finite positive value")
    return result


def minimum_turn_radius_m(wheelbase_m: float, center_steering_limit_rad: float) -> float:
    wheelbase = _positive("wheelbase_m", wheelbase_m)
    steering = _positive("center_steering_limit_rad", center_steering_limit_rad)
    if steering >= math.pi / 2:
        raise ValueError("center_steering_limit_rad must be below pi/2")
    return wheelbase / math.tan(steering)


def wheel_inclusive_envelope_m(*, chassis_length_m: float, chassis_width_m: float,
                               wheel_centers_xy_m, wheel_radius_m: float,
                               wheel_width_m: float) -> dict:
    chassis_length = _positive("chassis_length_m", chassis_length_m)
    chassis_width = _positive("chassis_width_m", chassis_width_m)
    radius = _positive("wheel_radius_m", wheel_radius_m)
    width = _positive("wheel_width_m", wheel_width_m)
    centers = tuple((float(x), float(y)) for x, y in wheel_centers_xy_m)
    if not centers or not all(math.isfinite(v) for point in centers for v in point):
        raise ValueError("at least one finite wheel center is required")
    half_x = max(chassis_length / 2, *(abs(x) + radius for x, _y in centers))
    half_y = max(chassis_width / 2, *(abs(y) + width / 2 for _x, y in centers))
    length = 2 * half_x
    full_width = 2 * half_y
    return {
        "length_m": length,
        "width_m": full_width,
        "footprint_radius_m": math.hypot(length, full_width) / 2,
    }


def lane_change_minimum_lengths_m(lateral_shift_m: float, minimum_turn_radius_m: float) -> dict:
    shift = float(lateral_shift_m)
    radius = _positive("minimum_turn_radius_m", minimum_turn_radius_m)
    if not math.isfinite(shift) or shift < 0:
        raise ValueError("lateral_shift_m must be finite and nonnegative")
    if shift == 0:
        return {"arc_angle_rad": 0.0, "longitudinal_length_m": 0.0, "path_length_m": 0.0}
    if shift > 2 * radius:
        raise ValueError("lateral shift exceeds twice the minimum turn radius")
    theta = math.acos(1 - shift / (2 * radius))
    return {
        "arc_angle_rad": theta,
        "longitudinal_length_m": 2 * radius * math.sin(theta),
        "path_length_m": 2 * radius * theta,
    }


def apply_layout_margin(minimum_route_length_m: float, margin_ratio: float = LAYOUT_MARGIN_RATIO) -> float:
    minimum = float(minimum_route_length_m)
    margin = float(margin_ratio)
    if not math.isfinite(minimum) or minimum <= 0:
        raise ValueError("minimum route length must be finite and positive")
    if not math.isfinite(margin) or margin < 0:
        raise ValueError("layout margin ratio must be finite and nonnegative")
    return minimum * (1 + margin)


def stopping_distance_m(speed_mps: float, braking_deceleration_mps2: float,
                        latency_s: float) -> float:
    speed = float(speed_mps)
    deceleration = _positive("braking_deceleration_mps2", braking_deceleration_mps2)
    latency = float(latency_s)
    if not math.isfinite(speed) or speed < 0:
        raise ValueError("speed_mps must be finite and nonnegative")
    if not math.isfinite(latency) or latency < 0:
        raise ValueError("latency_s must be finite and nonnegative")
    return speed * speed / (2 * deceleration) + speed * latency


def _parameter(value, name: str) -> float:
    if isinstance(value, dict):
        if value.get("unit") not in {"m", "m/s"} or not value.get("provenance"):
            raise ValueError(f"{name} requires a supported unit and provenance")
        value = value.get("value")
    return float(value)


def derive_scenario_sizing(scenario: dict, profile: dict, geometry: dict) -> dict:
    route = scenario.get("route", {})
    layout = route.get("sizing", {})
    try:
        route_length = _parameter(route["length_m"], "route.length_m")
        speed = _parameter(layout["entry_speed_mps"], "route.sizing.entry_speed_mps")
        approach = _parameter(layout["approach_distance_m"], "route.sizing.approach_distance_m")
        lateral_shift = _parameter(layout.get("lateral_shift_m", {"value": 0, "unit": "m", "provenance": "no lane change"}), "route.sizing.lateral_shift_m")
        recovery = _parameter(layout["recovery_distance_m"], "route.sizing.recovery_distance_m")
        lane_change_count = int(layout.get("lane_change_count", 1 if lateral_shift > 0 else 0))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"scenario {scenario.get('scenario_id', '<unknown>')} has incomplete route sizing: {exc}") from exc
    for name, value in (("route.length_m", route_length), ("entry_speed_mps", speed),
                        ("approach_distance_m", approach), ("recovery_distance_m", recovery)):
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be finite and nonnegative")
    if route_length <= 0 or speed <= 0:
        raise ValueError("route length and entry speed must be positive")
    if lane_change_count < 0:
        raise ValueError("lane_change_count must be nonnegative")
    if lane_change_count and lateral_shift == 0:
        raise ValueError("lane_change_count must be zero when lateral_shift_m is zero")

    vehicle = geometry["vehicle"]
    vehicle_length = _parameter(vehicle["length_m"], "vehicle.length_m")
    vehicle_width = _parameter(vehicle["width_m"], "vehicle.width_m")
    if vehicle_length <= 0 or vehicle_width <= 0:
        raise ValueError("vehicle length and width must be positive")
    dynamic = profile["dynamics"]
    braking = dynamic["braking_deceleration_mps2"]
    latency = dynamic["request_to_deceleration_latency_s"]
    if braking.get("status") != "measured_sim" or latency.get("status") != "measured_sim":
        raise ValueError("vehicle profile must have measured_sim braking and command latency")
    d_stop = stopping_distance_m(speed, braking["value"], latency["value"])
    radius_record = profile.get("derived_geometry", {}).get("minimum_turn_radius_m")
    if isinstance(radius_record, dict):
        radius = _positive("minimum_turn_radius_m", radius_record["value"])
    else:
        radius = minimum_turn_radius_m(
            profile["geometry"]["wheelbase_m"]["value"],
            profile["steering"]["effective_center_wheel_limit_rad"]["value"],
        )
    lane_change = lane_change_minimum_lengths_m(lateral_shift, radius)
    minimum = approach + d_stop + lane_change["path_length_m"] * lane_change_count + recovery
    bound = apply_layout_margin(minimum)
    if route_length > bound:
        raise ValueError(
            f"scenario {scenario.get('scenario_id', '<unknown>')} route {route_length:.6f}m "
            f"exceeds 1.25 x derived minimum {bound:.6f}m"
        )
    if route_length >= COURSE_CAP_M:
        raise ValueError(
            f"scenario {scenario.get('scenario_id', '<unknown>')} route {route_length:.6f}m "
            f"reaches the strict {COURSE_CAP_M:.0f}m course cap"
        )
    return {
        "scenario_id": scenario.get("scenario_id"),
        "route_length_m": route_length,
        "entry_speed_mps": speed,
        "d_stop_m": d_stop,
        "minimum_turn_radius_m": radius,
        "lane_change": lane_change,
        "lane_change_count": lane_change_count,
        "minimum_feasible_route_length_m": minimum,
        "layout_margin_ratio": LAYOUT_MARGIN_RATIO,
        "maximum_route_length_m": bound,
        "vehicle_length_m": vehicle_length,
        "vehicle_width_m": vehicle_width,
        "course_cap_m": COURSE_CAP_M,
    }


def _tracked(path: Path) -> bool:
    result = subprocess.run(
        ["git", "ls-files", "--error-unmatch", path.relative_to(REPO).as_posix()],
        cwd=REPO, capture_output=True, text=True, check=False,
    )
    return result.returncode == 0


def _load(path: Path):
    if not _tracked(path.resolve()):
        raise ValueError(f"input config is not tracked: {path}")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--geometry", required=True, type=Path)
    parser.add_argument("--scenarios", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        profile_path = args.profile.resolve()
        geometry_path = args.geometry.resolve()
        scenario_dir = args.scenarios.resolve()
        profile = _load(profile_path)
        geometry = _load(geometry_path)
        files = sorted(path for path in scenario_dir.glob("*.yaml") if path.name != "registry.yaml")
        if not files:
            raise ValueError(f"no scenario YAML files found in {scenario_dir}")
        results = []
        for path in files:
            scenario = _load(path)
            results.append(derive_scenario_sizing(scenario, profile, geometry))
        print(json.dumps({"result": "PASS", "scenarios": results}, indent=2))
        print(f"COURSE SIZING PASS ({len(results)} scenarios; cap < {COURSE_CAP_M:.0f} m)")
        return 0
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"check_obstacle_course_sizing: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
