"""Gate passability checks using class evidence and individual LiDAR returns."""

from __future__ import annotations

import math
from typing import Any


def configured_gate_opening_width(scenario: dict[str, Any]) -> float | None:
    """Return the configured clear width between two gate leaves, if present."""
    gates = [item for item in scenario.get("obstacles", []) if item.get("class") == "gate"]
    if len(gates) < 2:
        return None
    gates.sort(key=lambda item: float(item["pose"]["y"]["value"]))
    left, right = gates[0], gates[-1]
    center_delta = abs(float(right["pose"]["y"]["value"]) - float(left["pose"]["y"]["value"]))
    left_half = float(left["dimensions_m"]["width"]["value"]) / 2.0
    right_half = float(right["dimensions_m"]["width"]["value"]) / 2.0
    return max(0.0, center_delta - left_half - right_half)


def inspect_gate_corridor(
    *,
    camera_objects: list[dict[str, Any]],
    lidar_points: list[dict[str, float]],
    opening_width_m: float | None,
    vehicle_width_m: float,
    clearance_multiple: float,
) -> dict[str, Any]:
    """Classify a gate as passable only when its opening and LiDAR corridor are clear.

    LiDAR points near the posts are deliberately excluded by measuring against the
    narrower, configured opening corridor. A point inside that opening overrules
    the camera gate classification and returns its measured range for attribution.
    """
    gate_objects = [item for item in camera_objects if item.get("class") == "gate"]
    if not gate_objects:
        return {"gate_seen": False, "passable": False, "clear": False,
                "opening_width_m": opening_width_m, "required_width_m": None,
                "return_range_m": None, "center_bearing_rad": None}

    required = vehicle_width_m * (1.0 + clearance_multiple)
    bearings = [float(item["bearing_rad"]) for item in gate_objects]
    center_bearing = sum(bearings) / len(bearings)
    passable = opening_width_m is None or opening_width_m >= required
    if not passable:
        return {"gate_seen": True, "passable": False, "clear": False,
                "opening_width_m": opening_width_m, "required_width_m": required,
                "return_range_m": None, "center_bearing_rad": center_bearing}

    half_width = required / 2.0
    center_slope = math.tan(center_bearing)
    inside = [
        point for point in lidar_points
        if float(point["x_m"]) > 0.0
        and abs(float(point["y_m"]) - float(point["x_m"]) * center_slope) <= half_width
    ]
    nearest = min((float(point["range_m"]) for point in inside), default=None)
    return {"gate_seen": True, "passable": True, "clear": nearest is None,
            "opening_width_m": opening_width_m, "required_width_m": required,
            "return_range_m": nearest, "center_bearing_rad": center_bearing}
