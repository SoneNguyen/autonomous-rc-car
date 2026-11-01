"""Tests for vehicle-envelope and bounded-course geometry derivations."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest


TOOL = Path(__file__).resolve().parents[1] / "check_obstacle_course_sizing.py"
SPEC = importlib.util.spec_from_file_location("check_obstacle_course_sizing", TOOL)
assert SPEC and SPEC.loader
geometry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(geometry)


def test_minimum_turn_radius_uses_wheelbase_and_center_steering_limit():
    assert geometry.minimum_turn_radius_m(0.27, 0.45) == pytest.approx(0.55894248755)


def test_envelope_contains_chassis_and_all_wheel_extents():
    envelope = geometry.wheel_inclusive_envelope_m(
        chassis_length_m=0.38,
        chassis_width_m=0.22,
        wheel_centers_xy_m=((0.135, 0.1), (0.135, -0.1), (-0.135, 0.1), (-0.135, -0.1)),
        wheel_radius_m=0.033,
        wheel_width_m=0.025,
    )
    assert envelope["length_m"] == pytest.approx(0.38)
    assert envelope["width_m"] == pytest.approx(0.225)
    assert envelope["footprint_radius_m"] == pytest.approx(0.220808, abs=1e-6)


def test_lane_change_uses_two_opposite_max_curvature_arcs():
    result = geometry.lane_change_minimum_lengths_m(0.5, 0.5)
    theta = math.acos(0.5)
    assert result["arc_angle_rad"] == pytest.approx(theta)
    assert result["longitudinal_length_m"] == pytest.approx(2 * 0.5 * math.sin(theta))
    assert result["path_length_m"] == pytest.approx(2 * 0.5 * theta)


def test_lane_change_rejects_impossible_lateral_shift():
    with pytest.raises(ValueError, match="twice"):
        geometry.lane_change_minimum_lengths_m(1.01, 0.5)


def test_layout_margin_is_a_separate_multiplicative_bound():
    assert geometry.apply_layout_margin(4.0, 0.25) == pytest.approx(5.0)
    with pytest.raises(ValueError):
        geometry.apply_layout_margin(-1.0, 0.25)
