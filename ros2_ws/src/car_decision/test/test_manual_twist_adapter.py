"""Tests for bounded manual input forwarded through the safe controller."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
from car_decision.manual_twist_adapter import bounded_manual_request  # noqa: E402


def test_manual_input_is_clamped_to_rc_profile():
    speed, steering = bounded_manual_request(
        4.0, -1.0, wheelbase_m=0.27, max_speed_mps=0.45,
        max_steering_angle_rad=0.2, max_yaw_rate_radps=0.6,
    )
    assert speed == 0.45
    assert steering == -0.2


def test_reverse_and_turn_direction_are_preserved():
    speed, steering = bounded_manual_request(
        -0.2, 0.3, wheelbase_m=0.27, max_speed_mps=0.45,
        max_steering_angle_rad=0.5, max_yaw_rate_radps=0.6,
    )
    assert speed == -0.2
    assert steering < 0.0


def test_nonfinite_manual_commands_are_rejected():
    with pytest.raises(ValueError, match="finite"):
        bounded_manual_request(
            math.nan, 0.0, wheelbase_m=0.27, max_speed_mps=0.45,
            max_steering_angle_rad=0.5, max_yaw_rate_radps=0.6,
        )


def test_manual_node_publishes_only_a_bounded_controller_request():
    source = (PACKAGE_ROOT / "car_decision/manual_twist_adapter.py").read_text(encoding="utf-8")
    assert "/manual/cmd_vel_intent" in source
    assert "/ackermann_cmd" in source
    assert "/model/gazebo_truck/cmd_vel" not in source
