"""Tests for the collisionless, spawn-relative Obstacle course vehicle visual."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
from car_decision.vehicle_visual_overlay import predict_pose, transform_pose  # noqa: E402


def test_overlay_pose_uses_nonzero_odometry_and_world_origins():
    result = transform_pose(
        odometry_origin=(11.0, -4.0, 0.4),
        odometry_current=(12.0, -3.0, math.pi / 2),
        world_origin=(7.0, 9.0, -0.2),
    )
    assert result[0] == 8.0
    assert result[1] == 10.0
    assert result[2] == math.pi / 2 - 0.6


def test_overlay_pose_has_no_jump_at_origin_capture():
    origin = (3.2, -1.1, 0.25)
    assert transform_pose(origin, origin, (6.0, 8.0, -0.4)) == pytest.approx((6.0, 8.0, -0.4))


def test_overlay_extrapolates_one_display_frame_without_pose_hop():
    result = predict_pose((1.0, 2.0, 0.0), 0.4, 0.5, 0.02)
    assert result == pytest.approx((1.0079998, 2.00004, 0.01), abs=1e-5)


def test_overlay_prediction_is_bounded_to_fresh_sample_window():
    current = predict_pose((0.0, 0.0, 0.0), 1.0, 0.0, 1.0)
    assert current == pytest.approx((0.03, 0.0, 0.0))


def test_overlay_node_uses_pose_service_and_never_adds_collision():
    source = (PACKAGE_ROOT / "car_decision/vehicle_visual_overlay.py").read_text(encoding="utf-8")
    assert "SetEntityPose" in source
    assert "gazebo_truck_visual" in source
    assert "PoseStamped" in source
    assert "pose_components(message.pose)" in source
    assert "world_origin_x_m" not in source
    world = (PACKAGE_ROOT.parents[2] / "simulation/worlds/obstacle_condensed.sdf")
    if world.exists():
        import xml.etree.ElementTree as ET
        root = ET.parse(world).getroot()
        visual = root.find(".//world/model[@name='gazebo_truck_visual']")
        assert visual is not None and visual.find("./link/collision") is None
