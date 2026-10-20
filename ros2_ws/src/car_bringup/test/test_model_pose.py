"""Scenario pose conversion tests."""

import math

import pytest

from car_bringup.set_model_pose import make_pose_request, quaternion_from_rpy


def test_yaw_is_converted_to_a_unit_quaternion():
    q = quaternion_from_rpy(0.0, 0.0, math.pi / 2)
    assert q == pytest.approx((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))
    request = make_pose_request(x=-10, y=2, z=1.96, roll=0, pitch=0, yaw=math.pi / 2)
    assert 'name: "gazebo_truck"' in request
    assert "orientation {" in request


def test_pose_rejects_non_finite_values():
    with pytest.raises(ValueError, match="finite"):
        make_pose_request(x=math.nan, y=0, z=0, roll=0, pitch=0, yaw=0)
