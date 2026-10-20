"""Point-cloud geometry and safety tests for the lidar stop baseline."""

import math
import struct

import pytest

from sensor_msgs.msg import PointCloud2, PointField

from car_autonomy_nodes.lidar_corridor import (
    EXPECTED_FRAME,
    lidar_requested_speed,
    nearest_forward_obstacle,
    should_stop,
)


def _cloud(points, *, width=None, frame=EXPECTED_FRAME, bigendian=False):
    width = width or len(points)
    height = math.ceil(len(points) / width)
    padded = list(points) + [(math.nan, math.nan, math.nan)] * (width * height - len(points))
    message = PointCloud2()
    message.header.frame_id = frame
    message.height = height
    message.width = width
    message.fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z"))
    ]
    message.is_bigendian = bigendian
    message.point_step = 12
    message.row_step = width * message.point_step
    message.is_dense = False
    endian = ">" if bigendian else "<"
    message.data = b"".join(struct.pack(endian + "fff", *point) for point in padded)
    return message


def _result(points, **kwargs):
    return nearest_forward_obstacle(
        _cloud(points, **kwargs), corridor_half_width_m=5.5, ground_z_min_m=0.2
    )


def test_uses_returns_from_all_vertical_layers_and_measures_from_front_plane():
    # Sensor-frame x=8.05 becomes base_link x=15.25; the front plane is x=5.25.
    # The closest point is deliberately in the last of 16 vertical rows.
    points = [(30.0, 0.0, -2.0)] * 15 + [(8.05, 0.0, -3.0)]
    result = _result(points, width=1)
    assert result.valid
    assert result.points_seen == 16
    assert result.points_in_corridor == 16
    assert result.nearest_distance_m == pytest.approx(10.0)
    assert not should_stop(result, 5.0)
    assert should_stop(result, 10.0)


def test_rejects_ground_and_points_outside_the_forward_corridor():
    result = _result([
        (8.0, 0.0, -3.4),   # base z=0.1 m: ground reject
        (8.0, 8.0, -3.0),   # outside lateral corridor
        (-10.0, 0.0, -3.0), # behind the front collision plane
    ])
    assert result.valid
    assert result.points_seen == 3
    assert result.points_in_corridor == 0
    assert result.nearest_distance_m is None
    assert not should_stop(result, 5.0)


def test_invalid_frame_or_empty_nonfinite_cloud_fails_safe():
    wrong_frame = _result([(10.0, 0.0, 0.0)], frame="unverified_frame")
    no_finite_points = _result([(math.nan, math.inf, 0.0)])
    assert not wrong_frame.valid
    assert wrong_frame.reason == "unverified_frame"
    assert should_stop(wrong_frame, 5.0)
    assert not no_finite_points.valid
    assert no_finite_points.reason == "no_finite_points"
    assert should_stop(no_finite_points, 5.0)


def test_big_endian_packed_cloud_and_bad_trigger_are_handled():
    result = _result([(9.25, 0.0, -3.0)], bigendian=True)
    assert result.valid
    assert result.nearest_distance_m == pytest.approx(11.2)
    assert should_stop(result, 0.0)


def test_malformed_pointcloud_layout_is_invalid():
    message = _cloud([(1.0, 0.0, 0.0)])
    message.row_step = 1
    result = nearest_forward_obstacle(
        message, corridor_half_width_m=5.5, ground_z_min_m=0.2
    )
    assert not result.valid
    assert result.reason == "invalid_row_step"


def test_lidar_stop_dominates_forward_request_and_invalid_cloud_stops():
    clear = _result([(20.0, 0.0, -3.0)])
    close = _result([(3.0, 0.0, -3.0)])
    invalid = _result([(math.nan, 0.0, 0.0)])
    assert lidar_requested_speed(
        clear, stop_distance_m=5.0, commanded_speed_mps=1.5
    ) == 1.5
    assert lidar_requested_speed(
        close, stop_distance_m=5.0, commanded_speed_mps=1.5
    ) == 0.0
    assert lidar_requested_speed(
        invalid, stop_distance_m=5.0, commanded_speed_mps=1.5
    ) == 0.0
