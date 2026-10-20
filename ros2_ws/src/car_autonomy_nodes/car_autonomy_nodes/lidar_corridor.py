"""Decode the full multi-layer cloud and find forward-corridor obstacles."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct


EXPECTED_FRAME = "gazebo_truck/chassis/front_lidar"
SENSOR_X_IN_BASE_M = 7.2
SENSOR_Y_IN_BASE_M = 0.0
SENSOR_Z_IN_BASE_M = 3.5
FRONT_COLLISION_PLANE_X_M = 5.25


@dataclass(frozen=True)
class CorridorResult:
    """Validated nearest obstacle return in the vehicle's forward corridor."""

    valid: bool
    nearest_distance_m: float | None
    points_seen: int
    points_in_corridor: int
    reason: str = "ok"


def _field_offsets(message) -> dict[str, tuple[int, int]]:
    fields = {field.name: (field.offset, field.datatype) for field in message.fields}
    required = {"x", "y", "z"}
    if not required.issubset(fields):
        raise ValueError("PointCloud2 must contain x, y, and z fields")
    float32 = 7  # sensor_msgs/PointField.FLOAT32
    if any(fields[name][1] != float32 for name in required):
        raise ValueError("PointCloud2 x/y/z fields must be FLOAT32")
    if any(fields[name][0] < 0 or fields[name][0] + 4 > message.point_step for name in required):
        raise ValueError("PointCloud2 x/y/z offsets exceed point_step")
    return fields


def nearest_forward_obstacle(
    message,
    *,
    corridor_half_width_m: float,
    ground_z_min_m: float,
) -> CorridorResult:
    """Return nearest obstacle distance from chassis front plane in metres.

    The bridge publishes the SDF sensor frame ``gazebo_truck/chassis/front_lidar``.
    The verified static transform to base_link is +7.2 m x, 0 m y, +3.5 m z.
    The current sensor and base frames are parallel, so no rotation is applied.
    """
    if not math.isfinite(corridor_half_width_m) or corridor_half_width_m <= 0:
        return CorridorResult(False, None, 0, 0, "invalid_corridor_width")
    if not math.isfinite(ground_z_min_m):
        return CorridorResult(False, None, 0, 0, "invalid_ground_filter")
    if message.header.frame_id != EXPECTED_FRAME:
        return CorridorResult(False, None, 0, 0, "unverified_frame")
    if message.width <= 0 or message.height <= 0 or message.point_step <= 0:
        return CorridorResult(False, None, 0, 0, "empty_cloud")
    if message.row_step < message.width * message.point_step:
        return CorridorResult(False, None, 0, 0, "invalid_row_step")
    if len(message.data) < message.row_step * message.height:
        return CorridorResult(False, None, 0, 0, "truncated_cloud")
    try:
        fields = _field_offsets(message)
    except ValueError as exc:
        return CorridorResult(False, None, 0, 0, str(exc))

    endian = ">" if message.is_bigendian else "<"
    offsets = [fields[name][0] for name in ("x", "y", "z")]
    nearest: float | None = None
    seen = 0
    in_corridor = 0
    data = memoryview(message.data)
    for row in range(message.height):
        row_start = row * message.row_step
        for column in range(message.width):
            point_start = row_start + column * message.point_step
            x, y, z = (
                struct.unpack_from(endian + "f", data, point_start + offset)[0]
                for offset in offsets
            )
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            seen += 1
            base_x = x + SENSOR_X_IN_BASE_M
            base_y = y + SENSOR_Y_IN_BASE_M
            base_z = z + SENSOR_Z_IN_BASE_M
            if base_z < ground_z_min_m or abs(base_y) > corridor_half_width_m:
                continue
            distance = base_x - FRONT_COLLISION_PLANE_X_M
            if distance < 0:
                continue
            in_corridor += 1
            nearest = distance if nearest is None else min(nearest, distance)

    if seen == 0:
        return CorridorResult(False, None, 0, 0, "no_finite_points")
    return CorridorResult(True, nearest, seen, in_corridor)


def should_stop(result: CorridorResult, stop_distance_m: float) -> bool:
    """Fail safe on invalid data; otherwise stop at or inside the trigger range."""
    if not math.isfinite(stop_distance_m) or stop_distance_m <= 0:
        return True
    return not result.valid or (
        result.nearest_distance_m is not None
        and result.nearest_distance_m <= stop_distance_m + 1e-6
    )


def lidar_requested_speed(
    result: CorridorResult, *, stop_distance_m: float, commanded_speed_mps: float
) -> float:
    """Return the baseline's forward speed; obstacle/invalid data always wins."""
    if not math.isfinite(commanded_speed_mps):
        return 0.0
    if should_stop(result, stop_distance_m):
        return 0.0
    return max(0.0, min(2.0, commanded_speed_mps))
