#!/usr/bin/env python3
"""Publish the configured RC sensor frames relative to base_link."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def transforms_from_profile(profile: dict) -> list[dict]:
    rc = profile["presets"]["rc_car"]
    base_frame = str(rc["frames"]["base"]["value"])
    result = []
    for sensor_name in ("lidar", "camera", "imu"):
        sensor = rc["sensors"][sensor_name]
        x, y, z, roll, pitch, yaw = sensor["pose_base_link_m_rad"]["value"]
        result.append(
            {
                "parent": base_frame,
                "child": str(sensor["frame"]),
                "translation": (float(x), float(y), float(z)),
                "rotation": quaternion_from_rpy(float(roll), float(pitch), float(yaw)),
            }
        )
    return result


class SensorTfNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "sensor_tf_node",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.declare_parameter("profile_path", "")
        profile_path = Path(str(self.get_parameter("profile_path").value))
        if not profile_path.is_file():
            raise FileNotFoundError(f"RC vehicle profile not found: {profile_path}")
        profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        self.broadcaster = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for item in transforms_from_profile(profile):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = item["parent"]
            transform.child_frame_id = item["child"]
            transform.transform.translation.x = item["translation"][0]
            transform.transform.translation.y = item["translation"][1]
            transform.transform.translation.z = item["translation"][2]
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ) = item["rotation"]
            transforms.append(transform)
        self.broadcaster.sendTransform(transforms)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = SensorTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
