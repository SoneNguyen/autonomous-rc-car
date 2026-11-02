"""Normalize Gazebo truck pose with wheel-odometry speed and drift evidence."""

from __future__ import annotations

import math
from typing import Any

from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseStamped

from .evidence_common import message_stamp_ns, route_progress, value_of
from .evidence_node_base import EvidenceNode, spin_node


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pose_components(pose: Pose) -> tuple[float, float, float]:
    """Extract planar x, y, and yaw from one Gazebo-published truck pose."""
    return (
        float(pose.position.x),
        float(pose.position.y),
        yaw_from_quaternion(pose.orientation),
    )


def relative_pose(
    origin: tuple[float, float, float], current: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Express a world pose in the fixed vehicle-relative scenario frame."""
    ox, oy, oyaw = origin
    x, y, yaw = current
    dx, dy = x - ox, y - oy
    c, s = math.cos(oyaw), math.sin(oyaw)
    return (
        c * dx + s * dy,
        -s * dx + c * dy,
        math.atan2(math.sin(yaw - oyaw), math.cos(yaw - oyaw)),
    )


class OdometryEvidenceNode(EvidenceNode):
    """Use the named Gazebo model pose for navigation and wheel odometry for speed.

    The Ackermann odometry plugin's integrated pose has diverged from the
    collision model in the WSL simulation. The world-specific PosePublisher
    pose is therefore the control pose; raw wheel odometry remains visible as a
    measured pose-error field and continues to supply speed.
    """

    def __init__(self) -> None:
        super().__init__("odometry", "/vehicle/ground_truth_pose + /odom", "geometry_msgs/msg/PoseStamped + nav_msgs/msg/Odometry")
        self._source_topic = "/vehicle/ground_truth_pose + /odom"
        self._message_type = "geometry_msgs/msg/PoseStamped + nav_msgs/msg/Odometry"
        self._origin = tuple(
            float(value_of(self.scenario["start_pose"][key])) for key in ("x", "y", "yaw")
        )
        self._route = self.scenario["route"]
        self._length = float(value_of(self._route["length_m"]))
        vehicle = self.profile["vehicle_profile"]
        self._braking = float(value_of(vehicle["dynamics"]["braking_deceleration_mps2"]))
        self._latency = float(value_of(vehicle["dynamics"]["request_to_deceleration_latency_s"]))
        if self._braking <= 0:
            raise ValueError("measured simulation braking deceleration must be positive")
        self._wheel_sample: dict[str, Any] | None = None
        self._truck_pose: dict[str, Any] | None = None
        self._wheel_origin: tuple[float, float, float] | None = None
        self._wheel_relative_origin: tuple[float, float, float] | None = None
        self._odom_subscription = self.create_subscription(
            Odometry, "/odom", self._on_odometry, qos_profile_sensor_data
        )
        self._pose_subscription = self.create_subscription(
            PoseStamped, "/vehicle/ground_truth_pose", self._on_ground_truth_pose,
            qos_profile_sensor_data,
        )

    def _empty_payload(self):
        return {
            "world_pose": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            "start_relative_pose": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            "speed_mps": 0.0,
            "route_progress_m": 0.0,
            "d_stop_m": 0.0,
            "profile_id": str(self.profile["vehicle_profile"]["profile_id"]),
            "pose_source": "gazebo_named_model_pose",
            "speed_source": "ackermann_odometry_twist",
            "wheel_odometry_pose_error_m": 0.0,
        }

    def _on_odometry(self, message: Odometry) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.is_faulted(now_ns):
            return
        pose = message.pose.pose
        wheel_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        if self._wheel_origin is None:
            self._wheel_origin = wheel_pose
        self._wheel_sample = {
            "pose": wheel_pose,
            "speed_mps": math.hypot(
                float(message.twist.twist.linear.x), float(message.twist.twist.linear.y)
            ),
            "stamp_ns": message_stamp_ns(message.header, now_ns),
            "frame_id": message.header.frame_id,
        }
        self._record_combined_sample()

    def _on_ground_truth_pose(self, message: PoseStamped) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.is_faulted(now_ns):
            return
        self._truck_pose = {
            "pose": pose_components(message.pose),
            "stamp_ns": message_stamp_ns(message.header, now_ns),
            "frame_id": message.header.frame_id,
        }
        self._record_combined_sample()

    def _record_combined_sample(self) -> None:
        if self._wheel_sample is None or self._truck_pose is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        wheel = self._wheel_sample
        truck = self._truck_pose
        # Do not publish a fresh combined state if either source stopped.
        if any(
            now_ns - int(sample["stamp_ns"]) > self._timeout_s * 1_000_000_000
            for sample in (wheel, truck)
        ):
            return

        x, y, yaw = truck["pose"]
        relative_x, relative_y, relative_yaw = relative_pose(self._origin, truck["pose"])
        wheel_relative = relative_pose(self._wheel_origin, wheel["pose"])
        if self._wheel_relative_origin is None:
            self._wheel_relative_origin = wheel_relative
        # The diagnostic is relative to the first wheel sample, so the report
        # shows accumulated drift rather than an unknown initial frame offset.
        wheel_dx = wheel_relative[0] - self._wheel_relative_origin[0]
        wheel_dy = wheel_relative[1] - self._wheel_relative_origin[1]
        wheel_error = math.hypot(wheel_dx - relative_x, wheel_dy - relative_y)

        speed = float(wheel["speed_mps"])
        stopping_distance = speed * speed / (2.0 * self._braking) + speed * self._latency
        progress = min(self._length, route_progress(self._route, relative_x, relative_y))
        stamp_ns = min(int(wheel["stamp_ns"]), int(truck["stamp_ns"]))
        self.record_sample(
            {
                "world_pose": {"x_m": x, "y_m": y, "yaw_rad": yaw},
                "start_relative_pose": {
                    "x_m": relative_x,
                    "y_m": relative_y,
                    "yaw_rad": relative_yaw,
                },
                "speed_mps": speed,
                "route_progress_m": progress,
                "d_stop_m": stopping_distance,
                "profile_id": str(self.profile["vehicle_profile"]["profile_id"]),
                "pose_source": "gazebo_named_model_pose",
                "speed_source": "ackermann_odometry_twist",
                "wheel_odometry_pose_error_m": wheel_error,
            },
            stamp_ns,
            truck["frame_id"] or wheel["frame_id"],
        )


def main(args=None) -> None:
    spin_node(OdometryEvidenceNode, args)
