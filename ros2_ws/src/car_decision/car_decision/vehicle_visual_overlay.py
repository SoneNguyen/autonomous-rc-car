"""Place a collisionless research pickup visual at the RC vehicle pose."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from .odometry_evidence_node import pose_components


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def transform_pose(
    odometry_origin: tuple[float, float, float],
    odometry_current: tuple[float, float, float],
    world_origin: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Transform an odometry pose delta onto the configured world spawn pose."""
    ox, oy, oyaw = odometry_origin
    cx, cy, cyaw = odometry_current
    wx, wy, wyaw = world_origin
    return wx + cx - ox, wy + cy - oy, _wrap_angle(wyaw + cyaw - oyaw)


def predict_pose(
    pose: tuple[float, float, float],
    speed_mps: float,
    yaw_rate_rps: float,
    elapsed_s: float,
    *,
    maximum_prediction_s: float = 0.03,
) -> tuple[float, float, float]:
    """Extrapolate a fresh odometry pose by at most one rendering interval."""
    x, y, yaw = (float(value) for value in pose)
    dt = min(max(0.0, float(elapsed_s)), max(0.0, float(maximum_prediction_s)))
    speed = float(speed_mps)
    yaw_rate = float(yaw_rate_rps)
    midpoint_yaw = yaw + 0.5 * yaw_rate * dt
    return (
        x + speed * math.cos(midpoint_yaw) * dt,
        y + speed * math.sin(midpoint_yaw) * dt,
        _wrap_angle(yaw + yaw_rate * dt),
    )


class VehicleVisualOverlay(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_visual_overlay")
        self.declare_parameter("world_origin_z_m", 0.0675)
        self.declare_parameter("world_pose_topic", "/world/obstacle_course/pose/info")
        self.declare_parameter("pose_service", "/world/obstacle_course/set_pose")
        self._world_z = float(self.get_parameter("world_origin_z_m").value)
        self._truck_pose: tuple[float, float, float] | None = None
        self._last_overlay_pose: tuple[float, float, float] | None = None
        self._pose_in_flight = False
        self._client = self.create_client(
            SetEntityPose, str(self.get_parameter("pose_service").value)
        )
        self._subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("world_pose_topic").value),
            self._on_ground_truth_pose,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(1.0 / 30.0, self._update_overlay)

    def _on_ground_truth_pose(self, message: PoseStamped) -> None:
        self._truck_pose = pose_components(message.pose)

    def _update_overlay(self) -> None:
        if self._truck_pose is None:
            return
        if self._pose_in_flight or not self._client.service_is_ready():
            return
        x, y, yaw = self._truck_pose
        if self._last_overlay_pose is not None and all(
            abs(a - b) < 1e-4 for a, b in zip(self._last_overlay_pose, self._truck_pose)
        ):
            return
        request = SetEntityPose.Request()
        request.entity.name = "gazebo_truck_visual"
        request.entity.type = Entity.MODEL
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = self._world_z
        request.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.orientation.w = math.cos(yaw / 2.0)
        self._pose_in_flight = True
        self._last_overlay_pose = self._truck_pose
        future = self._client.call_async(request)
        future.add_done_callback(self._on_pose_response)

    def _on_pose_response(self, future) -> None:
        self._pose_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # service may be going away as Gazebo closes
            self.get_logger().debug(f"vehicle overlay pose update failed: {exc}")
            return
        if not response.success:
            self.get_logger().warning("Gazebo rejected gazebo_truck_visual pose update")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleVisualOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # A launch shutdown can close Gazebo's SetEntityPose bridge at the same
        # time as an in-flight response callback. Treat that teardown race as a
        # normal stop; runtime failures while ROS is active still propagate.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
