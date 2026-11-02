"""Bound manual Twist input into the existing RC Ackermann request path."""

from __future__ import annotations

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node


def bounded_manual_request(
    speed_mps: float,
    yaw_rate_radps: float,
    *,
    wheelbase_m: float,
    max_speed_mps: float,
    max_steering_angle_rad: float,
    max_yaw_rate_radps: float,
) -> tuple[float, float]:
    """Convert a planar Twist request to bounded Ackermann speed and steering."""
    values = (
        speed_mps, yaw_rate_radps, wheelbase_m, max_speed_mps,
        max_steering_angle_rad, max_yaw_rate_radps,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("manual command and limits must be finite")
    if min(wheelbase_m, max_speed_mps, max_steering_angle_rad, max_yaw_rate_radps) <= 0.0:
        raise ValueError("wheelbase and manual command limits must be positive")
    speed = max(-max_speed_mps, min(max_speed_mps, float(speed_mps)))
    yaw_rate = max(-max_yaw_rate_radps, min(max_yaw_rate_radps, float(yaw_rate_radps)))
    if abs(speed) < 1e-6:
        steering = 0.0
    else:
        steering = math.atan(wheelbase_m * yaw_rate / speed)
    steering = max(-max_steering_angle_rad, min(max_steering_angle_rad, steering))
    return speed, steering


class ManualTwistAdapter(Node):
    def __init__(self) -> None:
        super().__init__("manual_twist_adapter")
        self.declare_parameter("wheelbase_m", 0.27)
        self.declare_parameter("max_speed_mps", 1.0)
        self.declare_parameter("max_steering_angle_rad", 0.45)
        self.declare_parameter("max_yaw_rate_radps", 1.0)
        self.declare_parameter("input_topic", "/manual/cmd_vel_intent")
        self.declare_parameter("output_topic", "/ackermann_cmd")
        self._limits = {
            "wheelbase_m": float(self.get_parameter("wheelbase_m").value),
            "max_speed_mps": float(self.get_parameter("max_speed_mps").value),
            "max_steering_angle_rad": float(self.get_parameter("max_steering_angle_rad").value),
            "max_yaw_rate_radps": float(self.get_parameter("max_yaw_rate_radps").value),
        }
        self._publisher = self.create_publisher(
            AckermannDriveStamped, str(self.get_parameter("output_topic").value), 10
        )
        self._subscription = self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self._on_twist, 10
        )

    def _on_twist(self, message: Twist) -> None:
        speed, steering = bounded_manual_request(
            float(message.linear.x), float(message.angular.z), **self._limits
        )
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.drive.speed = speed
        command.drive.steering_angle = steering
        self._publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualTwistAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
