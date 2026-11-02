"""Bound and expire planner drive targets before the safe-drive controller."""

from __future__ import annotations

import json
import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String


def bounded_drive_target(target: dict, *, max_speed_mps: float, max_steering_angle_rad: float, now_ns: int) -> tuple[float, float]:
    """Validate freshness/finite values and apply the profile's command bounds."""
    speed = float(target["speed_mps"])
    steering = float(target["steering_angle_rad"])
    valid_until = int(target["valid_until_ns"])
    if not all(math.isfinite(value) for value in (speed, steering, max_speed_mps, max_steering_angle_rad)):
        raise ValueError("drive target and bounds must be finite")
    if max_speed_mps <= 0 or max_steering_angle_rad <= 0:
        raise ValueError("drive target bounds must be positive")
    if valid_until < now_ns:
        return 0.0, 0.0
    return (
        max(-max_speed_mps, min(max_speed_mps, speed)),
        max(-max_steering_angle_rad, min(max_steering_angle_rad, steering)),
    )


class ControllerRequestAdapter(Node):
    def __init__(self) -> None:
        super().__init__("controller_request_adapter")
        for name, default in (("max_speed_mps", 1.0), ("max_steering_angle_rad", 0.45), ("input_topic", "/decision/target_drive")):
            self.declare_parameter(name, default)
        self.max_speed = float(self.get_parameter("max_speed_mps").value)
        self.max_steering = float(self.get_parameter("max_steering_angle_rad").value)
        self.latest = None
        self._publisher = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        self._subscription = self.create_subscription(String, str(self.get_parameter("input_topic").value), self._on_target, 10)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(0.05, self._publish, clock=self._steady_clock)

    def _on_target(self, message: String) -> None:
        try:
            target = json.loads(message.data)
            speed = float(target["speed_mps"])
            steering = float(target["steering_angle_rad"])
            expiry = int(target["valid_until_ns"])
            if not math.isfinite(speed) or not math.isfinite(steering):
                return
            self.latest = (speed, steering, expiry)
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self.latest = None

    def _publish(self) -> None:
        # Use this node's ROS clock directly. A separate /clock subscription
        # can lag behind the ROS time source under sensor/UI load, producing
        # Ackermann stamps that the safety controller correctly rejects.
        now_ns = self.get_clock().now().nanoseconds
        speed, steering = 0.0, 0.0
        if self.latest:
            speed, steering = bounded_drive_target(
                {"speed_mps": self.latest[0], "steering_angle_rad": self.latest[1], "valid_until_ns": self.latest[2]},
                max_speed_mps=self.max_speed,
                max_steering_angle_rad=self.max_steering,
                now_ns=now_ns,
            )
        command = AckermannDriveStamped()
        command.header.stamp.sec = now_ns // 1_000_000_000
        command.header.stamp.nanosec = now_ns % 1_000_000_000
        command.drive.speed = speed
        command.drive.steering_angle = steering
        self._publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerRequestAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
