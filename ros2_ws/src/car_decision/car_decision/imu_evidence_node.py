"""Normalize simulation IMU yaw rate and vehicle-frame acceleration."""

from __future__ import annotations

from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

from .evidence_common import message_stamp_ns
from .evidence_node_base import EvidenceNode, spin_node


class ImuEvidenceNode(EvidenceNode):
    def __init__(self) -> None:
        super().__init__("imu", "/imu/data", "sensor_msgs/msg/Imu")
        self._subscription = self.create_subscription(
            Imu, "/imu/data", self._on_imu, qos_profile_sensor_data
        )

    def _empty_payload(self):
        return {
            "yaw_rate_rad_s": 0.0,
            "lateral_acceleration_mps2": 0.0,
            "longitudinal_acceleration_mps2": 0.0,
        }

    def _on_imu(self, message: Imu) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.is_faulted(now_ns):
            return
        self.record_sample(
            {
                "yaw_rate_rad_s": float(message.angular_velocity.z),
                "lateral_acceleration_mps2": float(message.linear_acceleration.y),
                "longitudinal_acceleration_mps2": float(message.linear_acceleration.x),
            },
            message_stamp_ns(message.header, now_ns),
            message.header.frame_id,
        )


def main(args=None) -> None:
    spin_node(ImuEvidenceNode, args)
