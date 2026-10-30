"""Matrix fault relay that stops forwarding scans after a fixed sim interval."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class MatrixLidarRelay(Node):
    def __init__(self) -> None:
        super().__init__(
            "matrix_lidar_fault_relay",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.declare_parameter("drop_after_sim_s", 1.0)
        self.drop_after_ns = int(float(self.get_parameter("drop_after_sim_s").value) * 1e9)
        if self.drop_after_ns <= 0:
            raise ValueError("drop_after_sim_s must be positive")
        self.start_ns: int | None = None
        self.dropped = False
        self.received_count = 0
        self.forwarded_count = 0
        self.publisher = self.create_publisher(LaserScan, "/matrix/lidar_for_controller", qos_profile_sensor_data)
        self.state_publisher = self.create_publisher(String, "/matrix/lidar_fault_state", 10)
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)

    @staticmethod
    def _to_ns(message: Clock) -> int:
        return int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)

    def _on_clock(self, message: Clock) -> None:
        now = self._to_ns(message)
        if self.start_ns is None:
            self.start_ns = now
        if not self.dropped and now - self.start_ns >= self.drop_after_ns:
            self.dropped = True
            state = String()
            state.data = f"dropped;received={self.received_count};forwarded={self.forwarded_count}"
            self.state_publisher.publish(state)
            self.get_logger().info(state.data)

    def _on_scan(self, message: LaserScan) -> None:
        self.received_count += 1
        if self.start_ns is None:
            return
        now = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if now - self.start_ns < self.drop_after_ns:
            self.publisher.publish(message)
            self.forwarded_count += 1
            if self.forwarded_count == 1:
                self.get_logger().info(
                    "forwarded first scan "
                    f"stamp_sim_ns={now} start_sim_ns={self.start_ns}"
                )


def main(args=None) -> int:
    rclpy.init(args=args)
    node = MatrixLidarRelay()
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
