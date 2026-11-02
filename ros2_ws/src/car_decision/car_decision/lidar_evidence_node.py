"""Normalize the active RC LaserScan into five sectors and a route corridor."""

from __future__ import annotations

from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

from .evidence_common import lidar_payload, message_stamp_ns, value_of
from .evidence_node_base import EvidenceNode, spin_node


class LidarEvidenceNode(EvidenceNode):
    def __init__(self) -> None:
        super().__init__("lidar", "/scan", "sensor_msgs/msg/LaserScan")
        rc = self.sensor_profile["presets"]["rc_car"]
        lidar = rc["sensors"]["lidar"]
        navigation = rc["navigation"]
        geometry = self.profile["vehicle_geometry"]["vehicle"]
        decision_geometry = self.decision["geometry"]
        self._pose = value_of(lidar["pose_base_link_m_rad"])
        self._boundaries = value_of(navigation["sector_boundaries_rad"])
        self._corridor_half_width = (
            float(value_of(geometry["width_m"]))
            * float(value_of(decision_geometry["route_corridor_half_width_multiple_W"]))
        )
        self._minimum_range = float(value_of(lidar["range_min_m"]))
        self._maximum_range = float(value_of(lidar["range_max_m"]))
        self._self_filter = float(value_of(navigation["lidar_planner"]["self_filter_radius_m"]))
        self._subscription = self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )

    def _empty_payload(self):
        return {
            "input_adapter": "laser_scan",
            "sectors": {name: None for name in ("right_m", "front_right_m", "front_m", "front_left_m", "left_m")},
            "route_corridor": {"occupied": False, "nearest_range_m": None},
            "points_base_link": [],
            "ground_points_rejected": 0,
            "invalid_returns_rejected": 0,
            "usable_points": 0,
        }

    def _on_scan(self, message: LaserScan) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.is_faulted(now_ns):
            return
        payload = lidar_payload(
            message.ranges,
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            range_min=max(message.range_min, self._minimum_range),
            range_max=min(message.range_max, self._maximum_range),
            lidar_pose=self._pose,
            sector_boundaries=self._boundaries,
            corridor_half_width_m=self._corridor_half_width,
            self_filter_radius_m=self._self_filter,
        )
        stamp_ns = message_stamp_ns(message.header, now_ns)
        self.record_sample(payload, stamp_ns, message.header.frame_id)


def main(args=None) -> None:
    spin_node(LidarEvidenceNode, args)
