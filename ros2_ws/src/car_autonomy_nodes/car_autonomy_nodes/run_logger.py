#!/usr/bin/env python3
"""Passive simulation-time run logger for lidar stopping trials."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import uuid

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu, PointCloud2

from car_autonomy_nodes.lidar_corridor import nearest_forward_obstacle


COMMAND_TOPIC = "/model/gazebo_truck/cmd_vel"
CLOUD_TOPIC = "/model/gazebo_truck/sensors/front_lidar/scan/points"
ODOMETRY_TOPIC = "/model/gazebo_truck/odometry"
IMU_TOPIC = "/model/gazebo_truck/sensors/imu"
CONTACT_TOPIC = "/model/gazebo_truck/sensors/bumper_contact"
CLOCK_TOPIC = "/clock"
REST_SPEED_MPS = 0.05
REST_SETTLE_NS = 1_000_000_000
SENSOR_MAX_AGE_NS = 500_000_000
VERIFIED_CONTACTS = {
    "post_01_collision",
    "gate_left_collision",
    "gate_right_collision",
}
COURSE_OBSTACLE_COLLISIONS = VERIFIED_CONTACTS | {
    "barrier_right_collision",
    "barrier_left_collision",
}
CSV_FIELDS = [
    "run_id", "scenario_id", "scenario_sha256", "split", "mode", "seed",
    "lighting_id", "condition_tags", "start_pose", "obstacle_entity",
    "obstacle_collision", "commanded_speed_mps", "detected", "detection_source",
    "detection_stamp_ns", "stop_request_stamp_ns", "stopped_before_contact",
    "stopping_distance_m", "trial_status", "completion_reason", "collision_status",
    "collision_coverage", "collision_entity", "collision_stamp_ns",
    "ground_contact_excluded", "sensor_stale_events", "sim_duration_s",
    "package_versions",
]
CLOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


def contact_names(message: Contacts) -> list[tuple[str, str]]:
    """Extract collision pairs from ros_gz_interfaces/Contacts."""
    return [
        (contact.collision1.name, contact.collision2.name)
        for contact in message.contacts
    ]


def is_ground_contact(names: tuple[str, str]) -> bool:
    return any("ground" in name.lower() or "floor" in name.lower() for name in names)


def allowlisted_obstacle_contact(
    names: tuple[str, str], expected_collision: str
) -> bool:
    if is_ground_contact(names):
        return False
    return any(expected_collision in name for name in names)


def collision_status(*, observed: bool, coverage: str) -> str:
    if observed:
        return "collision"
    return "no_collision" if coverage == "verified" else "unknown"


def completion_status(reason: str) -> tuple[str, str]:
    """Normalize session-ending causes using the shared status vocabulary."""
    if reason == "rest_and_flush":
        return "completed", reason
    if reason == "simulation_time_limit":
        return "timed_out", reason
    return "aborted", reason


def new_run_id() -> str:
    """Return a fresh identifier for each logger process/run attempt."""
    return str(uuid.uuid4())


def _pose_tuple(pose: dict) -> str:
    return json.dumps(pose, sort_keys=True, separators=(",", ":"))


class RunLogger(Node):
    """Observe a single run; this node never publishes drive commands."""

    def __init__(self) -> None:
        super().__init__(
            "run_logger",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("mode", "lidar")
        self.declare_parameter("output_csv", "")
        self.declare_parameter("repository_root", "")
        self.declare_parameter("runtime_lighting_id", "obstacle_world_default")
        self.declare_parameter(
            "package_versions", "ros=lyrical;gazebo_sim=10.5.0;ros_gz=3.0.10"
        )
        scenario_path = Path(str(self.get_parameter("scenario_path").value)).resolve()
        self.scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
        self.scenario_hash = hashlib.sha256(scenario_path.read_bytes()).hexdigest()
        self.run_id = new_run_id()
        self.mode = str(self.get_parameter("mode").value)
        self.runtime_lighting = str(self.get_parameter("runtime_lighting_id").value)
        self.package_versions = str(self.get_parameter("package_versions").value)
        repository_root = Path(str(self.get_parameter("repository_root").value)).resolve()
        output_value = str(self.get_parameter("output_csv").value)
        if output_value:
            output_path = Path(output_value)
            self.output_path = output_path if output_path.is_absolute() else repository_root / output_path
        else:
            self.output_path = repository_root / "artifacts" / "runs" / f"{self.run_id}.csv"

        self.start_sim_ns: int | None = None
        self.last_clock_ns: int | None = None
        self.latest_sensor_stamps: dict[str, int] = {}
        self.latest_detection = False
        self.detection_stamp_ns: int | None = None
        self.stop_request_stamp_ns: int | None = None
        self.last_command_nonzero = False
        self.previous_pose: tuple[float, float] | None = None
        self.previous_odom_stamp_ns: int | None = None
        self.speed_mps = math.inf
        self.travel_after_stop_m = 0.0
        self.rest_since_ns: int | None = None
        self.collision_entity = ""
        self.collision_stamp_ns: int | None = None
        self.ground_contact_excluded = False
        self.stale_events: set[str] = set()
        self.finished = False
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.started_wall_ns = self._steady_clock.now().nanoseconds

        scenario_collision = self.scenario["obstacle"]["collision"]
        self.coverage = "verified" if scenario_collision in VERIFIED_CONTACTS else "limited"
        self.coverage_stale = False
        self.create_subscription(ClockMessage, CLOCK_TOPIC, self._on_clock, CLOCK_QOS)
        self.create_subscription(
            PointCloud2, CLOUD_TOPIC, self._on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, ODOMETRY_TOPIC, self._on_odometry, qos_profile_sensor_data
        )
        self.create_subscription(Imu, IMU_TOPIC, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Contacts, CONTACT_TOPIC, self._on_contacts, qos_profile_sensor_data
        )
        self.create_subscription(Twist, COMMAND_TOPIC, self._on_command, 10)
        self.create_timer(0.1, self._check_freshness, clock=self._steady_clock)

        if self.runtime_lighting != self.scenario["lighting_id"]:
            self._finish("lighting_mismatch")

    @staticmethod
    def _stamp(header) -> int:
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _on_clock(self, message: ClockMessage) -> None:
        now = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        if self.start_sim_ns is None:
            self.start_sim_ns = now
        elif self.last_clock_ns is not None and now < self.last_clock_ns:
            self._finish("simulation_reset")
            return
        self.last_clock_ns = now
        elapsed_s = (now - self.start_sim_ns) / 1e9
        if elapsed_s >= float(self.scenario["run_timeout_sim_s"]):
            self._finish("simulation_time_limit")
            return
        if (
            self.stop_request_stamp_ns is not None
            and self.speed_mps <= REST_SPEED_MPS
        ):
            if self.rest_since_ns is None:
                self.rest_since_ns = now
            elif now - self.rest_since_ns >= REST_SETTLE_NS:
                self._finish("rest_and_flush")
        else:
            self.rest_since_ns = None

    def _on_cloud(self, message: PointCloud2) -> None:
        stamp = self._stamp(message.header)
        self.latest_sensor_stamps["lidar"] = stamp
        result = nearest_forward_obstacle(
            message,
            corridor_half_width_m=float(self.scenario["corridor_half_width_m"]),
            ground_z_min_m=float(self.scenario["ground_z_min_m"]),
        )
        if not result.valid:
            self.stale_events.add(f"invalid_lidar:{result.reason}")
            return
        if (
            result.nearest_distance_m is not None
            and result.nearest_distance_m <= float(self.scenario["stop_distance_m"]) + 1e-6
        ):
            self.latest_detection = True
            if self.detection_stamp_ns is None:
                self.detection_stamp_ns = stamp

    def _on_imu(self, message: Imu) -> None:
        self.latest_sensor_stamps["imu"] = self._stamp(message.header)

    def _on_command(self, message: Twist) -> None:
        now = self._now_ns()
        nonzero = abs(message.linear.x) > 1e-6 or abs(message.angular.z) > 1e-6
        if nonzero:
            self.last_command_nonzero = True
        elif self.stop_request_stamp_ns is None and (
            self.last_command_nonzero or self.latest_detection
        ):
            self.stop_request_stamp_ns = now

    def _on_odometry(self, message: Odometry) -> None:
        stamp = self._stamp(message.header)
        self.latest_sensor_stamps["odometry"] = stamp
        position = message.pose.pose.position
        pose = (float(position.x), float(position.y))
        if self.previous_pose is not None and self.stop_request_stamp_ns is not None:
            dx = pose[0] - self.previous_pose[0]
            dy = pose[1] - self.previous_pose[1]
            self.travel_after_stop_m += math.hypot(dx, dy)
        self.previous_pose = pose
        self.previous_odom_stamp_ns = stamp
        twist = message.twist.twist.linear
        self.speed_mps = math.sqrt(twist.x * twist.x + twist.y * twist.y)

    def _on_contacts(self, message: Contacts) -> None:
        stamp = self._now_ns()
        expected = self.scenario["obstacle"]["collision"]
        for names in contact_names(message):
            if is_ground_contact(names):
                self.ground_contact_excluded = True
            elif any(name in " ".join(names) for name in COURSE_OBSTACLE_COLLISIONS):
                self.collision_entity = " | ".join(names)
                self.collision_stamp_ns = stamp
                if allowlisted_obstacle_contact(names, expected):
                    self.coverage = "verified" if expected in VERIFIED_CONTACTS else "limited"

    def _check_freshness(self) -> None:
        if self.finished or self.last_clock_ns is None:
            return
        now = self._now_ns()
        for name, stamp in self.latest_sensor_stamps.items():
            if stamp <= 0 or stamp > now or now - stamp > SENSOR_MAX_AGE_NS:
                self.stale_events.add(f"{name}_stale")

    def _record(self, status: str, reason: str) -> dict[str, str]:
        collision_seen = self.collision_stamp_ns is not None
        collision_cov = "stale" if self.coverage_stale else self.coverage
        row = {
            "run_id": self.run_id,
            "scenario_id": self.scenario["scenario_id"],
            "scenario_sha256": self.scenario_hash,
            "split": self.scenario["split"],
            "mode": self.mode,
            "seed": str(self.scenario["seed"]),
            "lighting_id": self.scenario["lighting_id"],
            "condition_tags": json.dumps(self.scenario["unseen_condition_tags"]),
            "start_pose": _pose_tuple(self.scenario["start_pose"]),
            "obstacle_entity": self.scenario["obstacle"]["entity"],
            "obstacle_collision": self.scenario["obstacle"]["collision"],
            "commanded_speed_mps": str(self.scenario["commanded_speed_mps"]),
            "detected": str(self.latest_detection).lower(),
            "detection_source": "lidar" if self.latest_detection else "none",
            "detection_stamp_ns": str(self.detection_stamp_ns or ""),
            "stop_request_stamp_ns": str(self.stop_request_stamp_ns or ""),
            "stopped_before_contact": str(
                self.latest_detection
                and self.stop_request_stamp_ns is not None
                and (not collision_seen or self.stop_request_stamp_ns < (self.collision_stamp_ns or 0))
            ).lower(),
            "stopping_distance_m": f"{self.travel_after_stop_m:.6f}" if self.stop_request_stamp_ns else "",
            "trial_status": status,
            "completion_reason": reason,
            "collision_status": collision_status(observed=collision_seen, coverage=collision_cov),
            "collision_coverage": collision_cov,
            "collision_entity": self.collision_entity,
            "collision_stamp_ns": str(self.collision_stamp_ns or ""),
            "ground_contact_excluded": str(self.ground_contact_excluded).lower(),
            "sensor_stale_events": json.dumps(sorted(self.stale_events)),
            "sim_duration_s": f"{max(0, (self.last_clock_ns or 0) - (self.start_sim_ns or 0)) / 1e9:.6f}",
            "package_versions": self.package_versions,
        }
        return row

    def _finish(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        status, normalized_reason = completion_status(reason)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        needs_header = not self.output_path.exists() or self.output_path.stat().st_size == 0
        with self.output_path.open("a", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            if needs_header:
                writer.writeheader()
            writer.writerow(self._record(status, normalized_reason))
            stream.flush()
            os.fsync(stream.fileno())
        self.get_logger().info(
            f"run {self.run_id} persisted as {status} ({normalized_reason}) to {self.output_path}"
        )
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> int:
    rclpy.init(args=args)
    logger = RunLogger()
    try:
        if not logger.finished:
            rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        if not logger.finished:
            logger._finish("shutdown")
        logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
