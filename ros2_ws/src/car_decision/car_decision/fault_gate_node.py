"""Apply only the tracked S5/S6 diagnostic faults; never publishes drive commands."""

from __future__ import annotations

import json
import math
import uuid

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import String
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from .evidence_common import encode_record, load_yaml, sha256_file, value_of


class FaultGateNode(Node):
    def __init__(self) -> None:
        super().__init__("fault_gate_node")
        self.declare_parameter("scenario_id", "")
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("run_id", "")
        self.declare_parameter("run_purpose", "diagnostic")
        self.declare_parameter("frozen_inputs_root_sha256", "")
        self.declare_parameter("seed", 0)
        self.declare_parameter("pose_service", "/world/obstacle_course/set_pose")
        self.scenario_id = str(self.get_parameter("scenario_id").value)
        self.scenario_path = str(self.get_parameter("scenario_path").value)
        self.scenario = load_yaml(self.scenario_path)
        self.run_id = str(self.get_parameter("run_id").value) or str(uuid.uuid4())
        self.run_purpose = str(self.get_parameter("run_purpose").value)
        self.root_hash = str(self.get_parameter("frozen_inputs_root_sha256").value).strip()
        self.seed = int(self.get_parameter("seed").value) or int(self.scenario["seed"])
        if self.run_purpose == "demonstration" and not self.root_hash:
            raise ValueError("demonstration fault events require frozen_inputs_root_sha256")
        self.config_hash = sha256_file(self.scenario_path)
        self.publisher = self.create_publisher(String, "/scenario/fault_event", 10)
        self._odom_sub = self.create_subscription(String, "/decision/evidence/odometry", self._on_odom, 10)
        self._pose_client = self.create_client(SetEntityPose, str(self.get_parameter("pose_service").value))
        self._seen_start: set[int] = set()
        self._seen_end: set[int] = set()
        self._reveal_pending: int | None = None
        self._reveal_requested = False
        self._last_odom: dict | None = None
        self._timer = self.create_timer(0.05, self._tick)

    def _on_odom(self, message: String) -> None:
        try:
            record = json.loads(message.data)
            if record.get("component") == "odometry" and record.get("valid"):
                self._last_odom = record.get("evidence", {})
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning("ignoring malformed odometry evidence")

    def _publish(self, event_type: str, component: str | None, entity_id: str | None,
                 route_progress_m: float | None, obstacle_range_m: float | None,
                 sim_time_ns: int) -> None:
        record = {
            "schema_version": "1.0",
            "event_id": str(uuid.uuid4()),
            "run_id": self.run_id,
            "run_purpose": self.run_purpose,
            "scenario_id": self.scenario_id,
            "event_type": event_type,
            "seed": self.seed,
            "sim_time_ns": max(0, int(sim_time_ns)),
            "component": component,
            "entity_id": entity_id,
            "route_progress_m": route_progress_m,
            "obstacle_range_m": obstacle_range_m,
            "config_sha256": self.config_hash,
        }
        if self.root_hash:
            record["frozen_inputs_root_sha256"] = self.root_hash
        message = String()
        message.data = encode_record(record)
        self.publisher.publish(message)

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        for index, item in enumerate(self.scenario.get("fault_injections", [])):
            start_ns = int(float(value_of(item["start_sim_time"])) * 1_000_000_000)
            end_ns = int(float(value_of(item["end_sim_time"])) * 1_000_000_000)
            action = item.get("action")
            if action == "drop":
                component = str(item["component"])
                if now_ns >= start_ns and index not in self._seen_start:
                    self._publish("sensor_dropout_started", component, None, None, None, now_ns)
                    self._seen_start.add(index)
                if now_ns >= end_ns and index in self._seen_start and index not in self._seen_end:
                    self._publish("sensor_dropout_ended", component, None, None, None, now_ns)
                    self._seen_end.add(index)
            elif action == "reveal_obstacle" and now_ns >= start_ns and not self._reveal_requested:
                self._reveal_pending = index
                self._maybe_reveal(item, now_ns)

    def _target_pose(self, item: dict) -> Pose:
        obstacle = next(
            (obstacle for obstacle in self.scenario.get("obstacles", [])
             if obstacle.get("entity_id") == item.get("entity_id")),
            None,
        )
        if obstacle is None:
            raise ValueError(f"reveal entity {item.get('entity_id')} is not in the scenario")
        x = float(value_of(obstacle["pose"]["x"]))
        y = float(value_of(obstacle["pose"]["y"]))
        yaw = float(value_of(obstacle["pose"]["yaw"]))
        start = self.scenario["start_pose"]
        ox, oy, oyaw = (float(value_of(start[name])) for name in ("x", "y", "yaw"))
        c, s = math.cos(oyaw), math.sin(oyaw)
        pose = Pose()
        pose.position.x = ox + c * x - s * y
        pose.position.y = oy + s * x + c * y
        pose.position.z = 0.06
        target_yaw = oyaw + yaw
        pose.orientation.z = math.sin(target_yaw / 2.0)
        pose.orientation.w = math.cos(target_yaw / 2.0)
        return pose

    def _maybe_reveal(self, item: dict, now_ns: int) -> None:
        if self._reveal_requested or self._last_odom is None or not self._pose_client.service_is_ready():
            return
        progress = float(self._last_odom.get("route_progress_m", 0.0))
        trigger_progress = float(value_of(item["trigger_pose_progress_m"]))
        if progress < trigger_progress:
            return
        request = SetEntityPose.Request()
        request.entity.name = str(item["entity_id"])
        request.entity.type = Entity.MODEL
        request.pose = self._target_pose(item)
        self._reveal_requested = True
        future = self._pose_client.call_async(request)
        future.add_done_callback(lambda result: self._on_reveal_result(result, item, now_ns, progress))

    def _on_reveal_result(self, future, item: dict, sim_time_ns: int, progress: float) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._reveal_requested = False
            self.get_logger().error(f"S5 reveal service failed: {exc}")
            return
        if not response.success:
            self._reveal_requested = False
            self.get_logger().error("Gazebo rejected the tracked S5 obstacle reveal pose")
            return
        current = self._last_odom or {}
        world_pose = current.get("world_pose", {})
        target = self._target_pose(item)
        distance = math.hypot(target.position.x - float(world_pose.get("x_m", target.position.x)),
                              target.position.y - float(world_pose.get("y_m", target.position.y)))
        self._publish("obstacle_revealed", None, str(item["entity_id"]), progress, distance, sim_time_ns)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaultGateNode()
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
