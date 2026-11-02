"""ROS wrapper for the deterministic Obstacle course decision engine."""

from __future__ import annotations

import json
from pathlib import Path
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String

from .decision_engine import DecisionEngine
from .evidence_common import load_yaml, sha256_file


class DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_course_decision")
        for name, default in (
            ("scenario_path", ""), ("decision_path", ""), ("profile_path", ""),
            ("sensor_profile_path", ""), ("trigger_matrix_path", ""),
            ("run_id", ""), ("run_purpose", "diagnostic"),
            ("frozen_inputs_root_sha256", ""), ("decision_rate_hz", 20.0),
        ):
            self.declare_parameter(name, default)
        params = {name: self.get_parameter(name).value for name in (
            "scenario_path", "decision_path", "profile_path", "sensor_profile_path",
            "trigger_matrix_path", "run_id", "run_purpose", "frozen_inputs_root_sha256",
        )}
        missing = [name for name in ("scenario_path", "decision_path", "profile_path", "sensor_profile_path", "trigger_matrix_path") if not params[name]]
        if missing:
            raise ValueError(f"decision node requires paths: {', '.join(missing)}")
        self.scenario = load_yaml(params["scenario_path"])
        self.decision = load_yaml(params["decision_path"])
        self.sensor_profile = load_yaml(params["sensor_profile_path"])
        self.matrix = load_yaml(params["trigger_matrix_path"])
        self.profile = json.loads(Path(params["profile_path"]).read_text(encoding="utf-8"))
        self.run_id = str(params["run_id"]) or str(uuid.uuid4())
        self.run_purpose = str(params["run_purpose"])
        self.root_hash = str(params["frozen_inputs_root_sha256"]).strip()
        if self.run_purpose == "demonstration" and not self.root_hash:
            raise ValueError("demonstration decision events require a frozen input root hash")
        self.engine = DecisionEngine(
            decision=self.decision,
            profile=self.profile,
            scenario=self.scenario,
            trigger_matrix=self.matrix,
            sensor_profile=self.sensor_profile,
            decision_sha256=sha256_file(params["decision_path"]),
            matrix_sha256=sha256_file(params["trigger_matrix_path"]),
            profile_sha256=str(self.profile["profile_sha256"]),
            run_id=self.run_id,
            run_purpose=self.run_purpose,
            frozen_root_sha256=self.root_hash,
        )
        self._records = {}
        self._sim_time_ns = 0
        self._waiting_ticks = 0
        self._seen_components = set()
        self._timer_ticks = 0
        self._subscriptions = [
            self.create_subscription(String, f"/decision/evidence/{component}", self._callback(component), qos_profile_sensor_data)
            for component in ("lidar", "camera", "imu", "odometry", "contact")
        ]
        self._clock_subscription = self.create_subscription(
            ClockMessage, "/clock", self._on_clock, qos_profile_sensor_data
        )
        self._state_pub = self.create_publisher(String, "/decision/state", 10)
        self._event_pub = self.create_publisher(String, "/decision/event", 10)
        self._intent_pub = self.create_publisher(String, "/decision/planner_intent", 10)
        hz = float(self.get_parameter("decision_rate_hz").value)
        if hz <= 0:
            raise ValueError("decision_rate_hz must be positive")
        # Schedule on a wall clock, evaluate freshness and stamps on ROS time.
        # This avoids catch-up bursts if Gazebo advances /clock faster than the
        # Python executor can drain timer callbacks.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(1.0 / hz, self._evaluate, clock=self._steady_clock)

    def _callback(self, component):
        def receive(message: String) -> None:
            try:
                record = json.loads(message.data)
                if record.get("component") != component:
                    return
                previous = self._records.get(component)
                if previous is None or int(record.get("sequence", -1)) >= int(previous.get("sequence", -1)):
                    self._records[component] = record
                if component not in self._seen_components:
                    self._seen_components.add(component)
                    self.get_logger().info(
                        f"received {component} evidence for {record.get('scenario_id')} "
                        f"run={str(record.get('run_id', ''))[:8]} valid={record.get('valid')}"
                    )
            except (json.JSONDecodeError, TypeError, ValueError) as exc:
                self.get_logger().warning(f"discarded malformed {component} evidence: {exc}")
        return receive

    def _on_clock(self, message: ClockMessage) -> None:
        self._sim_time_ns = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)

    def _evaluate(self) -> None:
        self._timer_ticks += 1
        now_ns = self._sim_time_ns or max(
            (int(record.get("sim_time_ns", 0)) for record in self._records.values()),
            default=0,
        )
        if self._timer_ticks % 100 == 0:
            self.get_logger().info(
                f"decision timer tick={self._timer_ticks} sim_time_ns={now_ns} "
                f"records={len(self._records)}"
            )
        result = self.engine.evaluate(self._records, now_ns)
        if result is None:
            self._waiting_ticks += 1
            if self._waiting_ticks % 100 == 0:
                summary = {
                    name: {
                        "run_id": str(record.get("run_id", ""))[:8],
                        "scenario_id": record.get("scenario_id"),
                        "run_purpose": record.get("run_purpose"),
                        "clock_epoch": record.get("clock_epoch"),
                        "valid": record.get("valid"),
                        "freshness": record.get("freshness", {}).get("state"),
                    }
                    for name, record in self._records.items()
                }
                self.get_logger().warning(f"waiting for coherent component evidence: {summary}")
            return
        self._waiting_ticks = 0
        state_message = String()
        state_message.data = json.dumps(result["state"], sort_keys=True, separators=(",", ":"), allow_nan=False)
        self._state_pub.publish(state_message)
        for event in result["events"]:
            message = String()
            message.data = json.dumps(event, sort_keys=True, separators=(",", ":"), allow_nan=False)
            self._event_pub.publish(message)
            self.get_logger().info(
                f"decision {event['previous_behavior']} -> {event['behavior']} "
                f"rule={event['rule_id']} reason={event['reason']}"
            )
        intent_message = String()
        intent_message.data = json.dumps(result["intent"], sort_keys=True, separators=(",", ":"), allow_nan=False)
        self._intent_pub.publish(intent_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionNode()
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
