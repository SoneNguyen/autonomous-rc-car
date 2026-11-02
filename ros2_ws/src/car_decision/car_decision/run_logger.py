"""Persist one Obstacle course run's component evidence and decision stream."""

from __future__ import annotations

import json
from pathlib import Path
import threading

import jsonschema
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class RunLogger(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_course_run_logger")
        for name, default in (
            ("output_dir", ""), ("contract_dir", ""), ("run_id", ""),
            ("scenario_id", ""), ("run_purpose", "diagnostic"),
            ("seed", 0), ("frozen_inputs_root_sha256", ""),
        ):
            self.declare_parameter(name, default)
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser().resolve()
        self.contract_dir = Path(str(self.get_parameter("contract_dir").value)).expanduser().resolve()
        self.run_id = str(self.get_parameter("run_id").value)
        self.scenario_id = str(self.get_parameter("scenario_id").value)
        self.run_purpose = str(self.get_parameter("run_purpose").value)
        self.seed = int(self.get_parameter("seed").value)
        self.root_hash = str(self.get_parameter("frozen_inputs_root_sha256").value).strip()
        if not self.output_dir.is_absolute() or not self.run_id or not self.scenario_id:
            raise ValueError("run logger requires an output directory, run ID, and scenario ID")
        if self.run_purpose == "demonstration" and not self.root_hash:
            raise ValueError("demonstration logger requires frozen_inputs_root_sha256")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._files = {
            "state": (self.output_dir / "states.jsonl").open("a", encoding="utf-8"),
            "event": (self.output_dir / "events.jsonl").open("a", encoding="utf-8"),
            "intent": (self.output_dir / "intents.jsonl").open("a", encoding="utf-8"),
            "fault_event": (self.output_dir / "fault-events.jsonl").open("a", encoding="utf-8"),
            "validation_error": (self.output_dir / "validation-errors.jsonl").open("a", encoding="utf-8"),
        }
        for component in ("lidar", "camera", "imu", "odometry", "contact"):
            self._files[f"evidence/{component}"] = (
                self.output_dir / f"evidence-{component}.jsonl"
            ).open("a", encoding="utf-8")
        self._validators = self._load_schemas()
        self._subscriptions = [
            self.create_subscription(String, "/decision/state", self._callback("state"), 10),
            self.create_subscription(String, "/decision/event", self._callback("event"), 10),
            self.create_subscription(String, "/decision/planner_intent", self._callback("intent"), 10),
            self.create_subscription(String, "/scenario/fault_event", self._callback("fault_event"), 10),
            *[
                self.create_subscription(
                    String, f"/decision/evidence/{name}", self._callback(f"evidence/{name}"), qos_profile_sensor_data
                ) for name in ("lidar", "camera", "imu", "odometry", "contact")
            ],
        ]

    def _load_schemas(self) -> dict[str, jsonschema.Draft202012Validator]:
        names = {
            "event": "decision-event.schema.json",
            "intent": "planner-intent.schema.json",
            "fault_event": "fault-event.schema.json",
            **{f"evidence/{name}": "component-evidence.schema.json" for name in ("lidar", "camera", "imu", "odometry", "contact")},
        }
        validators = {}
        format_checker = jsonschema.FormatChecker()
        for topic, name in names.items():
            path = self.contract_dir / name
            schema = json.loads(path.read_text(encoding="utf-8"))
            jsonschema.Draft202012Validator.check_schema(schema)
            validators[topic] = jsonschema.Draft202012Validator(
                schema, format_checker=format_checker
            )
        return validators

    def _write(self, stream: str, record: dict) -> None:
        with self._lock:
            self._files[stream].write(
                json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n"
            )
            # Sensor evidence arrives at high rate. Buffered writes keep disk
            # latency out of ROS callbacks; close() flushes every stream, while
            # rare decision/fault events are flushed immediately for visibility.
            if stream in {"event", "fault_event", "validation_error"}:
                self._files[stream].flush()

    def _callback(self, stream: str, check_identity: bool = True):
        def receive(message: String) -> None:
            try:
                record = json.loads(message.data)
                if check_identity:
                    if record.get("run_id") != self.run_id:
                        return
                    if record.get("scenario_id") != self.scenario_id:
                        return
                    if record.get("run_purpose") != self.run_purpose:
                        return
                    if self.root_hash and record.get("frozen_inputs_root_sha256") != self.root_hash:
                        self._write("validation_error", {"stream": stream, "error": "frozen input root mismatch"})
                        return
                if stream in self._validators:
                    self._validators[stream].validate(record)
                self._write(stream, record)
            except Exception as exc:
                self._write("validation_error", {"stream": stream, "error": str(exc), "message": message.data})
        return receive

    def destroy_node(self):
        for stream in getattr(self, "_files", {}).values():
            try:
                stream.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RunLogger()
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
