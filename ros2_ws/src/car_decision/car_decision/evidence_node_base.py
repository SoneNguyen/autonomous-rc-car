"""Simulation-time publisher base for normalized Obstacle course evidence."""

from __future__ import annotations

import uuid
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .evidence_common import (
    ClockEpochTracker,
    encode_record,
    fault_active,
    freshness,
    load_yaml,
    sha256_file,
)


class EvidenceNode(Node):
    def __init__(self, component: str, topic: str, message_type: str) -> None:
        super().__init__(f"{component}_evidence_node")
        self.component = component
        self.declare_parameter("scenario_id", "")
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("decision_path", "")
        self.declare_parameter("profile_path", "")
        self.declare_parameter("sensor_profile_path", "")
        self.declare_parameter("run_id", "")
        self.declare_parameter("run_purpose", "diagnostic")
        self.declare_parameter("frozen_inputs_root_sha256", "")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.scenario_id = str(self.get_parameter("scenario_id").value)
        self.scenario_path = str(self.get_parameter("scenario_path").value)
        self.decision_path = str(self.get_parameter("decision_path").value)
        self.profile_path = str(self.get_parameter("profile_path").value)
        self.sensor_profile_path = str(self.get_parameter("sensor_profile_path").value)
        self.scenario = load_yaml(self.scenario_path)
        self.decision = load_yaml(self.decision_path)
        self.profile = load_yaml(self.profile_path)
        self.sensor_profile = load_yaml(self.sensor_profile_path)
        self.run_id = str(self.get_parameter("run_id").value) or str(uuid.uuid4())
        self.run_purpose = str(self.get_parameter("run_purpose").value)
        self.root_hash = str(self.get_parameter("frozen_inputs_root_sha256").value).strip()
        if self.run_purpose == "demonstration" and not self.root_hash:
            raise ValueError("demonstration evidence requires frozen_inputs_root_sha256")
        self.config_hash = sha256_file(self.decision_path)
        self.profile_hash = sha256_file(self.profile_path)
        self.sequence = 0
        self.sample_sequence = 0
        self._clock_epoch = ClockEpochTracker()
        self._sample_ns: int | None = None
        self._frame_id: str | None = None
        self._payload: dict[str, Any] | None = None
        self._publisher = self.create_publisher(String, f"/decision/evidence/{component}", 10)
        self._timeout_s = float(self.decision["freshness"][f"{component}_timeout_s"]["value"])
        hz = float(self.get_parameter("publish_rate_hz").value)
        if hz <= 0:
            raise ValueError("publish_rate_hz must be positive")
        self._timer = self.create_timer(1.0 / hz, self._publish_status)
        self._source_topic = topic
        self._message_type = message_type

    def is_faulted(self, now_ns: int) -> bool:
        return fault_active(self.scenario, self.component, now_ns)

    def record_sample(self, payload: dict[str, Any], stamp_ns: int, frame_id: str | None) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._clock_epoch.observe(now_ns):
            self._sample_ns = None
            self._payload = None
        if self.is_faulted(now_ns):
            return
        self._sample_ns = int(stamp_ns)
        self.sample_sequence += 1
        self._frame_id = frame_id or None
        self._payload = payload

    def _empty_payload(self) -> dict[str, Any]:
        return {}

    def _publish_status(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._clock_epoch.observe(now_ns):
            self._sample_ns = None
            self._payload = None
        status = freshness(self._sample_ns, now_ns, self._timeout_s)
        if self.is_faulted(now_ns) and status["state"] == "fresh":
            status["state"] = "stale"
        record: dict[str, Any] = {
            "schema_version": "1.0",
            "component": self.component,
            "scenario_id": self.scenario_id,
            "run_id": self.run_id,
            "run_purpose": self.run_purpose,
            "sequence": self.sequence,
            "source_sample_sequence": self.sample_sequence,
            "sim_time_ns": max(0, int(now_ns)),
            "clock_epoch": self._clock_epoch.epoch,
            "freshness": status,
            "valid": status["state"] == "fresh" and self._payload is not None,
            "invalid_reason": None,
            "source": {
                "topic": self._source_topic,
                "message_type": self._message_type,
                "adapter_id": f"obstacle-course-{self.component}-v1",
            },
            "config_sha256": self.config_hash,
            "profile_sha256": self.profile_hash,
            "frame_id": self._frame_id,
            "evidence": self._payload if self._payload is not None else self._empty_payload(),
        }
        if status["state"] != "fresh":
            record["invalid_reason"] = "source sample unavailable or stale in simulation time"
        if self.root_hash:
            record["frozen_inputs_root_sha256"] = self.root_hash
        message = String()
        message.data = encode_record(record)
        self._publisher.publish(message)
        self.sequence += 1


def spin_node(node_type, args=None) -> None:
    rclpy.init(args=args)
    node = node_type()
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
