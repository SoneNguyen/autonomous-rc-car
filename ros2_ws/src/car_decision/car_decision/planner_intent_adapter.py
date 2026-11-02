"""Convert short-lived behavior intent and relative route into a drive target."""

from __future__ import annotations

import json
import math
from typing import Any

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String

from .evidence_common import load_yaml, value_of


def route_point_at(waypoints: list[dict[str, Any]], distance_m: float) -> tuple[float, float]:
    points = [(float(value_of(item["x"])), float(value_of(item["y"]))) for item in waypoints]
    remaining = max(0.0, distance_m)
    for start, end in zip(points, points[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            continue
        if remaining <= length:
            ratio = remaining / length
            return start[0] + ratio * dx, start[1] + ratio * dy
        remaining -= length
    return points[-1]


def project_route(waypoints: list[dict[str, Any]], x: float, y: float) -> tuple[float, float, float]:
    points = [(float(value_of(item["x"])), float(value_of(item["y"]))) for item in waypoints]
    traveled = 0.0
    best = (float("inf"), 0.0, 0.0)
    for start, end in zip(points, points[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_sq = dx * dx + dy * dy
        length = math.sqrt(length_sq)
        if length_sq <= 1e-12:
            continue
        ratio = max(0.0, min(1.0, ((x - start[0]) * dx + (y - start[1]) * dy) / length_sq))
        px, py = start[0] + ratio * dx, start[1] + ratio * dy
        distance = math.hypot(x - px, y - py)
        if distance < best[0]:
            best = (distance, traveled + ratio * length, math.atan2(dy, dx))
        traveled += length
    return best[1], best[2], best[0]


def target_drive(
    *,
    pose: dict[str, float],
    intent: dict[str, Any],
    route: dict[str, Any],
    vehicle_width_m: float,
    vehicle_length_m: float,
    wheelbase_m: float,
    entry_speed_mps: float,
    now_ns: int,
    target_offset_extra_m: float = 0.0,
    target_offset_min_m: float = 0.0,
    lookahead_min_multiple_L: float = 1.5,
    lookahead_max_multiple_L: float = 3.5,
) -> dict[str, float]:
    """Pure pursuit target; expired/unsafe intents always return zero speed."""
    if int(intent.get("valid_until_ns", -1)) < now_ns or not intent.get("behavior"):
        return {"speed_mps": 0.0, "steering_angle_rad": 0.0}
    behavior = intent["behavior"]
    if behavior == "STOP":
        return {"speed_mps": 0.0, "steering_angle_rad": 0.0}
    waypoints = route["waypoints"]
    route_length = float(value_of(route["length_m"]))
    x, y, yaw = float(pose["x_m"]), float(pose["y_m"]), float(pose["yaw_rad"])
    progress, route_yaw, _ = project_route(waypoints, x, y)
    side = intent.get("preferred_side", "none")
    maneuvering = behavior in {"AVOID_LEFT", "AVOID_RIGHT", "CAUTION"} and side in {"left", "right"}
    # Use a shorter pursuit horizon for a requested detour so the small RC
    # chassis reaches its clearance offset before the obstacle. Keep the longer
    # horizon for ordinary route tracking to avoid twitchy straight-line trim.
    lookahead_multiple = lookahead_min_multiple_L if maneuvering else 2.5
    lookahead = vehicle_length_m * max(
        lookahead_min_multiple_L,
        min(lookahead_max_multiple_L, lookahead_multiple),
    )
    target_x, target_y = route_point_at(waypoints, min(route_length, progress + lookahead))
    # A CAUTION label can occur briefly while the same obstacle's range moves
    # across the CAUTION/AVOID boundary. Preserve the active side target until
    # the decision engine explicitly clears it at RESUME.
    if maneuvering:
        side_sign = 1.0 if side == "left" else -1.0
        geometric_offset = vehicle_width_m / 2.0 + float(intent.get("class_margin_m", 0.0)) + max(0.0, target_offset_extra_m)
        offset = max(geometric_offset, max(0.0, target_offset_min_m))
        # Smoothly enter and leave the offset while retaining the route tangent.
        ramp = min(1.0, max(0.0, (progress + lookahead) / max(lookahead, 1e-6)))
        target_x += -math.sin(route_yaw) * side_sign * offset * ramp
        target_y += math.cos(route_yaw) * side_sign * offset * ramp
    dx, dy = target_x - x, target_y - y
    body_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    body_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    distance_sq = max(1e-6, body_x * body_x + body_y * body_y)
    curvature = 2.0 * body_y / distance_sq
    steering = math.atan(wheelbase_m * curvature)
    speed_cap = max(0.0, float(intent.get("speed_cap_mps", entry_speed_mps)))
    speed = min(entry_speed_mps, speed_cap)
    tolerance = vehicle_width_m * 0.5
    if progress >= route_length - tolerance:
        speed = 0.0
    return {"speed_mps": speed, "steering_angle_rad": steering}


class PlannerIntentAdapter(Node):
    def __init__(self) -> None:
        super().__init__("planner_intent_adapter")
        for name, default in (("scenario_path", ""), ("profile_path", ""), ("vehicle_geometry_path", ""), ("decision_path", "")):
            self.declare_parameter(name, default)
        paths = {name: str(self.get_parameter(name).value) for name in ("scenario_path", "profile_path", "vehicle_geometry_path", "decision_path")}
        if not all(paths.values()):
            raise ValueError("planner intent adapter requires scenario, profile, vehicle geometry, and decision config")
        self.scenario = load_yaml(paths["scenario_path"])
        self.profile = json.loads(open(paths["profile_path"], encoding="utf-8").read())
        geometry = load_yaml(paths["vehicle_geometry_path"])["vehicle"]
        self.decision = load_yaml(paths["decision_path"])
        self.width = float(value_of(geometry["width_m"]))
        self.length = float(value_of(geometry["length_m"]))
        vehicle = self.profile["vehicle_profile"]
        self.wheelbase = float(value_of(vehicle["geometry"]["wheelbase_m"]))
        self.entry_speed = float(value_of(self.scenario["entry_speed_mps"]))
        self.obstacle_half_width = max((float(value_of(item["dimensions_m"]["width"])) / 2.0 for item in self.scenario.get("obstacles", [])), default=0.0)
        self.configured_lateral_shift = float(value_of(self.scenario["route"]["sizing"]["lateral_shift_m"]))
        self.lookahead_min = float(value_of(self.decision["geometry"]["planner_intent"]["lookahead_min_multiple_L"]))
        self.lookahead_max = float(value_of(self.decision["geometry"]["planner_intent"]["lookahead_max_multiple_L"]))
        self._pose = None
        self._pose_sim_time_ns = 0
        self._intent = None
        self._sim_time_ns = 0
        self.create_subscription(String, "/decision/evidence/odometry", self._on_odometry, qos_profile_sensor_data)
        self.create_subscription(String, "/decision/planner_intent", self._on_intent, 10)
        self.create_subscription(ClockMessage, "/clock", self._on_clock, qos_profile_sensor_data)
        self._publisher = self.create_publisher(String, "/decision/target_drive", 10)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(0.05, self._publish_target, clock=self._steady_clock)

    def _on_clock(self, message: ClockMessage) -> None:
        self._sim_time_ns = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)

    def _on_odometry(self, message: String) -> None:
        try:
            record = json.loads(message.data)
            if record.get("valid") and record.get("freshness", {}).get("state") == "fresh":
                self._pose = record["evidence"]["start_relative_pose"]
                self._pose_sim_time_ns = int(record.get("sim_time_ns", 0))
        except (json.JSONDecodeError, KeyError, TypeError):
            return

    def _on_intent(self, message: String) -> None:
        try:
            self._intent = json.loads(message.data)
        except json.JSONDecodeError:
            self._intent = None

    def _publish_target(self) -> None:
        now_ns = self._sim_time_ns or max(
            self._pose_sim_time_ns,
            int(self._intent.get("sim_time_ns", 0)) if self._intent else 0,
        )
        target = {"speed_mps": 0.0, "steering_angle_rad": 0.0, "valid_until_ns": now_ns}
        if self._pose is not None and self._intent is not None:
            target.update(target_drive(
                pose=self._pose, intent=self._intent, route=self.scenario["route"],
                vehicle_width_m=self.width, vehicle_length_m=self.length,
                wheelbase_m=self.wheelbase, entry_speed_mps=self.entry_speed,
                now_ns=now_ns, target_offset_extra_m=self.obstacle_half_width,
                target_offset_min_m=self.configured_lateral_shift,
                lookahead_min_multiple_L=self.lookahead_min,
                lookahead_max_multiple_L=self.lookahead_max,
            ))
            target["valid_until_ns"] = int(self._intent.get("valid_until_ns", now_ns))
            target["run_id"] = self._intent.get("run_id")
        target["sim_time_ns"] = now_ns
        message = String()
        message.data = json.dumps(target, sort_keys=True, separators=(",", ":"), allow_nan=False)
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerIntentAdapter()
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
