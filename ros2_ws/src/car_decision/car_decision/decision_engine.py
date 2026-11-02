"""Deterministic rule-based behavior selection with evidence attribution."""

from __future__ import annotations

import math
import time
import uuid
from typing import Any

from .evidence_common import value_of
from .gate_corridor import configured_gate_opening_width, inspect_gate_corridor


BEHAVIORS = {"FOLLOW_ROUTE", "CAUTION", "AVOID_LEFT", "AVOID_RIGHT", "STOP", "RESUME"}
CRITICAL = ("lidar", "odometry", "contact")


def _value(config: dict[str, Any], *keys: str) -> float:
    item: Any = config
    for key in keys:
        item = item[key]
    return float(value_of(item))


def _evidence(record: dict[str, Any] | None) -> dict[str, Any]:
    return record.get("evidence", {}) if record else {}


class DecisionEngine:
    """Pure state machine; ROS message transport is kept in decision_node.py."""

    def __init__(
        self,
        *,
        decision: dict[str, Any],
        profile: dict[str, Any],
        scenario: dict[str, Any],
        trigger_matrix: dict[str, Any],
        sensor_profile: dict[str, Any],
        decision_sha256: str,
        matrix_sha256: str,
        profile_sha256: str,
        run_id: str,
        run_purpose: str,
        frozen_root_sha256: str = "",
    ) -> None:
        self.decision = decision
        self.profile = profile
        self.scenario = scenario
        self.matrix = trigger_matrix
        self.sensor_profile = sensor_profile
        self.decision_sha256 = decision_sha256
        self.matrix_sha256 = matrix_sha256
        self.profile_sha256 = profile_sha256
        self.run_id = run_id
        self.run_purpose = run_purpose
        self.frozen_root_sha256 = frozen_root_sha256
        self.scenario_id = str(scenario["scenario_id"])
        self.current: str | None = "FOLLOW_ROUTE"
        self.current_since_ns = 0
        self.last_transition_ns = 0
        self.last_gate_clear_scan_seq: int | None = None
        self.last_path_clear_scan_seq: int | None = None
        self.gate_clear_scans = 0
        self.path_clear_scans = 0
        self.stop_latched = False
        self.contact_latched = False
        self.latched_contact_pairs: list[dict[str, Any]] = []
        self.last_avoid_side: str | None = None
        self.next_avoid_side = "left"
        self.active_avoid_side: str | None = None
        self.active_avoid_entity_id: str | None = None
        self.active_avoid_class: str | None = None
        self.caution_exit_distance_m = 0.0
        self.camera_caution_latched = False
        self.camera_clear_frames = 0
        self.last_camera_sample_seq: int | None = None
        self.camera_caution_ref: dict[str, Any] | None = None
        self.last_evaluation_key: str | None = None
        self.last_overruled_key: str | None = None
        self.clock_epoch = 0
        self.event_sequence = 0
        self.sequence = 0
        self._started = False
        self.width_m = _value(profile, "vehicle_geometry", "vehicle", "width_m")
        self.length_m = _value(profile, "vehicle_geometry", "vehicle", "length_m")
        self.braking_mps2 = _value(profile, "vehicle_profile", "dynamics", "braking_deceleration_mps2")
        self.latency_s = _value(profile, "vehicle_profile", "dynamics", "request_to_deceleration_latency_s")
        self.gate_opening_width_m = configured_gate_opening_width(scenario)
        self._matrix_rules = {
            (row["component"], row["behavior"], row["role"]): row["rule"]
            for row in trigger_matrix["rows"]
        }

    def _ref(self, component: str, records: dict[str, dict[str, Any]], value: Any, unit: str) -> dict[str, Any]:
        record = records.get(component, {})
        fresh = record.get("freshness", {}).get("state", "unavailable")
        return {
            "component": component,
            "evidence_sequence": int(record.get("sequence", 0)),
            "value": value,
            "unit": unit,
            "freshness": fresh,
        }

    def _attribution(self, records, triggers=(), supporters=(), vetoes=(), overruled=(), stale=()):
        return {
            "triggers": list(triggers), "supporters": list(supporters),
            "vetoes": list(vetoes), "overruled": list(overruled),
            "stale_components": sorted(set(stale)),
        }

    def _event(self, *, now_ns, kind, previous, behavior, candidate, rule, reason, attribution, intent):
        event = {
            "schema_version": "1.0",
            "event_id": str(uuid.uuid4()),
            "run_id": self.run_id,
            "run_purpose": self.run_purpose,
            "scenario_id": self.scenario_id,
            "sequence": self.event_sequence,
            "event_kind": kind,
            "sim_time_ns": int(now_ns),
            "clock_epoch": self.clock_epoch,
            "published_monotonic_ns": time.monotonic_ns(),
            "previous_behavior": previous,
            "behavior": behavior,
            "candidate_behavior": candidate,
            "rule_id": rule,
            "reason": reason,
            "attribution": attribution,
            "planner_intent": {
                key: intent[key]
                for key in ("preferred_side", "class_margin_m", "speed_cap_mps", "valid_until_ns")
            } if intent is not None else None,
            "decision_config_sha256": self.decision_sha256,
            "trigger_matrix_sha256": self.matrix_sha256,
            "profile_sha256": self.profile_sha256,
        }
        if self.frozen_root_sha256:
            event["frozen_inputs_root_sha256"] = self.frozen_root_sha256
        self.event_sequence += 1
        return event

    def _allowed(self, attribution: dict[str, Any], behavior: str) -> None:
        for role in ("triggers", "supporters", "vetoes"):
            matrix_role = {"triggers": "trigger", "supporters": "support", "vetoes": "veto"}[role]
            for ref in attribution[role]:
                key = (ref["component"], behavior, matrix_role)
                if key not in self._matrix_rules:
                    raise ValueError(f"unauthorized attribution row: {key}")
        for ref in attribution["overruled"]:
            if ref["component"] != "camera" or ref["overruled_by"] != "lidar":
                raise ValueError("only a LiDAR overrule of camera gate evidence is supported")

    def evaluate(self, records: dict[str, dict[str, Any]], now_ns: int) -> dict[str, Any] | None:
        """Evaluate one coherent component snapshot; return None before all streams arrive."""
        if not all(name in records for name in ("lidar", "camera", "imu", "odometry", "contact")):
            return None
        identities = {(r.get("run_id"), r.get("scenario_id"), r.get("run_purpose")) for r in records.values()}
        if identities != {(self.run_id, self.scenario_id, self.run_purpose)}:
            return None
        epochs = {int(record.get("clock_epoch", 0)) for record in records.values()}
        if len(epochs) != 1:
            return None
        epoch = next(iter(epochs))
        if epoch != self.clock_epoch:
            if epoch < self.clock_epoch:
                return None
            self.clock_epoch = epoch
            self.current = "FOLLOW_ROUTE"
            self.current_since_ns = int(now_ns)
            self.stop_latched = False
            self.contact_latched = False
            self.latched_contact_pairs = []
            self.last_avoid_side = None
            self.next_avoid_side = "left"
            self.active_avoid_side = None
            self.active_avoid_entity_id = None
            self.active_avoid_class = None
            self.caution_exit_distance_m = 0.0
            self.camera_caution_latched = False
            self.camera_clear_frames = 0
            self.last_camera_sample_seq = None
            self.camera_caution_ref = None
            self.path_clear_scans = 0
            self.gate_clear_scans = 0
            self.last_gate_clear_scan_seq = None
            self.last_path_clear_scan_seq = None
            self.last_evaluation_key = None
            self.last_overruled_key = None
            self._started = False
        if not self._started:
            if not all(
                records[name].get("valid")
                and records[name].get("freshness", {}).get("state") == "fresh"
                for name in CRITICAL
            ):
                return None
            self._started = True
        stale = [name for name, record in records.items() if record.get("freshness", {}).get("state") != "fresh" or not record.get("valid")]
        if any(name in stale for name in CRITICAL):
            self.stop_latched = True
            self.gate_clear_scans = 0
            self.path_clear_scans = 0
            triggers = [self._ref(name, records, records[name].get("freshness", {}).get("state"), "freshness") for name in stale if name in CRITICAL]
            desired, rule, reason = "STOP", "STALE_CRITICAL_INPUT", "LiDAR or odometry evidence is stale or unavailable"
            attribution = self._attribution(records, triggers=triggers, stale=stale)
            hazard_range = None
            selected_class = None
            gate_info = None
        else:
            lidar = _evidence(records["lidar"])
            camera = _evidence(records["camera"])
            imu = _evidence(records["imu"])
            odom = _evidence(records["odometry"])
            contacts = _evidence(records["contact"])
            collision = bool(contacts.get("non_ground_collision"))
            if collision:
                self.stop_latched = True
                self.contact_latched = True
                self.latched_contact_pairs = list(contacts.get("contacts", []))
            camera_objects = camera.get("objects", []) if records["camera"].get("valid") else []
            lidar_points = lidar.get("points_base_link", [])
            gate_info = inspect_gate_corridor(
                camera_objects=camera_objects,
                lidar_points=lidar_points,
                opening_width_m=self.gate_opening_width_m,
                vehicle_width_m=self.width_m,
                clearance_multiple=_value(self.decision, "thresholds", "class_clearance_vehicle_width_multiple", "gate"),
            )
            lidar_seq = int(records["lidar"].get("source_sample_sequence", records["lidar"].get("sequence", -1)))
            if lidar_seq != self.last_gate_clear_scan_seq:
                self.last_gate_clear_scan_seq = lidar_seq
                if gate_info["passable"] and gate_info["clear"]:
                    self.gate_clear_scans += 1
                else:
                    self.gate_clear_scans = 0

            raw_range = lidar.get("route_corridor", {}).get("nearest_range_m")
            gate_clear = gate_info["passable"] and gate_info["clear"] and self.gate_clear_scans >= int(_value(self.decision, "temporal", "gate_clear_consecutive_scans"))
            in_opening_range = gate_info.get("return_range_m")
            hazard_range = float(in_opening_range) if in_opening_range is not None else (None if gate_clear else raw_range)
            overruled = []
            if gate_info["gate_seen"] and in_opening_range is not None:
                overruled = [{
                    "component": "camera", "overruled_by": "lidar",
                    "value": {"class": "gate", "return_range_m": in_opening_range},
                    "unit": "m", "reason": "LiDAR return lies inside the configured gate opening corridor",
                }]

            speed = abs(float(odom.get("speed_mps", 0.0)))
            d_stop = max(0.0, float(odom.get("d_stop_m", 0.0)))
            caution_dist = _value(self.decision, "thresholds", "caution_distance_multiple_d_stop") * d_stop
            avoid_dist = _value(self.decision, "thresholds", "avoidance_distance_multiple_d_stop") * d_stop
            lateral_accel = abs(float(imu.get("lateral_acceleration_mps2", 0.0)))
            lateral_limit = _value(self.decision, "thresholds", "lateral_acceleration_caution_mps2")
            selected_class = self._select_camera_class(camera_objects)
            if selected_class == "gate" and gate_clear:
                selected_class = None
            visual_range = self._estimate_visual_range(selected_class, camera_objects)
            camera_valid = (
                records["camera"].get("valid")
                and records["camera"].get("freshness", {}).get("state") == "fresh"
            )
            camera_seq = int(records["camera"].get("source_sample_sequence", records["camera"].get("sequence", -1)))
            if not camera_valid:
                # Camera dropout is non-critical: let LiDAR carry the route
                # decision instead of holding an old camera-only caution.
                self.camera_caution_latched = False
                self.camera_clear_frames = 0
                self.camera_caution_ref = None
            elif camera_seq != self.last_camera_sample_seq:
                self.last_camera_sample_seq = camera_seq
                if visual_range is not None and visual_range <= caution_dist:
                    self.camera_caution_latched = True
                    self.camera_clear_frames = 0
                    self.camera_caution_ref = self._ref(
                        "camera", records,
                        {"class": selected_class, "range_estimate_m": visual_range}, "m",
                    )
                elif self.camera_caution_latched:
                    self.camera_clear_frames += 1
                    clear_frames = int(_value(self.decision, "temporal", "camera_clear_consecutive_frames"))
                    if self.camera_clear_frames >= clear_frames:
                        self.camera_caution_latched = False
                        self.camera_clear_frames = 0
                        self.camera_caution_ref = None
            camera_caution_active = (
                visual_range is not None and visual_range <= caution_dist
            ) or (self.current == "CAUTION" and self.camera_caution_latched)
            active_object_ahead = (
                self.current in {"AVOID_LEFT", "AVOID_RIGHT", "CAUTION"}
                and self._active_object_ahead(
                    lidar=lidar, selected_class=selected_class,
                    visual_range_m=visual_range, caution_distance_m=caution_dist,
                    odometry=odom,
                )
            )
            if collision:
                desired, rule, reason = "STOP", "CONTACT_LATCH", "allowlisted non-ground contact latched a stop"
                triggers = [self._ref("contact", records, contacts.get("contacts", []), "contact_pair")]
                supporters = []
            elif hazard_range is None and camera_caution_active and self.current not in {"AVOID_LEFT", "AVOID_RIGHT"}:
                # Camera may caution, but it cannot create an avoidance or stop.
                desired, rule = "CAUTION", "CAMERA_CAUTION"
                reason = (
                    "camera class is inside its estimated caution distance"
                    if visual_range is not None and visual_range <= caution_dist
                    else "holding camera caution until consecutive clear frames confirm the path"
                )
                triggers = [self.camera_caution_ref or self._ref(
                    "camera", records,
                    {"class": selected_class, "range_estimate_m": visual_range}, "m",
                )]
                supporters = [self._ref("odometry", records, d_stop, "d_stop_m")]
            elif hazard_range is not None:
                self.path_clear_scans = 0
                available_brake_distance = float(hazard_range) - speed * self.latency_s
                required_braking = float("inf") if available_brake_distance <= 0 else speed * speed / (2.0 * available_brake_distance)
                braking_limit = self.braking_mps2 * _value(self.decision, "thresholds", "stop_required_deceleration_fraction")
                if required_braking >= braking_limit:
                    self.stop_latched = True
                    desired, rule, reason = "STOP", "STOPPING_DISTANCE", "hazard is inside the measured stopping envelope"
                    triggers = [self._ref("lidar", records, hazard_range, "m")]
                    supporters = [self._ref("odometry", records, {"speed_mps": speed, "d_stop_m": d_stop}, "m")]
                elif active_object_ahead and self.active_avoid_side:
                    # Hold the chosen maneuver as range estimates cross the
                    # caution/avoid envelopes. The planner keeps its lateral
                    # target until the tracked obstacle has passed the rear axle.
                    desired = "AVOID_LEFT" if self.active_avoid_side == "left" else "AVOID_RIGHT"
                    rule, reason = "ACTIVE_DETOUR_HOLD", "continue the selected detour until its tracked obstacle passes the rear axle"
                    triggers = [self._ref("lidar", records, hazard_range, "m")]
                    supporters = []
                elif hazard_range <= avoid_dist:
                    side, feasible = self._choose_avoidance(lidar, hazard_range, selected_class, camera_fresh=records["camera"].get("valid", False))
                    if not feasible:
                        self.stop_latched = True
                        desired, rule, reason = "STOP", "NO_FEASIBLE_DETOUR", "neither side satisfies the configured clearance margin"
                        triggers = [self._ref("lidar", records, hazard_range, "m")]
                        supporters = []
                    else:
                        desired = "AVOID_LEFT" if side == "left" else "AVOID_RIGHT"
                        rule, reason = "BLOCKED_ROUTE", f"route blocked at {hazard_range:.3f} m; {side} detour has clearance"
                        triggers = [self._ref("lidar", records, hazard_range, "m")]
                        supporters = []
                        if selected_class and records["camera"].get("valid"):
                            supporters.append(self._ref("camera", records, {"class": selected_class}, "class"))
                        if lateral_accel > lateral_limit:
                            supporters.append(self._ref("imu", records, lateral_accel, "m/s^2"))
                        self.last_avoid_side = side
                        self.active_avoid_class = selected_class
                elif hazard_range <= d_stop:
                    self.stop_latched = True
                    desired, rule, reason = "STOP", "STOPPING_DISTANCE", "hazard is inside the measured stopping envelope and no avoidance window remains"
                    triggers = [self._ref("lidar", records, hazard_range, "m")]
                    supporters = [self._ref("odometry", records, {"speed_mps": speed, "d_stop_m": d_stop}, "m")]
                elif hazard_range <= caution_dist or (
                    self.current == "CAUTION"
                    and self.caution_exit_distance_m > 0.0
                    and hazard_range <= self.caution_exit_distance_m
                ):
                    desired, rule, reason = "CAUTION", "CAUTION_DISTANCE", "LiDAR return is inside the configured caution envelope"
                    triggers = [self._ref("lidar", records, hazard_range, "m")]
                    supporters = [self._ref("odometry", records, d_stop, "d_stop_m")]
                elif active_object_ahead:
                    desired = self.current
                    rule, reason = "ACTIVE_DETOUR_HOLD", "keep the selected detour until its tracked obstacle passes the rear axle"
                    triggers, supporters = [], []
                else:
                    desired, rule, reason = "FOLLOW_ROUTE", "FOLLOW_CLEAR", "route corridor is clear"
                    triggers = [self._ref("odometry", records, float(odom.get("route_progress_m", 0.0)), "m")]
                    supporters = []
            elif lateral_accel > lateral_limit:
                desired, rule, reason = "CAUTION", "LATERAL_ACCELERATION", "measured lateral acceleration exceeded the configured limit"
                triggers = [self._ref("imu", records, lateral_accel, "m/s^2")]
                supporters = []
            elif self.current in {"AVOID_LEFT", "AVOID_RIGHT", "STOP", "CAUTION"}:
                if active_object_ahead:
                    # The object can leave the route corridor as the truck
                    # moves sideways, before it has actually passed the truck.
                    # Keep the detour until forward sensor evidence says it is
                    # alongside/behind rather than resuming through it.
                    self.path_clear_scans = 0
                elif lidar_seq != self.last_path_clear_scan_seq:
                    self.last_path_clear_scan_seq = lidar_seq
                    self.path_clear_scans += 1
                needed = int(_value(self.decision, "temporal", "clear_consecutive_scans"))
                dwell = _value(self.decision, "temporal", "stop_clear_dwell_s") if self.current == "STOP" else _value(self.decision, "temporal", "minimum_behavior_dwell_s")
                elapsed = max(0, now_ns - self.current_since_ns) / 1e9
                if self.path_clear_scans >= needed and elapsed >= dwell:
                    self.stop_latched = False
                    desired, rule, reason = "RESUME", "PATH_CLEAR_RESUME", "clear path and lateral clearance held through hysteresis"
                    triggers = [self._ref("lidar", records, self.path_clear_scans, "scans")]
                    supporters = [self._ref("odometry", records, odom.get("start_relative_pose", {}), "pose")]
                    if self.last_avoid_side is not None:
                        self.next_avoid_side = "right" if self.last_avoid_side == "left" else "left"
                        self.last_avoid_side = None
                else:
                    desired = self.current
                    rule, reason = "HYSTERESIS_HOLD", "waiting for path-clear count and minimum dwell"
                    triggers, supporters = [], []
            elif self.current == "RESUME":
                elapsed = max(0, now_ns - self.current_since_ns) / 1e9
                if elapsed >= _value(self.decision, "temporal", "minimum_behavior_dwell_s"):
                    desired, rule, reason = "FOLLOW_ROUTE", "PATH_RESUMED", "vehicle returned to the clear route"
                    triggers = [self._ref("odometry", records, float(odom.get("route_progress_m", 0.0)), "m")]
                    supporters = []
                else:
                    desired, rule, reason = "RESUME", "RESUME_DWELL", "holding resume state through its minimum dwell"
                    triggers, supporters = [], []
            else:
                desired, rule, reason = "FOLLOW_ROUTE", "FOLLOW_CLEAR", "route corridor is clear"
                triggers = [self._ref("odometry", records, float(odom.get("route_progress_m", 0.0)), "m")]
                supporters = []

            # A recorded collision is a run-level stop latch. The transient
            # contact sensor can go quiet while the vehicle is still pressed
            # against an obstacle, so a later empty scan must not silently
            # resume motion. A new scenario run (or simulation clock epoch)
            # is the reset boundary because this feature has no operator reset.
            if self.contact_latched:
                desired, rule = "STOP", "CONTACT_LATCH"
                reason = "non-ground contact was recorded; STOP remains latched for this run"
                triggers = [self._ref("contact", records, self.latched_contact_pairs, "contact_pair")]
                supporters = []

            if self.stop_latched and desired not in {"STOP", "RESUME"}:
                desired, rule, reason = "STOP", "STOP_LATCHED", "stop remains latched until the path-clear dwell completes"
                triggers = [self._ref("lidar", records, hazard_range, "m")]
                supporters = []
            attribution = self._attribution(
                records, triggers=triggers, supporters=supporters,
                overruled=overruled, stale=stale,
            )

        if self.current is not None and desired != self.current and desired not in {"STOP", "RESUME"}:
            dwell = _value(self.decision, "temporal", "minimum_behavior_dwell_s")
            if (now_ns - self.current_since_ns) / 1e9 < dwell:
                desired, rule, reason = self.current, "MINIMUM_DWELL", "minimum behavior dwell has not elapsed"
                attribution = self._attribution(records)

        if desired == "CAUTION" and not stale:
            if self.current != "CAUTION" or self.caution_exit_distance_m <= 0.0:
                hysteresis = _value(self.decision, "temporal", "distance_hysteresis_multiple_d_stop")
                self.caution_exit_distance_m = caution_dist + hysteresis * d_stop
        elif desired not in {"AVOID_LEFT", "AVOID_RIGHT"}:
            self.caution_exit_distance_m = 0.0

        if desired in {"AVOID_LEFT", "AVOID_RIGHT"} and not stale:
            obstacle = self._nearest_scenario_obstacle(odom)
            if obstacle is not None:
                self.active_avoid_entity_id = str(obstacle["entity_id"])
                self.active_avoid_class = str(obstacle.get("class", selected_class or "")) or None
            elif selected_class:
                self.active_avoid_class = selected_class
        if desired == "AVOID_LEFT":
            self.active_avoid_side = "left"
        elif desired == "AVOID_RIGHT":
            self.active_avoid_side = "right"
        elif desired == "RESUME":
            # RESUME means the path-clear dwell has completed. Drop the offset
            # and let pure pursuit guide the vehicle back to the route.
            self.active_avoid_side = None
            self.active_avoid_entity_id = None
            self.active_avoid_class = None
        preferred = (
            "left" if desired == "AVOID_LEFT" else
            "right" if desired == "AVOID_RIGHT" else
            self.active_avoid_side if desired == "CAUTION" and self.active_avoid_side else
            "none"
        )
        speed_cap = _value(self.decision, "thresholds", "turn_speed_cap_mps") if desired in {"CAUTION", "AVOID_LEFT", "AVOID_RIGHT"} else _value(self.scenario, "entry_speed_mps") if desired in {"FOLLOW_ROUTE", "RESUME"} else 0.0
        margin_class = selected_class or ("conservative_lidar_only" if stale and "camera" in stale else "barrier")
        margin_multiple = _value(self.decision, "thresholds", "class_clearance_vehicle_width_multiple", margin_class)
        intent = {
            "preferred_side": preferred,
            "class_margin_m": self.width_m * margin_multiple,
            "speed_cap_mps": speed_cap,
            "valid_until_ns": int(now_ns + _value(self.decision, "geometry", "planner_intent", "intent_validity_s") * 1e9),
        }

        events = []
        if desired != self.current:
            self._allowed(attribution, desired)
            previous = self.current
            self.current = desired
            self.current_since_ns = int(now_ns)
            self.last_transition_ns = int(now_ns)
            event = self._event(
                now_ns=now_ns, kind="transition", previous=previous, behavior=desired,
                candidate=None, rule=rule, reason=reason, attribution=attribution,
                intent=intent,
            )
            intent["decision_event_id"] = event["event_id"]
            events.append(event)
        evaluation = None
        if gate_info and gate_info["gate_seen"] and gate_info["passable"] and gate_info["clear"] and self.gate_clear_scans >= int(_value(self.decision, "temporal", "gate_clear_consecutive_scans")):
            key = f"gate-veto:{self.current}"
            if key != self.last_evaluation_key:
                refs = [self._ref("camera", records, {"class": "gate"}, "class")]
                attribution = self._attribution(records, vetoes=refs)
                self._allowed(attribution, "AVOID_LEFT")
                evaluation = self._event(
                    now_ns=now_ns, kind="evaluation", previous=self.current,
                    behavior=self.current, candidate="AVOID_LEFT", rule="GATE_CLEAR_VETO",
                    reason="camera identified a passable gate and consecutive LiDAR scans found its opening clear",
                    attribution=attribution, intent=intent,
                )
                events.append(evaluation)
                self.last_evaluation_key = key
        elif gate_info is None or not gate_info.get("gate_seen") or not gate_info.get("clear"):
            self.last_evaluation_key = None

        overruled_key = None
        if gate_info and gate_info.get("gate_seen") and gate_info.get("return_range_m") is not None:
            overruled_key = f"gate-return:{float(gate_info['return_range_m']):.3f}:{self.current}"
            if overruled_key != self.last_overruled_key and not any(event["event_kind"] == "transition" for event in events):
                candidate = "STOP" if hazard_range is not None and hazard_range <= d_stop else "AVOID_LEFT" if hazard_range is not None and hazard_range <= avoid_dist else "CAUTION"
                self._allowed(attribution, self.current)
                evaluation = self._event(
                    now_ns=now_ns, kind="evaluation", previous=self.current,
                    behavior=self.current, candidate=candidate, rule="LIDAR_GATE_RETURN_OVERRULE",
                    reason="LiDAR has an in-opening return; the camera gate classification is overruled",
                    attribution=attribution, intent=intent,
                )
                events.append(evaluation)
            self.last_overruled_key = overruled_key
        else:
            self.last_overruled_key = None

        intent.update({
            "schema_version": "1.0", "scenario_id": self.scenario_id,
            "run_id": self.run_id, "run_purpose": self.run_purpose,
            "decision_event_id": intent.get("decision_event_id") or (events[-1]["event_id"] if events else str(uuid.uuid4())),
            "sim_time_ns": int(now_ns), "clock_epoch": self.clock_epoch,
            "behavior": self.current, "selected_class": selected_class,
            "rule_id": rule, "decision_config_sha256": self.decision_sha256,
            "trigger_matrix_sha256": self.matrix_sha256, "profile_sha256": self.profile_sha256,
        })
        if self.frozen_root_sha256:
            intent["frozen_inputs_root_sha256"] = self.frozen_root_sha256
        state = {
            "schema_version": "1.0", "scenario_id": self.scenario_id,
            "run_id": self.run_id, "run_purpose": self.run_purpose,
            "sim_time_ns": int(now_ns), "clock_epoch": self.clock_epoch, "behavior": self.current,
            "reason": reason, "attribution": attribution,
            "freshness": {name: records[name].get("freshness", {}).get("state", "unavailable") for name in records},
            "decision_config_sha256": self.decision_sha256,
            "trigger_matrix_sha256": self.matrix_sha256,
            "profile_sha256": self.profile_sha256,
        }
        if self.frozen_root_sha256:
            state["frozen_inputs_root_sha256"] = self.frozen_root_sha256
        self.sequence += 1
        return {"state": state, "events": events, "intent": intent}

    @staticmethod
    def _select_camera_class(objects: list[dict[str, Any]]) -> str | None:
        actionable = [item for item in objects if item.get("class") in {"barrier", "post", "gate"}]
        forward = [item for item in actionable if abs(float(item.get("bearing_rad", 99.0))) <= 0.55]
        if not forward:
            return None
        priority = {"barrier": 3, "post": 2, "gate": 1}
        return max(forward, key=lambda item: (float(item.get("apparent_size_px", 0.0)), priority[item["class"]]))["class"]

    def _estimate_visual_range(self, selected_class, objects):
        if not selected_class:
            return None
        candidates = [item for item in objects if item.get("class") == selected_class and abs(float(item.get("bearing_rad", 99))) <= 0.55]
        if not candidates:
            return None
        camera_profile = self.sensor_profile["presets"]["rc_car"]["sensors"]["camera"]
        width_px = float(value_of(camera_profile["width_px"]))
        fov = float(value_of(camera_profile["horizontal_fov_rad"]))
        focal_px = width_px / (2.0 * math.tan(fov / 2.0))
        dimensions = [
            max(float(value_of(item["dimensions_m"][axis])) for axis in ("length", "width", "height"))
            for item in self.scenario.get("obstacles", [])
            if item.get("class") == selected_class
        ]
        if not dimensions:
            return None
        nominal_dimension = max(dimensions)
        distances = [
            nominal_dimension * focal_px / max(1.0, float(item.get("apparent_size_px", 0.0)))
            for item in candidates
        ]
        return min(distances) if distances else None

    def _choose_avoidance(self, lidar, distance, selected_class, *, camera_fresh):
        margins = self.decision["thresholds"]["class_clearance_vehicle_width_multiple"]
        kind = selected_class if camera_fresh and selected_class in margins else "conservative_lidar_only"
        clearance = self.width_m * _value(self.decision, "thresholds", "class_clearance_vehicle_width_multiple", kind)
        half_width = self.width_m / 2.0
        sectors = lidar.get("sectors", {})
        left = sectors.get("left_m")
        right = sectors.get("right_m")
        required = half_width + clearance
        left_free = left is None or float(left) > distance + required
        right_free = right is None or float(right) > distance + required
        if not left_free and not right_free:
            return "none", False
        if left_free and right_free:
            if self.current == "AVOID_LEFT":
                return "left", True
            if self.current == "AVOID_RIGHT":
                return "right", True
            # Keep one side for the entire hazard episode. Ranges may briefly
            # cross CAUTION/AVOID thresholds as the scan jitters; alternating
            # sides on those transitions makes the truck weave into the object.
            if self.last_avoid_side is not None:
                return self.last_avoid_side, True
            return self.next_avoid_side, True
        return ("left" if left_free else "right"), True

    def _route_progress(self, x: float, y: float) -> tuple[float, float]:
        """Project a start-relative point onto the configured route polyline."""
        points = [
            (float(value_of(item["x"])), float(value_of(item["y"])))
            for item in self.scenario["route"]["waypoints"]
        ]
        traveled = 0.0
        best_distance, best_progress = float("inf"), 0.0
        for start, end in zip(points, points[1:]):
            dx, dy = end[0] - start[0], end[1] - start[1]
            length_sq = dx * dx + dy * dy
            length = math.sqrt(length_sq)
            if length_sq <= 1e-12:
                continue
            ratio = max(0.0, min(1.0, ((x - start[0]) * dx + (y - start[1]) * dy) / length_sq))
            px, py = start[0] + ratio * dx, start[1] + ratio * dy
            distance = math.hypot(x - px, y - py)
            if distance < best_distance:
                best_distance, best_progress = distance, traveled + ratio * length
            traveled += length
        return best_progress, best_distance

    def _nearest_scenario_obstacle(self, odometry: dict[str, Any]) -> dict[str, Any] | None:
        pose = odometry.get("start_relative_pose", {})
        progress, _ = self._route_progress(float(pose.get("x_m", 0.0)), float(pose.get("y_m", 0.0)))
        candidates = []
        for obstacle in self.scenario.get("obstacles", []):
            obstacle_pose = obstacle.get("pose", {})
            x = float(value_of(obstacle_pose.get("x", 0.0)))
            y = float(value_of(obstacle_pose.get("y", 0.0)))
            obstacle_progress, lateral_distance = self._route_progress(x, y)
            half_extent = max(
                float(value_of(obstacle.get("dimensions_m", {}).get(axis, 0.0))) / 2.0
                for axis in ("length", "width")
            )
            along = obstacle_progress - progress
            if along >= -(self.length_m / 2.0 + half_extent) and lateral_distance <= self.width_m + half_extent:
                candidates.append((along, obstacle))
        return min(candidates, key=lambda item: item[0])[1] if candidates else None

    def _active_object_ahead(self, *, lidar, selected_class, visual_range_m, caution_distance_m, odometry):
        """Keep the detour until the configured scenario object has passed the rear axle."""
        if self.active_avoid_side is None:
            return False
        if self.active_avoid_entity_id:
            obstacle = next((
                item for item in self.scenario.get("obstacles", [])
                if item.get("entity_id") == self.active_avoid_entity_id
            ), None)
            if obstacle is not None:
                pose = odometry.get("start_relative_pose", {})
                vehicle_progress, _ = self._route_progress(
                    float(pose.get("x_m", 0.0)), float(pose.get("y_m", 0.0))
                )
                obstacle_pose = obstacle.get("pose", {})
                obstacle_progress, _ = self._route_progress(
                    float(value_of(obstacle_pose.get("x", 0.0))),
                    float(value_of(obstacle_pose.get("y", 0.0))),
                )
                half_extent = max(
                    float(value_of(obstacle.get("dimensions_m", {}).get(axis, 0.0))) / 2.0
                    for axis in ("length", "width")
                )
                along = obstacle_progress - vehicle_progress
                rear_clearance = self.length_m / 2.0 + half_extent
                # Once avoidance starts, the obstacle remains relevant for the
                # whole detour. A caution-distance upper bound is unsafe here:
                # the controller may already have moved outside the LiDAR route
                # corridor while the obstacle is still ahead, then resume into it.
                return along >= -rear_clearance
        active_class = self.active_avoid_class
        if (
            selected_class == active_class and visual_range_m is not None
            and visual_range_m <= caution_distance_m
        ):
            return True

        margins = self.decision["thresholds"]["class_clearance_vehicle_width_multiple"]
        margin_class = active_class if active_class in margins else "conservative_lidar_only"
        clearance = self.width_m * _value(
            self.decision, "thresholds", "class_clearance_vehicle_width_multiple", margin_class
        )
        object_half_width = max((
            max(float(value_of(item["dimensions_m"][axis])) for axis in ("length", "width")) / 2.0
            for item in self.scenario.get("obstacles", [])
            if item.get("class") == active_class
        ), default=0.0)
        lateral_limit = self.width_m / 2.0 + clearance + object_half_width
        for point in lidar.get("points_base_link", []):
            x = float(point.get("x_m", -1.0))
            y = float(point.get("y_m", float("inf")))
            distance = float(point.get("range_m", math.hypot(x, y)))
            if 0.0 < x <= caution_distance_m and abs(y) <= lateral_limit and distance <= caution_distance_m:
                return True
        return False
