"""Deterministic LiDAR geometry and status observations.

These helpers consume sensor samples and the module-matrix ground truth. They
do not change planner thresholds or sensor configuration.
"""
from __future__ import annotations

from bisect import bisect_right
import json
import math
from pathlib import Path
from typing import Any, Mapping

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SECTOR_NAMES = ("right", "front_right", "front", "front_left", "left")
STOP_REASON_SCHEMA = REPO_ROOT / "docs/contracts/rc-car-modules/stop-reasons.json"


def sector_for_angle(angle_rad: float, boundaries_rad: tuple[float, ...] | list[float]) -> str:
    """Map scan bearing to planner sector using its bisect_right convention."""
    if len(boundaries_rad) != len(SECTOR_NAMES) + 1:
        raise ValueError("five sectors require exactly six configured boundaries")
    if any(not math.isfinite(float(value)) for value in boundaries_rad):
        raise ValueError("sector boundaries must be finite")
    if any(right <= left for left, right in zip(boundaries_rad, boundaries_rad[1:])):
        raise ValueError("sector boundaries must be strictly increasing")
    if angle_rad < boundaries_rad[0] or angle_rad > boundaries_rad[-1]:
        raise ValueError("bearing is outside configured sector boundaries")
    index = min(len(SECTOR_NAMES) - 1, max(0, bisect_right(boundaries_rad[:-1], angle_rad) - 1))
    return SECTOR_NAMES[index]


def beam_for_angle(angle_rad: float, angle_min_rad: float, angle_increment_rad: float, sample_count: int) -> int:
    """Return the nearest scan beam; reject a bearing outside the scan."""
    if sample_count <= 0 or angle_increment_rad <= 0:
        raise ValueError("scan sample count and angular increment must be positive")
    angle_max = angle_min_rad + (sample_count - 1) * angle_increment_rad
    tolerance = abs(angle_increment_rad) * 1e-9
    if angle_rad < angle_min_rad - tolerance or angle_rad > angle_max + tolerance:
        raise ValueError("bearing is outside scan angle limits")
    return min(sample_count - 1, max(0, math.floor((angle_rad - angle_min_rad) / angle_increment_rad + 0.5)))


def nearest_range_in_sector(
    scan: Mapping[str, Any], *, center_angle: float, half_width_rad: float,
) -> float | None:
    """Return the nearest finite, in-range return around a target bearing."""
    ranges = scan["ranges"]
    minimum = float(scan["range_min"])
    maximum = float(scan["range_max"])
    angle_min = float(scan["angle_min"])
    increment = float(scan["angle_increment"])
    if half_width_rad < 0 or increment <= 0 or maximum < minimum:
        raise ValueError("invalid scan bounds or sector half-width")
    first = max(0, math.ceil((center_angle - half_width_rad - angle_min) / increment))
    last = min(len(ranges) - 1, math.floor((center_angle + half_width_rad - angle_min) / increment))
    found = []
    for index in range(first, last + 1):
        value = float(ranges[index])
        if math.isfinite(value) and minimum <= value <= maximum:
            found.append(value)
    return min(found) if found else None


def target_half_width_rad(scenario: Mapping[str, Any], *, range_m: float, angle_increment: float) -> float:
    """Return the angular half-width of the declared physical target footprint."""
    setup = scenario.get("setup", {})
    target_id = setup.get("target_entity")
    dimensions = setup.get("fixture_sizes_m", {}).get(target_id) if target_id else None
    if not dimensions or len(dimensions) < 2:
        return max(abs(angle_increment) * 1.5, 0.02)
    horizontal_radius = 0.5 * math.hypot(float(dimensions[0]), float(dimensions[1]))
    return math.atan2(horizontal_radius, max(float(range_m), 0.05)) + abs(angle_increment) / 2.0


def endpoint_observation(measured_m: float | None, *, endpoint_m: float, resolution_m: float) -> dict[str, Any]:
    """Classify a measured range relative to one configured endpoint."""
    if resolution_m <= 0 or not math.isfinite(endpoint_m) or not math.isfinite(resolution_m):
        raise ValueError("endpoint and resolution must be finite; resolution must be positive")
    if measured_m is None or not math.isfinite(float(measured_m)):
        return {"measured_m": None, "endpoint_m": endpoint_m, "classification": "unobserved"}
    measured = float(measured_m)
    if math.isclose(measured, endpoint_m, rel_tol=0.0, abs_tol=resolution_m / 2):
        classification = "boundary"
    elif measured < endpoint_m:
        classification = "below" if endpoint_m < 1.0 else "inside"
    else:
        classification = "beyond" if endpoint_m >= 1.0 else "inside"
    return {"measured_m": measured, "endpoint_m": endpoint_m, "classification": classification}


def stale_status(*, observed_age_s: float, max_age_s: float) -> dict[str, Any]:
    """Expose scan freshness using the tracked module-matrix stop-reason enum."""
    if not math.isfinite(observed_age_s) or observed_age_s < 0 or not math.isfinite(max_age_s) or max_age_s <= 0:
        raise ValueError("scan age and configured max age must be finite and valid")
    reasons = json.loads(STOP_REASON_SCHEMA.read_text(encoding="utf-8"))["enum"]
    reason = "sensor_stale" if observed_age_s > max_age_s else None
    if reason is not None and reason not in reasons:
        raise ValueError(f"stop reason {reason!r} is not in the module-matrix contract")
    return {"observed_age_s": observed_age_s, "max_age_s": max_age_s,
            "stale": observed_age_s > max_age_s, "stop_reason": reason}


def moving_target_order(samples: list[Mapping[str, float]]) -> list[dict[str, float]]:
    """Return range-rate observations sorted by simulation timestamp."""
    ordered = sorted(samples, key=lambda sample: float(sample["stamp_sim_s"]))
    result = []
    previous = None
    for sample in ordered:
        current = {"stamp_sim_s": float(sample["stamp_sim_s"]), "range_m": float(sample["range_m"])}
        if not math.isfinite(current["stamp_sim_s"]) or not math.isfinite(current["range_m"]):
            raise ValueError("moving-target samples must be finite")
        if previous is not None:
            dt = current["stamp_sim_s"] - previous["stamp_sim_s"]
            if dt <= 0:
                raise ValueError("moving-target simulation stamps must be strictly increasing")
            current["range_rate_mps"] = (current["range_m"] - previous["range_m"]) / dt
        previous = current
        result.append(current)
    return result


def evaluate_scan_case(scenario: Mapping[str, Any], scans: list[Mapping[str, Any]]) -> tuple[str, list[dict[str, Any]], str | None]:
    """Evaluate one captured LiDAR case against its declared geometry/tolerance."""
    if not scans:
        return "unobserved", [], "no LaserScan samples were captured"
    setup = scenario["setup"]
    measurement = scenario["measurements"][0]
    tolerance = scenario["tolerances"][0]
    scan = scans[-1]
    bearing = float(setup.get("target_bearing_rad", 0.0))
    center_beam = beam_for_angle(bearing, scan["angle_min"], scan["angle_increment"], len(scan["ranges"]))
    frame = scan.get("frame_id")
    expected = setup.get("target_surface_range_m")
    beam = center_beam
    measured = None
    if expected is not None and not setup.get("ground_truth_only"):
        half_width = target_half_width_rad(
            scenario, range_m=float(expected), angle_increment=float(scan["angle_increment"]),
        )
        first = max(0, math.ceil((bearing - half_width - float(scan["angle_min"])) / float(scan["angle_increment"])))
        last = min(len(scan["ranges"]) - 1,
                   math.floor((bearing + half_width - float(scan["angle_min"])) / float(scan["angle_increment"])))
        candidates = [
            (index, float(scan["ranges"][index]))
            for index in range(first, last + 1)
            if math.isfinite(float(scan["ranges"][index]))
            and float(scan["range_min"]) <= float(scan["ranges"][index]) <= float(scan["range_max"])
        ]
        if candidates:
            beam, measured = min(candidates, key=lambda item: abs(item[1] - float(expected)))
    else:
        value = float(scan["ranges"][center_beam])
        if math.isfinite(value) and float(scan["range_min"]) <= value <= float(scan["range_max"]):
            measured = value
    reason = None
    if setup.get("ground_truth_only"):
        valid = sum(
            math.isfinite(float(item)) and float(scan["range_min"]) <= float(item) <= float(scan["range_max"])
            for sample in scans for item in sample["ranges"]
        )
        passed = valid == 0
        observation_value = {"valid_returns": valid}
        ground_truth: Any = {"valid_returns": 0}
        unit = "count"
    elif expected is None:
        reason = "target surface range is missing"
        passed = False
        observation_value = measured
        ground_truth = {"expected_no_return": True}
        unit = "m"
    else:
        profile = yaml.safe_load((REPO_ROOT / "ros2_ws/src/car_bringup/config/rc_vehicle.yaml").read_text())
        endpoint_slop = float(profile["presets"]["rc_car"]["sensors"]["lidar"]["range_resolution_m"]["value"]) / 2.0
    if not setup.get("ground_truth_only") and expected is not None and (
        float(expected) < float(scan["range_min"]) - endpoint_slop
        or float(expected) > float(scan["range_max"]) + endpoint_slop
    ):
        passed = measured is None
        observation_value = measured
        ground_truth = {"expected_no_return": True, "target_surface_range_m": expected}
        unit = "m"
        if not passed:
            reason = f"an out-of-range fixture produced a return at {measured} m"
    elif not setup.get("ground_truth_only") and expected is not None:
        allowed_error = float(tolerance.get("absolute_error", 0.02))
        passed = measured is not None and abs(float(measured) - float(expected)) <= allowed_error
        observation_value = measured
        ground_truth = {"target_surface_range_m": expected, "beam_index": center_beam}
        if measured is not None:
            ground_truth["observed_beam_index"] = beam
        unit = "m"
        if measured is None:
            reason = "the declared target produced no finite in-range return"
        elif not passed:
            reason = f"range error {abs(float(measured) - float(expected)):.4f} m exceeds {allowed_error:.4f} m"
    result = {
        "observation_id": f"{scenario['scenario_id']}/range",
        "source": "/scan",
        "stamp_sim_ns": int(scan["stamp_sim_ns"]),
        "frame_id": frame,
        "value": observation_value,
        "ground_truth": ground_truth,
        "unit": unit,
        "tolerance_id": tolerance["tolerance_id"],
        "error": None if passed else reason,
        "result": "pass" if passed else "fail",
        "evidence_ref": None,
    }
    if not setup.get("ground_truth_only") and frame != "lidar_link":
        result["result"] = "fail"
        passed = False
        reason = f"unexpected scan frame_id {frame!r}; expected lidar_link"
        result["error"] = reason
    if setup.get("expected_sector"):
        profile = yaml.safe_load((REPO_ROOT / "ros2_ws/src/car_bringup/config/rc_vehicle.yaml").read_text())
        bounds = profile["presets"]["rc_car"]["navigation"]["sector_boundaries_rad"]["value"]
        observed_bearing = (
            float(scan["angle_min"]) + beam * float(scan["angle_increment"])
            if measured is not None else bearing
        )
        sector = sector_for_angle(observed_bearing, bounds)
        sector_result = sector == setup["expected_sector"]
        result["ground_truth"]["expected_sector"] = setup["expected_sector"]
        result["value"] = {"range_m": observation_value, "beam_index": beam, "sector": sector}
        if not sector_result:
            passed = False
            reason = f"bearing mapped to {sector}, expected {setup['expected_sector']}"
            result["error"] = reason
            result["result"] = "fail"
    return ("pass" if passed else "fail"), [result], reason


def run_lidar_case(scenario: dict[str, Any], *, output_dir: Path) -> dict[str, Any]:
    """Run one captured sensor case and score only observed range/status data."""
    setup = scenario["setup"]
    if "stale_after_s" in setup:
        return run_lidar_stale_case(scenario, output_dir=output_dir)
    try:
        from tools.checks.runtime_capture import capture_lidar
        capture = capture_lidar(scenario, output_dir=output_dir)
    except Exception as exc:
        return {"status": "blocked", "observations": [], "evidence_refs": [], "reason": str(exc)}
    output_dir.mkdir(parents=True, exist_ok=True)
    evidence_path = output_dir / "lidar-capture.json"
    evidence_path.write_text(json.dumps(capture, allow_nan=True, indent=2) + "\n", encoding="utf-8")
    if setup.get("moving_target_order"):
        status, observations, reason = evaluate_moving_case(scenario, capture)
    else:
        status, observations, reason = evaluate_scan_case(scenario, capture["scans"])
    for observation in observations:
        observation["evidence_ref"] = str(evidence_path.relative_to(REPO_ROOT))
    evidence_refs = [str(evidence_path.relative_to(REPO_ROOT)), capture["log_ref"]]
    if capture.get("render_configuration_ref"):
        evidence_refs.append(capture["render_configuration_ref"])
    return {
        "status": status, "observations": observations,
        "evidence_refs": evidence_refs,
        "reason": reason,
    }


def evaluate_moving_case(
    scenario: Mapping[str, Any], capture: Mapping[str, Any],
) -> tuple[str, list[dict[str, Any]], str | None]:
    """Require a live research target to close then recede in measured scan data."""
    scans = list(capture.get("scans", []))
    start_ns = int(capture.get("clock_start_sim_ns") or 0)
    setup = scenario["setup"]
    finite_samples = []
    for scan in scans:
        half_width = target_half_width_rad(
            scenario,
            range_m=float(setup.get("target_surface_range_m", 3.0)),
            angle_increment=float(scan["angle_increment"]),
        )
        value = nearest_range_in_sector(
            scan,
            center_angle=float(setup.get("target_bearing_rad", 0.0)),
            half_width_rad=half_width,
        )
        if value is not None:
            finite_samples.append({
                "stamp_sim_s": (int(scan["stamp_sim_ns"]) - start_ns) / 1e9,
                "range_m": value,
            })
    ordered = moving_target_order(finite_samples) if finite_samples else []
    closing = [row["range_rate_mps"] for row in ordered
               if "range_rate_mps" in row and 0.15 <= row["stamp_sim_s"] <= 1.0]
    # Allow the Gazebo pose service and the 10 Hz scan stream one half-second
    # after the turnaround to settle before scoring the receding phase.
    receding = [row["range_rate_mps"] for row in ordered
                if "range_rate_mps" in row and 1.5 <= row["stamp_sim_s"] <= 2.5]
    pose_commands = list(capture.get("pose_commands", []))
    pose_ok = len(pose_commands) >= 8 and all(row["service_success"] for row in pose_commands)
    closing_ok = len(closing) >= 2 and sum(closing) / len(closing) < -0.03
    receding_ok = len(receding) >= 2 and sum(receding) / len(receding) > 0.03
    passed = pose_ok and closing_ok and receding_ok
    reason = None
    if not pose_ok:
        reason = "Gazebo did not acknowledge the fixed-seed moving-target pose sequence"
    elif not finite_samples:
        reason = "the moving research target produced no finite in-range LiDAR return"
    elif not closing_ok or not receding_ok:
        reason = "measured LiDAR range did not close then recede across the configured motion phases"
    observation = {
        "observation_id": f"{scenario['scenario_id']}/moving-range-order",
        "source": "/scan and Gazebo set_pose service",
        "stamp_sim_ns": int(scans[-1]["stamp_sim_ns"] if scans else capture.get("clock_last_sim_ns", 0)),
        "frame_id": "lidar_link",
        "value": {
            "finite_target_returns": len(finite_samples),
            "range_samples": ordered,
            "closing_range_rates_mps": closing,
            "receding_range_rates_mps": receding,
            "pose_update_count": len(pose_commands),
            "pose_updates_acknowledged": pose_ok,
            "range_decreased_before_turnaround": closing_ok,
            "range_increased_after_turnaround": receding_ok,
        },
        "ground_truth": {
            "motion_profile": scenario["obstacle_placements"][
                next(i for i, item in enumerate(scenario["obstacle_placements"])
                     if item["entity_id"] == setup["target_entity"])
            ]["speed_profile"],
            "expected_range_rate_signs": {"closing": "negative", "receding": "positive"},
        },
        "unit": "m,m/s",
        "tolerance_id": scenario["tolerances"][0]["tolerance_id"],
        "error": reason,
        "result": "pass" if passed else "fail",
        "evidence_ref": None,
    }
    return ("pass" if passed else "fail"), [observation], reason


def run_lidar_stale_case(scenario: dict[str, Any], *, output_dir: Path) -> dict[str, Any]:
    """Drop only the controller's scan input and require its live stale-stop status."""
    try:
        from tools.checks.runtime_capture import capture_command_path
        from car_autonomy_nodes.safe_drive_controller import SENSOR_MAX_AGE_NS

        capture = capture_command_path(scenario, output_dir=output_dir)
        statuses = capture["statuses"]
        drop_events = [row for row in capture.get("fault_states", []) if row["state"].startswith("dropped")]
        threshold_s = SENSOR_MAX_AGE_NS / 1e9
        stale_rows = [row for row in statuses
                      if row["values"].get("sensor_lidar_state") == "stale"
                      and row["values"].get("readiness_blocker") == "sensor_lidar_stale"]
        ages = [float(row["values"]["sensor_lidar_age_s"]) for row in stale_rows]
        first_age = min(ages, default=None)
        fresh_before_drop = any(
            row["values"].get("sensor_lidar_state") == "fresh"
            for row in statuses if not drop_events or row["stamp_sim_ns"] < drop_events[0]["stamp_sim_ns"]
        )
        repeated_commands = len(capture["commands"]) >= int(scenario["setup"].get("minimum_commands_before_stop", 3))
        relayed_stamps = capture.get("relayed_scan_stamps_sim_ns", [])
        scans_forwarded_before_drop = bool(drop_events) and any(
            int(stamp) < int(drop_events[0]["stamp_sim_ns"]) for stamp in relayed_stamps
        )
        zero_after_stale = any(
            int(row["stamp_sim_ns"]) >= int(stale_rows[0]["stamp_sim_ns"])
            and abs(float(row["linear_x_mps"])) <= 1e-9
            and abs(float(row["angular_z_radps"])) <= 1e-9
            for row in capture.get("controller_twists", [])
        ) if stale_rows else False
        stale_stop_semantics = bool(stale_rows) and all(
            row["values"].get("stop_reason") == "controller_not_ready"
            and row["values"].get("controller_ready") == "false"
            for row in stale_rows
        )
        post_stale_commands = [
            row for row in capture["commands"]
            if stale_rows and int(row["stamp_sim_ns"]) >= int(stale_rows[0]["stamp_sim_ns"])
        ]
        repeated_after_stale = len(post_stale_commands) >= int(
            scenario["setup"].get("minimum_commands_after_stale", 2)
        )
        passed = (
            bool(drop_events) and scans_forwarded_before_drop and fresh_before_drop
            and repeated_commands and first_age is not None and zero_after_stale
            and stale_stop_semantics and repeated_after_stale
            and threshold_s <= first_age <= threshold_s + float(scenario["setup"].get("status_period_tolerance_s", 0.15))
        )
        reason = None if passed else "live controller did not report sensor_stale at the configured freshness boundary"
        observation = {
            "observation_id": f"{scenario['scenario_id']}/sensor-stale-stop",
            "source": "/vehicle_controller/status",
            "stamp_sim_ns": int(stale_rows[0]["stamp_sim_ns"] if stale_rows else capture["clock_last_sim_ns"]),
            "frame_id": "base_link",
            "value": {
                "relay_drop_sim_ns": drop_events[0]["stamp_sim_ns"] if drop_events else None,
                "stale_stop_observed": bool(stale_rows),
                "stop_reason": stale_rows[0]["values"].get("stop_reason") if stale_rows else None,
                "readiness_blocker": stale_rows[0]["values"].get("readiness_blocker") if stale_rows else None,
                "stop_reason_matches_existing_controller_contract": stale_stop_semantics,
                "zero_controller_output_after_stale": zero_after_stale,
                "observed_scan_age_sim_s": first_age,
                "command_count": len(capture["commands"]),
                "command_count_after_stale": len(post_stale_commands),
                "fresh_sensor_observed_before_drop": fresh_before_drop,
                "relayed_scan_count": len(relayed_stamps),
                "scans_forwarded_before_drop": scans_forwarded_before_drop,
            },
            "ground_truth": {
                "maximum_sensor_age_sim_s": threshold_s,
                "expected_readiness_blocker": "sensor_lidar_stale",
                "expected_stop_reason": "controller_not_ready",
                "status_period_tolerance_sim_s": scenario["setup"].get("status_period_tolerance_s", 0.15),
            },
            "unit": "s",
            "tolerance_id": scenario["tolerances"][0]["tolerance_id"],
            "error": reason,
            "result": "pass" if passed else "fail",
            "evidence_ref": capture["evidence_ref"],
        }
        return {
            "status": "pass" if passed else "fail",
            "observations": [observation],
            "evidence_refs": [capture["evidence_ref"], capture["log_ref"]],
            "reason": reason,
        }
    except Exception as exc:
        return {"status": "blocked", "observations": [], "evidence_refs": [], "reason": str(exc)}
