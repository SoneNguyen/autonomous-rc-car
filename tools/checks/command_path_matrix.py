"""Optional controller-bound, watchdog, and command-response qualification."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]


def _vehicle_bounds() -> dict[str, float]:
    profile = yaml.safe_load((REPO_ROOT / "ros2_ws/src/car_bringup/config/rc_vehicle.yaml").read_text())
    rc = profile["presets"]["rc_car"]
    return {
        "max_speed_mps": float(rc["steering"]["max_speed_mps"]["value"]),
        "max_steering_angle_rad": float(rc["steering"]["effective_center_wheel_limit_rad"]["value"]),
        "max_yaw_rate_radps": float(rc["steering"]["max_yaw_rate_radps"]["value"]),
        "wheelbase_m": float(rc["geometry"]["wheelbase_m"]["value"]),
    }


def run_command_path_case(scenario: dict[str, Any], *, output_dir: Path) -> dict[str, Any]:
    """Run one command through the single controller authority and score its output."""
    try:
        from tools.checks.runtime_capture import capture_command_path

        capture = capture_command_path(scenario, output_dir=output_dir)
        setup = scenario["setup"]
        bounds = _vehicle_bounds()
        statuses = capture["statuses"]
        odometry = capture["odometry"]
        command_ns = int(capture["command_sent_sim_ns"])
        usable = [row for row in statuses if row["stamp_sim_ns"] >= command_ns]
        kind = setup["command_test"]
        tolerance = scenario["tolerances"][0]
        value: dict[str, Any]
        truth: dict[str, Any]
        if kind == "bounds_and_response":
            ready = [row for row in usable if row["values"].get("controller_ready") == "true"]
            requested = [float(row["values"]["requested_speed_mps"]) for row in ready
                         if _finite_number(row["values"].get("requested_speed_mps"))]
            observed_speed = [abs(float(row["linear_speed_mps"])) for row in odometry]
            observed_yaw = [abs(float(row["yaw_rate_radps"])) for row in odometry]
            from car_autonomy_nodes.safe_drive_controller import ackermann_to_twist
            expected_output = ackermann_to_twist(
                float(setup["command_speed_mps"]), float(setup["command_steering_rad"]),
                wheelbase_m=bounds["wheelbase_m"],
                max_speed_mps=bounds["max_speed_mps"],
                max_steering_angle_rad=bounds["max_steering_angle_rad"],
                max_yaw_rate_radps=bounds["max_yaw_rate_radps"],
            )
            response = next((row for row in odometry
                             if row["stamp_sim_ns"] >= command_ns and abs(row["linear_speed_mps"]) >= 0.02), None)
            latency = None if response is None else (response["stamp_sim_ns"] - command_ns) / 1e9
            speed_ok = bool(requested) and max(requested) <= bounds["max_speed_mps"] + 1e-6
            clamp_match = bool(requested) and all(
                abs(value - expected_output.linear.x) <= 0.02 for value in requested if value > 0.0
            ) and expected_output.linear.x <= bounds["max_speed_mps"]
            motion_ok = bool(response) and latency is not None and latency <= float(setup["maximum_step_response_latency_s"])
            bounds_ok = max(observed_speed, default=0.0) <= bounds["max_speed_mps"] + 0.05
            yaw_ok = abs(float(expected_output.angular.z)) <= bounds["max_yaw_rate_radps"] + 1e-6
            passed = speed_ok and clamp_match and motion_ok and bounds_ok and yaw_ok
            value = {"controller_requested_speed_samples_mps": requested,
                     "expected_clamped_twist": {"linear_x_mps": float(expected_output.linear.x),
                                                 "angular_z_radps": float(expected_output.angular.z)},
                     "clamp_matches_controller_contract": clamp_match,
                     "maximum_observed_speed_mps": max(observed_speed, default=0.0),
                     "maximum_observed_yaw_rate_radps": max(observed_yaw, default=0.0),
                     "step_response_latency_sim_s": latency}
            truth = {**bounds, "maximum_step_response_latency_s": setup["maximum_step_response_latency_s"]}
            reason = None if passed else "clamped controller output or step-response latency is outside its frozen bounds"
        elif kind == "watchdog_before":
            threshold_s = _controller_lease_s()
            reasons = [row["values"].get("stop_reason", row["message"]) for row in usable]
            ages = [float(row["values"]["accepted_command_age_s"]) for row in usable
                    if _finite_number(row["values"].get("accepted_command_age_s"))]
            last_age = max(ages, default=-1.0)
            passed = last_age >= float(setup["minimum_observed_command_age_sim_s"]) and not any(
                reason == "safety_watchdog_stop" for reason in reasons
            ) and last_age < threshold_s
            value = {"last_observed_command_age_sim_s": last_age,
                     "watchdog_stop_observed": "safety_watchdog_stop" in reasons,
                     "observed_stop_reasons": reasons}
            truth = {"controller_lease_threshold_sim_s": threshold_s,
                     "expected_stop_before_threshold": False}
            reason = None if passed else "controller stopped before the lease threshold or the run did not bracket it"
        elif kind == "watchdog_after":
            threshold_s = _controller_lease_s()
            watchdog_rows = [row for row in usable
                             if row["values"].get("stop_reason", row["message"]) == "safety_watchdog_stop"]
            ages = [float(row["values"]["accepted_command_age_s"]) for row in watchdog_rows
                    if _finite_number(row["values"].get("accepted_command_age_s"))]
            onset = min(ages, default=None)
            passed = onset is not None and threshold_s <= onset <= threshold_s + float(setup["watchdog_observation_tolerance_sim_s"])
            value = {"watchdog_stop_observed": bool(watchdog_rows),
                     "first_reported_stop_age_sim_s": onset,
                     "stop_reason": "safety_watchdog_stop" if watchdog_rows else None}
            truth = {"controller_lease_threshold_sim_s": threshold_s,
                     "observation_tolerance_sim_s": setup["watchdog_observation_tolerance_sim_s"]}
            reason = None if passed else "controller watchdog stop did not occur within the observed lease interval"
        else:
            raise RuntimeError(f"unsupported command-path test {kind!r}")
        observation = {
            "observation_id": f"{scenario['scenario_id']}/{kind}",
            "source": "/vehicle_controller/status and /odom",
            "stamp_sim_ns": int(usable[-1]["stamp_sim_ns"] if usable else capture["clock_last_sim_ns"]),
            "frame_id": "base_link",
            "value": value,
            "ground_truth": truth,
            "unit": "m/s,rad/s,s",
            "tolerance_id": tolerance["tolerance_id"],
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


def _finite_number(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _controller_lease_s() -> float:
    try:
        from car_autonomy_nodes.safe_drive_controller import COMMAND_LEASE_NS
    except ImportError as exc:
        raise RuntimeError("source ROS 2 and build car_autonomy_nodes to read the configured command lease") from exc
    return float(COMMAND_LEASE_NS) / 1e9
