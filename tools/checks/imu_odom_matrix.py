"""Unscored simulator-only IMU and odometry baseline capture."""
from __future__ import annotations

import math
from pathlib import Path
import statistics
from typing import Any

from tools.rc_module_matrix import SIMULATION_ONLY_STATEMENT

REPO_ROOT = Path(__file__).resolve().parents[2]


def _axis_summary(rows: list[dict[str, Any]], key: str, labels: tuple[str, ...]) -> dict[str, Any]:
    values = [row[key] for row in rows]
    return {
        label: {
            "mean": statistics.fmean(row[index] for row in values),
            "population_stddev": statistics.pstdev(row[index] for row in values),
            "minimum": min(row[index] for row in values),
            "maximum": max(row[index] for row in values),
        }
        for index, label in enumerate(labels)
    }


def _measured_rate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    stamps = [int(row["stamp_sim_ns"]) for row in rows]
    intervals = [(right - left) / 1e9 for left, right in zip(stamps, stamps[1:])]
    if not intervals or any(interval <= 0 for interval in intervals):
        return {"sample_count": len(rows), "rate_hz": None, "median_period_s": None}
    return {
        "sample_count": len(rows),
        "rate_hz": (len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9),
        "median_period_s": statistics.median(intervals),
        "minimum_period_s": min(intervals),
        "maximum_period_s": max(intervals),
    }


def _yaw(quaternion: list[float]) -> float:
    x, y, z, w = map(float, quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def run_imu_odom_pilot(scenario: dict[str, Any], *, output_dir: Path) -> dict[str, Any]:
    """Record at-rest sensor statistics without deriving or applying a tolerance."""
    try:
        from tools.checks.runtime_capture import capture_imu_odom

        capture = capture_imu_odom(scenario, output_dir=output_dir)
        imu = capture["imu_samples"]
        odom = capture["odometry_samples"]
        first, last = odom[0], odom[-1]
        position_delta = [last["position_m"][i] - first["position_m"][i] for i in range(3)]
        yaw_delta = _yaw(last["orientation_xyzw"]) - _yaw(first["orientation_xyzw"])
        duration_s = max(0.0, (capture["clock_last_sim_ns"] - capture["clock_start_sim_ns"]) / 1e9)
        value = {
            "duration_sim_s": duration_s,
            "imu": {
                "frame_ids": sorted({row["frame_id"] for row in imu}),
                "measured_rate": _measured_rate(imu),
                "angular_velocity_radps": _axis_summary(
                    imu, "angular_velocity_radps", ("x", "y", "z"),
                ),
                "linear_acceleration_mps2": _axis_summary(
                    imu, "linear_acceleration_mps2", ("x", "y", "z"),
                ),
            },
            "odometry": {
                "frame_pairs": sorted({(row["frame_id"], row["child_frame_id"]) for row in odom}),
                "measured_rate": _measured_rate(odom),
                "position_delta_m": position_delta,
                "yaw_delta_rad": yaw_delta,
            },
            "interpretation": SIMULATION_ONLY_STATEMENT,
        }
        observation = {
            "observation_id": f"{scenario['scenario_id']}/unscored-baseline",
            "source": "/imu/data and /odom",
            "stamp_sim_ns": int(imu[-1]["stamp_sim_ns"]),
            "frame_id": imu[-1]["frame_id"],
            "value": value,
            "ground_truth": {
                "initial_vehicle_pose": scenario["start_pose"],
                "commanded_speed_mps": 0.0,
                "scoring": "not_applicable; pilot measures the configured simulator model only",
            },
            "unit": "mixed",
            "tolerance_id": None,
            "result": "not_applicable",
            "evidence_ref": capture["evidence_ref"],
        }
        return {
            "status": "unscored",
            "observations": [observation],
            "evidence_refs": [capture["evidence_ref"], capture["log_ref"]],
            "reason": "unscored simulator-only baseline; tolerances are not frozen",
        }
    except Exception as exc:
        return {"status": "blocked", "observations": [], "evidence_refs": [], "reason": str(exc)}


def run_imu_odom_case(scenario: dict[str, Any], *, output_dir: Path) -> dict[str, Any]:
    """Score stamped IMU/odometry data against the sensor model and command profile."""
    try:
        from tools.checks.runtime_capture import capture_imu_odom

        capture = capture_imu_odom(scenario, output_dir=output_dir)
        imu = capture["imu_samples"]
        odom = capture["odometry_samples"]
        if len(imu) < 2 or len(odom) < 2:
            raise RuntimeError("qualification capture needs multiple IMU and odometry samples")
        setup = scenario["setup"]
        profile = scenario["vehicle_motion"]["speed_profile"]
        mode = setup["motion_mode"]
        duration = float(setup["capture_duration_sim_s"])
        yaw_rate = float(setup.get("yaw_rate_rps", 0.0))
        position_delta = [odom[-1]["position_m"][i] - odom[0]["position_m"][i] for i in range(3)]
        yaw_delta = math.atan2(
            math.sin(_yaw(odom[-1]["orientation_xyzw"]) - _yaw(odom[0]["orientation_xyzw"])),
            math.cos(_yaw(odom[-1]["orientation_xyzw"]) - _yaw(odom[0]["orientation_xyzw"])),
        )
        actual_duration = max(0.0, (capture["clock_last_sim_ns"] - capture["clock_start_sim_ns"]) / 1e9)
        imustamps = [int(row["stamp_sim_ns"]) for row in imu]
        odomstamps = [int(row["stamp_sim_ns"]) for row in odom]
        def one_clock_epoch(stamps: list[int]) -> bool:
            return bool(stamps) and all(b > a for a, b in zip(stamps, stamps[1:]))

        def monotonic_epochs(stamps: list[int]) -> tuple[bool, int]:
            resets = sum(right < left for left, right in zip(stamps, stamps[1:]))
            chunks: list[list[int]] = [[]]
            for stamp in stamps:
                if chunks[-1] and stamp < chunks[-1][-1]:
                    chunks.append([])
                chunks[-1].append(stamp)
            return resets == 1 and all(one_clock_epoch(chunk) for chunk in chunks), resets

        if mode == "pause_reset":
            monotonic, stamp_reset_count = monotonic_epochs(imustamps)
            odom_monotonic, odom_reset_count = monotonic_epochs(odomstamps)
            clock_monotonic, clock_reset_count = monotonic_epochs(capture["clock_samples_sim_ns"])
            monotonic = monotonic and odom_monotonic and clock_monotonic
        else:
            monotonic = one_clock_epoch(imustamps) and one_clock_epoch(odomstamps)
            stamp_reset_count = odom_reset_count = clock_reset_count = 0
        angular_peak = max(math.sqrt(sum(axis * axis for axis in row["angular_velocity_radps"])) for row in imu)
        acceleration_means = [statistics.fmean(row["linear_acceleration_mps2"][axis] for row in imu) for axis in range(3)]
        angular_limit = float(setup.get("rest_angular_velocity_bound_radps", 1e-5))
        gravity_limit = float(setup.get("rest_gravity_error_bound_mps2", 0.01))
        drift_limit = float(setup.get("rest_position_drift_bound_m", 0.001))
        yaw_drift_limit = float(setup.get("rest_yaw_drift_bound_rad", 0.001))
        observations = []

        def add(observation_id: str, measurement_id: str, value: Any, truth: Any, unit: str,
                passed: bool, detail: str | None = None) -> None:
            tolerance = next((item for item in scenario["tolerances"] if item["measurement_id"] == measurement_id), None)
            observations.append({
                "observation_id": f"{scenario['scenario_id']}/{observation_id}",
                "source": "/imu/data" if measurement_id.startswith("imu_") else "/odom",
                "stamp_sim_ns": int((imu if measurement_id.startswith("imu_") else odom)[-1]["stamp_sim_ns"]),
                "frame_id": "imu_link" if measurement_id.startswith("imu_") else "odom",
                "value": value,
                "ground_truth": {"expected": truth, "interpretation": SIMULATION_ONLY_STATEMENT},
                "unit": unit,
                "tolerance_id": tolerance["tolerance_id"] if tolerance else None,
                "error": detail,
                "result": "pass" if passed else "fail",
                "evidence_ref": capture["evidence_ref"],
            })

        if mode == "pause_reset":
            final_epoch = []
            for row in imu:
                if final_epoch and row["stamp_sim_ns"] < final_epoch[-1]["stamp_sim_ns"]:
                    final_epoch = []
                final_epoch.append(row)
            imu_rate = _measured_rate(final_epoch)["rate_hz"]
        else:
            imu_rate = _measured_rate(imu)["rate_hz"]
        expected_rate = float(scenario.get("expected_sensor_rates_hz", {}).get("/imu/data", 100.0))
        rate_error = None if imu_rate is None else abs(imu_rate - expected_rate) / expected_rate
        rate_pass = rate_error is not None and rate_error <= float(setup.get("imu_rate_relative_error_bound", 0.02))
        add("sim-time-stamps", "imu_timestamps", {"strictly_increasing": monotonic,
            "measured_rate_hz": imu_rate, "relative_rate_error": rate_error,
            "sample_count": len(imu)},
            {"strictly_increasing": True, "configured_rate_hz": expected_rate,
             "maximum_relative_rate_error": setup.get("imu_rate_relative_error_bound", 0.02)}, "Hz",
            monotonic and rate_pass,
            None if monotonic and rate_pass else "simulation-time IMU stamps or configured rate are outside bounds")

        if mode == "rest":
            angular_pass = angular_peak <= angular_limit
            gravity_error = abs(acceleration_means[2] - 9.8)
            gravity_pass = max(abs(acceleration_means[0]), abs(acceleration_means[1]), gravity_error) <= gravity_limit
            drift = math.sqrt(sum(value * value for value in position_delta))
            drift_pass = drift <= drift_limit
            yaw_pass = abs(yaw_delta) <= yaw_drift_limit
            add("at-rest-angular-noise", "imu_rest_noise", {"peak_magnitude_radps": angular_peak},
                {"maximum_radps": angular_limit}, "rad/s", angular_pass,
                None if angular_pass else f"angular-rate peak {angular_peak:.6g} exceeds {angular_limit:.6g}")
            add("at-rest-gravity", "imu_gravity", {"mean_acceleration_mps2": acceleration_means},
                {"expected_magnitude_mps2": 9.8, "maximum_axis_error_mps2": gravity_limit}, "m/s^2", gravity_pass,
                None if gravity_pass else "at-rest acceleration is outside the configured gravity tolerance")
            add("at-rest-odometry-drift", "odometry_drift", {"position_delta_m": position_delta, "norm_m": drift},
                {"maximum_norm_m": drift_limit, "maximum_yaw_delta_rad": yaw_drift_limit}, "m", drift_pass and yaw_pass,
                None if drift_pass and yaw_pass else "at-rest odometry drift exceeds the frozen simulation bound")
        else:
            expected_distance = sum(
                (float(a["speed_mps"]) + float(b["speed_mps"])) * 0.5
                * (float(b["sim_time_s"]) - float(a["sim_time_s"]))
                for a, b in zip(profile, profile[1:])
            )
            if mode == "straight":
                expected_yaw = 0.0
                distance_tolerance = float(setup["distance_error_bound_m"])
                error = abs(position_delta[0] - expected_distance)
                passed = monotonic and error <= distance_tolerance and abs(yaw_delta) <= float(setup["yaw_error_bound_rad"])
                truth = {"command_integral_m": expected_distance, "distance_error_bound_m": distance_tolerance,
                         "yaw_error_bound_rad": setup["yaw_error_bound_rad"]}
                add("straight-line-wheel-odometry", "odometry_motion",
                    {"position_delta_m": position_delta, "yaw_delta_rad": yaw_delta, "distance_error_m": error},
                    truth, "m,rad", passed,
                    None if passed else "straight-line odometry differs from the fixed command integral beyond tolerance")
            elif mode == "turn":
                expected_yaw = yaw_rate * duration
                yaw_error = abs(yaw_delta - expected_yaw)
                tolerance = float(setup["yaw_error_bound_rad"])
                passed = monotonic and yaw_error <= tolerance
                add("turning-bicycle-model-yaw", "odometry_motion",
                    {"yaw_delta_rad": yaw_delta, "expected_yaw_delta_rad": expected_yaw, "absolute_error_rad": yaw_error},
                    {"yaw_rate_rps": yaw_rate, "duration_sim_s": duration, "maximum_error_rad": tolerance},
                    "rad", passed,
                    None if passed else "turn yaw differs from the commanded bicycle-model yaw beyond tolerance")
            elif mode == "pause_reset":
                reset_pass = monotonic and clock_reset_count == 1 and stamp_reset_count == 1 and odom_reset_count == 1
                add("pause-clock-reset", "imu_timestamps", {
                        "gazebo_clock_reset_count": capture["clock_reset_count"],
                        "clock_timestamp_reset_count": clock_reset_count,
                        "imu_timestamp_reset_count": stamp_reset_count,
                        "odometry_timestamp_reset_count": odom_reset_count,
                        "imu_resumed": bool(imu), "odometry_resumed": bool(odom),
                    },
                    {"clock_pause_and_reset_count": 1, "sensor_timestamps_increasing_within_each_epoch": True},
                    "epoch-count", reset_pass,
                    None if reset_pass else "pause/reset did not produce one clean timestamp epoch transition for all sources")
            else:
                raise RuntimeError(f"unsupported IMU/odometry motion mode {mode!r}")

        passed = all(row["result"] == "pass" for row in observations)
        return {
            "status": "pass" if passed else "fail",
            "observations": observations,
            "evidence_refs": [capture["evidence_ref"], capture["log_ref"]],
            "reason": None if passed else next(row["error"] for row in observations if row["result"] == "fail"),
        }
    except Exception as exc:
        return {"status": "blocked", "observations": [], "evidence_refs": [], "reason": str(exc)}
