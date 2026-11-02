#!/usr/bin/env python3
"""Measure RC simulation braking through the existing safe-drive-controller path.

The tracked brake-probe scenario is the only scenario input. The tool makes an
ephemeral, obstacle-free derivative of its pinned source world, starts the
existing Gazebo bridge and controller, and records simulation-time odometry.
The resulting numbers describe this Gazebo setup, not physical RC hardware.
"""

from __future__ import annotations

import argparse
import atexit
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import yaml


REPO = Path(__file__).resolve().parents[1]
PROFILE_SOURCE = Path("ros2_ws/src/car_bringup/config/rc_vehicle.yaml")
MODEL_SOURCE = Path("simulation/models/rc_car/model.sdf")
CONTROLLER_SOURCE = Path(
    "ros2_ws/src/car_autonomy_nodes/car_autonomy_nodes/safe_drive_controller.py"
)
BRIDGE_CONFIG = Path("ros2_ws/src/car_bringup/config/obstacle_course/bridge.yaml")
RC_MODEL_PATH = Path("simulation/models/rc_car")
ASSET_MODELS_PATH = Path("artifacts/obstacle-course/assets/models")


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def stopping_distance_m(speed_mps: float, braking_deceleration_mps2: float,
                        latency_s: float) -> float:
    """Return ``v²/(2a) + v*t`` after validating physical units/ranges."""
    values = (speed_mps, braking_deceleration_mps2, latency_s)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("speed, braking deceleration, and latency must be finite")
    if speed_mps < 0 or braking_deceleration_mps2 <= 0 or latency_s < 0:
        raise ValueError("speed/latency must be nonnegative and deceleration positive")
    return speed_mps * speed_mps / (2.0 * braking_deceleration_mps2) + speed_mps * latency_s


def _fit_positive_deceleration(samples: list[dict], onset_delta_mps: float,
                               stop_speed_threshold_mps: float) -> float:
    """Fit speed against simulation time from first deceleration to first stop."""
    request_s = float(samples[0]["sim_time_s"])
    initial_speed = float(samples[0]["speed_mps"])
    onset = next(
        (i for i, sample in enumerate(samples)
         if i > 0 and initial_speed - float(sample["speed_mps"]) >= onset_delta_mps),
        None,
    )
    if onset is None:
        raise ValueError("no measurable deceleration onset")
    stop = next(
        (i for i in range(onset, len(samples))
         if float(samples[i]["speed_mps"]) <= stop_speed_threshold_mps),
        None,
    )
    if stop is None or stop - onset < 2:
        raise ValueError("not enough samples between deceleration and stop")
    points = samples[onset:stop + 1]
    times = [float(sample["sim_time_s"]) - request_s for sample in points]
    speeds = [max(0.0, float(sample["speed_mps"])) for sample in points]
    mean_t = statistics.fmean(times)
    mean_v = statistics.fmean(speeds)
    denominator = sum((value - mean_t) ** 2 for value in times)
    if denominator <= 1e-12:
        raise ValueError("insufficient time spread to fit deceleration")
    slope = sum((t - mean_t) * (v - mean_v) for t, v in zip(times, speeds)) / denominator
    deceleration = -slope
    if not math.isfinite(deceleration) or deceleration <= 0:
        raise ValueError("measured braking deceleration is not positive")
    return deceleration


def _validate_trial(trial: dict, *, stop_speed_threshold_mps: float,
                    onset_delta_mps: float, maximum_gap_s: float,
                    minimum_samples: int,
                    maximum_distance_m: float | None = None) -> dict:
    if trial.get("accepted") is not True:
        raise ValueError(f"trial {trial.get('trial_index')} was not accepted by controller")
    samples = trial.get("samples")
    if not isinstance(samples, list) or len(samples) < minimum_samples:
        raise ValueError(f"trial {trial.get('trial_index')} has too few samples")
    times: list[float] = []
    speeds: list[float] = []
    distances: list[float] = []
    for sample in samples:
        try:
            sim_time = float(sample["sim_time_s"])
            speed = float(sample["speed_mps"])
            distance = float(sample["distance_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("trial sample is missing numeric simulation fields") from exc
        if not all(math.isfinite(value) for value in (sim_time, speed, distance)):
            raise ValueError("trial sample contains a non-finite value")
        if speed < -0.03 or distance < -0.02:
            raise ValueError("trial sample has invalid reverse speed or distance")
        times.append(sim_time)
        speeds.append(max(0.0, speed))
        distances.append(max(0.0, distance))
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("trial sample simulation times are not strictly increasing")
    gaps = [b - a for a, b in zip(times, times[1:])]
    if gaps and max(gaps) > maximum_gap_s:
        raise ValueError(f"trial sample gap {max(gaps):.3f}s exceeds {maximum_gap_s:.3f}s")
    initial_speed = float(trial.get("speed_before_brake_mps", speeds[0]))
    onset_index = next(
        (i for i, speed in enumerate(speeds)
         if initial_speed - speed >= onset_delta_mps),
        None,
    )
    if onset_index is None:
        raise ValueError(f"trial {trial.get('trial_index')} has no deceleration onset")
    latency_s = times[onset_index] - float(trial["brake_request_sim_time_s"])
    if latency_s < 0:
        raise ValueError("measured deceleration precedes the brake request")
    if not any(speed <= stop_speed_threshold_mps for speed in speeds[onset_index:]):
        raise ValueError(f"trial {trial.get('trial_index')} did not reach the stop threshold")
    if maximum_distance_m is not None and max(distances) > maximum_distance_m:
        raise ValueError(
            f"trial {trial.get('trial_index')} traveled {max(distances):.3f}m, "
            f"above the configured {maximum_distance_m:.3f}m probe limit"
        )
    deceleration = _fit_positive_deceleration(
        [{**sample, "speed_mps": speed} for sample, speed in zip(samples, speeds)],
        onset_delta_mps,
        stop_speed_threshold_mps,
    )
    trial["measured_deceleration_mps2"] = deceleration
    trial["measured_latency_s"] = latency_s
    trial["stop_distance_m"] = distances[-1]
    return trial


def aggregate_trials(trials: list[dict], *, expected_repeats: int,
                     expected_source_hashes: dict[str, str],
                     expected_simulator_identity: str,
                     stop_speed_threshold_mps: float, onset_delta_mps: float,
                     maximum_gap_s: float, minimum_samples: int,
                     maximum_distance_m: float | None = None) -> dict:
    """Validate all repeat provenance and produce conservative measurement bounds."""
    if len(trials) != expected_repeats:
        raise ValueError(f"expected {expected_repeats} trials, got {len(trials)}")
    validated = []
    for trial in trials:
        if trial.get("source_hashes") != expected_source_hashes:
            raise ValueError(f"trial {trial.get('trial_index')} source hash mismatch")
        if trial.get("simulator_identity") != expected_simulator_identity:
            raise ValueError(f"trial {trial.get('trial_index')} simulator identity mismatch")
        validated.append(_validate_trial(
            trial,
            stop_speed_threshold_mps=stop_speed_threshold_mps,
            onset_delta_mps=onset_delta_mps,
            maximum_gap_s=maximum_gap_s,
            minimum_samples=minimum_samples,
            maximum_distance_m=maximum_distance_m,
        ))
    decelerations = [trial["measured_deceleration_mps2"] for trial in validated]
    latencies = [trial["measured_latency_s"] for trial in validated]
    stops = [trial["stop_distance_m"] for trial in validated]
    return {
        "accepted_trial_count": len(validated),
        "braking_deceleration_mps2": statistics.median(decelerations),
        "conservative_braking_deceleration_mps2": min(decelerations),
        "request_to_deceleration_latency_s": statistics.median(latencies),
        "conservative_request_to_deceleration_latency_s": max(latencies),
        "stop_distance_median_m": statistics.median(stops),
        "stop_distance_max_m": max(stops),
        "per_trial_deceleration_mps2": decelerations,
        "per_trial_latency_s": latencies,
        "per_trial_stop_distance_m": stops,
        "trials": validated,
    }


def _generate_probe_world(config: dict, output: Path) -> None:
    source = REPO / config["world"]["source_path"]
    if sha256_file(source) != config["world"]["source_sha256"]:
        raise RuntimeError("protected probe source world hash does not match the tracked config")
    tree = ET.parse(source)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("probe source SDF has no world element")
    vehicle = config["vehicle"]
    pose = vehicle["initial_pose"]
    for include in list(world.findall("include")):
        name = include.findtext("name", "")
        if name != vehicle["entity"]:
            world.remove(include)
            continue
        include_pose = include.find("pose")
        if include_pose is None:
            include_pose = ET.SubElement(include, "pose")
        include_pose.text = (
            f"{pose['x_m']} {pose['y_m']} {pose['z_m']} 0 0 {pose['yaw_rad']}"
        )
    # The obstacle-course road references optional research textures. The brake
    # probe needs no visual asset, so remove material PBR maps in this generated
    # derivative while leaving all protected source files untouched.
    for pbr in list(root.iter("pbr")):
        parent = next((node for node in root.iter() if pbr in list(node)), None)
        if parent is not None:
            parent.remove(pbr)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)


def _version(command: list[str]) -> str:
    try:
        result = subprocess.run(command, check=False, capture_output=True, text=True, timeout=8)
    except (OSError, subprocess.TimeoutExpired):
        return "unavailable"
    return (result.stdout + result.stderr).strip().splitlines()[0] if (result.stdout or result.stderr) else "unavailable"


def _pinned_runtime_identity(seed: int) -> str:
    ros_distro = os.environ.get("ROS_DISTRO", "")
    bridge_version = ""
    try:
        completed = subprocess.run(
            ["ros2", "pkg", "xml", "ros_gz_bridge"], check=True,
            capture_output=True, text=True, timeout=8,
        )
        match = re.search(r"<version>([^<]+)</version>", completed.stdout)
        bridge_version = match.group(1).strip() if match else ""
    except (OSError, subprocess.SubprocessError):
        pass
    gazebo_version = _version(["gz", "sim", "--versions"])
    if ros_distro != "lyrical" or bridge_version != "3.0.10" or "10.5.0" not in gazebo_version:
        raise RuntimeError(
            "runtime pin mismatch: expected ROS lyrical / ros_gz_bridge 3.0.10 / "
            f"Gazebo Sim 10.5.0, observed ROS={ros_distro or 'unset'}, "
            f"bridge={bridge_version or 'unavailable'}, Gazebo={gazebo_version}"
        )
    return (
        f"Gazebo Sim 10.5.0; ROS 2 {ros_distro}; ros_gz_bridge {bridge_version}; "
        f"physics=source-world; seed={seed}"
    )


def _run_simulation(config: dict, profile_path: Path, scenario_path: Path,
                    output_path: Path, repeats: int) -> dict:
    # Imported only after verifying the environment was sourced.
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from diagnostic_msgs.msg import DiagnosticStatus
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data

    yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    rc = yaml.safe_load((REPO / PROFILE_SOURCE).read_text(encoding="utf-8"))["presets"]["rc_car"]
    profile_hash = sha256_file(profile_path)
    scenario_hash = sha256_file(scenario_path)
    source_hashes = {
        "profile": profile_hash,
        "scenario": scenario_hash,
        "vehicle_profile": sha256_file(REPO / PROFILE_SOURCE),
        "vehicle_model": sha256_file(REPO / MODEL_SOURCE),
        "controller": sha256_file(REPO / CONTROLLER_SOURCE),
        "source_world": sha256_file(REPO / config["world"]["source_path"]),
    }
    sim_identity = _pinned_runtime_identity(int(config["seed"]))
    world_output = REPO / config["world"]["generated_output"]
    _generate_probe_world(config, world_output)
    generated_world_hash = sha256_file(world_output)
    logs = output_path.parent / "process-logs"
    logs.mkdir(parents=True, exist_ok=True)
    child_env = os.environ.copy()
    search_paths = [str(REPO / RC_MODEL_PATH)]
    asset_models = REPO / ASSET_MODELS_PATH
    if asset_models.is_dir():
        search_paths.insert(0, str(asset_models))
    inherited = child_env.get("GZ_SIM_RESOURCE_PATH", "")
    if inherited:
        search_paths.append(inherited)
    child_env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(search_paths)
    child_env["SDF_PATH"] = os.pathsep.join([str(REPO / "simulation/models"), child_env.get("SDF_PATH", "")]).rstrip(os.pathsep)

    def start(name: str, command: list[str]):
        log = open(logs / f"{name}.log", "wb")
        process = subprocess.Popen(command, cwd=REPO, env=child_env, stdout=log, stderr=subprocess.STDOUT)
        processes.append((process, log))
        return process

    processes: list[tuple[subprocess.Popen, object]] = []

    def cleanup_processes():
        for process, _log in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
        for _process, log in processes:
            if not log.closed:
                log.close()

    atexit.register(cleanup_processes)

    world_arg = str(world_output.relative_to(REPO))
    start("gazebo", [
        "gz", "sim", "--force-version", "10.5.0", "-s", "--headless-rendering",
        "-r", "--seed", str(int(config["seed"])), world_arg,
    ])
    bridge = REPO / BRIDGE_CONFIG
    time.sleep(2.0)
    if processes[0][0].poll() is not None:
        raise RuntimeError("Gazebo exited before the bridge started; see process-logs/gazebo.log")
    start("bridge", [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
        "-p", f"config_file:={bridge}", "-p", "use_sim_time:=true",
    ])
    time.sleep(1.5)
    controller = start("controller", [
        "ros2", "run", "car_autonomy_nodes", "safe_drive_controller", "--ros-args",
        "-p", "use_sim_time:=true", "-p", "mode:=rc_navigation",
        "-p", f"wheelbase_m:={rc['geometry']['wheelbase_m']['value']}",
        "-p", f"max_steering_angle_rad:={rc['steering']['effective_center_wheel_limit_rad']['value']}",
        "-p", f"rc_max_speed_mps:={rc['steering']['max_speed_mps']['value']}",
        "-p", f"rc_max_yaw_rate_radps:={rc['steering']['max_yaw_rate_radps']['value']}",
    ])

    class BrakeProbe(Node):
        def __init__(self):
            super().__init__("rc_vehicle_brake_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.publisher = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
            self.odom_sub = self.create_subscription(
                Odometry, "/odom", self.on_odom, qos_profile_sensor_data
            )
            self.status_sub = self.create_subscription(DiagnosticStatus, "/vehicle_controller/status", self.on_status, 20)
            self.sim_ns = 0
            self.speed_mps: float | None = None
            self.x_m: float | None = None
            self.ready = False
            self.active_trial: dict | None = None
            self.last_publish_ns = -10**30
            self.stable_samples = 0

        def on_odom(self, message):
            self.speed_mps = float(message.twist.twist.linear.x)
            self.x_m = float(message.pose.pose.position.x)
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            self.sim_ns = max(self.sim_ns, stamp)
            if self.speed_mps >= float(config["drive"]["minimum_stable_speed_mps"]):
                self.stable_samples += 1
            else:
                self.stable_samples = 0
            if self.active_trial is not None and self.active_trial.get("request_ns") is not None:
                distance = max(0.0, abs(self.x_m - self.active_trial["x_start_m"]))
                self.active_trial["samples"].append({
                    "sim_time_s": self.sim_ns / 1e9,
                    "speed_mps": self.speed_mps,
                    "distance_m": distance,
                })

        def on_status(self, message):
            values = {entry.key: entry.value for entry in message.values}
            self.ready = values.get("controller_ready") == "true" and values.get("command_authority_count") == "1"

        def pump(self, timeout=0.025):
            if controller.poll() is not None:
                path = logs / "controller.log"
                text = path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""
                raise RuntimeError(f"safe-drive controller exited early: {text[-2000:]}")
            if processes[0][0].poll() is not None:
                raise RuntimeError("Gazebo exited during the brake probe; see process-logs/gazebo.log")
            rclpy.spin_once(self, timeout_sec=timeout)
            clock = self.get_clock().now().nanoseconds
            self.sim_ns = max(self.sim_ns, clock)

        def send(self, speed: float, steering: float = 0.0):
            message = AckermannDriveStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.drive.speed = float(speed)
            message.drive.steering_angle = float(steering)
            self.publisher.publish(message)
            self.last_publish_ns = self.sim_ns
            if self.active_trial is not None:
                self.active_trial["command_trace"].append({
                    "sim_time_s": self.sim_ns / 1e9,
                    "speed_request_mps": float(speed),
                    "steering_request_rad": float(steering),
                    "topic": "/ackermann_cmd",
                })

        def send_periodically(self, speed: float, period_s: float):
            if self.sim_ns - self.last_publish_ns >= int(period_s * 1e9):
                self.send(speed)

    rclpy.init(args=None)
    node = BrakeProbe()
    trials: list[dict] = []
    schedule = config["trial_schedule"]
    wall_deadline = time.monotonic() + max(180.0, repeats * 18.0)
    try:
        while not (node.ready and node.sim_ns > 0):
            if time.monotonic() > wall_deadline:
                raise RuntimeError("controller and simulated sensor streams did not become ready before the wall timeout")
            node.pump()
        while node.sim_ns < int(float(schedule["warmup_sim_time_s"]) * 1e9):
            node.pump()
        for trial_index in range(1, repeats + 1):
            start_ns = node.sim_ns
            stable_deadline_ns = start_ns + int(float(schedule["approach_timeout_sim_time_s"]) * 1e9)
            node.stable_samples = 0
            while node.stable_samples < 3 and node.sim_ns < stable_deadline_ns:
                node.send_periodically(float(config["drive"]["entry_speed_mps"]), float(schedule["approach_command_period_sim_time_s"]))
                node.pump()
            if node.stable_samples < 3 or node.speed_mps is None or node.x_m is None:
                raise RuntimeError(f"trial {trial_index} did not reach a stable entry speed")
            trial = {
                "trial_index": trial_index,
                "accepted": True,
                "source_hashes": source_hashes,
                "simulator_identity": sim_identity,
                "speed_before_brake_mps": node.speed_mps,
                "brake_request_sim_time_s": node.sim_ns / 1e9,
                "request_ns": node.sim_ns,
                "x_start_m": node.x_m,
                "command_trace": [],
                "samples": [{
                    "sim_time_s": node.sim_ns / 1e9,
                    "speed_mps": node.speed_mps,
                    "distance_m": 0.0,
                }],
            }
            node.active_trial = trial
            node.send(float(config["drive"]["brake_speed_mps"]), float(config["drive"]["steering_angle_rad"]))
            brake_ns = node.sim_ns
            stop_seen_since_ns = None
            stop_elapsed_ns = int(float(schedule["brake_hold_sim_time_s"]) * 1e9)
            rest_ns = int(float(schedule["rest_sim_time_s"]) * 1e9)
            while node.sim_ns - brake_ns < stop_elapsed_ns + rest_ns:
                node.send_periodically(float(config["drive"]["brake_speed_mps"]), float(schedule["approach_command_period_sim_time_s"]))
                node.pump()
                if node.speed_mps is not None and abs(node.speed_mps) <= float(schedule["stop_speed_threshold_mps"]):
                    if stop_seen_since_ns is None:
                        stop_seen_since_ns = node.sim_ns
                else:
                    stop_seen_since_ns = None
                if stop_seen_since_ns is not None and node.sim_ns - stop_seen_since_ns >= rest_ns:
                    break
            node.active_trial = None
            trial.pop("x_start_m", None)
            trial["samples"] = sorted(
                {sample["sim_time_s"]: sample for sample in trial["samples"]}.values(),
                key=lambda sample: sample["sim_time_s"],
            )
            trials.append(trial)
            print(
                f"trial {trial_index}/{repeats}: speed={trial['speed_before_brake_mps']:.3f} m/s "
                f"samples={len(trial['samples'])}", flush=True,
            )
        tolerances = config["expected_observations"]
        measurements = aggregate_trials(
            trials,
            expected_repeats=repeats,
            expected_source_hashes=source_hashes,
            expected_simulator_identity=sim_identity,
            stop_speed_threshold_mps=float(schedule["stop_speed_threshold_mps"]),
            onset_delta_mps=float(schedule["deceleration_onset_delta_mps"]),
            maximum_gap_s=float(tolerances["maximum_sample_gap_sim_time_s"]),
            minimum_samples=int(tolerances["minimum_samples_per_brake"]),
            maximum_distance_m=float(schedule["maximum_trial_distance_m"]),
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cleanup_processes()

    return {
        "schema_version": "1.0",
        "measurement_id": "rc_vehicle_brake_reference",
        "scenario_id": config["scenario_id"],
        "run_purpose": "diagnostic",
        "seed": int(config["seed"]),
        "environment": sim_identity,
        "generated_world": str(world_output.relative_to(REPO)),
        "generated_world_sha256": generated_world_hash,
        "source_hashes": source_hashes,
        "profile_input_sha256": profile_hash,
        "measurements": measurements,
        "stopping_distance_curve": [
            {
                "speed_mps": float(speed),
                "distance_m": stopping_distance_m(
                    float(speed),
                    float(measurements["conservative_braking_deceleration_mps2"]),
                    float(measurements["conservative_request_to_deceleration_latency_s"]),
                ),
                "unit": "m",
                "provenance": "measured_sim braking and request latency from this report",
            }
            for speed in (0.1, 0.25, 0.5, 0.75, 1.0)
        ],
    }


def _write_measured_profile(profile_path: Path, report_path: Path, report: dict) -> None:
    document = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    measurements = report["measurements"]
    document["status"] = "measured_sim"
    dynamics = document["dynamics"]
    dynamics["braking_deceleration_mps2"] = {
        "value": measurements["conservative_braking_deceleration_mps2"],
        "unit": "m/s^2",
        "status": "measured_sim",
        "provenance": f"{report_path.as_posix()}#measurements.conservative_braking_deceleration_mps2",
    }
    dynamics["request_to_deceleration_latency_s"] = {
        "value": measurements["conservative_request_to_deceleration_latency_s"],
        "unit": "s",
        "status": "measured_sim",
        "provenance": f"{report_path.as_posix()}#measurements.conservative_request_to_deceleration_latency_s",
    }
    dynamics["measurement_report"] = {
        "path": report_path.as_posix(),
        "sha256": sha256_file(report_path),
    }
    dynamics["simulator_identity"] = report["environment"]
    dynamics["measurement_source_hashes"] = report["source_hashes"]
    geometry = document["geometry"]
    wheelbase = float(geometry["wheelbase_m"]["value"])
    steering = float(document["steering"]["effective_center_wheel_limit_rad"]["value"])
    minimum_turn_radius = wheelbase / math.tan(steering)
    document["derived_geometry"]["minimum_turn_radius_m"] = {
        "value": minimum_turn_radius,
        "unit": "m",
        "status": "derived_sim_geometry",
        "provenance": "wheelbase_m / tan(effective_center_wheel_limit_rad)",
    }
    stop = document["stopping_distance"]
    stop["status"] = "measured_sim"
    stop["curve"] = report["stopping_distance_curve"]
    profile_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-config", required=True, type=Path)
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--write-profile", action="store_true")
    args = parser.parse_args(argv)
    scenario_path = args.scenario_config.resolve()
    profile_path = args.profile.resolve()
    output_path = args.output if args.output.is_absolute() else REPO / args.output
    try:
        scenario_rel = scenario_path.relative_to(REPO)
        expected_scenario = REPO / "ros2_ws/src/car_bringup/config/obstacle_course/diagnostics/rc_vehicle_brake.yaml"
        if scenario_path != expected_scenario.resolve():
            raise RuntimeError("only the obstacle-course diagnostic brake scenario is accepted")
        profile_path.relative_to(REPO)
        if args.repeats <= 0:
            raise RuntimeError("--repeats must be positive")
        config = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
        if config.get("scenario_id") != "rc_vehicle_brake" or config.get("run_purpose") != "diagnostic":
            raise RuntimeError("only the tracked diagnostic rc_vehicle_brake scenario is accepted")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        report = _run_simulation(config, profile_path, scenario_path, output_path, args.repeats)
        report["measurement_report_path"] = output_path.relative_to(REPO).as_posix()
        output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if args.write_profile:
            _write_measured_profile(profile_path, output_path.relative_to(REPO), report)
        result = report["measurements"]
        print(
            f"RC braking measurement PASS ({result['accepted_trial_count']} trials): "
            f"a_brake={result['conservative_braking_deceleration_mps2']:.4f} m/s^2, "
            f"latency={result['conservative_request_to_deceleration_latency_s']:.4f} s; "
            "simulation-only values",
            flush=True,
        )
        print(f"Report: {output_path.relative_to(REPO)}", flush=True)
        return 0
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as exc:
        print(f"measure_rc_vehicle_dynamics: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
