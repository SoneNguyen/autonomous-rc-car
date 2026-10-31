#!/usr/bin/env python3
"""Open a live Gazebo GUI and the matching ROS topic viewer for one RC module."""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
from typing import Any
import uuid

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = ROOT / "ros2_ws/src/car_bringup/config/rc_module_matrix"
BRIDGE_CONFIG = CONFIG_ROOT / "bridge.yaml"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

MODULE_VIEWS: dict[str, dict[str, Any]] = {
    "lidar": {
        "config": "lidar.yaml",
        "scenario": "lidar/sector-front",
        "behavior": "sensors",
        "sensor_set": "lidar",
        "bridge_topics": {"/scan", "/clock"},
        "viewers": [["rqt_plot", "/scan/ranges[180]"], ["rqt_topic"]],
        "description": "Gazebo shows the research car and front-sector target; rqt_plot shows live front-beam range.",
    },
    "camera": {
        "config": "camera.yaml",
        "scenario": "camera/cone-near",
        "behavior": "camera",
        "sensor_set": "camera",
        "bridge_topics": {"/camera/image_raw", "/camera/labels", "/clock"},
        "viewers": [
            ["rqt_image_view", "/camera/image_raw"],
            ["rqt_image_view", "/camera/labels"],
            ["rqt_topic"],
        ],
        "description": "Image windows show RGB and segmentation; rqt_topic exposes /detections and perception status.",
    },
    "imu_odometry": {
        "config": "imu_odometry.yaml",
        "scenario": "imu_odometry/turning-bicycle",
        "behavior": "sensors",
        "sensor_set": "imu",
        "bridge_topics": {"/imu/data", "/odom", "/clock", "/model/gazebo_truck/cmd_vel"},
        "viewers": [[
            "rqt_plot", "/imu/data/angular_velocity/z",
            "/imu/data/linear_acceleration/x", "/odom/twist/twist/linear/x",
        ]],
        "motion": {"speed_mps": 0.2, "yaw_rate_rps": 0.08, "duration_s": 5.0},
        "description": "The car performs a short, bounded turn while the plots show IMU yaw, acceleration, and odometry speed.",
    },
    "contact_tf": {
        "config": "contact_tf.yaml",
        "scenario": "contact_tf/cone-contact",
        "behavior": "sensors",
        "sensor_set": "contact",
        "bridge_topics": {"/contacts", "/clock"},
        "viewers": [["rqt_topic"]],
        "description": "Gazebo shows the research cone/car contact pose; rqt_topic exposes /contacts and TF topics.",
    },
    "actuator": {
        "config": "command_path.yaml",
        "scenario": "command_path/bounds-and-step-response",
        "behavior": "sensors",
        "sensor_set": "none",
        "bridge_topics": {"/model/gazebo_truck/cmd_vel", "/odom", "/clock"},
        "viewers": [["rqt_plot", "/odom/twist/twist/linear/x", "/odom/twist/twist/angular/z"]],
        "motion": {"speed_mps": 0.2, "yaw_rate_rps": 0.12, "duration_s": 4.0},
        "description": "A short direct Gazebo Twist pulse checks simulated drive response; plots show measured odometry.",
    },
}


def _scenarios(path: Path) -> dict[str, dict[str, Any]]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if isinstance(document, list):
        rows = document
    elif isinstance(document, dict) and isinstance(document.get("scenarios"), list):
        rows = document["scenarios"]
    else:
        rows = [document]
    return {
        str(row["scenario_id"]): row
        for row in rows if isinstance(row, dict) and row.get("scenario_id")
    }


def _scene_camera_pose(scenario: dict[str, Any]) -> str:
    """Frame the controlled vehicle and the fixtures that enter the test world."""
    start = scenario["start_pose"]
    points = [(float(start["x_m"]), float(start["y_m"]))]
    for fixture in scenario.get("obstacle_placements", []):
        pose = fixture["pose"]
        x, y = float(pose["x_m"]), float(pose["y_m"])
        if -14.0 <= x <= 14.0 and -2.5 <= y <= 2.5:
            points.append((x, y))

    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    focus_x = (min_x + max_x) * 0.5
    focus_y = (min_y + max_y) * 0.5
    span = max(max_x - min_x, max_y - min_y, 3.0)
    # Keep close scenes framed on the vehicle and target so the 1:10 research
    # meshes are legible in the first view.
    distance = max(3.0, span * 0.65 + 0.4)
    # A quarter-turn view makes the vehicle and its longitudinal target sit at
    # similar camera distances, instead of hiding the target down the road.
    yaw = float(start.get("yaw_rad", 0.0)) + math.pi * 0.5
    pitch = 0.48
    camera_x = focus_x - distance * math.cos(yaw)
    camera_y = focus_y - distance * math.sin(yaw)
    camera_z = 0.8 + distance * math.tan(pitch)
    return " ".join(f"{value:.3f}" for value in (
        camera_x, camera_y, camera_z, 0.0, pitch, yaw,
    ))


def _write_gui_config(scenario: dict[str, Any], run_id: str) -> Path:
    template_path = ROOT / "simulation/gui/rc_module_matrix.config.in"
    template = template_path.read_text(encoding="utf-8")
    marker = "__CAMERA_POSE__"
    if template.count(marker) != 1:
        raise RuntimeError(f"GUI config template must contain exactly one {marker} camera marker")
    output = ROOT / "artifacts/rc-car-matrix/gui" / run_id / "gui.config"
    output.parent.mkdir(parents=True, exist_ok=False)
    output.write_text(template.replace(marker, _scene_camera_pose(scenario)), encoding="utf-8")
    return output


def _render_environment() -> dict[str, str]:
    env = os.environ.copy()
    if Path("/mnt/wslg").is_dir() and (env.get("DISPLAY") or env.get("WAYLAND_DISPLAY")):
        env.setdefault("GALLIUM_DRIVER", "d3d12")
        if not env.get("MESA_D3D12_DEFAULT_ADAPTER_NAME"):
            try:
                result = subprocess.run(
                    ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
                    capture_output=True, text=True, timeout=3, check=False,
                )
            except (OSError, subprocess.TimeoutExpired):
                result = None
            if result and result.stdout.strip():
                env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = result.stdout.splitlines()[0].strip()
    return env


def _write_module_bridge_config(module: str, output_dir: Path) -> Path:
    allowed_topics = MODULE_VIEWS[module]["bridge_topics"]
    rows = yaml.safe_load(BRIDGE_CONFIG.read_text(encoding="utf-8"))
    filtered = [row for row in rows if row.get("ros_topic_name") in allowed_topics]
    found_topics = {row["ros_topic_name"] for row in filtered}
    if found_topics != allowed_topics:
        raise RuntimeError(
            f"bridge config is missing topics for {module}: {sorted(allowed_topics - found_topics)}"
        )
    output = output_dir / "bridge.yaml"
    output.write_text(yaml.safe_dump(filtered, sort_keys=False), encoding="utf-8")
    return output


def _wait_for_topic(
    topic: str, env: dict[str, str], launch: subprocess.Popen, timeout_s: float = 45.0,
) -> bool:
    """Wait for a real sample; bridge-created ROS topic names alone prove no simulator is alive."""
    if topic == "/contacts":
        # Contact publishers only emit during a collision, so use simulation clock for startup.
        topic = "/clock"
    if launch.poll() is not None:
        return False
    try:
        subscriber = subprocess.Popen(
            ["ros2", "topic", "echo", "--once", topic,
             "--qos-reliability", "best_effort"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except OSError:
        return False
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                return False
            if subscriber.poll() is not None:
                return subscriber.returncode == 0
            time.sleep(0.2)
        return False
    finally:
        _stop_process_group(subscriber, timeout_s=2.0)


def _group_exists(process: subprocess.Popen) -> bool:
    try:
        os.killpg(process.pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _stop_process_group(process: subprocess.Popen, *, timeout_s: float = 12.0) -> None:
    """Stop a ROS CLI process and any child it started, without leaving orphans."""
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            pass

    # ros2 launch may exit while Gazebo or an rqt child is still shutting down.
    deadline = time.monotonic() + 2.0
    while _group_exists(process) and time.monotonic() < deadline:
        time.sleep(0.1)
    if _group_exists(process):
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass
    if _group_exists(process):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    if process.poll() is None:
        process.wait()


def _publish_motion(
    env: dict[str, str], *, speed_mps: float, yaw_rate_rps: float, duration_s: float,
) -> None:
    topic = "/model/gazebo_truck/cmd_vel"
    active = subprocess.Popen([
        "ros2", "topic", "pub", "--rate", "10", topic,
        "geometry_msgs/msg/Twist",
        f"{{linear: {{x: {speed_mps}}}, angular: {{z: {yaw_rate_rps}}}}}",
    ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        time.sleep(duration_s)
    finally:
        _stop_process_group(active, timeout_s=3.0)
        subprocess.run([
            "ros2", "topic", "pub", "--once", topic,
            "geometry_msgs/msg/Twist", "{linear: {x: 0.0}, angular: {z: 0.0}}",
        ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5, check=False)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("module", choices=MODULE_VIEWS)
    parser.add_argument("--scenario", help="override the module's default visible scenario")
    parser.add_argument(
        "--quality", choices=("low", "high"), default="low",
        help="viewer profile (low is smoother; high keeps the qualification camera resolution/rate)",
    )
    args = parser.parse_args(argv)

    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("no GUI display found; start this command from a WSLg terminal")
    if not (ROOT / "ros2_ws/install/car_bringup/share/car_bringup").is_dir():
        parser.error("workspace is not built/sourced; source /opt/ros/lyrical/setup.bash and ros2_ws/install/setup.bash")
    for executable in ("ros2", "gz"):
        if shutil.which(executable) is None:
            parser.error(
                f"{executable} is not on PATH; source /opt/ros/lyrical/setup.bash "
                "and ros2_ws/install/setup.bash before opening a module demo"
            )

    view = MODULE_VIEWS[args.module]
    for package in {item[0] for item in view["viewers"]}:
        try:
            available = subprocess.run(
                ["ros2", "pkg", "executables", package],
                capture_output=True, text=True, timeout=10, check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            parser.error(f"cannot check ROS viewer package {package}: {exc}")
        expected = f"{package} {package}"
        if available.returncode != 0 or expected not in available.stdout.splitlines():
            package_name = package.replace("_", "-")
            parser.error(
                f"ROS viewer {package} is unavailable; install ros-lyrical-{package_name} "
                "and source /opt/ros/lyrical/setup.bash"
            )

    scenario = args.scenario or view["scenario"]
    scenarios = _scenarios(CONFIG_ROOT / view["config"])
    if scenario not in scenarios:
        parser.error(f"{scenario!r} is not a checked-in scenario for {args.module}")

    scenario_config = dict(scenarios[scenario])
    scenario_config["setup"] = dict(scenario_config.get("setup", {}))
    scenario_config["setup"]["quality_preset"] = args.quality

    env = _render_environment()
    from tools.checks.runtime_capture import write_runtime_scenario
    runtime_id = str(uuid.uuid4())
    runtime_scenario_name, runtime_scenario_path = write_runtime_scenario(
        scenario_config, runtime_id,
    )
    gui_config_path = _write_gui_config(scenario_config, runtime_id)
    bridge_config_path = _write_module_bridge_config(
        args.module, gui_config_path.parent,
    )
    launch: subprocess.Popen | None = None
    viewers: list[subprocess.Popen] = []
    topic = {
        "lidar": "/scan",
        "camera": "/camera/image_raw",
        "imu_odometry": "/imu/data",
        "contact_tf": "/contacts",
        "actuator": "/odom",
    }[args.module]
    print(f"Opening Gazebo GUI for {scenario} (quality={args.quality}).", flush=True)
    print(view["description"], flush=True)
    try:
        launch = subprocess.Popen([
            "ros2", "launch", "car_bringup", "obstacle_course.launch.py",
            f"behavior:={view['behavior']}", f"scenario:={runtime_scenario_name}",
            "headless:=false", f"gui_config:={gui_config_path}",
            f"bridge_config:={bridge_config_path}",
            f"sensor_set:={view['sensor_set']}", f"quality_preset:={args.quality}",
        ], cwd=ROOT, env=env, start_new_session=True)
        if not _wait_for_topic(topic, env, launch):
            raise RuntimeError(
                f"{topic} did not produce a live sample; Gazebo may have exited, "
                "check the launch output above"
            )
        for viewer in view["viewers"]:
            package, *viewer_args = viewer
            viewers.append(subprocess.Popen(
                ["ros2", "run", package, package, *viewer_args],
                cwd=ROOT, env=env, start_new_session=True,
            ))
        if motion := view.get("motion"):
            print("Sending a short low-speed simulated drive pulse, then publishing zero Twist.", flush=True)
            _publish_motion(env, **motion)
        while launch.poll() is None:
            time.sleep(0.5)
        return launch.returncode or 0
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"rc_module_gui: {exc}", file=sys.stderr)
        return 2
    finally:
        for process in viewers:
            _stop_process_group(process, timeout_s=5.0)
        if launch is not None and launch.poll() is None:
            _stop_process_group(launch)
        runtime_scenario_path.unlink(missing_ok=True)
        gui_config_path.unlink(missing_ok=True)
        bridge_config_path.unlink(missing_ok=True)
        try:
            gui_config_path.parent.rmdir()
        except OSError:
            pass
        parent = runtime_scenario_path.parent
        installed_root = runtime_scenario_path.parents[4]
        while parent != installed_root:
            try:
                parent.rmdir()
            except OSError:
                break
            parent = parent.parent


if __name__ == "__main__":
    raise SystemExit(main())
