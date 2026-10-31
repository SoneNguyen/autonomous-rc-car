#!/usr/bin/env python3
"""Open a live Gazebo GUI and the matching ROS topic viewer for one RC module."""
from __future__ import annotations

import argparse
from copy import deepcopy
import math
import os
from pathlib import Path
import re
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
ROS_TOPIC_TYPES = {
    "/camera/image_raw": "sensor_msgs/msg/Image",
    "/imu/data": "sensor_msgs/msg/Imu",
    "/odom": "nav_msgs/msg/Odometry",
    "/scan": "sensor_msgs/msg/LaserScan",
    "/clock": "rosgraph_msgs/msg/Clock",
}
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

MODULE_VIEWS: dict[str, dict[str, Any]] = {
    "lidar": {
        "config": "lidar.yaml",
        # The front-right target stays clear of a straight driving line but
        # remains in a known LiDAR beam throughout the demo.
        "scenario": "lidar/sector-front_right",
        "behavior": "sensors",
        "sensor_set": "lidar",
        "bridge_topics": {"/scan", "/clock", "/model/gazebo_truck/cmd_vel"},
        "viewers": [["rqt_plot", "/scan/ranges[140]"], ["rqt_topic"]],
        "motion": {"speed_mps": 0.2, "yaw_rate_rps": 0.0},
        "description": "The truck drives straight past the front-right cone while the plot shows its live LiDAR range.",
        "work_case": "Drive past the cone placed 3 m away at -20 degrees, in the front-right LiDAR sector.",
        "measure": "Plot /scan/ranges[140]. A finite return near 3 m confirms the target beam sees the cone.",
    },
    "camera": {
        "config": "camera.yaml",
        "scenario": "camera/cone-near",
        "behavior": "camera",
        "sensor_set": "camera",
        "bridge_topics": {
            "/camera/image_raw", "/camera/labels", "/clock", "/model/gazebo_truck/cmd_vel",
        },
        "viewers": [
            ["rqt_image_view", "/camera/image_raw"],
            ["rqt_image_view", "/camera/labels"],
            ["rqt_topic"],
        ],
        "motion": {"speed_mps": 0.2, "yaw_rate_rps": 0.0},
        "description": "The truck drives straight past a cone while RGB, segmentation, and detections update live.",
        "work_case": "Pass a near cone without hitting it while the camera and frozen detector observe it.",
        "measure": "Compare /camera/image_raw with /camera/labels; /detections should include class cone at the frozen 0.8 threshold.",
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
        "motion": {"speed_mps": 0.2, "yaw_rate_rps": 0.08, "turn_duration_s": 5.0},
        "description": "The truck follows a gentle turn while the plots show IMU yaw rate, acceleration, and odometry speed.",
        "work_case": "Apply a 5-second, 0.08 rad/s left turn, then continue straight.",
        "measure": "During the turn, /imu/data/angular_velocity/z and /odom angular.z should be positive; after it, yaw rate should return near zero while /odom linear.x stays near 0.2 m/s.",
    },
    "contact_tf": {
        "config": "contact_tf.yaml",
        "scenario": "contact_tf/cone-contact",
        "behavior": "sensors",
        "sensor_set": "contact",
        "bridge_topics": {"/contacts", "/clock", "/model/gazebo_truck/cmd_vel"},
        "viewers": [["rqt_topic"]],
        "motion": {"speed_mps": 0.1, "yaw_rate_rps": 0.0},
        "description": "The truck rolls slowly into the cone so the contact event and sensor transforms are visible.",
        "work_case": "Approach the cone and deliberately touch it with the truck bumper.",
        "measure": "Inspect /contacts for gazebo_truck chassis and cone_01 collisions; resolve base_link to lidar_link, camera_link, and imu_link with TF tools.",
    },
    "actuator": {
        "config": "command_path.yaml",
        "scenario": "command_path/bounds-and-step-response",
        "behavior": "sensors",
        "sensor_set": "none",
        "bridge_topics": {"/model/gazebo_truck/cmd_vel", "/odom", "/clock"},
        "viewers": [["rqt_plot", "/odom/twist/twist/linear/x", "/odom/twist/twist/angular/z"]],
        "motion": {"speed_mps": 0.2, "yaw_rate_rps": 0.0},
        "description": "The truck drives straight continuously; plots show the measured speed and yaw response.",
        "work_case": "Send a direct straight Gazebo Twist command through the bridge.",
        "measure": "Plot /odom linear.x approaching 0.2 m/s and angular.z remaining near zero.",
    },
}

MODULE_LABELS = {
    "lidar": "LiDAR pass (front-right target)",
    "camera": "Camera pass (cone)",
    "imu_odometry": "IMU and odometry (gentle turn)",
    "contact_tf": "Contact and TF (slow cone approach)",
    "actuator": "Drive actuator (straight drive)",
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
        # Keep Gazebo and rqt on the same X11 compositor. Besides avoiding
        # mixed Wayland/X11 window placement in WSLg, this lets the launcher
        # tile the viewer windows beside the Gazebo scene.
        if env.get("WAYLAND_DISPLAY") and env.get("DISPLAY"):
            env.setdefault("QT_QPA_PLATFORM", "xcb")
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
    message_type = ROS_TOPIC_TYPES.get(topic)
    if message_type is None:
        return False
    if launch.poll() is not None:
        return False
    try:
        subscriber = subprocess.Popen(
            ["ros2", "topic", "echo", "--once", topic, message_type,
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


def _resume_simulation(env: dict[str, str], launch: subprocess.Popen) -> None:
    """Resume Gazebo and confirm simulation time advances before opening viewers."""
    deadline = time.monotonic() + 30.0
    detail = "Gazebo world control is not available yet"
    last_sim_time_ns: int | None = None
    next_resume_attempt = 0.0
    while time.monotonic() < deadline:
        if launch.poll() is not None:
            raise RuntimeError("Gazebo exited before its simulation could be started")
        now = time.monotonic()
        if now >= next_resume_attempt:
            try:
                result = subprocess.run(
                    [
                        "gz", "service", "-s", "/world/rc_module_matrix/control",
                        "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                        "--timeout", "1000", "--req", "pause: false",
                    ],
                    env=env, capture_output=True, text=True, timeout=2.5, check=False,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                detail = str(exc)
            else:
                detail = (result.stdout + result.stderr).strip() or f"exit status {result.returncode}"
            next_resume_attempt = now + 1.0

        # A successful WorldControl response only means the request was
        # accepted. WSLg / sensor startup can still leave the world paused at
        # its first rendered frame, so verify that /clock actually moves.
        try:
            clock = subprocess.run(
                ["ros2", "topic", "echo", "--once", "/clock", "rosgraph_msgs/msg/Clock"],
                env=env, capture_output=True, text=True, timeout=2.0, check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            detail = str(exc)
        else:
            match = re.search(r"sec:\s*(\d+)\s+nanosec:\s*(\d+)", clock.stdout)
            if clock.returncode == 0 and match:
                sim_time_ns = int(match.group(1)) * 1_000_000_000 + int(match.group(2))
                if last_sim_time_ns is not None and sim_time_ns > last_sim_time_ns:
                    return
                last_sim_time_ns = sim_time_ns
            elif clock.returncode != 0:
                detail = (clock.stdout + clock.stderr).strip() or detail
        time.sleep(0.25)
    raise RuntimeError(f"Gazebo simulation clock did not advance after resume: {detail}")


def _print_module_menu() -> None:
    print("Available visible component demos:")
    for index, (module, label) in enumerate(MODULE_LABELS.items(), start=1):
        print(f"  {index}. {label}  (python3 tools/rc_module_gui.py {module})")


def _choose_module() -> str | None:
    _print_module_menu()
    print("Choose a number or module name. Press Enter or q to quit.")
    try:
        selection = input("> ").strip()
    except EOFError:
        return None
    if not selection or selection.lower() in {"q", "quit", "exit"}:
        return None
    modules = list(MODULE_LABELS)
    if selection.isdigit() and 1 <= int(selection) <= len(modules):
        return modules[int(selection) - 1]
    if selection in MODULE_VIEWS:
        return selection
    print(f"Unknown component {selection!r}; run with --list to see choices.", file=sys.stderr)
    return None


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


def _stop_launch(launch: subprocess.Popen) -> None:
    """Let ROS launch signal Gazebo and its nodes in order before escalation."""
    if launch.poll() is None:
        from tools.gui_window_layout import request_close_component_windows

        request_close_component_windows()
        close_deadline = time.monotonic() + 8.0
        while launch.poll() is None and time.monotonic() < close_deadline:
            time.sleep(0.1)
    if launch.poll() is None:
        try:
            launch.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            launch.wait(timeout=12.0)
        except subprocess.TimeoutExpired:
            pass

    deadline = time.monotonic() + 5.0
    while _group_exists(launch) and time.monotonic() < deadline:
        time.sleep(0.1)
    if _group_exists(launch):
        try:
            os.killpg(launch.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            launch.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pass
    if _group_exists(launch):
        try:
            os.killpg(launch.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    if launch.poll() is None:
        launch.wait()


def _start_motion(
    env: dict[str, str], *, speed_mps: float, yaw_rate_rps: float,
) -> subprocess.Popen:
    topic = "/model/gazebo_truck/cmd_vel"
    return subprocess.Popen([
        "ros2", "topic", "pub", "--rate", "10", topic,
        "geometry_msgs/msg/Twist",
        f"{{linear: {{x: {speed_mps}}}, angular: {{z: {yaw_rate_rps}}}}}",
    ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def _stop_motion(env: dict[str, str], active: subprocess.Popen | None) -> None:
    if active is None:
        return
    _stop_process_group(active, timeout_s=3.0)
    subprocess.run([
        "ros2", "topic", "pub", "--once", "/model/gazebo_truck/cmd_vel",
        "geometry_msgs/msg/Twist", "{linear: {x: 0.0}, angular: {z: 0.0}}",
    ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5, check=False)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("module", nargs="?", choices=MODULE_VIEWS)
    parser.add_argument("--list", action="store_true", help="list component demos without launching Gazebo")
    parser.add_argument("--scenario", help="override the module's default visible scenario")
    parser.add_argument(
        "--quality", choices=("low", "high"), default="low",
        help="viewer profile (low is smoother; high keeps the qualification camera resolution/rate)",
    )
    args = parser.parse_args(argv)

    if args.list:
        _print_module_menu()
        return 0
    module = args.module or _choose_module()
    if module is None:
        return 0

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

    view = MODULE_VIEWS[module]
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
        parser.error(f"{scenario!r} is not a checked-in scenario for {module}")

    scenario_config = deepcopy(scenarios[scenario])
    scenario_config["setup"] = dict(scenario_config.get("setup", {}))
    scenario_config["setup"]["quality_preset"] = args.quality
    if module == "camera":
        # The runtime-only demo passes the locked cone without driving through
        # it, while it remains inside the forward camera view. Qualification
        # scenario files are left unchanged.
        target = scenario_config["setup"].get("target_entity")
        for fixture in scenario_config.get("obstacle_placements", []):
            if fixture.get("entity_id") == target:
                fixture["pose"]["y_m"] = float(scenario_config["start_pose"]["y_m"]) + 0.65
                break
    elif module == "contact_tf" and scenario == "contact_tf/cone-contact":
        # Give the truck a visible, slow approach before its deliberate bumper
        # contact instead of spawning it already touching the cone.
        target_pose = scenario_config["obstacle_placements"][0]["pose"]
        scenario_config["start_pose"]["x_m"] = float(target_pose["x_m"]) - 1.4

    env = _render_environment()
    from tools.camera_hotkeys import GazeboCameraHotkeys
    from tools.gui_window_layout import tile_component_windows
    from tools.checks.runtime_capture import write_runtime_scenario
    runtime_id = str(uuid.uuid4())
    runtime_scenario_name, runtime_scenario_path = write_runtime_scenario(
        scenario_config, runtime_id,
    )
    gui_config_path = _write_gui_config(scenario_config, runtime_id)
    bridge_config_path = _write_module_bridge_config(
        module, gui_config_path.parent,
    )
    launch: subprocess.Popen | None = None
    motion_publisher: subprocess.Popen | None = None
    camera_controls: GazeboCameraHotkeys | None = None
    viewers: list[subprocess.Popen] = []
    topic = {
        "lidar": "/scan",
        "camera": "/camera/image_raw",
        "imu_odometry": "/imu/data",
        "contact_tf": "/contacts",
        "actuator": "/odom",
    }[module]
    print(f"Opening Gazebo GUI for {scenario} (quality={args.quality}).", flush=True)
    print(view["description"], flush=True)
    print(f"Work case: {view['work_case']}", flush=True)
    print(f"Confirm with: {view['measure']}", flush=True)
    try:
        launch = subprocess.Popen([
            "ros2", "launch", "car_bringup", "obstacle_course.launch.py",
            f"behavior:={view['behavior']}", f"scenario:={runtime_scenario_name}",
            "headless:=false", f"gui_config:={gui_config_path}",
            f"bridge_config:={bridge_config_path}",
            f"sensor_set:={view['sensor_set']}", f"quality_preset:={args.quality}",
        ], cwd=ROOT, env=env, start_new_session=True)
        _resume_simulation(env, launch)
        if not _wait_for_topic(topic, env, launch):
            raise RuntimeError(
                f"{topic} did not produce a live sample; Gazebo may have exited, "
                "check the launch output above"
            )
        camera_pose = tuple(float(value) for value in _scene_camera_pose(scenario_config).split())
        camera_controls = GazeboCameraHotkeys(camera_pose)
        camera_controls.start()
        for viewer in view["viewers"]:
            package, *viewer_args = viewer
            viewer_index = len(viewers)
            viewer_config = gui_config_path.parent / "rqt-config" / str(viewer_index)
            viewer_config.mkdir(parents=True, exist_ok=True)
            viewer_env = env.copy()
            # rqt persists dock layouts in QSettings. Start each viewer with a
            # clean, run-local profile so an old user layout cannot restore extra
            # copies of plugins or minimized dock widgets into this demo.
            viewer_env["XDG_CONFIG_HOME"] = str(viewer_config)
            viewers.append(subprocess.Popen(
                ["ros2", "run", package, package, *viewer_args],
                cwd=ROOT, env=viewer_env, start_new_session=True,
            ))
        layout_message = tile_component_windows(
            [viewer[0] for viewer in view["viewers"]],
        )
        if layout_message:
            print(layout_message, flush=True)
        print(
            "Live data received; the demo is running. Press Ctrl-C to close it, then run the menu again to choose another component.",
            flush=True,
        )
        print(
            "Camera: click the Gazebo scene, then WASD pan, Q/E rotate, PgUp/PgDn or +/- zoom, "
            "Shift moves faster, 0 resets, and T toggles truck follow without resetting zoom. "
            "Mouse wheel zooms; "
            "middle drag orbits and right drag zooms.",
            flush=True,
        )
        if args.quality == "high":
            print(
                "High camera quality uses 640x480 at 30 Hz; if WSLg lags, restart without --quality high.",
                flush=True,
            )
        motion_deadline = None
        motion_transitioned = False
        if motion := view.get("motion"):
            if abs(float(motion.get("yaw_rate_rps", 0.0))) > 1e-3:
                motion_description = (
                    f"Turning left at {motion['yaw_rate_rps']:.2f} rad/s for "
                    f"{motion.get('turn_duration_s', 0):.1f} s, at {motion['speed_mps']:.2f} m/s, "
                    "then continuing straight."
                )
            else:
                motion_description = f"Driving straight at {motion['speed_mps']:.2f} m/s."
            print(f"{motion_description} Press Ctrl-C to stop the truck and close the demo.", flush=True)
            motion_publisher = _start_motion(
                env, speed_mps=motion["speed_mps"], yaw_rate_rps=motion["yaw_rate_rps"],
            )
            if float(motion.get("turn_duration_s", 0.0)) > 0.0:
                motion_deadline = time.monotonic() + float(motion["turn_duration_s"])
        while launch.poll() is None:
            if motion_deadline is not None and not motion_transitioned and time.monotonic() >= motion_deadline:
                _stop_process_group(motion_publisher, timeout_s=3.0)
                motion_publisher = _start_motion(env, speed_mps=motion["speed_mps"], yaw_rate_rps=0.0)
                motion_transitioned = True
                print("Turn complete; truck is now driving straight and yaw rate should return near zero.", flush=True)
            time.sleep(0.5)
        return launch.returncode or 0
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"rc_module_gui: {exc}", file=sys.stderr)
        return 2
    finally:
        if camera_controls is not None:
            camera_controls.stop()
        _stop_motion(env, motion_publisher)
        from tools.gui_window_layout import request_close_component_windows

        request_close_component_windows([viewer[0] for viewer in view["viewers"]])
        for process in viewers:
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                _stop_process_group(process, timeout_s=5.0)
        if launch is not None:
            _stop_launch(launch)
        runtime_scenario_path.unlink(missing_ok=True)
        # This per-run directory also holds the temporary rqt config profiles.
        shutil.rmtree(gui_config_path.parent, ignore_errors=True)
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
