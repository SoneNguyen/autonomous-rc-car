"""Ephemeral scenario adaptation and headless ROS sensor capture."""
from __future__ import annotations

from collections import OrderedDict
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]


def _select_sensor_renderer(output_dir: Path) -> tuple[str, dict[str, str], str]:
    """Select and record the WSLg GPU renderer used by Gazebo sensor captures."""
    wslg = Path("/mnt/wslg").is_dir()
    default_mode = "glx" if wslg and (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ) else "headless"
    mode = os.environ.get("RC_MODULE_MATRIX_RENDER_MODE", default_mode)
    if mode not in {"headless", "glx"}:
        raise RuntimeError("RC_MODULE_MATRIX_RENDER_MODE must be 'headless' or 'glx'")
    child_env = os.environ.copy()
    if mode == "glx" and wslg:
        child_env.setdefault("GALLIUM_DRIVER", "d3d12")
        if not child_env.get("MESA_D3D12_DEFAULT_ADAPTER_NAME"):
            try:
                adapters = subprocess.run(
                    ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
                    check=False, capture_output=True, text=True, timeout=3.0,
                ).stdout.splitlines()
            except (OSError, subprocess.TimeoutExpired):
                adapters = []
            if adapters:
                child_env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = adapters[0].strip()
    config_path = output_dir / "render-configuration.json"
    config_path.write_text(json.dumps({
        "mode": mode,
        "server_only": True,
        "wslg_detected": wslg,
        "display": os.environ.get("DISPLAY"),
        "wayland_display": os.environ.get("WAYLAND_DISPLAY"),
        "gallium_driver": child_env.get("GALLIUM_DRIVER"),
        "mesa_d3d12_adapter": child_env.get("MESA_D3D12_DEFAULT_ADAPTER_NAME"),
    }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return mode, child_env, str(config_path.relative_to(REPO_ROOT))


def runtime_scenario(scenario: dict[str, Any], runtime_id: str) -> tuple[str, dict[str, Any]]:
    """Translate one checked matrix case to the transient Gazebo launch input."""
    scenario_name = f"runtime/{runtime_id}/{scenario['scenario_id']}"
    speed = float(scenario["vehicle_motion"]["speed_profile"][0]["speed_mps"])
    placements = [
        item for item in scenario["obstacle_placements"]
        if -14.0 <= float(item["pose"]["x_m"]) <= 14.0
        and -2.5 <= float(item["pose"]["y_m"]) <= 2.5
    ]
    launch_config = {
        "scenario_id": scenario_name,
        "split": "runtime",
        "world_id": scenario["world_id"],
        "world": scenario["world_path"],
        "world_name": "rc_module_matrix",
        "seed": int(scenario["seed"]),
        "start_pose": scenario["start_pose"],
        "goal_pose": {"x_m": scenario["start_pose"]["x_m"] + 1.0,
                      "y_m": scenario["start_pose"]["y_m"], "z_m": scenario["start_pose"]["z_m"],
                      "yaw_rad": scenario["start_pose"]["yaw_rad"]},
        "obstacles": [
            {"entity": item["entity_id"], "pose": item["pose"]}
            for item in placements
        ],
        "obstacle_placements": placements,
        "target_speed_mps": max(abs(speed), 0.01),
        "commanded_speed_mps": max(abs(speed), 0.01),
        "speed_cap_mps": max(abs(speed), 0.01),
        "stop_distance_m": 0.5,
        "corridor_half_width_m": 1.0,
        "ground_z_min_m": -0.01,
        "run_timeout_sim_s": float(scenario.get("setup", {}).get("capture_duration_sim_s", 2.0)) + 5.0,
        "course_bounds_m": {"x": [-14.0, 14.0], "y": [-2.5, 2.5]},
        "expected_stop_state": "drive",
    }
    if scenario["world_id"] == "rc_module_matrix":
        setup = scenario.get("setup", {})
        launch_config["lighting_preset"] = setup.get("lighting_preset", "neutral_day")
        launch_config["quality_preset"] = setup.get("quality_preset", "high")
        launch_config["floor_texture"] = setup.get("floor_texture", "floor_asphalt_001")
        launch_config["object_instances"] = [
            {
                "entity": item["entity_id"],
                "asset_id": item.get("asset_id") or _default_asset(item["entity_id"]),
            }
            for item in placements
        ]
        # Runtime controls are interpreted by the capture node and do not
        # change a checked-in vehicle or model profile.
        launch_config["matrix_setup"] = setup
        launch_config["vehicle_motion_profile"] = scenario["vehicle_motion"]["speed_profile"]
        for obstacle in placements:
            if obstacle.get("speed_profile"):
                launch_config.setdefault("moving_entities", {})[obstacle["entity_id"]] = {
                    "start_pose": obstacle["pose"],
                    "speed_profile": obstacle["speed_profile"],
                }
    return scenario_name, launch_config


def _default_asset(entity_id: str) -> str:
    """Select a locked high-fidelity visual for an RC fixture class."""
    if entity_id.startswith("cone"):
        return "cone_clean_01"
    if entity_id.startswith("pedestrian"):
        return "pedestrian_cross_step_01"
    if entity_id.startswith("vehicle"):
        return "vehicle_compact_suv_01"
    raise RuntimeError(f"no research visual is locked for fixture {entity_id!r}")


def package_config_root() -> Path:
    """Find the installed car_bringup config tree; fail until it was rebuilt."""
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share = Path(get_package_share_directory("car_bringup"))
    except (ImportError, LookupError) as exc:
        raise RuntimeError("source ROS 2 Lyrical and build car_bringup before running the matrix") from exc
    try:
        package_share.resolve().relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise RuntimeError(
            "car_bringup is resolving from another workspace; open a fresh terminal, "
            "source /opt/ros/lyrical/setup.bash, then source this checkout's "
            "ros2_ws/install/setup.bash"
        ) from exc
    root = package_share / "config"
    if not (root / "rc_module_matrix").is_dir():
        raise RuntimeError("installed matrix configs are missing; rebuild car_bringup to install them")
    return root


def write_runtime_scenario(scenario: dict[str, Any], runtime_id: str) -> tuple[str, Path]:
    """Write only to the ignored colcon install tree, then remove after launch."""
    scenario_name, launch_config = runtime_scenario(scenario, runtime_id)
    root = package_config_root() / "rc_module_matrix" / "runtime" / runtime_id
    scenario_path = root.joinpath(*scenario_name.split("/")[2:]).with_suffix(".yaml")
    scenario_path.parent.mkdir(parents=True, exist_ok=False)
    scenario_path.write_text(yaml.safe_dump(launch_config, sort_keys=False), encoding="utf-8")
    return scenario_name, scenario_path


def capture_lidar(
    scenario: dict[str, Any], *, output_dir: Path, wall_timeout_s: float = 45.0,
) -> dict[str, Any]:
    """Run one passive sensor scene and return stamped LaserScan samples.

    RC_MODULE_MATRIX_RENDER_MODE=glx selects server-only WSLg GLX rendering;
    the default remains Gazebo's EGL headless-rendering mode.
    """
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
    except ImportError as exc:
        raise RuntimeError("ROS 2 sensor message packages are unavailable in this Python environment") from exc

    runtime_id = str(uuid.uuid4())
    scenario_name, scenario_path = write_runtime_scenario(scenario, runtime_id)
    output_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / "gazebo-launch.log"
    scans: list[dict[str, Any]] = []
    pose_commands: list[dict[str, Any]] = []
    clock_ns = 0
    sim_start_ns: int | None = None
    next_pose_elapsed_s = 0.0
    node = None
    process = None
    rclpy.init(args=None)
    try:
        node = Node("rc_module_matrix_capture")

        moving = scenario.get("setup", {}).get("moving_target_order")
        target_id = scenario.get("setup", {}).get("target_entity")
        target = next((item for item in scenario.get("obstacle_placements", [])
                       if item["entity_id"] == target_id), None)
        pose_interval_s = float(scenario.get("setup", {}).get("pose_update_interval_s", 0.2))
        if moving and (target is None or not target.get("speed_profile") or pose_interval_s <= 0):
            raise RuntimeError("moving LiDAR case needs a target pose, speed profile, and positive update interval")

        def target_x_at(elapsed_s: float) -> float:
            assert target is not None
            x = float(target["pose"]["x_m"])
            profile = target["speed_profile"]
            for index, segment in enumerate(profile):
                begin = float(segment["sim_time_s"])
                end = float(profile[index + 1]["sim_time_s"]) if index + 1 < len(profile) else elapsed_s
                duration = max(0.0, min(elapsed_s, end) - begin)
                x += float(segment["speed_mps"]) * duration
                if elapsed_s <= end:
                    break
            return x

        def on_clock(message: Clock) -> None:
            nonlocal clock_ns, sim_start_ns, next_pose_elapsed_s
            clock_ns = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
            if sim_start_ns is None and clock_ns > 0:
                sim_start_ns = clock_ns
            if moving and sim_start_ns is not None:
                elapsed_s = (clock_ns - sim_start_ns) / 1e9
                if elapsed_s + 1e-9 >= next_pose_elapsed_s:
                    assert target is not None
                    x_m = target_x_at(elapsed_s)
                    pose = target["pose"]
                    request = (
                        f'name: "{target_id}" position {{ x: {x_m:.6f} y: {float(pose["y_m"]):.6f} '
                        f'z: {float(pose.get("z_m", 0.0)):.6f} }} orientation {{ x: 0 y: 0 '
                        f'z: {math.sin(float(pose.get("yaw_rad", 0.0)) / 2.0):.9f} '
                        f'w: {math.cos(float(pose.get("yaw_rad", 0.0)) / 2.0):.9f} }}'
                    )
                    completed = subprocess.run(
                        ["gz", "service", "-s", "/world/rc_module_matrix/set_pose/blocking",
                         "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                         "--timeout", "10000", "--req", request],
                        check=False, capture_output=True, text=True, timeout=12.0,
                    )
                    response = (completed.stdout + completed.stderr).strip()
                    success = completed.returncode == 0 and "data: true" in response.lower()
                    pose_commands.append({
                        "stamp_sim_ns": clock_ns,
                        "elapsed_sim_s": elapsed_s,
                        "entity_id": target_id,
                        "requested_x_m": x_m,
                        "service_success": success,
                        "service_response": response,
                    })
                    if not success:
                        raise RuntimeError(
                            "Gazebo rejected a fixed-seed moving target pose: "
                            f"{response or f'exit={completed.returncode}'}"
                        )
                    next_pose_elapsed_s += pose_interval_s

        def on_scan(message: LaserScan) -> None:
            scans.append({
                "stamp_sim_ns": int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec),
                "frame_id": message.header.frame_id,
                "angle_min": float(message.angle_min),
                "angle_max": float(message.angle_max),
                "angle_increment": float(message.angle_increment),
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
                "ranges": [float(value) for value in message.ranges],
            })

        node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
        node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
        render_mode, child_env, render_config_ref = _select_sensor_renderer(output_dir)
        render_configuration_path = REPO_ROOT / render_config_ref
        command = [
            "ros2", "launch", "car_bringup", "obstacle_course.launch.py",
            "behavior:=sensors",
            f"scenario:={scenario_name}",
            f"headless:={'true' if render_mode == 'headless' else 'glx'}",
        ]
        with log_path.open("wb") as log_file:
            process = subprocess.Popen(
                command, env=child_env, stdout=log_file, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            started = time.monotonic()
            duration_ns = int(float(scenario.get("setup", {}).get("capture_duration_sim_s", 2.0)) * 1_000_000_000)
            while time.monotonic() - started < wall_timeout_s:
                rclpy.spin_once(node, timeout_sec=0.05)
                if process.poll() is not None:
                    raise RuntimeError(f"Gazebo launch exited with status {process.returncode}; see {log_path}")
                if clock_ns:
                    if scans and clock_ns - (sim_start_ns or clock_ns) >= duration_ns:
                        break
            else:
                raise RuntimeError(f"timed out waiting for stamped /scan messages; see {log_path}")
    finally:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            scenario_path.unlink(missing_ok=True)
            parent = scenario_path.parent
            while parent != package_config_root() and parent.is_dir() and not any(parent.iterdir()):
                parent.rmdir()
                parent = parent.parent
        except OSError:
            pass
    if not scans:
        raise RuntimeError(f"Gazebo produced no /scan messages; see {log_path}")
    scans.sort(key=lambda item: item["stamp_sim_ns"])
    return {"clock_start_sim_ns": sim_start_ns, "clock_last_sim_ns": clock_ns,
            "scans": scans, "pose_commands": pose_commands, "log_ref": str(log_path),
            "render_configuration_ref": str(render_configuration_path)}


def capture_imu_odom(
    scenario: dict[str, Any], *, output_dir: Path, wall_timeout_s: float = 60.0,
) -> dict[str, Any]:
    """Capture unmodified IMU and odometry messages with simulation timestamps."""
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu
    except ImportError as exc:
        raise RuntimeError("ROS IMU/odometry capture message packages are unavailable") from exc

    runtime_id = str(uuid.uuid4())
    scenario_name, scenario_path = write_runtime_scenario(scenario, runtime_id)
    output_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / "gazebo-launch.log"
    imu_samples: list[dict[str, Any]] = []
    odom_samples: list[dict[str, Any]] = []
    clock_ns = 0
    clock_samples: list[int] = []
    node = None
    process = None
    rclpy.init(args=None)
    try:
        node = Node("rc_matrix_imu_odom_capture")
        command_pub = node.create_publisher(Twist, "/model/gazebo_truck/cmd_vel", 10)
        motion_profile = scenario["vehicle_motion"]["speed_profile"]
        yaw_rate = float(scenario.get("setup", {}).get("yaw_rate_rps", 0.0))
        next_motion = 0
        motion_start_ns = None
        clock_reset_triggered = False
        clock_reset_count = 0

        def time_ns(stamp: Any) -> int:
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

        def on_clock(message: Clock) -> None:
            nonlocal clock_ns, motion_start_ns, next_motion, clock_reset_triggered, clock_reset_count
            clock_ns = time_ns(message.clock)
            clock_samples.append(clock_ns)
            pause_at = scenario.get("setup", {}).get("pause_and_reset_at_sim_s")
            if pause_at is not None and not clock_reset_triggered and motion_start_ns is not None:
                if (clock_ns - motion_start_ns) / 1e9 >= float(pause_at):
                    clock_reset_triggered = True
                    def control(request: str) -> None:
                        result = subprocess.run([
                            "gz", "service", "-s", "/world/rc_module_matrix/control",
                            "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                            "--timeout", "2000", "--req", request,
                        ], capture_output=True, text=True, timeout=4.0)
                        if result.returncode != 0 or "data: true" not in result.stdout.lower():
                            raise RuntimeError(f"Gazebo world-control request failed: {result.stdout} {result.stderr}")
                    try:
                        control("pause: true")
                        pause_wall_s = float(scenario["setup"].get("pause_wall_s", 0.25))
                        time.sleep(pause_wall_s)
                        control("reset { time_only: true }")
                        clock_reset_count += 1
                        control("pause: false")
                    except Exception as exc:
                        raise RuntimeError(f"could not observe configured pause/clock reset: {exc}") from exc
            if motion_start_ns is None and clock_ns > 0:
                motion_start_ns = clock_ns
            if motion_start_ns is None:
                return
            elapsed_s = (clock_ns - motion_start_ns) / 1e9
            while next_motion < len(motion_profile) and elapsed_s >= float(motion_profile[next_motion]["sim_time_s"]):
                command = Twist()
                command.linear.x = float(motion_profile[next_motion]["speed_mps"])
                command.angular.z = yaw_rate
                command_pub.publish(command)
                next_motion += 1

        def on_imu(message: Imu) -> None:
            imu_samples.append({
                "stamp_sim_ns": time_ns(message.header.stamp),
                "frame_id": str(message.header.frame_id),
                "angular_velocity_radps": [
                    float(message.angular_velocity.x), float(message.angular_velocity.y),
                    float(message.angular_velocity.z),
                ],
                "linear_acceleration_mps2": [
                    float(message.linear_acceleration.x), float(message.linear_acceleration.y),
                    float(message.linear_acceleration.z),
                ],
                "orientation_xyzw": [
                    float(message.orientation.x), float(message.orientation.y),
                    float(message.orientation.z), float(message.orientation.w),
                ],
                "angular_velocity_covariance": [float(value) for value in message.angular_velocity_covariance],
                "linear_acceleration_covariance": [float(value) for value in message.linear_acceleration_covariance],
            })

        def on_odom(message: Odometry) -> None:
            pose = message.pose.pose
            twist = message.twist.twist
            odom_samples.append({
                "stamp_sim_ns": time_ns(message.header.stamp),
                "frame_id": str(message.header.frame_id),
                "child_frame_id": str(message.child_frame_id),
                "position_m": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
                "orientation_xyzw": [
                    float(pose.orientation.x), float(pose.orientation.y),
                    float(pose.orientation.z), float(pose.orientation.w),
                ],
                "linear_velocity_mps": [float(twist.linear.x), float(twist.linear.y), float(twist.linear.z)],
                "angular_velocity_radps": [float(twist.angular.x), float(twist.angular.y), float(twist.angular.z)],
            })

        node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
        node.create_subscription(Imu, "/imu/data", on_imu, qos_profile_sensor_data)
        node.create_subscription(Odometry, "/odom", on_odom, qos_profile_sensor_data)
        command = [
            "ros2", "launch", "car_bringup", "obstacle_course.launch.py",
            "behavior:=sensors", f"scenario:={scenario_name}", "headless:=true",
        ]
        with log_path.open("wb") as log_file:
            process = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT, start_new_session=True)
            started = time.monotonic()
            sim_start_ns = None
            clock_start_ns = None
            duration_ns = int(float(scenario.get("setup", {}).get("capture_duration_sim_s", 3.0)) * 1e9)
            while time.monotonic() - started < wall_timeout_s:
                rclpy.spin_once(node, timeout_sec=0.05)
                if process.poll() is not None:
                    raise RuntimeError(f"IMU/odometry launch exited with status {process.returncode}; see {log_path}")
                if clock_ns:
                    if sim_start_ns is None:
                        sim_start_ns = clock_ns
                        clock_start_ns = clock_ns
                    if (
                        clock_ns - sim_start_ns >= duration_ns
                        and imu_samples and odom_samples
                    ):
                        break
            else:
                raise RuntimeError(f"timed out waiting for simulation time, /imu/data, and /odom; see {log_path}")
    finally:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            scenario_path.unlink(missing_ok=True)
            parent = scenario_path.parent
            installed_root = package_config_root()
            while parent != installed_root and parent.is_dir() and not any(parent.iterdir()):
                parent.rmdir()
                parent = parent.parent
        except (OSError, RuntimeError):
            pass

    if not imu_samples or not odom_samples:
        raise RuntimeError("the simulator did not provide both IMU and odometry samples")
    if not scenario.get("setup", {}).get("pause_and_reset_at_sim_s"):
        imu_samples.sort(key=lambda item: item["stamp_sim_ns"])
        odom_samples.sort(key=lambda item: item["stamp_sim_ns"])
    capture = {
        "clock_start_sim_ns": clock_start_ns,
        "clock_last_sim_ns": clock_ns,
        "clock_samples_sim_ns": clock_samples,
        "clock_reset_count": clock_reset_count,
        "imu_samples": imu_samples,
        "odometry_samples": odom_samples,
        "log_ref": str(log_path.relative_to(REPO_ROOT)),
    }
    evidence_path = output_dir / "imu-odom-capture.json"
    evidence_path.write_text(json.dumps(capture, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    capture["evidence_ref"] = str(evidence_path.relative_to(REPO_ROOT))
    return capture


def capture_contact_tf(
    scenario: dict[str, Any], *, output_dir: Path, wall_timeout_s: float = 60.0,
) -> dict[str, Any]:
    """Capture contact identities and resolve every sensor frame at each sample time."""
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from ros_gz_interfaces.msg import Contacts
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, Imu, LaserScan
        from tf2_ros import Buffer, TransformListener
    except ImportError as exc:
        raise RuntimeError("ROS contact/TF capture dependencies are unavailable") from exc

    runtime_id = str(uuid.uuid4())
    scenario_name, scenario_path = write_runtime_scenario(scenario, runtime_id)
    output_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / "gazebo-launch.log"
    contacts: list[dict[str, Any]] = []
    sensor_samples: list[dict[str, Any]] = []
    clock_ns = 0
    clock_start_ns = None
    node = None
    process = None
    rclpy.init(args=None)
    try:
        node = Node("rc_matrix_contact_tf_capture")
        tf_buffer = Buffer(node=node)
        tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
        command_pub = node.create_publisher(Twist, "/model/gazebo_truck/cmd_vel", 10)
        motion_profile = scenario["vehicle_motion"]["speed_profile"]
        yaw_rate = float(scenario.get("setup", {}).get("yaw_rate_rps", 0.0))
        next_motion = 0

        def stamp(message: Any) -> int:
            return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

        def on_clock(message: Clock) -> None:
            nonlocal clock_ns, clock_start_ns, next_motion
            clock_ns = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
            if clock_start_ns is None and clock_ns > 0:
                clock_start_ns = clock_ns
            if clock_start_ns is None:
                return
            elapsed = (clock_ns - clock_start_ns) / 1e9
            while next_motion < len(motion_profile) and elapsed >= float(motion_profile[next_motion]["sim_time_s"]):
                command = Twist()
                command.linear.x = float(motion_profile[next_motion]["speed_mps"])
                command.angular.z = yaw_rate
                command_pub.publish(command)
                next_motion += 1

        def on_contacts(message: Contacts) -> None:
            for contact in message.contacts:
                contacts.append({
                    "stamp_sim_ns": stamp(message),
                    "collision1": {"name": str(contact.collision1.name), "id": int(contact.collision1.id)},
                    "collision2": {"name": str(contact.collision2.name), "id": int(contact.collision2.id)},
                    "position_count": len(contact.positions),
                })

        def sample(topic: str, message: Any) -> None:
            frame = str(message.header.frame_id)
            sensor_samples.append({"source": topic, "stamp_sim_ns": stamp(message), "frame_id": frame})

        node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
        node.create_subscription(Contacts, "/contacts", on_contacts, qos_profile_sensor_data)
        node.create_subscription(LaserScan, "/scan", lambda message: sample("/scan", message), qos_profile_sensor_data)
        node.create_subscription(Imu, "/imu/data", lambda message: sample("/imu/data", message), qos_profile_sensor_data)
        node.create_subscription(Image, "/camera/image_raw", lambda message: sample("/camera/image_raw", message), qos_profile_sensor_data)
        node.create_subscription(Image, "/camera/labels", lambda message: sample("/camera/labels", message), qos_profile_sensor_data)

        command = [
            "ros2", "launch", "car_bringup", "obstacle_course.launch.py",
            "behavior:=sensors", f"scenario:={scenario_name}", "headless:=true",
        ]
        with log_path.open("wb") as log_file:
            process = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT, start_new_session=True)
            started = time.monotonic()
            start = None
            duration_ns = int(float(scenario["setup"].get("capture_duration_sim_s", 2.0)) * 1e9)
            while time.monotonic() - started < wall_timeout_s:
                rclpy.spin_once(node, timeout_sec=0.05)
                if process.poll() is not None:
                    raise RuntimeError(f"contact/TF launch exited with status {process.returncode}; see {log_path}")
                if clock_ns:
                    if start is None:
                        start = clock_ns
                    if clock_ns - start >= duration_ns and sensor_samples:
                        break
            else:
                raise RuntimeError(f"timed out waiting for contact/TF samples; see {log_path}")

        required_frames = tuple(scenario["setup"].get("required_sensor_frames", ("lidar_link", "camera_link", "imu_link")))
        tf_results = []
        for row in sensor_samples:
            if row["source"] not in {"/scan", "/camera/image_raw", "/imu/data"}:
                continue
            try:
                tf_buffer.lookup_transform(
                    "base_link", row["frame_id"], Time(nanoseconds=row["stamp_sim_ns"]),
                    timeout=Duration(seconds=0.05),
                )
                resolved = True
                error = None
            except Exception as exc:
                resolved = False
                error = str(exc)
            tf_results.append({**row, "resolved_to_base_link": resolved, "error": error})
        resolved_frames = {row["frame_id"] for row in tf_results if row["resolved_to_base_link"]}
        missing_frames = sorted(set(required_frames) - resolved_frames)
        gaps = [row for row in tf_results if not row["resolved_to_base_link"]]
        capture = {
            "clock_start_sim_ns": clock_start_ns,
            "clock_last_sim_ns": clock_ns,
            "contacts": contacts,
            "sensor_sample_count": len(sensor_samples),
            "tf_samples": tf_results,
            "required_sensor_frames": list(required_frames),
            "resolved_sensor_frames": sorted(resolved_frames),
            "missing_sensor_frames": missing_frames,
            "tf_gap_count": len(gaps),
            "log_ref": str(log_path.relative_to(REPO_ROOT)),
        }
        evidence_path = output_dir / "contact-tf-capture.json"
        evidence_path.write_text(json.dumps(capture, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        capture["evidence_ref"] = str(evidence_path.relative_to(REPO_ROOT))
        return capture
    finally:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            scenario_path.unlink(missing_ok=True)
            parent = scenario_path.parent
            installed_root = package_config_root()
            while parent != installed_root and parent.is_dir() and not any(parent.iterdir()):
                parent.rmdir()
                parent = parent.parent
        except (OSError, RuntimeError):
            pass


def capture_command_path(
    scenario: dict[str, Any], *, output_dir: Path, wall_timeout_s: float = 60.0,
) -> dict[str, Any]:
    """Drive the configured controller and capture status/odometry response."""
    try:
        import rclpy
        from ackermann_msgs.msg import AckermannDriveStamped
        from diagnostic_msgs.msg import DiagnosticStatus
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError("ROS command-path capture dependencies are unavailable") from exc

    runtime_id = str(uuid.uuid4())
    scenario_name, scenario_path = write_runtime_scenario(scenario, runtime_id)
    output_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / "gazebo-launch.log"
    clock_ns = 0
    clock_start_ns = None
    command_sent_ns = None
    statuses: list[dict[str, Any]] = []
    odometry: list[dict[str, Any]] = []
    controller_twists: list[dict[str, Any]] = []
    command_records: list[dict[str, Any]] = []
    fault_states: list[dict[str, Any]] = []
    relayed_scan_stamps: list[int] = []
    node = None
    process = None
    next_command_elapsed_s = None
    rclpy.init(args=None)
    try:
        node = Node("rc_matrix_command_path_capture")
        command_pub = node.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        setup = scenario["setup"]

        def to_ns(stamp: Any) -> int:
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

        def on_clock(message: Clock) -> None:
            nonlocal clock_ns, clock_start_ns, command_sent_ns, next_command_elapsed_s
            clock_ns = to_ns(message.clock)
            if clock_start_ns is None and clock_ns > 0:
                clock_start_ns = clock_ns
            if clock_start_ns is None:
                return
            elapsed = (clock_ns - clock_start_ns) / 1e9
            command_at = float(setup.get("command_at_sim_s", 0.5))
            repeat_s = float(setup.get("command_repeat_interval_s", 0.0))
            repeat_until = float(setup.get("command_repeat_until_sim_s", command_at))
            if elapsed < command_at or (command_sent_ns is not None and (
                repeat_s <= 0 or elapsed > repeat_until or elapsed < float(next_command_elapsed_s)
            )):
                return
            command = AckermannDriveStamped()
            command.header.stamp = message.clock
            command.header.frame_id = "base_link"
            command.drive.speed = float(setup.get("command_speed_mps", 0.0))
            command.drive.steering_angle = float(setup.get("command_steering_rad", 0.0))
            command_pub.publish(command)
            if command_sent_ns is None:
                command_sent_ns = clock_ns
            next_command_elapsed_s = elapsed + repeat_s
            command_records.append({
                "stamp_sim_ns": clock_ns,
                "speed_mps": float(command.drive.speed),
                "steering_angle_rad": float(command.drive.steering_angle),
            })

        def on_status(message: DiagnosticStatus) -> None:
            raw_level = message.level
            level = int.from_bytes(raw_level, "little") if isinstance(raw_level, bytes) else int(raw_level)
            statuses.append({
                "stamp_sim_ns": clock_ns,
                "level": level,
                "message": str(message.message),
                "values": {str(item.key): str(item.value) for item in message.values},
            })

        def on_odom(message: Odometry) -> None:
            odometry.append({
                "stamp_sim_ns": to_ns(message.header.stamp),
                "frame_id": str(message.header.frame_id),
                "child_frame_id": str(message.child_frame_id),
                "position_m": [float(message.pose.pose.position.x), float(message.pose.pose.position.y)],
                "linear_speed_mps": float(message.twist.twist.linear.x),
                "yaw_rate_radps": float(message.twist.twist.angular.z),
            })

        def on_controller_twist(message: Twist) -> None:
            controller_twists.append({
                "stamp_sim_ns": clock_ns,
                "linear_x_mps": float(message.linear.x),
                "angular_z_radps": float(message.angular.z),
            })

        def on_fault_state(message: String) -> None:
            fault_states.append({"stamp_sim_ns": clock_ns, "state": str(message.data)})

        def on_relayed_scan(message: Any) -> None:
            relayed_scan_stamps.append(to_ns(message.header.stamp))

        node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
        node.create_subscription(DiagnosticStatus, "/vehicle_controller/status", on_status, 10)
        node.create_subscription(Odometry, "/odom", on_odom, qos_profile_sensor_data)
        node.create_subscription(
            Twist, "/model/gazebo_truck/cmd_vel", on_controller_twist, 10
        )
        node.create_subscription(String, "/matrix/lidar_fault_state", on_fault_state, 10)
        node.create_subscription(
            LaserScan, "/matrix/lidar_for_controller", on_relayed_scan, qos_profile_sensor_data
        )
        command = [
            "ros2", "launch", "car_bringup", "obstacle_course.launch.py",
            "behavior:=command_path", f"scenario:={scenario_name}", "headless:=true",
        ]
        with log_path.open("wb") as log_file:
            process = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT, start_new_session=True)
            started = time.monotonic()
            duration_ns = int(float(setup.get("capture_duration_sim_s", 2.0)) * 1e9)
            while time.monotonic() - started < wall_timeout_s:
                rclpy.spin_once(node, timeout_sec=0.05)
                if process.poll() is not None:
                    raise RuntimeError(f"command-path launch exited with status {process.returncode}; see {log_path}")
                if clock_start_ns is not None and clock_ns - clock_start_ns >= duration_ns and statuses:
                    break
            else:
                raise RuntimeError(f"timed out waiting for controller status/odometry; see {log_path}")
        if command_sent_ns is None:
            raise RuntimeError("the command scenario did not reach its scheduled command time")
        capture = {
            "clock_start_sim_ns": clock_start_ns,
            "clock_last_sim_ns": clock_ns,
            "command_sent_sim_ns": command_sent_ns,
            "commands": command_records,
            "statuses": statuses,
            "odometry": odometry,
            "controller_twists": controller_twists,
            "fault_states": fault_states,
            "relayed_scan_stamps_sim_ns": relayed_scan_stamps,
            "log_ref": str(log_path.relative_to(REPO_ROOT)),
        }
        evidence_path = output_dir / "command-path-capture.json"
        evidence_path.write_text(json.dumps(capture, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        capture["evidence_ref"] = str(evidence_path.relative_to(REPO_ROOT))
        return capture
    finally:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            scenario_path.unlink(missing_ok=True)
            parent = scenario_path.parent
            installed_root = package_config_root()
            while parent != installed_root and parent.is_dir() and not any(parent.iterdir()):
                parent.rmdir()
                parent = parent.parent
        except (OSError, RuntimeError):
            pass


def capture_camera(
    scenario: dict[str, Any], *, output_dir: Path, wall_timeout_s: float = 120.0,
    require_aligned_frames: bool = True,
) -> dict[str, Any]:
    """Capture camera frames, optionally requiring exact RGB/label stamp alignment."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from vision_msgs.msg import Detection2DArray
    except ImportError as exc:
        raise RuntimeError("ROS camera capture message packages are unavailable") from exc

    runtime_id = str(uuid.uuid4())
    scenario_name, scenario_path = write_runtime_scenario(scenario, runtime_id)
    output_dir.mkdir(parents=True, exist_ok=True)
    evidence_dir = output_dir / "frames"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / "gazebo-launch.log"
    images: OrderedDict[int, dict[str, Any]] = OrderedDict()
    labels: OrderedDict[int, dict[str, Any]] = OrderedDict()
    detections: OrderedDict[int, list[dict[str, Any]]] = OrderedDict()
    statuses: list[dict[str, Any]] = []
    clock_start_ns = None
    clock_ns = 0
    node = None
    process = None
    rclpy.init(args=None)
    try:
        node = Node("rc_matrix_camera_capture")

        def stamp(message: Image | Detection2DArray) -> int:
            return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

        def retain(mapping: OrderedDict, key: int, value: Any) -> None:
            mapping[key] = value
            mapping.move_to_end(key)
            while len(mapping) > 12:
                mapping.popitem(last=False)

        def on_clock(message: Clock) -> None:
            nonlocal clock_ns
            clock_ns = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)

        def on_status(message: String) -> None:
            statuses.append({"stamp_sim_ns": clock_ns, "value": str(message.data)})

        def on_image(message: Image) -> None:
            message_stamp = stamp(message)
            if message_stamp > 0:
                retain(images, message_stamp, {
                    "data": bytes(message.data), "width": int(message.width),
                    "height": int(message.height), "step": int(message.step),
                    "encoding": str(message.encoding), "frame_id": str(message.header.frame_id),
                })

        def on_label(message: Image) -> None:
            message_stamp = stamp(message)
            if message_stamp > 0:
                retain(labels, message_stamp, {
                    "data": bytes(message.data), "width": int(message.width),
                    "height": int(message.height), "step": int(message.step),
                    "encoding": str(message.encoding), "frame_id": str(message.header.frame_id),
                })

        def on_detection(message: Detection2DArray) -> None:
            message_stamp = stamp(message)
            if message_stamp <= 0:
                return
            rows = []
            for item in message.detections:
                if not item.results:
                    continue
                hypothesis = item.results[0].hypothesis
                center_x = float(item.bbox.center.position.x)
                center_y = float(item.bbox.center.position.y)
                width = float(item.bbox.size_x)
                height = float(item.bbox.size_y)
                x0 = round(center_x - (width - 1.0) / 2.0)
                y0 = round(center_y - (height - 1.0) / 2.0)
                rows.append({
                    "class_name": str(hypothesis.class_id),
                    "score": float(hypothesis.score),
                    "bbox_xyxy_inclusive_px": [x0, y0, x0 + round(width) - 1, y0 + round(height) - 1],
                })
            retain(detections, message_stamp, rows)

        node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
        node.create_subscription(Image, "/camera/image_raw", on_image, qos_profile_sensor_data)
        node.create_subscription(Image, "/camera/labels", on_label, qos_profile_sensor_data)
        node.create_subscription(Detection2DArray, "/detections", on_detection, qos_profile_sensor_data)
        node.create_subscription(String, "/perception/status", on_status, 10)

        setup = scenario.get("setup", {})
        expects_stale = bool(setup.get("expect_detector_stale"))
        capture_detector = bool(setup.get("capture_detector", not expects_stale))
        render_mode, child_env, render_config_ref = _select_sensor_renderer(output_dir)
        command = [
            "ros2", "launch", "car_bringup", "obstacle_course.launch.py",
            f"scenario:={scenario_name}",
            f"behavior:={'camera' if capture_detector else 'sensors'}",
            f"headless:={'true' if render_mode == 'headless' else 'glx'}",
        ]
        with log_path.open("wb") as log_file:
            process = subprocess.Popen(
                command, env=child_env, stdout=log_file, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            started = time.monotonic()
            sim_start_ns = None
            clock_start_ns = None
            duration_ns = int(float(setup.get("capture_duration_sim_s", 2.0)) * 1_000_000_000)
            while time.monotonic() - started < wall_timeout_s:
                rclpy.spin_once(node, timeout_sec=0.05)
                if process.poll() is not None:
                    raise RuntimeError(f"camera launch exited with status {process.returncode}; see {log_path}")
                if clock_ns:
                    if sim_start_ns is None:
                        sim_start_ns = clock_ns
                        clock_start_ns = clock_ns
                    detector_observed = bool(detections)
                    aligned_camera_frame_observed = (
                        bool(set(images).intersection(labels))
                        if require_aligned_frames else bool(labels)
                    )
                    detector_condition_met = (
                        not capture_detector or expects_stale or detector_observed
                    )
                    if (
                        clock_ns - sim_start_ns >= duration_ns
                        and aligned_camera_frame_observed
                        and detector_condition_met
                    ):
                        break
            else:
                diagnostic_path = output_dir / "camera-capture-diagnostics.json"
                diagnostic_path.write_text(json.dumps({
                    "rgb_frame_count": len(images),
                    "label_frame_count": len(labels),
                    "detector_frame_count": len(detections),
                    "rgb_stamp_sim_ns": sorted(images),
                    "label_stamp_sim_ns": sorted(labels),
                    "detector_stamp_sim_ns": sorted(detections),
                    "clock_start_sim_ns": clock_start_ns,
                    "clock_last_sim_ns": clock_ns,
                }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
                raise RuntimeError(
                    f"timed out waiting for {'aligned RGB/label' if require_aligned_frames else 'label'} frames "
                    f"(rgb={len(images)}, labels={len(labels)}, sim_time={clock_ns}, "
                    f"diagnostics={diagnostic_path}); see {log_path}"
                )

        paired_stamps = (
            sorted(set(images) & set(labels))
            if require_aligned_frames else sorted(labels)
        )
        if require_aligned_frames and not paired_stamps:
            diagnostic_path = output_dir / "camera-capture-diagnostics.json"
            diagnostic_path.write_text(json.dumps({
                "rgb_frame_count": len(images),
                "label_frame_count": len(labels),
                "detector_frame_count": len(detections),
                "rgb_stamp_sim_ns": sorted(images),
                "label_stamp_sim_ns": sorted(labels),
                "detector_stamp_sim_ns": sorted(detections),
                "clock_start_sim_ns": clock_start_ns,
                "clock_last_sim_ns": clock_ns,
            }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            raise RuntimeError(
                "no RGB and label messages shared a simulation timestamp "
                f"(rgb={len(images)}, labels={len(labels)}, diagnostics={diagnostic_path})"
            )
        if capture_detector and expects_stale and detections:
            raise RuntimeError("detector output arrived in a case declared stale")
        if capture_detector and not expects_stale and not detections:
            raise RuntimeError("the frozen detector emitted no stamped results")
        if capture_detector and not expects_stale and not any(status["value"] == "ready" for status in statuses):
            raise RuntimeError("the frozen detector did not publish its ready status")
        frames = []
        selected_stamps = paired_stamps[-3:]
        if capture_detector and not expects_stale:
            selected_stamps = [message_stamp for message_stamp in selected_stamps if message_stamp in detections]
        for message_stamp in selected_stamps:
            label = labels[message_stamp]
            rgb = images.get(message_stamp)
            if require_aligned_frames and rgb is None:
                raise RuntimeError("aligned RGB image was not retained for a selected label frame")
            if rgb and (rgb["width"] != label["width"] or rgb["height"] != label["height"]):
                raise RuntimeError("RGB and label image dimensions do not match")
            stem = f"frame-{message_stamp}"
            label_path = evidence_dir / f"{stem}.labels-rgb8"
            label_path.write_bytes(label["data"])
            frame = {
                "stamp_sim_ns": message_stamp,
                "frame_id": (rgb or label)["frame_id"],
                "width": label["width"], "height": label["height"], "step": label["step"],
                "label_encoding": label["encoding"], "label_raw_path": str(label_path.relative_to(REPO_ROOT)),
                "detections": detections.get(message_stamp, []),
            }
            if rgb is not None:
                rgb_path = evidence_dir / f"{stem}.rgb8"
                rgb_path.write_bytes(rgb["data"])
                frame["rgb_encoding"] = rgb["encoding"]
                frame["rgb_raw_path"] = str(rgb_path.relative_to(REPO_ROOT))
            frames.append(frame)
        status_path = output_dir / "camera-capture.json"
        stale_observation = None
        if expects_stale:
            stale_observation = {
                "state": "stale",
                "source": "matrix_observer",
                "fault_injection": "detector process intentionally omitted",
                "detection_message_count": 0,
                "observation_window_sim_s": max(
                    0.0, (clock_ns - (clock_start_ns or clock_ns)) / 1_000_000_000,
                ),
            }
            statuses.append({
                "stamp_sim_ns": clock_ns,
                "value": "stale",
                "source": "matrix_observer",
                "reason": stale_observation["fault_injection"],
            })
        status_doc = {
            "frames": frames, "perception_status": statuses,
            "clock_start_sim_ns": clock_start_ns, "clock_last_sim_ns": clock_ns,
            "detector_staleness": stale_observation,
        }
        status_path.write_text(json.dumps(status_doc, indent=2) + "\n", encoding="utf-8")
        for frame in frames:
            frame["evidence_ref"] = str(status_path.relative_to(REPO_ROOT))
        return {
            "frames": frames, "log_ref": str(log_path.relative_to(REPO_ROOT)),
            "render_configuration_ref": render_config_ref,
            "clock_start_sim_ns": clock_start_ns, "clock_last_sim_ns": clock_ns,
            "perception_status": statuses, "detector_staleness": stale_observation,
        }
    finally:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            scenario_path.unlink(missing_ok=True)
            parent = scenario_path.parent
            installed_root = package_config_root()
            while parent != installed_root and parent.is_dir() and not any(parent.iterdir()):
                parent.rmdir()
                parent = parent.parent
        except (OSError, RuntimeError):
            pass
