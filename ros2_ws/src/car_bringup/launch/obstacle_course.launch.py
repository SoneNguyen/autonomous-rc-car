"""Launch one deterministic RC sensor/actuation matrix scenario."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from car_bringup.research_vehicle_overlay import create_matrix_vehicle_overlay


GAZEBO_VERSION = "10.5.0"
_ACTIVE_RUNTIME_OVERLAYS = []


def _cleanup_runtime_overlays(_context, *_args, **_kwargs):
    while _ACTIVE_RUNTIME_OVERLAYS:
        _ACTIVE_RUNTIME_OVERLAYS.pop().cleanup()
    return []


def _repository_root(package_share: Path) -> Path:
    for candidate in (package_share, *package_share.parents):
        if (candidate / "simulation/worlds/rc_module_matrix.sdf").is_file():
            return candidate
    raise RuntimeError("repository root not found; keep the checkout on the WSL Linux filesystem")


def _launch_setup(context, *_args, **_kwargs):
    scenario_name = LaunchConfiguration("scenario").perform(context)
    behavior = LaunchConfiguration("behavior").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower()
    gui_config = LaunchConfiguration("gui_config").perform(context).strip()
    bridge_config = LaunchConfiguration("bridge_config").perform(context).strip()
    sensor_set = LaunchConfiguration("sensor_set").perform(context).strip().lower()
    quality_preset = LaunchConfiguration("quality_preset").perform(context).strip().lower()
    if headless not in {"true", "false", "glx"}:
        raise RuntimeError("headless must be true, false, or glx (server-only WSLg rendering)")
    if behavior not in {"sensors", "camera", "command_path"}:
        raise RuntimeError(f"unsupported matrix behavior: {behavior}")
    if behavior == "command_path" and headless not in {"true", "glx"}:
        raise RuntimeError("command-path runs must be server-only")
    if gui_config and headless != "false":
        raise RuntimeError("gui_config can only be used when headless:=false")
    if sensor_set not in {"", "lidar", "camera", "imu", "contact", "none"}:
        raise RuntimeError("sensor_set must be empty, lidar, camera, imu, contact, or none")
    if quality_preset not in {"", "high", "low"}:
        raise RuntimeError("quality_preset must be empty, high, or low")
    if (sensor_set or quality_preset) and headless != "false":
        raise RuntimeError("sensor_set and quality_preset overrides are restricted to visible GUI demos")
    if not scenario_name or Path(scenario_name).is_absolute():
        raise RuntimeError("scenario must be a relative matrix scenario path")
    scenario_parts = Path(scenario_name).parts
    if any(part in {".", ".."} for part in scenario_parts):
        raise RuntimeError("scenario path cannot contain '.' or '..'")

    package_share = Path(get_package_share_directory("car_bringup"))
    repo = _repository_root(package_share)
    config_share = package_share / "config/rc_module_matrix"
    config_source = repo / "ros2_ws/src/car_bringup/config/rc_module_matrix"
    scenario_path = None
    for root in (config_share, config_source):
        candidate = (root / f"{scenario_name}.yaml").resolve()
        if candidate.is_relative_to(root.resolve()) and candidate.is_file():
            scenario_path = candidate
            break
    if scenario_path is None:
        raise RuntimeError(f"matrix scenario does not exist: {scenario_name}")
    scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    if scenario.get("scenario_id") != scenario_name or scenario.get("split") not in {
        "qualification", "pilot", "runtime"
    }:
        raise RuntimeError("runtime scenario identity or purpose is invalid")
    if scenario.get("world") != "simulation/worlds/rc_module_matrix.sdf":
        raise RuntimeError("matrix scenarios must use the research-asset world")
    matrix_setup = scenario.get("matrix_setup", {})

    world_path = repo / "artifacts/rc-car-matrix/worlds" / (scenario_name.replace("/", "_") + ".sdf")
    build = [
        "python3", str(repo / "tools/build_rc_matrix_world.py"),
        "--scenario", str(scenario_path),
        "--lock", str(config_source / "research_assets.lock.yaml"),
        "--output", str(world_path),
    ]
    if quality_preset:
        build.extend(["--quality-preset", quality_preset])
    subprocess.run(build, cwd=repo, check=True)

    vehicle_lock_path = config_source / "research_vehicle.lock.yaml"
    vehicle_lock = yaml.safe_load(vehicle_lock_path.read_text(encoding="utf-8"))
    vehicles = [asset for asset in vehicle_lock.get("assets", [])
                if asset.get("role") == "controlled_vehicle_visual_only"]
    if len(vehicles) != 1:
        raise RuntimeError("research vehicle lock must identify exactly one visual-only vehicle")
    vehicle = vehicles[0]
    vehicle_mesh = (
        repo / "artifacts/rc-car-matrix/assets/models/research" / vehicle["class_id"]
        / vehicle["asset_id"] / vehicle["file_name"]
    )
    if not vehicle_mesh.is_file():
        raise RuntimeError(
            "research vehicle mesh is missing; run python3 tools/fetch_matrix_assets.py "
            "--lock ros2_ws/src/car_bringup/config/rc_module_matrix/research_vehicle.lock.yaml"
        )
    if hashlib.sha256(vehicle_mesh.read_bytes()).hexdigest() != vehicle.get("file_sha256"):
        raise RuntimeError("research vehicle mesh does not match its SHA-256 lock")
    mesh_uri = f"model://research/{vehicle['class_id']}/{vehicle['asset_id']}/{vehicle['file_name']}"
    mesh_scale = float(vehicle["dimensions"]["visual_scale_factor"]["value"])

    quality = yaml.safe_load((config_source / "quality.yaml").read_text(encoding="utf-8"))
    quality_id = scenario.get("quality_preset", scenario.get("setup", {}).get("quality_preset", "high"))
    if quality_preset:
        quality_id = quality_preset
    if quality_id not in quality.get("presets", {}):
        raise RuntimeError(f"unknown camera quality preset {quality_id!r}")
    enabled_sensors = None
    if sensor_set:
        enabled_sensors = {
            "lidar": {"lidar"},
            "camera": {"camera"},
            "imu": {"imu"},
            "contact": {"contact"},
            "none": set(),
        }[sensor_set]
    overlay = create_matrix_vehicle_overlay(
        repo / "simulation/models/rc_car", quality["presets"][quality_id],
        research_vehicle_mesh_uri=mesh_uri, research_vehicle_scale=mesh_scale,
        enabled_sensors=enabled_sensors,
    )
    _ACTIVE_RUNTIME_OVERLAYS.append(overlay)

    models = [str(overlay.models_dir), str(repo / "simulation/models"),
              str(repo / "artifacts/rc-car-matrix/assets/models")]
    inherited = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join([*models, inherited] if inherited else models)
    sdf_path = os.pathsep.join([*models, os.environ.get("SDF_PATH", "")]).rstrip(os.pathsep)

    gz_args = ["sim", "--force-version", GAZEBO_VERSION, "-v", "3", "-r", "--seed", str(scenario["seed"])]
    if headless == "true":
        gz_args.extend(["-s", "--headless-rendering"])
    elif headless == "glx":
        # Use WSLg's accelerated GLX surface for sensors. Gazebo's --headless-rendering
        # selects EGL device rendering, which may fall back to software under WSLg.
        gz_args.append("-s")
    elif gui_config:
        gui_config_path = Path(gui_config).expanduser().resolve()
        if not gui_config_path.is_file():
            raise RuntimeError(f"GUI configuration does not exist: {gui_config_path}")
        gz_args.extend(["--gui-config", str(gui_config_path)])
    gz_args.append(str(world_path))
    gazebo = ExecuteProcess(cmd=["gz", *gz_args], cwd=str(repo), output="screen", emulate_tty=True)
    bridge_config_path = Path(bridge_config).expanduser().resolve() if bridge_config else config_share / "bridge.yaml"
    if not bridge_config_path.is_file():
        raise RuntimeError(f"matrix bridge configuration does not exist: {bridge_config_path}")
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="matrix_bridge",
        parameters=[{"config_file": str(bridge_config_path), "use_sim_time": True}],
        output="screen",
    )

    profile_path = package_share / "config/rc_vehicle.yaml"
    sensor_tf = Node(
        package="car_navigation", executable="sensor_tf_node", name="sensor_tf_node",
        parameters=[{"use_sim_time": True, "profile_path": str(profile_path)}], output="screen",
    )
    matrix_nodes = [sensor_tf]
    if behavior == "camera":
        detector = Node(
            package="car_navigation", executable="object_detection_node", name="object_detection_node",
            parameters=[{
                "use_sim_time": True, "profile_path": str(profile_path),
                "model_path": str(repo / "models/rc_module_matrix/detector.onnx"),
            }], output="screen",
        )
        matrix_nodes.append(detector)
    elif behavior == "command_path":
        rc = yaml.safe_load(profile_path.read_text(encoding="utf-8"))["presets"]["rc_car"]
        sensor_remaps = []
        if matrix_setup.get("fault_injection") == "suppress_lidar_after_sim_time":
            relay = Node(
                package="car_navigation", executable="matrix_lidar_fault_relay",
                name="matrix_lidar_fault_relay", parameters=[{
                    "use_sim_time": True,
                    "drop_after_sim_s": float(matrix_setup["stale_after_s"]),
                }], output="screen",
            )
            matrix_nodes.append(relay)
            sensor_remaps = [("/scan", "/matrix/lidar_for_controller")]
        controller = Node(
            package="car_autonomy_nodes", executable="safe_drive_controller",
            name="vehicle_controller_node", parameters=[{
                "use_sim_time": True, "mode": "rc_navigation",
                "wheelbase_m": rc["geometry"]["wheelbase_m"]["value"],
                "max_steering_angle_rad": rc["steering"]["effective_center_wheel_limit_rad"]["value"],
                "rc_max_speed_mps": rc["steering"]["max_speed_mps"]["value"],
                "rc_max_yaw_rate_radps": rc["steering"]["max_yaw_rate_radps"]["value"],
            }], remappings=sensor_remaps, output="screen",
        )
        matrix_nodes.append(controller)

    start_bridge = RegisterEventHandler(OnProcessStart(
        target_action=gazebo,
        on_start=[LogInfo(msg="Simulator started; bringing up the matrix bridge"), bridge],
    ))
    start_nodes = RegisterEventHandler(OnProcessStart(
        target_action=bridge,
        on_start=[LogInfo(msg=f"Matrix bridge started; behavior={behavior}"),
                  TimerAction(period=1.0, actions=matrix_nodes)],
    ))
    shutdown_handlers = [start_bridge, start_nodes]
    for node in matrix_nodes:
        if node.name == "vehicle_controller_node":
            shutdown_handlers.append(RegisterEventHandler(OnProcessExit(
                target_action=node,
                on_exit=[Shutdown(reason="matrix command controller exited")],
            )))
    shutdown_handlers.append(RegisterEventHandler(OnProcessExit(
        target_action=gazebo,
        on_exit=[
            OpaqueFunction(function=_cleanup_runtime_overlays),
            Shutdown(reason="Gazebo exited; stopping matrix nodes"),
        ],
    )))
    gui_render_environment = []
    if headless == "false" and Path("/mnt/wslg").is_dir():
        if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
            gui_render_environment.append(SetEnvironmentVariable(
                "GALLIUM_DRIVER", os.environ.get("GALLIUM_DRIVER", "d3d12")
            ))
            adapter = os.environ.get("MESA_D3D12_DEFAULT_ADAPTER_NAME")
            if not adapter:
                try:
                    adapters = subprocess.run(
                        ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
                        check=False, capture_output=True, text=True, timeout=3.0,
                    ).stdout.splitlines()
                except (OSError, subprocess.TimeoutExpired):
                    adapters = []
                if adapters:
                    adapter = adapters[0].strip()
            if adapter:
                gui_render_environment.append(SetEnvironmentVariable(
                    "MESA_D3D12_DEFAULT_ADAPTER_NAME", adapter
                ))
    return [
        *gui_render_environment,
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("SDF_PATH", sdf_path),
        LogInfo(msg=f"Matrix scenario {scenario['scenario_id']} seed={scenario['seed']} research vehicle={vehicle['asset_id']}"),
        *shutdown_handlers,
        gazebo,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="lidar/range-min-at"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("behavior", default_value="sensors"),
        DeclareLaunchArgument("gui_config", default_value=""),
        DeclareLaunchArgument("bridge_config", default_value=""),
        DeclareLaunchArgument("sensor_set", default_value=""),
        DeclareLaunchArgument("quality_preset", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
