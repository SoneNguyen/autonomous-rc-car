"""Launch the protected obstacle-course world and its contract bridge."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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

from car_bringup.scenario import load_scenario
from car_bringup.runtime_overlay import create_filtered_model_overlay


GAZEBO_VERSION = "10.5.0"
AUTONOMOUS_MODES = {"command_test", "lidar", "ai", "evaluation"}
IMPLEMENTED_MODES = {"sensors", "command_test"}
_ACTIVE_RUNTIME_OVERLAYS = []


def _cleanup_runtime_overlays(_context, *_args, **_kwargs):
    while _ACTIVE_RUNTIME_OVERLAYS:
        _ACTIVE_RUNTIME_OVERLAYS.pop().cleanup()
    return []


def _find_repository_root(package_share: Path) -> Path:
    for candidate in (package_share, *package_share.parents):
        if (candidate / "simulation" / "worlds" / "obstacle.sdf").is_file():
            return candidate
    raise RuntimeError(
        "could not find the repository root above car_bringup package share; "
        "keep the ROS checkout in ~/src/gz-ws inside WSL"
    )


def _launch_setup(context, *_args, **_kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    scenario_name = LaunchConfiguration("scenario").perform(context)
    headless_text = LaunchConfiguration("headless").perform(context).lower()
    if headless_text not in {"true", "false"}:
        raise RuntimeError("headless must be exactly true or false")
    headless = headless_text == "true"

    if mode not in IMPLEMENTED_MODES | AUTONOMOUS_MODES:
        raise RuntimeError(f"unsupported mode: {mode}")
    if mode in AUTONOMOUS_MODES and not headless:
        raise RuntimeError(
            f"mode {mode} is autonomous and must be server-only/headless; "
            "the Gazebo GUI keyboard is another command source"
        )
    if mode not in IMPLEMENTED_MODES:
        raise RuntimeError(f"mode {mode} is not implemented yet")

    if not scenario_name or Path(scenario_name).is_absolute():
        raise RuntimeError("scenario must be a relative name such as training/sensor_smoke")
    scenario_parts = Path(scenario_name).parts
    if any(part in {".", ".."} for part in scenario_parts):
        raise RuntimeError("scenario path cannot contain '.' or '..'")

    package_share = Path(get_package_share_directory("car_bringup"))
    repository_root = _find_repository_root(package_share)
    scenarios_dir = package_share / "config" / "scenarios"
    scenario_path = scenarios_dir / f"{scenario_name}.yaml"
    try:
        scenario_path.relative_to(scenarios_dir)
    except ValueError as exc:
        raise RuntimeError("scenario must resolve inside the installed scenarios directory") from exc
    if not scenario_path.is_file():
        raise RuntimeError(f"scenario file does not exist: {scenario_name}")
    resolved_scenario = scenario_path.resolve()
    source_scenarios = repository_root / "ros2_ws" / "src" / "car_bringup" / "config" / "scenarios"
    allowed_scenario_roots = (scenarios_dir.resolve(), source_scenarios.resolve())
    if not any(
        resolved_scenario == allowed_root or resolved_scenario.is_relative_to(allowed_root)
        for allowed_root in allowed_scenario_roots
    ):
        raise RuntimeError("scenario file resolves outside the checked-in scenario directory")

    scenario = load_scenario(resolved_scenario)
    if scenario["split"] != scenario_parts[0]:
        raise RuntimeError(
            f"scenario split mismatch: path says {scenario_parts[0]}, "
            f"config says {scenario['split']}"
        )
    world_path = (repository_root / scenario["world"]).resolve()
    try:
        world_path.relative_to(repository_root.resolve())
    except ValueError as exc:
        raise RuntimeError("scenario world must remain inside the repository") from exc
    if not world_path.is_file():
        raise RuntimeError(f"scenario world does not exist: {world_path}")

    source_models_path = repository_root / "simulation" / "models"
    models_paths = []
    runtime_overlay = None
    if mode in AUTONOMOUS_MODES:
        runtime_overlay = create_filtered_model_overlay(source_models_path / "car")
        _ACTIVE_RUNTIME_OVERLAYS.append(runtime_overlay)
        models_paths.append(str(runtime_overlay.models_dir))
    models_paths.append(str(source_models_path))
    inherited_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join([*models_paths, inherited_path] if inherited_path else models_paths)
    bridge_config = package_share / "config" / "ros_gz_bridge.yaml"

    gz_args = [
        "sim",
        "--force-version",
        GAZEBO_VERSION,
        "-v",
        "4",
        "-r",
        "--seed",
        str(scenario["seed"]),
    ]
    if headless:
        gz_args.extend(["-s", "--headless-rendering"])
    gz_args.append(str(world_path))

    gazebo = ExecuteProcess(
        cmd=["gz", *gz_args],
        cwd=str(repository_root),
        output="screen",
        emulate_tty=True,
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_ros_bridge",
        parameters=[{"config_file": str(bridge_config)}, {"use_sim_time": True}],
        output="screen",
    )
    start_bridge_after_gazebo = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[LogInfo(msg="Gazebo started; starting ros_gz_bridge"), bridge],
        )
    )

    event_handlers = [start_bridge_after_gazebo]
    if mode == "command_test":
        controller = Node(
            package="car_autonomy_nodes",
            executable="safe_drive_controller",
            name="safe_drive_controller",
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
        start_controller_after_bridge = RegisterEventHandler(
            OnProcessStart(
                target_action=bridge,
                on_start=[
                    LogInfo(msg="Bridge process started; checking authority before controller startup"),
                    TimerAction(period=2.0, actions=[controller]),
                ],
            )
        )
        shutdown_if_controller_exits = RegisterEventHandler(
            OnProcessExit(
                target_action=controller,
                on_exit=[Shutdown(reason="safe_drive_controller exited; autonomous run stopped")],
            )
        )
        event_handlers.extend([start_controller_after_bridge, shutdown_if_controller_exits])
    if runtime_overlay is not None:
        event_handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gazebo,
                    on_exit=[OpaqueFunction(function=_cleanup_runtime_overlays)],
                )
            )
        )

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        LogInfo(
            msg=(
                f"Scenario {scenario['scenario_id']} seed={scenario['seed']}"
                + ("; autonomous model copy filters exactly five keyboard-trigger plugins" if runtime_overlay else "")
            )
        ),
        *event_handlers,
        gazebo,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="sensors",
                description="sensors is inspection-only; autonomy modes must be headless",
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value="training/sensor_smoke",
                description="Scenario path under car_bringup/config/scenarios without .yaml",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="true enables server-only headless rendering; false is GUI inspection",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
