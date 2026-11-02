"""Launch a fixed-seed RC obstacle-course scenario and behavior demo."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml



GAZEBO_VERSION = "10.5.0"

def _wslg_render_environment(headless: str):
    """Select WSLg's D3D12 adapter for visible Gazebo launches when available."""
    if headless != "false" or not Path("/mnt/wslg").is_dir():
        return []
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return []

    actions = [SetEnvironmentVariable(
        "GALLIUM_DRIVER", os.environ.get("GALLIUM_DRIVER", "d3d12")
    )]
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
        actions.append(SetEnvironmentVariable("MESA_D3D12_DEFAULT_ADAPTER_NAME", adapter))
    return actions


def _repository_root(package_share: Path) -> Path:
    source_file = Path(__file__).resolve()
    candidates = (Path.cwd(), source_file.parent, *source_file.parents, package_share, *package_share.parents)
    for candidate in candidates:
        if (candidate / "simulation/worlds/obstacle_condensed.sdf").is_file():
            return candidate
    raise RuntimeError("repository root not found; keep the checkout on the WSL Linux filesystem")


def _launch_obstacle_course_setup(context, *_args, **_kwargs):
    control = LaunchConfiguration("control").perform(context).strip().lower()
    scenario_id = LaunchConfiguration("scenario").perform(context).strip()
    supplied_scenario_config = LaunchConfiguration("scenario_config").perform(context).strip()
    supplied_decision_config = LaunchConfiguration("decision_config").perform(context).strip()
    run_purpose = LaunchConfiguration("run_purpose").perform(context).strip().lower()
    headless = LaunchConfiguration("headless").perform(context).strip().lower()
    dashboard_enabled = LaunchConfiguration("dashboard").perform(context).strip().lower()
    supplied_world = LaunchConfiguration("world_path").perform(context).strip()
    supplied_run_id = LaunchConfiguration("run_id").perform(context).strip()
    supplied_seed = LaunchConfiguration("seed").perform(context).strip()
    supplied_output = LaunchConfiguration("run_output_dir").perform(context).strip()
    input_manifest = LaunchConfiguration("input_manifest").perform(context).strip()
    configured_root = LaunchConfiguration("frozen_inputs_root_sha256").perform(context).strip()
    if control not in {"manual", "autonomous"}:
        raise RuntimeError("control must be manual or autonomous")
    if run_purpose not in {"diagnostic", "development", "demonstration"}:
        raise RuntimeError("run_purpose must be diagnostic, development, or demonstration")
    if headless not in {"true", "false"}:
        raise RuntimeError("headless must be true or false")
    if dashboard_enabled not in {"true", "false"}:
        raise RuntimeError("dashboard must be true or false")
    if not scenario_id or Path(scenario_id).name != scenario_id:
        raise RuntimeError("scenario must be a tracked obstacle-course scenario ID such as S1")

    package_share = Path(get_package_share_directory("car_bringup"))
    repo = _repository_root(package_share)
    config_root = (repo / "ros2_ws/src/car_bringup/config/obstacle_course").resolve()
    scenario_root = (config_root / "scenarios").resolve()
    development_scenario_root = (config_root / "development/scenarios").resolve()
    scenario_path = (
        (repo / supplied_scenario_config).resolve()
        if supplied_scenario_config
        else (scenario_root / f"{scenario_id}.yaml").resolve()
    )
    is_development_scenario = scenario_path.is_relative_to(development_scenario_root)
    if (
        (not scenario_path.is_relative_to(scenario_root) and not is_development_scenario)
        or not scenario_path.is_file()
    ):
        raise RuntimeError(f"unknown obstacle-course scenario {scenario_id!r}")
    scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    if scenario.get("scenario_id") != scenario_id or scenario_path.stem != scenario_id:
        raise RuntimeError("scenario ID must match the selected scenario config")
    decision_config_path = (
        (repo / supplied_decision_config).resolve()
        if supplied_decision_config
        else (config_root / "decision.yaml").resolve()
    )
    development_config_root = (config_root / "development").resolve()
    is_development_decision = decision_config_path.is_relative_to(development_config_root)
    if (
        decision_config_path != (config_root / "decision.yaml").resolve()
        and not is_development_decision
    ) or not decision_config_path.is_file():
        raise RuntimeError("decision_config must be the production config or a development config")
    if is_development_scenario != is_development_decision:
        raise RuntimeError("development scenarios must use a development decision config, and vice versa")
    if is_development_scenario and run_purpose != "development":
        raise RuntimeError("development configs require run_purpose:=development")

    rc = yaml.safe_load(
        (repo / "ros2_ws/src/car_bringup/config/rc_vehicle.yaml").read_text(encoding="utf-8")
    )["presets"]["rc_car"]
    run_id = supplied_run_id or str(uuid.uuid4())
    try:
        uuid.UUID(run_id)
    except ValueError as exc:
        raise RuntimeError("run_id must be a UUID") from exc
    simulation_seed = int(supplied_seed) if supplied_seed else int(scenario["seed"])
    if simulation_seed < 0:
        raise RuntimeError("seed must be nonnegative")
    frozen_root = configured_root
    if input_manifest:
        manifest_path = Path(input_manifest).expanduser().resolve()
        if not manifest_path.is_file():
            raise RuntimeError(f"frozen input manifest is missing: {manifest_path}")
        subprocess.run([
            sys.executable, str(repo / "tools/verify_obstacle_course_inputs.py"),
            "--manifest", str(manifest_path),
        ], cwd=repo, check=True)
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if frozen_root and frozen_root != manifest.get("root_sha256"):
            raise RuntimeError("configured frozen root does not match the input manifest")
        frozen_root = str(manifest["root_sha256"])
    if run_purpose == "demonstration" and (not input_manifest or not frozen_root):
        raise RuntimeError("demonstration runs require a verified input_manifest")
    world_path = (
        repo / "artifacts/obstacle-course/worlds" / f"{scenario_id}.sdf"
        if not supplied_world else Path(supplied_world).expanduser().resolve()
    )
    if not supplied_world:
        subprocess.run([
            sys.executable, str(repo / "tools/generate_obstacle_course_world.py"),
            "--scenario-config", str(scenario_path), "--output", str(world_path),
            "--run-purpose", run_purpose,
        ], cwd=repo, check=True)
    elif not world_path.is_relative_to((repo / "artifacts/obstacle-course/worlds").resolve()):
        raise RuntimeError("world_path must be inside artifacts/obstacle-course/worlds")
    if not world_path.is_file():
        raise RuntimeError(f"generated world is missing: {world_path}")

    profile_output = repo / "artifacts/obstacle-course/config" / f"{scenario_id}.json"
    subprocess.run([
        sys.executable, str(repo / "tools/build_obstacle_course_profile.py"),
        "--scenario-config", str(scenario_path), "--profile",
        str(repo / "ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml"),
        "--decision-config", str(decision_config_path),
        "--output", str(profile_output), "--run-purpose", run_purpose,
    ], cwd=repo, check=True)
    subprocess.run([
        sys.executable, str(repo / "tools/validate_obstacle_course_config.py"), "--all",
    ], cwd=repo, check=True)
    subprocess.run([
        sys.executable, str(repo / "tools/fetch_obstacle_course_assets.py"), "--verify-only",
    ], cwd=repo, check=True)
    subprocess.run([
        sys.executable, str(repo / "tools/fetch_obstacle_course_assets.py"), "--lock",
        str(repo / "ros2_ws/src/car_bringup/config/obstacle_course/research_vehicle.lock.yaml"),
        "--output", str(repo / "artifacts/obstacle-course/assets"), "--verify-only",
    ], cwd=repo, check=True)
    subprocess.run([
        sys.executable, str(repo / "tools/fetch_obstacle_course_assets.py"), "--lock",
        str(repo / "ros2_ws/src/car_bringup/config/obstacle_course/research_assets.lock.yaml"),
        "--output", str(repo / "artifacts/obstacle-course/assets"), "--verify-only",
    ], cwd=repo, check=True)
    subprocess.run([
        sys.executable, str(repo / "tools/fetch_obstacle_course_assets.py"), "--lock",
        str(repo / "ros2_ws/src/car_bringup/config/obstacle_course/assets.lock.yaml"),
        "--output", str(repo / "artifacts/obstacle-course/assets"), "--verify-only",
    ], cwd=repo, check=True)
    resource_roots = [
        repo / "simulation/models",
        repo / "artifacts/obstacle-course/assets/models",
    ]
    inherited = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        [*(str(path) for path in resource_roots), inherited]
        if inherited else [*(str(path) for path in resource_roots)]
    )
    sdf_path = os.pathsep.join(
        [*(str(path) for path in resource_roots), os.environ.get("SDF_PATH", "")]
    ).rstrip(os.pathsep)
    sdf_environment = os.environ.copy()
    sdf_environment["GZ_SIM_RESOURCE_PATH"] = resource_path
    sdf_environment["SDF_PATH"] = sdf_path
    subprocess.run(["gz", "sdf", "-k", str(world_path)], cwd=repo, env=sdf_environment, check=True)
    gz_args = ["sim", "--force-version", GAZEBO_VERSION, "-v", "3", "-r", "--seed", str(simulation_seed)]
    if headless == "true":
        gz_args.extend(["-s", "--headless-rendering"])
    gz_args.append(str(world_path))
    # Keep Gazebo in this launch's process group. Detaching with `setsid` leaves
    # the GUI child rendering after Ctrl+C and can pin a CPU core in WSLg.
    gazebo = ExecuteProcess(cmd=["gz", *gz_args], cwd=str(repo), output="screen", emulate_tty=True)

    config_share = Path(get_package_share_directory("car_bringup")) / "config/obstacle_course"
    bridge_path = config_share / "bridge.yaml"
    if not bridge_path.is_file():
        bridge_path = repo / "ros2_ws/src/car_bringup/config/obstacle_course/bridge.yaml"
    if not bridge_path.is_file():
        raise RuntimeError(f"Obstacle-course bridge file is missing: {bridge_path}")
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="obstacle_course_bridge",
        parameters=[{"config_file": str(bridge_path), "use_sim_time": True}],
        prefix="setsid", output="screen",
    )
    set_pose_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="obstacle_course_set_pose_bridge",
        arguments=["/world/obstacle_course/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
        prefix="setsid", output="screen",
    )
    run_output_dir = Path(supplied_output).expanduser().resolve() if supplied_output else (
        repo / "artifacts/obstacle-course/runs" / run_id
    )
    runs_root = (repo / "artifacts/obstacle-course/runs").resolve()
    if not run_output_dir.is_relative_to(runs_root):
        raise RuntimeError("run_output_dir must be inside artifacts/obstacle-course/runs")
    vehicle_visual = Node(
        package="car_decision", executable="vehicle_visual_overlay", name="vehicle_visual_overlay",
        parameters=[{
            "use_sim_time": True,
            "world_origin_z_m": 0.0675,
        }], prefix="setsid", output="screen",
    )
    if control == "manual":
        command_nodes = [Node(
            package="car_decision", executable="manual_twist_adapter", name="manual_twist_adapter",
            parameters=[{
                "use_sim_time": True,
                "wheelbase_m": float(rc["geometry"]["wheelbase_m"]["value"]),
                "max_speed_mps": float(rc["steering"]["max_speed_mps"]["value"]),
                "max_steering_angle_rad": float(rc["steering"]["effective_center_wheel_limit_rad"]["value"]),
                "max_yaw_rate_radps": float(rc["steering"]["max_yaw_rate_radps"]["value"]),
            }], prefix="setsid", output="screen",
        )]
    else:
        decision_config = decision_config_path
        trigger_matrix = repo / "ros2_ws/src/car_bringup/config/obstacle_course/trigger_matrix.yaml"
        geometry_config = repo / "ros2_ws/src/car_bringup/config/obstacle_course/vehicle_geometry.yaml"
        decision_node = Node(
            package="car_decision", executable="decision_node", name="obstacle_course_decision",
            parameters=[{
                "use_sim_time": True,
                "scenario_path": str(scenario_path),
                "decision_path": str(decision_config),
                "profile_path": str(profile_output),
                "sensor_profile_path": str(repo / "ros2_ws/src/car_bringup/config/rc_vehicle.yaml"),
                "trigger_matrix_path": str(trigger_matrix),
                "run_id": run_id,
                "run_purpose": run_purpose,
                "frozen_inputs_root_sha256": frozen_root,
            }], prefix="setsid", output="screen",
        )
        planner = Node(
            package="car_decision", executable="planner_intent_adapter", name="planner_intent_adapter",
            parameters=[{
                "use_sim_time": True,
                "scenario_path": str(scenario_path),
                "profile_path": str(profile_output),
                "vehicle_geometry_path": str(geometry_config),
                "decision_path": str(decision_config),
            }], prefix="setsid", output="screen",
        )
        request_adapter = Node(
            package="car_decision", executable="controller_request_adapter", name="controller_request_adapter",
            parameters=[{
                "use_sim_time": True,
                "max_speed_mps": float(rc["steering"]["max_speed_mps"]["value"]),
                "max_steering_angle_rad": float(rc["steering"]["effective_center_wheel_limit_rad"]["value"]),
            }], prefix="setsid", output="screen",
        )
        command_nodes = [decision_node, planner, request_adapter]
    controller = Node(
        package="car_autonomy_nodes", executable="safe_drive_controller", name="vehicle_controller_node",
        parameters=[{
            "use_sim_time": True,
            "mode": "rc_navigation",
            "wheelbase_m": float(rc["geometry"]["wheelbase_m"]["value"]),
            "max_steering_angle_rad": float(rc["steering"]["effective_center_wheel_limit_rad"]["value"]),
            "rc_max_speed_mps": float(rc["steering"]["max_speed_mps"]["value"]),
            "rc_max_yaw_rate_radps": float(rc["steering"]["max_yaw_rate_radps"]["value"]),
        }], prefix="setsid", output="screen",
    )
    evidence_parameters = {
        "use_sim_time": True,
        "scenario_id": scenario_id,
        "scenario_path": str(scenario_path),
        "decision_path": str(decision_config_path),
        "profile_path": str(profile_output),
        "sensor_profile_path": str(repo / "ros2_ws/src/car_bringup/config/rc_vehicle.yaml"),
        "run_id": run_id,
        "run_purpose": run_purpose,
        "frozen_inputs_root_sha256": frozen_root,
    }
    sensor_tf = Node(
        package="car_navigation", executable="sensor_tf_node", name="obstacle_course_sensor_tf",
        parameters=[{"use_sim_time": True, "profile_path": str(repo / "ros2_ws/src/car_bringup/config/rc_vehicle.yaml")}],
        prefix="setsid", output="screen",
    )
    evidence_nodes = [
        Node(package="car_decision", executable="lidar_evidence_node", name="lidar_evidence_node",
             parameters=[evidence_parameters], prefix="setsid", output="screen"),
        Node(package="car_decision", executable="camera_evidence_node", name="camera_evidence_node",
             parameters=[evidence_parameters], prefix="setsid", output="screen"),
        Node(package="car_decision", executable="imu_evidence_node", name="imu_evidence_node",
             parameters=[evidence_parameters], prefix="setsid", output="screen"),
        Node(package="car_decision", executable="odometry_evidence_node", name="odometry_evidence_node",
             parameters=[evidence_parameters], prefix="setsid", output="screen"),
        Node(package="car_decision", executable="contact_evidence_node", name="contact_evidence_node",
             parameters=[evidence_parameters], prefix="setsid", output="screen"),
        Node(package="car_decision", executable="fault_gate_node", name="fault_gate_node",
             parameters=[{
                 "use_sim_time": True,
                 "scenario_id": scenario_id,
                 "scenario_path": str(scenario_path),
                 "run_id": run_id,
                 "run_purpose": run_purpose,
                 "frozen_inputs_root_sha256": evidence_parameters["frozen_inputs_root_sha256"],
                 "seed": simulation_seed,
                 "pose_service": "/world/obstacle_course/set_pose",
             }], prefix="setsid", output="screen"),
    ]
    run_logger = Node(
        package="car_decision", executable="run_logger", name="obstacle_course_run_logger",
        parameters=[{
            "use_sim_time": True,
            "output_dir": str(run_output_dir),
            "contract_dir": str(repo / "ros2_ws/src/car_bringup/config/obstacle_course/contracts"),
            "run_id": run_id, "scenario_id": scenario_id, "run_purpose": run_purpose,
            "seed": simulation_seed,
            "frozen_inputs_root_sha256": frozen_root,
        }], prefix="setsid", output="screen",
    )
    evidence_nodes.append(run_logger)
    evidence_nodes.extend(command_nodes)
    if dashboard_enabled == "true":
        evidence_nodes.append(Node(
            package="car_decision", executable="dashboard_node", name="obstacle_course_dashboard",
            parameters=[{"host": "127.0.0.1", "port": 8765}], prefix="setsid", output="screen",
        ))
    start_bridge = RegisterEventHandler(OnProcessStart(target_action=gazebo, on_start=[bridge]))
    start_course_nodes = RegisterEventHandler(OnProcessStart(
        target_action=bridge,
        on_start=[TimerAction(period=1.0, actions=[
            set_pose_bridge, sensor_tf, vehicle_visual, controller, *evidence_nodes
        ])],
    ))
    return [
        *_wslg_render_environment(headless),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("SDF_PATH", sdf_path),
        LogInfo(msg=f"Obstacle course {scenario_id} seed={simulation_seed} run_id={run_id} purpose={run_purpose} control={control}; controller is the only Gazebo command authority"),
        start_bridge,
        start_course_nodes,
        gazebo,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="S0"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("control", default_value="manual"),
        DeclareLaunchArgument("run_purpose", default_value="diagnostic"),
        DeclareLaunchArgument("dashboard", default_value="false"),
        DeclareLaunchArgument("scenario_config", default_value=""),
        DeclareLaunchArgument("decision_config", default_value=""),
        DeclareLaunchArgument("run_id", default_value=""),
        DeclareLaunchArgument("seed", default_value=""),
        DeclareLaunchArgument("run_output_dir", default_value=""),
        DeclareLaunchArgument("input_manifest", default_value=""),
        DeclareLaunchArgument("frozen_inputs_root_sha256", default_value=""),
        DeclareLaunchArgument("world_path", default_value=""),
        OpaqueFunction(function=_launch_obstacle_course_setup),
    ])
