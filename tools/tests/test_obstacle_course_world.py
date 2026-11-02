"""Obstacle course world generation, asset, and launch-contract tests."""

from __future__ import annotations

import hashlib
import yaml
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from generate_obstacle_course_world import (  # noqa: E402
    WorldGenerationError,
    build_world,
    load_tracked_scenario,
)
from build_obstacle_course_profile import build_profile  # noqa: E402


CONFIGS = ROOT / "ros2_ws/src/car_bringup/config/obstacle_course"
SCENARIOS = CONFIGS / "scenarios"


def test_only_tracked_fixed_seed_scenarios_are_accepted(monkeypatch):
    source = SCENARIOS / "S0.yaml"
    monkeypatch.setattr("generate_obstacle_course_world._is_tracked", lambda _path: False)
    with pytest.raises(WorldGenerationError, match="tracked"):
        load_tracked_scenario(source)


def test_world_generation_is_deterministic_and_scenario_isolated(tmp_path):
    first = tmp_path / "S1-a.sdf"
    second = tmp_path / "S1-b.sdf"
    unrelated = tmp_path / "S0.sdf"
    first_meta = build_world(SCENARIOS / "S1.yaml", first)
    second_meta = build_world(SCENARIOS / "S1.yaml", second)
    clear_meta = build_world(SCENARIOS / "S0.yaml", unrelated)
    assert hashlib.sha256(first.read_bytes()).hexdigest() == hashlib.sha256(second.read_bytes()).hexdigest()
    assert first_meta["world_sha256"] == second_meta["world_sha256"]
    assert first_meta["entity_ids"] == ["gazebo_truck", "gazebo_truck_visual", "barrier_01"]
    assert clear_meta["entity_ids"] == ["gazebo_truck", "gazebo_truck_visual"]
    assert "barrier_red" in first_meta["fragment_ids"]
    assert not clear_meta["fragment_ids"]


def test_tracked_development_variant_builds_world_and_uses_its_decision_config(tmp_path):
    scenario_path = CONFIGS / "development/scenarios/barrier_tuning.yaml"
    decision_path = CONFIGS / "development/decision.yaml"
    world_path = tmp_path / "barrier_tuning.sdf"
    world_meta = build_world(scenario_path, world_path, run_purpose="development")
    profile = build_profile(
        scenario_path,
        CONFIGS / "rc_vehicle_reference.yaml",
        tmp_path / "barrier_tuning-profile.json",
        run_purpose="development",
        decision_config_path=decision_path,
    )

    assert world_meta["scenario_id"] == "barrier_tuning"
    assert world_meta["seed"] == 87001
    assert world_meta["entity_ids"] == [
        "gazebo_truck", "gazebo_truck_visual", "barrier_stage_1",
        "cone_stage_2", "vehicle_stage_3", "gate_stage_4",
    ]
    scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    assert len(scenario["obstacles"]) == 4
    assert scenario["route"]["length_m"]["value"] > 10.0
    assert {float(point["y"]["value"]) for point in scenario["route"]["waypoints"]} == {0.0}
    assert scenario["route"]["sizing"]["lane_change_count"] == 8
    assert profile["decision"]["decision_config_id"] == "obstacle-course-trigger-showcase-development-v2"
    assert profile["source_hashes"]["decision_config"]["path"] == decision_path.relative_to(ROOT).as_posix()


def test_course_include_publishes_only_the_physical_truck_pose_at_a_bounded_rate():
    world = ET.parse(ROOT / "simulation/worlds/obstacle_condensed.sdf").getroot()
    include = world.find(".//world/include[name='gazebo_truck']")
    assert include is not None
    plugin = include.find("plugin[@filename='gz-sim-pose-publisher-system']")
    assert plugin is not None
    assert plugin.get("name") == "gz::sim::systems::PosePublisher"
    assert plugin.findtext("publish_model_pose") == "true"
    assert plugin.findtext("publish_link_pose") == "false"
    assert plugin.findtext("publish_sensor_pose") == "false"
    assert plugin.findtext("topic") == "/model/gazebo_truck/ground_truth_pose"
    assert float(plugin.findtext("update_frequency")) == pytest.approx(30.0)


def test_run_purpose_override_changes_metadata_without_changing_world(tmp_path):
    demonstration = tmp_path / "demonstration.sdf"
    diagnostic = tmp_path / "diagnostic.sdf"
    demo_meta = build_world(SCENARIOS / "S1.yaml", demonstration, run_purpose="demonstration")
    diagnostic_meta = build_world(SCENARIOS / "S1.yaml", diagnostic, run_purpose="diagnostic")
    assert demo_meta["run_purpose"] == "demonstration"
    assert diagnostic_meta["run_purpose"] == "diagnostic"
    assert demonstration.read_bytes() == diagnostic.read_bytes()
    assert demo_meta["world_sha256"] == diagnostic_meta["world_sha256"]


def test_s5_reveal_obstacle_starts_at_off_course_initial_pose(tmp_path):
    output = tmp_path / "S5.sdf"
    build_world(SCENARIOS / "S5.yaml", output)
    root = ET.parse(output).getroot()
    hidden = root.find(".//world/model[@name='late_barrier']/pose")
    assert hidden is not None
    values = [float(value) for value in hidden.text.split()]
    assert abs(values[1]) > 1.0


def test_research_vehicle_visual_uses_original_locked_mesh_and_no_collision():
    lock_path = ROOT / "ros2_ws/src/car_bringup/config/obstacle_course/research_vehicle.lock.yaml"
    lock = yaml.safe_load(lock_path.read_text(encoding="utf-8"))
    truck = next(asset for asset in lock["assets"] if asset["role"] == "controlled_vehicle_visual_only")
    mesh = ROOT / "artifacts/obstacle-course/assets/models/research/rc_vehicle/research_pickup_truck_01/pickup-original.glb"
    assert truck["asset_id"] == "research_pickup_truck_01"
    assert hashlib.sha256(mesh.read_bytes()).hexdigest() == truck["file_sha256"]
    assert truck["dimensions"]["visual_scale_factor"]["value"] == 0.08
    world = ET.parse(ROOT / "simulation/worlds/obstacle_condensed.sdf").getroot()
    visual = world.find(".//world/model[@name='gazebo_truck_visual']")
    assert visual is not None
    uri = visual.findtext("./link/visual/geometry/mesh/uri")
    assert uri == "model://research/rc_vehicle/research_pickup_truck_01/pickup-original.glb"
    assert visual.findtext("./link/visual/geometry/mesh/scale") == "0.08 0.08 0.08"
    assert visual.findtext("./link/visual/pose") == "0 0 -0.0675 1.57079632679 0 1.57079632679"
    assert visual.find("./link/collision") is None


def test_generated_s5_keeps_visual_entity_at_staging_pose_until_fault(tmp_path):
    output = tmp_path / "S5.sdf"
    build_world(SCENARIOS / "S5.yaml", output)
    root = ET.parse(output).getroot()
    hidden = root.find(".//world/model[@name='late_barrier']/pose")
    assert hidden is not None
    assert float(hidden.text.split()[1]) == pytest.approx(3.0)


def test_each_semantic_fragment_keeps_a_locked_mesh_visual_and_proxy_collision():
    fragments = ROOT / "simulation/worlds/obstacle_course/fragments"
    checks = {
        "barrier_red.sdf": "model://research/obstacle/road_barrier_set/road-barrier-set.glb",
        "post_orange.sdf": "model://research/cone/cone_reflector_01/cone_reflector_01.glb",
        "gate_blue.sdf": "model://research/obstacle/security_grille_gate/security-grille-gate.glb",
        "gate_blocked.sdf": "model://research/obstacle/road_barrier_set/road-barrier-set.glb",
        "generic_obstacle.sdf": "model://research/vehicle/vehicle_compact_suv_01/vehicle_compact_suv_01.glb",
    }
    for filename, uri in checks.items():
        model = ET.parse(fragments / filename).getroot().find("model")
        assert model is not None
        assert model.findtext("./link/visual/geometry/mesh/uri") == uri
        assert model.find("./link/collision") is not None


def test_research_cone_visual_is_upright_with_its_base_at_ground_level():
    fragment = ROOT / "simulation/worlds/obstacle_course/fragments/post_orange.sdf"
    model = ET.parse(fragment).getroot().find("model")
    assert model is not None
    visual_pose = [float(value) for value in model.findtext("./link/visual/pose").split()]
    collision_pose = [float(value) for value in model.findtext("./link/collision/pose").split()]
    collision_height = float(model.findtext("./link/collision/geometry/cylinder/length"))
    assert visual_pose[3] == pytest.approx(1.57079632679)  # GLB Y-up axis to Gazebo Z-up.
    assert visual_pose[2] == pytest.approx(collision_height / 2.0)
    assert collision_pose[2] == pytest.approx(collision_height / 2.0)


def test_research_barrier_selects_upright_red_submesh_in_the_lidar_plane():
    fragment = ROOT / "simulation/worlds/obstacle_course/fragments/barrier_red.sdf"
    model = ET.parse(fragment).getroot().find("model")
    assert model is not None
    mesh = model.find("./link/visual/geometry/mesh")
    assert mesh is not None
    assert mesh.findtext("submesh/name") == "Water-filled barrier (red)"
    assert mesh.findtext("submesh/center") == "true"
    visual_pose = [float(value) for value in model.findtext("./link/visual/pose").split()]
    scale = [float(value) for value in mesh.findtext("scale").split()]
    # Source GLB is Y-up; after the X rotation its Y extent is the upright Z extent.
    visual_height = 0.83 * scale[1]
    collision_height = float(model.findtext("./link/collision/geometry/box/size").split()[2])
    assert visual_pose[3] == pytest.approx(1.57079632679)
    assert visual_pose[2] == pytest.approx(collision_height / 2.0)
    assert visual_pose[2] - visual_height / 2.0 == pytest.approx(0.0, abs=1e-3)
    assert visual_pose[2] + visual_height / 2.0 > 0.1225


def test_research_suv_visual_is_upright_grounded_and_crosses_the_lidar_plane():
    fragment = ROOT / "simulation/worlds/obstacle_course/fragments/generic_obstacle.sdf"
    model = ET.parse(fragment).getroot().find("model")
    assert model is not None
    visual = model.find("./link/visual")
    assert visual is not None
    visual_pose = [float(value) for value in visual.findtext("pose").split()]
    mesh = visual.find("./geometry/mesh")
    assert mesh is not None
    assert mesh.findtext("scale") == "0.1 0.1 0.1"
    # The locked GLB is X-forward, Y-up, Z-wide. Rotating around X maps its
    # source vertical axis to Gazebo Z without changing the research asset.
    assert visual_pose[3] == pytest.approx(1.57079632679)
    assert visual_pose[2] == pytest.approx(-0.0003)

    lock = yaml.safe_load(
        (ROOT / "ros2_ws/src/car_bringup/config/obstacle_course/research_assets.lock.yaml")
        .read_text(encoding="utf-8")
    )
    asset = next(item for item in lock["assets"] if item["asset_id"] == "vehicle_compact_suv_01")
    bounds = asset["dimensions"]["miniature_bounds_m"]["value"]
    visual_bottom = visual_pose[2] + 0.003 * 0.1  # GLB root translation is -3 mm on source Y.
    visual_top = visual_pose[2] + bounds[2]
    rc = yaml.safe_load((ROOT / "ros2_ws/src/car_bringup/config/rc_vehicle.yaml").read_text())
    sensor_z = float(rc["presets"]["rc_car"]["sensors"]["lidar"]["pose_base_link_m_rad"]["value"][2])
    vehicle_z = 0.0675
    assert visual_bottom == pytest.approx(0.0, abs=1e-6)
    assert visual_bottom < vehicle_z + sensor_z < visual_top

    collision = model.find("./link/collision")
    assert collision is not None
    assert float(collision.findtext("pose").split()[2]) - float(
        collision.findtext("./geometry/box/size").split()[2]
    ) / 2.0 == pytest.approx(0.0)


def test_single_plane_lidar_crosses_the_configured_barrier_and_post_collision_envelopes():
    rc = yaml.safe_load((ROOT / "ros2_ws/src/car_bringup/config/rc_vehicle.yaml").read_text())
    sensor_z = float(rc["presets"]["rc_car"]["sensors"]["lidar"]["pose_base_link_m_rad"]["value"][2])
    world = ET.parse(ROOT / "simulation/worlds/obstacle_condensed.sdf").getroot()
    vehicle_pose = world.findtext(".//world/include[name='gazebo_truck']/pose")
    vehicle_z = float(vehicle_pose.split()[2])
    scan_plane_z = vehicle_z + sensor_z
    fragments = ROOT / "simulation/worlds/obstacle_course/fragments"
    for name in ("barrier_red.sdf", "gate_blocked.sdf", "post_orange.sdf"):
        root = ET.parse(fragments / name).getroot()
        collision = root.find(".//collision")
        assert collision is not None
        collision_pose = collision.findtext("pose", "0 0 0 0 0 0").split()
        center_z = float(collision_pose[2])
        box = collision.find("./geometry/box")
        cylinder = collision.find("./geometry/cylinder")
        if box is not None:
            height = float(box.findtext("size").split()[2])
        else:
            height = float(cylinder.findtext("length"))
        assert center_z - height / 2.0 <= 0.005, f"{name} collision proxy must sit on the ground"
        assert center_z + height / 2.0 > scan_plane_z + 0.005, f"{name} does not intersect the active RC single-plane LiDAR"


def test_obstacle_course_world_and_fragments_parse_with_gz():
    world = ROOT / "simulation/worlds/obstacle_condensed.sdf"
    if not world.exists():
        pytest.skip("P1 world is authored after scenario inputs are frozen")
    assert ET.parse(world).getroot().find("world[@name='obstacle_course']") is not None
    assert not list((ROOT / "simulation/worlds/obstacle_course/fragments").glob("*.sdf")) == []


def test_launch_keeps_one_command_authority_and_selects_one_manual_or_autonomous_request_path():
    launch = (ROOT / "ros2_ws/src/car_bringup/launch/obstacle_course.launch.py").read_text(encoding="utf-8")
    assert "scenario_config" in launch
    assert "control == \"autonomous\"" in launch or "control not in" in launch
    assert "safe_drive_controller" in launch
    assert "manual_twist_adapter" in launch
    assert "vehicle_visual_overlay" in launch
    assert "/world/obstacle_course/set_pose@ros_gz_interfaces/srv/SetEntityPose" in launch
    demo_launch = launch.split("def _launch_obstacle_course_setup", 1)[1]
    # Gazebo stays owned by the launch process group so its GUI child exits too.
    assert 'ExecuteProcess(cmd=["gz", *gz_args]' in demo_launch
    assert 'ExecuteProcess(cmd=["setsid", "gz", *gz_args]' not in demo_launch
    # Twelve shared/manual actions cover the bridge, controller, evidence, and
    # logger; three belong to the autonomous path, one launches Gazebo, and one
    # optional read-only dashboard is added when enabled.
    assert demo_launch.count('prefix="setsid"') == 17
    assert 'if control == "manual"' in demo_launch
    assert 'else:\n        decision_config' in demo_launch
    # Visible runs select the WSLg renderer; headless runs do not request a display.
    assert launch.count("_wslg_render_environment(headless)") == 1
    assert '"GALLIUM_DRIVER", os.environ.get("GALLIUM_DRIVER", "d3d12")' in launch
    assert '"MESA_D3D12_DEFAULT_ADAPTER_NAME", adapter' in launch
    bridge = yaml.safe_load((CONFIGS / "bridge.yaml").read_text(encoding="utf-8"))
    command_bridges = [item for item in bridge if item.get("direction") == "ROS_TO_GZ"]
    assert len(command_bridges) == 1
    assert command_bridges[0]["gz_topic_name"] == "/model/gazebo_truck/cmd_vel"
    pose_bridge = next(item for item in bridge if item.get("gz_topic_name") == "/model/gazebo_truck/ground_truth_pose")
    assert pose_bridge["ros_topic_name"] == "/vehicle/ground_truth_pose"
    assert pose_bridge["ros_type_name"] == "geometry_msgs/msg/PoseStamped"
