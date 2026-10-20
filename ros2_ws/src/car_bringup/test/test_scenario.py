"""Tests for the shared scenario configuration contract."""

import copy

import pytest

from car_bringup.scenario import (
    ScenarioValidationError,
    validate_scenario,
    validate_scenario_obstacle,
)


VALID_SCENARIO = {
    "scenario_id": "training/sensor_smoke",
    "split": "training",
    "world": "simulation/worlds/obstacle.sdf",
    "seed": 1701,
    "lighting_id": "obstacle_world_default",
    "unseen_condition_tags": [],
    "start_pose": {
        "x_m": -105.0,
        "y_m": -10.0,
        "z_m": 1.96,
        "roll_rad": 0.0,
        "pitch_rad": 0.0,
        "yaw_rad": 0.0,
    },
    "obstacle": {
        "entity": "course_obstacles",
        "collision": "barrier_collision",
        "pose": {
            "x_m": -80.0,
            "y_m": -10.0,
            "z_m": 0.5,
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "yaw_rad": 0.0,
        },
    },
    "commanded_speed_mps": 1.0,
    "stop_distance_m": 5.0,
    "corridor_half_width_m": 5.5,
    "ground_z_min_m": 0.2,
    "run_timeout_sim_s": 60.0,
}


def test_valid_fixed_seed_scenario_is_returned_unchanged():
    scenario = copy.deepcopy(VALID_SCENARIO)
    assert validate_scenario(scenario) is scenario


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("world", "simulation/worlds/car_demo.sdf"),
        ("seed", -1),
        ("split", "testing"),
        ("lighting_id", "randomized"),
        ("commanded_speed_mps", 2.1),
        ("stop_distance_m", 0.0),
        ("corridor_half_width_m", -1.0),
        ("run_timeout_sim_s", 0),
    ],
)
def test_rejects_invalid_world_seed_split_and_run_limits(field, value):
    scenario = copy.deepcopy(VALID_SCENARIO)
    scenario[field] = value
    with pytest.raises(ScenarioValidationError):
        validate_scenario(scenario)


def test_requires_named_obstacle_and_full_start_pose():
    scenario = copy.deepcopy(VALID_SCENARIO)
    del scenario["obstacle"]["collision"]
    del scenario["start_pose"]["yaw_rad"]
    with pytest.raises(ScenarioValidationError, match="collision|yaw_rad"):
        validate_scenario(scenario)


def test_rejects_non_finite_numeric_values():
    scenario = copy.deepcopy(VALID_SCENARIO)
    scenario["obstacle"]["pose"]["x_m"] = float("nan")
    with pytest.raises(ScenarioValidationError, match="finite"):
        validate_scenario(scenario)


def test_rejects_unknown_fields_to_keep_scenarios_auditable():
    scenario = copy.deepcopy(VALID_SCENARIO)
    scenario["seed_override"] = 1702
    with pytest.raises(ScenarioValidationError, match="Additional properties"):
        validate_scenario(scenario)


def test_scenario_obstacle_pose_matches_world_without_rewriting_sdf(tmp_path):
    world = tmp_path / "world.sdf"
    world.write_text(
        "<sdf version='1.10'><gz:policies/><world name='test'><model name='course_obstacles'>"
        "<link name='link'><collision name='barrier_collision'>"
        "<pose>-80 -10 0.5 0 0 0</pose></collision></link></model></world></sdf>",
        encoding="utf-8",
    )
    before = world.read_bytes()

    validate_scenario_obstacle(VALID_SCENARIO, world)

    assert world.read_bytes() == before


def test_scenario_obstacle_pose_mismatch_is_rejected(tmp_path):
    world = tmp_path / "world.sdf"
    world.write_text(
        "<sdf><world name='test'><model name='course_obstacles'><link name='link'>"
        "<collision name='barrier_collision'><pose>-80 -10 0.5 0 0 0</pose>"
        "</collision></link></model></world></sdf>",
        encoding="utf-8",
    )
    scenario = copy.deepcopy(VALID_SCENARIO)
    scenario["obstacle"]["pose"]["x_m"] = -79.0

    with pytest.raises(ScenarioValidationError, match="does not match protected SDF"):
        validate_scenario_obstacle(scenario, world)
