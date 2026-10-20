"""Unit tests for the command lease and its safety interlocks."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from car_autonomy_nodes.safe_drive_controller import DriveSafetyState


NS = 1_000_000_000
SENSORS = {"lidar": 20 * NS, "odometry": 20 * NS, "imu": 20 * NS}


def _ready_state(now_ns: int = 20 * NS) -> DriveSafetyState:
    state = DriveSafetyState()
    state.update_clock(now_ns - 1, 100 * NS)
    state.update_clock(now_ns, 101 * NS)
    for name, stamp in SENSORS.items():
        state.update_sensor(name, stamp, now_ns)
    return state


def _twist(**kwargs):
    values = dict(linear_x=0.0, linear_y=0.0, linear_z=0.0,
                  angular_x=0.0, angular_y=0.0, angular_z=0.0)
    values.update(kwargs)
    return SimpleNamespace(
        linear=SimpleNamespace(x=values["linear_x"], y=values["linear_y"], z=values["linear_z"]),
        angular=SimpleNamespace(x=values["angular_x"], y=values["angular_y"], z=values["angular_z"]),
    )


def test_request_is_clamped_and_expires_after_half_sim_second():
    state = _ready_state()
    assert state.accept_request(_twist(linear_x=4.5, angular_z=-0.5), 20 * NS, 101 * NS)
    assert state.command_for_tick(20 * NS, 101 * NS) == (2.0, -0.15)
    assert state.command_for_tick(20_499_999_999, 101 * NS) == (2.0, -0.15)
    assert state.command_for_tick(20_500_000_000, 101 * NS) == (0.0, 0.0)
    assert state.command is None


def test_rejects_non_finite_and_unsupported_twist_axes():
    state = _ready_state()
    assert not state.accept_request(_twist(linear_x=float("nan")), 20 * NS, 101 * NS)
    assert not state.accept_request(_twist(angular_x=0.01), 20 * NS, 101 * NS)
    assert state.command is None


def test_stale_sensor_blocks_and_clears_active_command():
    state = _ready_state()
    assert state.accept_request(_twist(linear_x=1.0), 20 * NS, 101 * NS)
    assert state.command_for_tick(20_600_000_000, 101 * NS) == (0.0, 0.0)
    assert state.command is None


@pytest.mark.parametrize("sensor", ("lidar", "odometry", "imu"))
def test_each_required_sensor_fails_closed_when_stale(sensor):
    state = _ready_state()
    state.sensors[sensor] = 20 * NS - 500_000_001
    assert not state.accept_request(_twist(linear_x=1.0), 20 * NS, 101 * NS)
    assert state.command_for_tick(20 * NS, 101 * NS) == (0.0, 0.0)


@pytest.mark.parametrize("sensor", ("lidar", "odometry", "imu"))
def test_each_required_sensor_rejects_invalid_and_future_timestamps(sensor):
    state = _ready_state()
    assert not state.update_sensor(sensor, 0, 20 * NS)
    assert not state.update_sensor(sensor, 21 * NS, 20 * NS)
    assert state.sensors[sensor] == 20 * NS


def test_clock_rollback_clears_command_and_sensor_history():
    state = _ready_state()
    assert state.accept_request(_twist(linear_x=1.0), 20 * NS, 101 * NS)
    assert state.update_clock(1, 102 * NS) is True
    assert state.command is None
    assert not state.sensors
    assert state.command_for_tick(1, 102 * NS) == (0.0, 0.0)


def test_stalled_clock_blocks_commands_until_clock_and_sensors_refresh():
    state = _ready_state()
    assert state.accept_request(_twist(linear_x=1.0), 20 * NS, 101 * NS)
    state.check_clock_stall(1_000 * NS)
    assert state.command is None
    assert state.command_for_tick(20 * NS, 1_000 * NS) == (0.0, 0.0)
    state.update_clock(20 * NS + 1, 1_001 * NS)
    for name in SENSORS:
        state.update_sensor(name, 20 * NS + 1, 20 * NS + 1)
    assert state.accept_request(_twist(linear_x=1.0), 20 * NS + 1, 1_001 * NS)


def test_pause_resume_rejects_pre_pause_request_until_every_sensor_is_fresh():
    state = _ready_state()
    assert state.accept_request(_twist(linear_x=1.0), 20 * NS, 101 * NS)
    state.check_clock_stall(101 * NS + 400_000_001)
    assert state.command_for_tick(20 * NS, 101 * NS + 400_000_001) == (0.0, 0.0)

    resumed_sim_ns = 20 * NS + 1
    resumed_wall_ns = 101 * NS + 500_000_000
    state.update_clock(resumed_sim_ns, resumed_wall_ns)
    assert state.command_for_tick(resumed_sim_ns, resumed_wall_ns) == (0.0, 0.0)
    for name in SENSORS:
        state.update_sensor(name, resumed_sim_ns, resumed_sim_ns)
    assert state.accept_request(
        _twist(linear_x=1.0), resumed_sim_ns, resumed_wall_ns
    )


def test_future_sensor_stamp_is_rejected_and_never_refreshes_old_data():
    state = _ready_state()
    assert not state.update_sensor("lidar", 21 * NS, 20 * NS)
    assert state.sensors["lidar"] == 20 * NS
    assert not state.update_sensor("odometry", 0, 20 * NS)
