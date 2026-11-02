"""P4 repeat scheduling and exact behavior/trigger scoring tests."""

from __future__ import annotations

import json
from pathlib import Path
import signal
import sys
import uuid

import pytest


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "tools"))
import run_obstacle_course as runner  # noqa: E402
from run_obstacle_course import (  # noqa: E402
    _scenario_launch_overrides,
    _descendant_pids,
    _route_end_is_stable,
    _stop_remaining_descendants,
    score_repeat,
    seed_for_repeat,
)


def _scenario(scenario_id="S1", contact_rule="no_non_ground_contact"):
    return {
        "scenario_id": scenario_id,
        "seed": 70002,
        "expected_behavior_sequence": ["FOLLOW_ROUTE", "CAUTION", "AVOID_LEFT", "RESUME", "FOLLOW_ROUTE"],
        "expected_trigger_components": ["lidar", "camera"],
        "contact_rule": contact_rule,
    }


def _transition(sequence, trigger):
    return {
        "event_id": str(uuid.uuid4()),
        "event_kind": "transition",
        "behavior": sequence,
        "attribution": {"triggers": [{"component": trigger}]},
    }


def test_repeat_seed_schedule_is_base_plus_repeat_index_minus_one():
    assert [seed_for_repeat(70002, n) for n in (1, 2, 10)] == [70002, 70003, 70011]
    with pytest.raises(ValueError):
        seed_for_repeat(70002, 0)


def test_development_scenario_and_decision_configs_are_forwarded_to_launch(tmp_path):
    scenario_path = tmp_path / "barrier_tuning.yaml"
    scenario_path.write_text("scenario_id: barrier_tuning\n", encoding="utf-8")
    decision_path = tmp_path / "decision.yaml"

    overrides = _scenario_launch_overrides(
        scenario_path, decision_path, run_purpose="development", headless=True,
        seed=87001, run_id="test-run", run_dir=tmp_path / "run-output",
    )

    assert f"scenario_config:={scenario_path}" in overrides
    assert f"decision_config:={decision_path}" in overrides
    assert "scenario:=barrier_tuning" in overrides
    assert "run_purpose:=development" in overrides
    assert "seed:=87001" in overrides

    visible = _scenario_launch_overrides(
        scenario_path, decision_path, run_purpose="development", headless=False,
        seed=87001, run_id="visible-run", run_dir=tmp_path / "visible-output",
    )
    assert "headless:=false" in visible
    assert "dashboard:=true" in visible


def test_launch_descendant_snapshot_captures_ros_process_groups_before_shutdown():
    table = """ 100 1
 101 100
 102 101
 200 1
"""
    assert _descendant_pids(100, process_table=table) == [101, 102]


def test_launch_cleanup_terminates_processes_that_survive_ros_shutdown(monkeypatch):
    sent = []
    monkeypatch.setattr(runner, "_pid_running", lambda _pid: True)
    monkeypatch.setattr(runner.os, "kill", lambda pid, sig: sent.append((pid, sig)))

    _stop_remaining_descendants([123, 456], grace_s=0)

    assert sent == [
        (123, signal.SIGTERM), (456, signal.SIGTERM),
        (123, signal.SIGKILL), (456, signal.SIGKILL),
    ]


def test_visible_runner_stops_after_a_stable_route_finish_but_not_a_midcourse_stop(tmp_path):
    evidence_path = tmp_path / "evidence-odometry.jsonl"
    samples = [
        {"sim_time_ns": i * 50_000_000,
         "evidence": {"route_progress_m": 14.9, "speed_mps": 0.0}}
        for i in range(1, 11)
    ]
    evidence_path.write_text("".join(json.dumps(sample) + "\n" for sample in samples), encoding="utf-8")
    assert _route_end_is_stable(tmp_path, 14.943321)

    midcourse = [{**sample, "evidence": {"route_progress_m": 5.0, "speed_mps": 0.0}} for sample in samples]
    evidence_path.write_text("".join(json.dumps(sample) + "\n" for sample in midcourse), encoding="utf-8")
    assert not _route_end_is_stable(tmp_path, 14.943321)


def test_scoring_uses_initial_state_and_transition_events_only():
    states = [{"behavior": "FOLLOW_ROUTE"}, {"behavior": "STOP"}]
    events = [
        {"event_kind": "evaluation", "behavior": "AVOID_RIGHT", "attribution": {"triggers": [{"component": "contact"}]}},
        _transition("CAUTION", "lidar"),
        _transition("AVOID_LEFT", "camera"),
        {"event_kind": "evaluation", "behavior": "RESUME", "attribution": {"triggers": [{"component": "imu"}]}},
        _transition("RESUME", "lidar"),
        _transition("FOLLOW_ROUTE", "odometry"),
    ]
    result = score_repeat(
        _scenario(), states, events, [], [], repeat_index=1, run_id=str(uuid.uuid4()),
        seed=70002, run_purpose="demonstration", launched=True,
    )
    assert result["observed_behavior_sequence"] == _scenario()["expected_behavior_sequence"]
    assert result["behavior_sequence_matches"] is True
    assert result["actual_trigger_components"] == ["camera", "lidar", "odometry"]
    assert result["trigger_set_matches"] is False
    assert result["qualified"] is False


def test_quick_runs_never_qualify_and_contact_must_follow_s5_rule():
    scenario = _scenario("S5", "contact_expected_and_latched")
    scenario["expected_behavior_sequence"] = ["FOLLOW_ROUTE", "STOP"]
    scenario["expected_trigger_components"] = ["lidar", "contact"]
    events = [_transition("STOP", "lidar")]
    result = score_repeat(
        scenario, [{"behavior": "FOLLOW_ROUTE"}], events,
        [{"event_type": "obstacle_revealed", "obstacle_range_m": 0.01, "sim_time_ns": 1}],
        [{"evidence": {"non_ground_collision": True}}], repeat_index=1,
        run_id=str(uuid.uuid4()), seed=70005, run_purpose="diagnostic", launched=True,
        odometry_evidence=[{"sim_time_ns": 1, "evidence": {"d_stop_m": 0.02}}],
    )
    assert result["status"] == "smoke"
    assert result["qualified"] is False
    assert result["contact_outcome"] == "permitted_inside_d_stop"


def test_s6_lidar_dropout_requires_stop_for_each_repeat():
    scenario = _scenario("S6-lidar-drop")
    scenario["expected_behavior_sequence"] = ["FOLLOW_ROUTE", "STOP"]
    scenario["expected_trigger_components"] = ["lidar"]
    result = score_repeat(
        scenario, [{"behavior": "FOLLOW_ROUTE"}], [], [], [], repeat_index=1,
        run_id=str(uuid.uuid4()), seed=70006, run_purpose="demonstration", launched=True,
    )
    assert result["behavior_sequence_matches"] is False
    assert result["qualified"] is False
