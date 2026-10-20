"""Safety and output-contract tests for the lidar repeated-run wrapper."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from run_evaluation import (  # noqa: E402
    _write_summary,
    launch_command,
    ros_command_authority_ok,
    ros_command_publisher_count,
    transport_writer_count,
)


def test_transport_writer_count_filters_by_twist_type():
    info = """Publishers [Address, Message Type]:
  tcp://127.0.0.1:1234, gz.msgs.Twist
  tcp://127.0.0.1:1235, gz.msgs.StringMsg
Subscribers [Address, Message Type]:
"""
    assert transport_writer_count(info) == 1
    assert transport_writer_count(info, "") == 2
    assert transport_writer_count("Subscribers [Address, Message Type]:\n") == 0


def test_ros_authority_requires_exactly_one_controller_publisher():
    only_controller = "Publisher count: 1\nNode name: safe_drive_controller\n"
    two_writers = "Publisher count: 2\nNode name: safe_drive_controller\n"
    wrong_node = "Publisher count: 1\nNode name: teleop_keyboard\n"
    assert ros_command_authority_ok(only_controller)
    assert not ros_command_authority_ok(two_writers)
    assert not ros_command_authority_ok(wrong_node)
    assert ros_command_publisher_count(wrong_node) == 1
    assert ros_command_publisher_count("not available") is None


def test_launcher_always_uses_scenario_backed_headless_lidar_mode():
    command = launch_command("evaluation/eval_01", Path("/tmp/results.csv"))
    assert command[:4] == ["ros2", "launch", "car_bringup", "obstacle_course.launch.py"]
    assert "mode:=lidar" in command
    assert "scenario:=evaluation/eval_01" in command
    assert "headless:=true" in command
    assert not any("gz sim" in argument or "headless:=false" in argument for argument in command)


def test_summary_preserves_unseen_obstacle_group_metrics(tmp_path):
    output = tmp_path / "baseline.csv"
    rows = [
        {
            "condition_tags": '["unseen_obstacle_placement"]',
            "detected": "true",
            "stopped_before_contact": "true",
            "trial_status": "completed",
            "collision_status": "no_collision",
            "collision_coverage": "verified",
            "stopping_distance_m": "0.2",
        },
        {
            "condition_tags": "[]",
            "detected": "false",
            "stopped_before_contact": "false",
            "trial_status": "timed_out",
            "collision_status": "unknown",
            "collision_coverage": "limited",
        },
    ]
    summary = _write_summary(output, rows).read_text(encoding="utf-8")
    assert "unseen_obstacle_placement | 1 | 1 | 1 | 1 | 0.200 m | 1 | 0" in summary
    assert "seen_or_unlabelled | 1 | 0 | 0 | 0 | n/a | 0 | 1" in summary
    assert "Observed P3 checkpoint: **FAIL**" in summary
    assert "must not be used to tune" in summary
