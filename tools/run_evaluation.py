#!/usr/bin/env python3
"""Run frozen scenarios through the headless lidar baseline and log each trial."""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import sys
import time
import uuid

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = REPO_ROOT / "ros2_ws" / "src" / "car_bringup" / "config" / "scenarios"
DEFAULT_OUTPUT = REPO_ROOT / "docs" / "evaluation" / "lidar-baseline.csv"
COMMAND_TOPIC = "/model/gazebo_truck/cmd_vel"
EXPECTED_GZ_TYPE = "gz.msgs.Twist"
EXPECTED_EVALUATION_COUNT = 10

sys.path.insert(0, str(REPO_ROOT / "ml"))
from collect_dataset import SplitValidationError, validate_splits  # noqa: E402

CSV_FIELDS = [
    "run_id", "scenario_id", "scenario_sha256", "split", "mode", "seed",
    "lighting_id", "condition_tags", "start_pose", "obstacle_entity",
    "obstacle_collision", "commanded_speed_mps", "detected", "detection_source",
    "detection_stamp_ns", "stop_request_stamp_ns", "stopped_before_contact",
    "stopping_distance_m", "trial_status", "completion_reason", "collision_status",
    "collision_coverage", "collision_entity", "collision_stamp_ns",
    "ground_contact_excluded", "sensor_stale_events", "sim_duration_s",
    "package_versions",
]


def transport_writer_count(topic_info: str, message_type: str = EXPECTED_GZ_TYPE) -> int:
    match = re.search(
        r"Publishers \[Address, Message Type\]:\s*(.*?)(?:\nSubscribers|\Z)",
        topic_info,
        re.S,
    )
    if match is None:
        return 0
    return sum(
        "://" in line and (not message_type or message_type in line)
        for line in match.group(1).splitlines()
    )


def ros_command_authority_ok(topic_info: str) -> bool:
    count_match = re.search(r"Publisher count:\s*(\d+)", topic_info)
    if count_match is None or int(count_match.group(1)) != 1:
        return False
    return "safe_drive_controller" in topic_info


def ros_command_publisher_count(topic_info: str) -> int | None:
    match = re.search(r"Publisher count:\s*(\d+)", topic_info)
    return None if match is None else int(match.group(1))


def launch_command(scenario_id: str, output: Path) -> list[str]:
    return [
        "ros2", "launch", "car_bringup", "obstacle_course.launch.py",
        "mode:=lidar", f"scenario:={scenario_id}", "headless:=true",
        f"output:={output}",
    ]


def _write_row(output: Path, row: dict[str, str]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    needs_header = not output.exists() or output.stat().st_size == 0
    with output.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        if needs_header:
            writer.writeheader()
        writer.writerow({key: row.get(key, "") for key in CSV_FIELDS})
        stream.flush()
        os.fsync(stream.fileno())


def _planned_abort_row(scenario_path: Path, scenario: dict, reason: str) -> dict[str, str]:
    import hashlib

    pose = json.dumps(scenario["start_pose"], sort_keys=True, separators=(",", ":"))
    return {
        "run_id": str(uuid.uuid4()),
        "scenario_id": scenario["scenario_id"],
        "scenario_sha256": hashlib.sha256(scenario_path.read_bytes()).hexdigest(),
        "split": scenario["split"],
        "mode": "lidar",
        "seed": str(scenario["seed"]),
        "lighting_id": scenario["lighting_id"],
        "condition_tags": json.dumps(scenario["unseen_condition_tags"]),
        "start_pose": pose,
        "obstacle_entity": scenario["obstacle"]["entity"],
        "obstacle_collision": scenario["obstacle"]["collision"],
        "commanded_speed_mps": str(scenario["commanded_speed_mps"]),
        "detected": "false",
        "detection_source": "none",
        "stopped_before_contact": "false",
        "trial_status": "timed_out" if reason == "wall_watchdog_timeout" else "aborted",
        "completion_reason": reason,
        "collision_status": "unknown",
        "collision_coverage": "limited",
        "ground_contact_excluded": "false",
        "sensor_stale_events": "[]",
        "package_versions": "ros=lyrical;gazebo_sim=10.5.0;ros_gz=3.0.10",
    }


def _read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _replace_rows(path: Path, rows: list[dict[str, str]]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows({key: row.get(key, "") for key in CSV_FIELDS} for row in rows)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _terminate_process_group(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


def _topic_info(command: list[str], timeout: float = 3.0) -> str:
    try:
        completed = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout, check=False
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    return completed.stdout if completed.returncode == 0 else ""


def _write_summary(output: Path, rows: list[dict[str, str]]) -> Path:
    summary_path = output.with_name("summary.md")
    groups: dict[str, list[dict[str, str]]] = {"unseen_obstacle_placement": [], "seen_or_unlabelled": []}
    for row in rows:
        tags = set(json.loads(row.get("condition_tags") or "[]"))
        group = "unseen_obstacle_placement" if "unseen_obstacle_placement" in tags else "seen_or_unlabelled"
        groups[group].append(row)
    lines = [
        "# Lidar Baseline Results",
        "",
        "Results apply only to the truck-scale Gazebo prototype. The evaluation configs are frozen and were recorded read-only; these outcomes must not be used to tune controller or scenario parameters.",
        "",
        f"Planned rows: {len(rows)} / 10. A pass requires at least 9 pre-contact stops, ten verified coverage rows, ten `no_collision` rows, and no timed-out/aborted trial.",
        "",
        "| Condition group | Rows | Detected | Pre-contact stops | Completed | Mean stopping travel | Verified no-collision | Unknown/collision |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, group_rows in groups.items():
        group_travel = [
            float(row["stopping_distance_m"])
            for row in group_rows
            if row.get("stopping_distance_m") not in {None, ""}
        ]
        mean_travel = f"{statistics.mean(group_travel):.3f} m" if group_travel else "n/a"
        lines.append(
            f"| {name} | {len(group_rows)} | "
            f"{sum(row.get('detected') == 'true' for row in group_rows)} | "
            f"{sum(row.get('stopped_before_contact') == 'true' for row in group_rows)} | "
            f"{sum(row.get('trial_status') == 'completed' for row in group_rows)} | "
            f"{mean_travel} | "
            f"{sum(row.get('collision_status') == 'no_collision' and row.get('collision_coverage') == 'verified' for row in group_rows)} | "
            f"{sum(row.get('collision_status') in {'unknown', 'collision'} for row in group_rows)} |"
        )
    lines.extend(["", "Trial rows are in [`" + output.name + "`](`" + output.name + "`).", ""])
    stopping_travel = [
        float(row["stopping_distance_m"])
        for row in rows
        if row.get("stopping_distance_m") not in {None, ""}
    ]
    detected = sum(row.get("detected") == "true" for row in rows)
    stops = sum(row.get("stopped_before_contact") == "true" for row in rows)
    completed = sum(row.get("trial_status") == "completed" for row in rows)
    verified_clear = sum(
        row.get("collision_status") == "no_collision"
        and row.get("collision_coverage") == "verified"
        for row in rows
    )
    p3_pass = (
        len(rows) == EXPECTED_EVALUATION_COUNT
        and stops >= 9
        and completed == EXPECTED_EVALUATION_COUNT
        and verified_clear == EXPECTED_EVALUATION_COUNT
    )
    travel_range = (
        f"{min(stopping_travel):.3f}–{max(stopping_travel):.3f} m"
        if stopping_travel
        else "n/a"
    )
    travel_mean = f"{statistics.mean(stopping_travel):.3f} m" if stopping_travel else "n/a"
    lines.extend(
        [
            f"Observed P3 checkpoint: **{'PASS' if p3_pass else 'FAIL'}** — {detected}/10 detected, {stops}/10 pre-contact stops, {completed}/10 completed, and {verified_clear}/10 verified no-collision rows.",
            f"Measured odometry stopping travel: mean {travel_mean}; range {travel_range} across {len(stopping_travel)} rows with a recorded stop.",
            "",
        ]
    )
    # Replace the intentionally code-styled path with a normal relative link.
    lines[lines.index("Trial rows are in [`" + output.name + "`](`" + output.name + "`).")] = (
        f"Trial rows are in [{output.name}]({output.name})."
    )
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text("\n".join(lines), encoding="utf-8")
    return summary_path


def run_trials(*, scenario_dir: Path, manifest_path: Path, output: Path) -> list[dict[str, str]]:
    validate_splits(scenario_dir, manifest_path, require_frozen=True)
    scenarios = sorted((scenario_dir / "evaluation").glob("*.yaml"))
    if output.exists():
        raise RuntimeError(f"refusing to overwrite existing results: {output}")
    log_dir = REPO_ROOT / "artifacts" / "runs"
    log_dir.mkdir(parents=True, exist_ok=True)
    for scenario_path in scenarios:
        scenario_id = scenario_path.relative_to(scenario_dir).with_suffix("").as_posix()
        scenario = yaml_safe_load(scenario_path)
        if scenario["lighting_id"] != "obstacle_world_default":
            _write_row(output, _planned_abort_row(scenario_path, scenario, "lighting_mismatch"))
            continue
        log_path = log_dir / f"{scenario_id.replace('/', '_')}-{uuid.uuid4()}.log"
        command = launch_command(scenario_id, output.resolve())
        with log_path.open("w", encoding="utf-8") as log_stream:
            process = subprocess.Popen(
                command,
                cwd=REPO_ROOT,
                env=os.environ.copy(),
                stdout=log_stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
            start = time.monotonic()
            wall_deadline = start + max(180.0, float(scenario["run_timeout_sim_s"]) * 5.0 + 90.0)
            authority_seen = False
            abort_reason = ""
            while process.poll() is None:
                now = time.monotonic()
                existing = [
                    row for row in _read_rows(output)
                    if row.get("scenario_id") == scenario_id
                ]
                if existing and existing[-1].get("trial_status") in {
                    "completed", "timed_out", "aborted"
                }:
                    break
                gz_info = _topic_info(["gz", "topic", "-i", "-t", COMMAND_TOPIC])
                gz_count = transport_writer_count(gz_info)
                if gz_count == 1:
                    ros_info = _topic_info(["ros2", "topic", "info", "-v", COMMAND_TOPIC])
                    if ros_command_authority_ok(ros_info):
                        authority_seen = True
                    elif ros_command_publisher_count(ros_info) not in {None, 0, 1}:
                        abort_reason = "competing_writer"
                        break
                    elif (
                        ros_command_publisher_count(ros_info) == 1
                        and "safe_drive_controller" not in ros_info
                    ):
                        abort_reason = "competing_writer"
                        break
                    elif authority_seen:
                        abort_reason = "competing_writer"
                        break
                elif gz_count > 1 or authority_seen:
                    abort_reason = "competing_writer"
                    break
                elif now - start > 60.0:
                    abort_reason = "command_authority_unavailable"
                    break
                if now >= wall_deadline:
                    abort_reason = "wall_watchdog_timeout"
                    break
                time.sleep(1.0)
            if abort_reason:
                _terminate_process_group(process)
                rows = _read_rows(output)
                existing_indices = [
                    index for index, row in enumerate(rows)
                    if row.get("scenario_id") == scenario_id
                ]
                if not existing_indices:
                    _write_row(output, _planned_abort_row(scenario_path, scenario, abort_reason))
                elif len(existing_indices) == 1 and rows[existing_indices[0]].get("trial_status") != "completed":
                    replacement = _planned_abort_row(scenario_path, scenario, abort_reason)
                    replacement["run_id"] = rows[existing_indices[0]].get("run_id", replacement["run_id"])
                    rows[existing_indices[0]] = replacement
                    _replace_rows(output, rows)

        rows = _read_rows(output)
        this_scenario_rows = [row for row in rows if row.get("scenario_id") == scenario_id]
        if not this_scenario_rows:
            reason = "launch_failed_without_run_record"
            _write_row(output, _planned_abort_row(scenario_path, scenario, reason))
        elif len(this_scenario_rows) > 1:
            raise RuntimeError(f"duplicate records for planned scenario {scenario_id}")
    rows = _read_rows(output)
    if len(rows) != EXPECTED_EVALUATION_COUNT:
        raise RuntimeError(
            f"expected {EXPECTED_EVALUATION_COUNT} planned rows, found {len(rows)}; inspect the persisted CSV"
        )
    _write_summary(output, rows)
    return rows


def yaml_safe_load(path: Path) -> dict:
    import yaml

    return yaml.safe_load(path.read_text(encoding="utf-8"))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--split", choices=["evaluation"])
    parser.add_argument("--behavior", choices=["lidar"])
    parser.add_argument("--summarize", action="store_true")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--scenario-dir", type=Path, default=SCENARIO_DIR)
    parser.add_argument("--manifest", type=Path, default=REPO_ROOT / "artifacts" / "scenario-splits" / "split-manifest.json")
    args = parser.parse_args(argv)
    output = (args.output if args.output.is_absolute() else REPO_ROOT / args.output).resolve()
    if args.summarize:
        if not output.is_file():
            parser.error(f"results file does not exist: {output}")
        summary = _write_summary(output, _read_rows(output))
        print(f"summary: {summary}")
        return 0
    if args.split is None or args.behavior is None:
        parser.error("--split and --behavior are required unless --summarize is used")
    try:
        rows = run_trials(
            scenario_dir=args.scenario_dir.resolve(),
            manifest_path=args.manifest.resolve(),
            output=output,
        )
    except (RuntimeError, SplitValidationError) as exc:
        print(f"lidar evaluation failed: {exc}", file=sys.stderr)
        return 2
    print(f"recorded {len(rows)} lidar trials; results: {args.output}")
    print(f"summary: {(args.output.parent / 'summary.md')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
