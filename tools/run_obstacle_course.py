#!/usr/bin/env python3
"""Run fixed-seed Obstacle course scenarios and score their recorded behavior."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import yaml

from freeze_obstacle_course_inputs import REPO, root_hash_for_inputs
from verify_obstacle_course_inputs import verify_manifest


CONFIG = Path("ros2_ws/src/car_bringup/config/obstacle_course")
SCENARIOS = ("S0", "S1", "S2", "S3", "S3b", "S4", "S5", "S6-lidar-drop", "S6-camera-drop", "S7")
BEHAVIORS = {"FOLLOW_ROUTE", "CAUTION", "AVOID_LEFT", "AVOID_RIGHT", "STOP", "RESUME"}


def seed_for_repeat(base_seed: int, repeat_index: int) -> int:
    if base_seed < 0 or repeat_index < 1:
        raise ValueError("base seed must be nonnegative and repeat index must be positive")
    return int(base_seed) + int(repeat_index) - 1


def _read_jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def _contact_outcome(scenario: dict, evidence: list[dict]) -> str:
    observations = [row.get("evidence", {}) for row in evidence]
    if not observations:
        return "unavailable"
    non_ground = any(item.get("non_ground_collision") for item in observations)
    ground = any(item.get("ground_excluded") for item in observations)
    if not non_ground:
        return "ground_only" if ground else "none"
    return "non_ground"


def score_repeat(
    scenario: dict,
    states: list[dict],
    events: list[dict],
    fault_events: list[dict],
    contact_evidence: list[dict],
    *, repeat_index: int, run_id: str, seed: int, run_purpose: str, launched: bool,
    event_log_path: str | None = None, events_sha256: str | None = None,
    odometry_evidence: list[dict] | None = None,
    component_evidence: dict[str, list[dict]] | None = None,
) -> dict:
    transitions = [event for event in events if event.get("event_kind") == "transition"]
    initial = next((state for state in states if state.get("behavior") in BEHAVIORS), None)
    observed = ([initial["behavior"]] if initial else []) + [
        event["behavior"] for event in transitions if event.get("behavior") in BEHAVIORS
    ]
    actual_triggers = sorted({
        str(ref.get("component"))
        for event in transitions
        for ref in event.get("attribution", {}).get("triggers", [])
        if ref.get("component") in {"lidar", "camera", "imu", "odometry", "contact"}
    })
    expected = list(scenario.get("expected_behavior_sequence", []))
    expected_triggers = sorted(set(scenario.get("expected_trigger_components", [])))
    sequence_matches = observed == expected
    trigger_matches = actual_triggers == expected_triggers
    contact_outcome = _contact_outcome(scenario, contact_evidence)
    if scenario.get("contact_rule") == "contact_expected_and_latched":
        reveal = next((event for event in fault_events if event.get("event_type") == "obstacle_revealed"), None)
        d_stop = None
        if reveal and odometry_evidence:
            closest = min(odometry_evidence, key=lambda item: abs(int(item.get("sim_time_ns", 0)) - int(reveal.get("sim_time_ns", 0))))
            d_stop = closest.get("evidence", {}).get("d_stop_m")
        within = reveal and isinstance(d_stop, (int, float)) and isinstance(reveal.get("obstacle_range_m"), (int, float)) and reveal["obstacle_range_m"] <= d_stop
        has_stop = "STOP" in observed
        has_contact = any(item.get("evidence", {}).get("non_ground_collision") for item in contact_evidence)
        contact_ok = bool(has_contact and within and has_stop)
        if has_contact and within:
            contact_outcome = "permitted_inside_d_stop"
    else:
        contact_ok = contact_outcome in {"none", "ground_only"}
    if scenario.get("scenario_id") == "S6-lidar-drop":
        contact_ok = contact_ok and "STOP" in observed
    status = "fail" if launched else "blocked"
    if not launched:
        qualified = False
    else:
        passed = sequence_matches and trigger_matches and contact_ok
        status = "pass" if passed else "fail"
        qualified = run_purpose == "demonstration" and passed
    if run_purpose != "demonstration" and launched:
        status, qualified = "smoke", False
    elif not launched:
        status, qualified = "blocked", False
    return {
        "repeat_index": int(repeat_index), "run_id": str(run_id), "seed": int(seed),
        "status": status, "qualified": qualified,
        "observed_behavior_sequence": observed,
        "behavior_sequence_matches": sequence_matches,
        "actual_trigger_components": actual_triggers,
        "trigger_set_matches": trigger_matches,
        "contact_outcome": contact_outcome,
        "fault_events": fault_events,
        "events_sha256": events_sha256,
        "event_refs": [event_log_path] if event_log_path else [],
        "stale_components": sorted({
            component for component, rows in (component_evidence or {}).items()
            if any(item.get("freshness", {}).get("state") == "stale" for item in rows)
        }),
        "reason": None if sequence_matches and trigger_matches and contact_ok else "behavior, exact trigger set, or contact rule did not match",
    }


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source_identity(
    scenario_path: Path,
    decision_config_path: Path,
    profile_path: Path,
    world_path: Path,
) -> dict:
    import platform
    ros = os.environ.get("ROS_DISTRO", "unknown")
    gazebo = subprocess.run(["gz", "sim", "--versions"], capture_output=True, text=True, check=False).stdout.strip()
    try:
        ubuntu = next(line.split("=", 1)[1].strip().strip('"') for line in Path("/etc/os-release").read_text().splitlines() if line.startswith("VERSION_ID="))
    except (OSError, StopIteration):
        ubuntu = platform.platform()
    matrix = REPO / CONFIG / "trigger_matrix.yaml"
    import subprocess as sp
    revision = sp.run(["git", "rev-parse", "HEAD"], cwd=REPO, text=True, capture_output=True, check=False).stdout.strip() or "working-tree"
    return {
        "ubuntu": ubuntu, "ros_distro": ros, "gazebo_version": gazebo,
        "profile_sha256": json.loads(profile_path.read_text())["profile_sha256"],
        "scenario_sha256": _sha(scenario_path),
        "decision_config_sha256": _sha(decision_config_path),
        "decision_config_path": decision_config_path.relative_to(REPO).as_posix(),
        "trigger_matrix_sha256": _sha(matrix), "world_sha256": _sha(world_path),
        "source_revision": revision,
    }


def _scenario_launch_overrides(
    scenario_path: Path,
    decision_config_path: Path,
    *,
    run_purpose: str,
    headless: bool,
    seed: int,
    run_id: str,
    run_dir: Path,
) -> list[str]:
    scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    overrides = [
        "control:=autonomous",
        f"scenario:={scenario['scenario_id']}", f"run_purpose:={run_purpose}",
        f"scenario_config:={scenario_path}", f"decision_config:={decision_config_path}",
        f"headless:={'true' if headless else 'false'}", f"seed:={seed}", f"run_id:={run_id}",
        f"run_output_dir:={run_dir}",
    ]
    if not headless:
        overrides.append("dashboard:=true")
    return overrides


def _descendant_pids(root_pid: int, *, process_table: str | None = None) -> list[int]:
    """Return the launch process tree before ROS launch reparents stragglers."""
    if process_table is None:
        result = subprocess.run(
            ["ps", "-eo", "pid=,ppid="], cwd=REPO,
            capture_output=True, text=True, check=False,
        )
        process_table = result.stdout
    children: dict[int, list[int]] = {}
    for line in process_table.splitlines():
        fields = line.split()
        if len(fields) != 2:
            continue
        try:
            pid, parent = map(int, fields)
        except ValueError:
            continue
        children.setdefault(parent, []).append(pid)
    descendants: list[int] = []
    pending = list(children.get(int(root_pid), ()))
    while pending:
        pid = pending.pop()
        if pid in descendants:
            continue
        descendants.append(pid)
        pending.extend(children.get(pid, ()))
    return descendants


def _pid_running(pid: int) -> bool:
    """Treat a zombie as stopped; containers may not reap it promptly."""
    try:
        status = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    except OSError:
        return False
    fields = status.rsplit(")", 1)[-1].split()
    return bool(fields) and fields[0] != "Z"


def _stop_remaining_descendants(pids: list[int], *, grace_s: float = 2.0) -> None:
    """Stop nodes that survive ros2 launch shutdown because they used new sessions."""
    remaining = [pid for pid in pids if _pid_running(pid)]
    for pid in remaining:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + max(0.0, grace_s)
    while time.monotonic() < deadline and any(_pid_running(pid) for pid in remaining):
        time.sleep(0.05)
    for pid in remaining:
        if _pid_running(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def _route_end_is_stable(run_dir: Path, route_length_m: float) -> bool:
    """Detect a completed, stopped route without treating a mid-course STOP as done."""
    path = run_dir / "evidence-odometry.jsonl"
    try:
        with path.open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            end = stream.tell()
            stream.seek(max(0, end - 16_384))
            lines = stream.read().splitlines()[-16:]
    except OSError:
        return False
    samples = []
    for line in lines:
        try:
            row = json.loads(line)
            evidence = row["evidence"]
            samples.append((
                int(row["sim_time_ns"]),
                float(evidence["route_progress_m"]),
                float(evidence["speed_mps"]),
            ))
        except (ValueError, KeyError, TypeError, json.JSONDecodeError):
            continue
    if len(samples) < 4:
        return False
    recent = samples[-8:]
    if recent[-1][0] - recent[0][0] < 350_000_000:
        return False
    return all(progress >= route_length_m - 0.15 and speed <= 0.03 for _, progress, speed in recent)


def _run_one(scenario_path: Path, decision_config_path: Path, *, repeat_index: int, seed: int, run_purpose: str,
             frozen_root: str, manifest_path: Path | None,
             wall_timeout_s: float, headless: bool) -> tuple[dict, dict]:
    scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    run_id = str(uuid.uuid4())
    run_dir = REPO / "artifacts/obstacle-course/runs" / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    cmd = ["ros2", "launch", "car_bringup", "obstacle_course.launch.py", *
           _scenario_launch_overrides(
               scenario_path, decision_config_path, run_purpose=run_purpose,
               headless=headless, seed=seed, run_id=run_id, run_dir=run_dir,
           )]
    if frozen_root:
        cmd.append(f"frozen_inputs_root_sha256:={frozen_root}")
    if manifest_path is not None:
        manifest_arg = manifest_path if manifest_path.is_absolute() else (REPO / manifest_path)
        cmd.append(f"input_manifest:={manifest_arg}")
    process = subprocess.Popen(cmd, cwd=REPO, start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    timed_out = False
    route_completed = False
    descendants: set[int] = set()
    route_length = float(scenario["route"]["length_m"].get("value", scenario["route"]["length_m"]))
    deadline = time.monotonic() + wall_timeout_s
    try:
        output = ""
        while True:
            descendants.update(_descendant_pids(process.pid))
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                timed_out = True
                break
            try:
                output, _ = process.communicate(timeout=min(0.5, remaining))
                break
            except subprocess.TimeoutExpired:
                if _route_end_is_stable(run_dir, route_length):
                    route_completed = True
                    break
        try:
            if timed_out or route_completed:
                os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        if timed_out or route_completed:
            try:
                output, _ = process.communicate(timeout=15)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    process.kill()
                output, _ = process.communicate()
    except KeyboardInterrupt:
        descendants.update(_descendant_pids(process.pid))
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.communicate()
        raise
    finally:
        _stop_remaining_descendants(sorted(descendants))
    (run_dir / "launch.log").write_text(output or "", encoding="utf-8")
    profile_path = REPO / "artifacts/obstacle-course/config" / f"{scenario['scenario_id']}.json"
    world_path = REPO / "artifacts/obstacle-course/worlds" / f"{scenario['scenario_id']}.sdf"
    events = _read_jsonl(run_dir / "events.jsonl")
    states = _read_jsonl(run_dir / "states.jsonl")
    faults = _read_jsonl(run_dir / "fault-events.jsonl")
    contact = _read_jsonl(run_dir / "evidence-contact.jsonl")
    odometry = _read_jsonl(run_dir / "evidence-odometry.jsonl")
    evidence = {
        component: _read_jsonl(run_dir / f"evidence-{component}.jsonl")
        for component in ("lidar", "camera", "imu", "odometry", "contact")
    }
    report = score_repeat(
        scenario, states, events, faults, contact,
        repeat_index=repeat_index, run_id=run_id, seed=seed,
        run_purpose=run_purpose, launched=bool(states),
        event_log_path=f"artifacts/obstacle-course/runs/{run_id}/events.jsonl",
        events_sha256=_sha(run_dir / "events.jsonl") if (run_dir / "events.jsonl").is_file() else hashlib.sha256(b"").hexdigest(),
        odometry_evidence=odometry, component_evidence=evidence,
    )
    return report, {"run_dir": run_dir, "scenario": scenario, "scenario_path": scenario_path,
                    "profile_path": profile_path, "world_path": world_path, "timed_out": timed_out,
                    "route_completed": route_completed}


def _write_report(scenario, repeats, mode, run_purpose, frozen_root, identity, report_dir):
    passes = sum(1 for repeat in repeats if repeat["behavior_sequence_matches"] and repeat["trigger_set_matches"] and repeat["contact_outcome"] in {"none", "ground_only", "permitted_inside_d_stop"})
    threshold = 10 if scenario["scenario_id"] == "S6-lidar-drop" else 8
    threshold_pass = len(repeats) >= 10 and passes >= threshold
    if mode == "quick" and any(item["status"] != "blocked" for item in repeats): status, qualified = "smoke", False
    elif not repeats or all(item["status"] == "blocked" for item in repeats): status, qualified = "blocked", False
    else:
        status = "pass" if threshold_pass else "partial"
        qualified = run_purpose == "demonstration" and threshold_pass
    report = {
        "schema_version": "1.0", "report_id": str(uuid.uuid4()),
        "created_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "scenario_id": scenario["scenario_id"], "run_purpose": run_purpose,
        "mode": mode, "status": status, "qualified": qualified,
        "expected_behavior_sequence": scenario["expected_behavior_sequence"],
        "expected_trigger_components": scenario["expected_trigger_components"],
        "repeats": repeats, "source_identity": identity,
        "protected_metrics_written": False,
        "report_path": f"artifacts/obstacle-course/runs/{report_dir.name}/report.json",
        "summary_ref": None,
    }
    if frozen_root:
        report["frozen_inputs_root_sha256"] = frozen_root
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--scenario", choices=SCENARIOS)
    selection.add_argument("--all", action="store_true")
    selection.add_argument("--scenario-config", type=Path)
    parser.add_argument("--decision-config", type=Path)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--run-purpose", choices=("diagnostic", "development", "demonstration"))
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--wall-timeout-s", type=float, default=35.0)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args(argv)
    purpose = args.run_purpose or ("diagnostic" if args.quick else "demonstration")
    if args.repeats < 1 or args.wall_timeout_s <= 0:
        parser.error("repeats and wall timeout must be positive")
    if args.quick:
        args.repeats = 1
        purpose = args.run_purpose or "diagnostic"
    frozen_root = ""
    if purpose == "demonstration":
        if not args.manifest:
            parser.error("demonstration runs require --manifest")
        check = verify_manifest(args.manifest)
        if not check["valid"]:
            print(json.dumps(check, indent=2), file=sys.stderr)
            return 2
        frozen_root = check["root_sha256"]
    elif args.manifest:
        check = verify_manifest(args.manifest)
        if not check["valid"]:
            print(json.dumps(check, indent=2), file=sys.stderr)
            return 2
        frozen_root = check["root_sha256"]

    scenario_paths = []
    if args.scenario_config:
        scenario_paths = [args.scenario_config if args.scenario_config.is_absolute() else REPO / args.scenario_config]
    elif args.all:
        scenario_paths = [REPO / CONFIG / "scenarios" / f"{name}.yaml" for name in SCENARIOS]
    else:
        scenario_paths = [REPO / CONFIG / "scenarios" / f"{args.scenario}.yaml"]
    scenario_paths = [path.resolve() for path in scenario_paths]
    development_root = (REPO / CONFIG / "development").resolve()
    development_scenario_root = (development_root / "scenarios").resolve()
    decision_config_path = (
        args.decision_config if args.decision_config and args.decision_config.is_absolute()
        else REPO / args.decision_config if args.decision_config
        else REPO / CONFIG / "decision.yaml"
    ).resolve()
    is_development_decision = decision_config_path.is_relative_to(development_root)
    if not decision_config_path.is_file():
        parser.error(f"decision config does not exist: {decision_config_path}")
    for scenario_path in scenario_paths:
        if not scenario_path.is_file():
            parser.error(f"scenario config does not exist: {scenario_path}")
        is_development_scenario = scenario_path.is_relative_to(development_scenario_root)
        if is_development_scenario != is_development_decision:
            parser.error("development scenarios and decision configs must be selected together")
        if is_development_scenario and purpose != "development":
            parser.error("development configs require --run-purpose development")
        if not scenario_path.is_relative_to((REPO / CONFIG / "scenarios").resolve()) and not is_development_scenario:
            parser.error(f"scenario config must be in the production or development scenario directory: {scenario_path}")
    mode = "quick" if args.quick else ("all" if args.all else ("single" if len(scenario_paths) == 1 and args.repeats == 1 else "scenario"))
    aggregate_status = 0
    for scenario_path in scenario_paths:
        scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
        repeat_reports = []
        last_details = None
        for repeat_index in range(1, args.repeats + 1):
            seed = seed_for_repeat(int(scenario["seed"]), repeat_index)
            if purpose == "demonstration":
                check = verify_manifest(args.manifest)
                if not check["valid"] or check["root_sha256"] != frozen_root:
                    print(json.dumps({"error": "frozen inputs changed before run", **check}), file=sys.stderr)
                    return 2
            repeat, details = _run_one(
                scenario_path, decision_config_path, repeat_index=repeat_index, seed=seed,
                run_purpose=purpose, frozen_root=frozen_root,
                manifest_path=args.manifest,
                wall_timeout_s=args.wall_timeout_s, headless=args.headless,
            )
            repeat_reports.append(repeat)
            last_details = details
            print(json.dumps({"scenario_id": scenario["scenario_id"], "repeat": repeat_index, "seed": seed,
                              "status": repeat["status"], "observed_behavior_sequence": repeat["observed_behavior_sequence"],
                              "actual_trigger_components": repeat["actual_trigger_components"],
                              "contact_outcome": repeat["contact_outcome"]}, sort_keys=True), flush=True)
        if last_details is None:
            continue
        report_dir = REPO / "artifacts/obstacle-course/runs" / str(uuid.uuid4())
        report_dir.mkdir(parents=True, exist_ok=False)
        identity = _source_identity(
            scenario_path, decision_config_path,
            last_details["profile_path"], last_details["world_path"],
        )
        report = _write_report(scenario, repeat_reports, mode, purpose, frozen_root, identity, report_dir)
        report["report_path"] = f"artifacts/obstacle-course/runs/{report_dir.name}/report.json"
        schema = json.loads((REPO / "ros2_ws/src/car_bringup/config/obstacle_course/contracts/run-report.schema.json").read_text())
        import jsonschema
        from referencing import Registry, Resource
        fault_schema = json.loads((REPO / "ros2_ws/src/car_bringup/config/obstacle_course/contracts/fault-event.schema.json").read_text())
        registry = Registry().with_resource(
            "fault-event.schema.json", Resource.from_contents(fault_schema)
        )
        jsonschema.Draft202012Validator(
            schema, format_checker=jsonschema.FormatChecker(), registry=registry
        ).validate(report)
        path = report_dir / "report.json"
        path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"report": str(path.relative_to(REPO)), "status": report["status"], "qualified": report["qualified"]}, sort_keys=True), flush=True)
        if report["status"] not in {"pass", "smoke"}:
            aggregate_status = 1
    return aggregate_status


if __name__ == "__main__":
    raise SystemExit(main())
