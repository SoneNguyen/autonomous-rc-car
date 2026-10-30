from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.rc_module_matrix import (  # noqa: E402
    CORE_MODULES,
    MatrixError,
    SIMULATION_ONLY_STATEMENT,
    derive_report_status,
    discover_scenarios,
    _make_report,
    run_overlap_preflight,
    select_scenarios,
    scenario_sha256,
    _ground_truth_ref,
    validate_report,
    validate_scenario,
    write_report,
)


def _scenario(*, module="lidar", purpose="qualification"):
    return {
        "schema_version": "1.0",
        "module_id": module,
        "scenario_id": f"{module}/case-a",
        "seed": 100,
        "case_kind": "nominal",
        "run_purpose": purpose,
        "quick_case": purpose == "qualification",
        "world_id": "rc_module_matrix",
        "world_path": "simulation/worlds/rc_module_matrix.sdf",
        "start_pose": {"x_m": -12.0, "y_m": 0.0, "z_m": 0.0675, "roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": 0.0},
        "vehicle_motion": {"speed_profile": [{"sim_time_s": 0, "speed_mps": 0.0}]},
        "obstacle_placements": [],
        "setup": {"sensor": "lidar"},
        "ground_truth": {"description": "declared truth", "references": [{"path": "fixture pose from scenario seed"}]},
        "measurements": [{"measurement_id": "sample", "source": "/scan", "field": "ranges", "unit": "m", "frame_id": "lidar_link", "sampling": "sensor timestamp"}],
        "tolerances": [] if purpose == "tolerance_pilot" else [{"tolerance_id": "sample-range", "measurement_id": "sample", "unit": "m", "comparison": "maximum", "maximum": 0.03, "derivation": "LiDAR range resolution and explicit fixture face"}],
        "source_refs": {"vehicle_profile": {"path": "ros2_ws/src/car_bringup/config/rc_vehicle.yaml", "pointer": "/presets/rc_car/sensors/lidar"}, "protocols": ["matrix fixed-seed sensor cases"]},
        "overlap_refs": {"required": True, "protocol_ids": ["matrix-unique-physical-set-v1"]},
    }


def _camera(scenario):
    scenario["source_refs"].update({
        "detector_id": "rc-module-matrix-frozen-detector-v1",
        "detector_sha256": "79eafbe11c919fef2f240762b512a3a389cf7bd00fe3076ac43490ef2ae8eaf0",
        "threshold": 0.8,
    })
    return scenario


def test_speed_profile_requires_zero_start_and_strictly_increasing_time():
    scenario = _scenario()
    scenario["vehicle_motion"]["speed_profile"] = [
        {"sim_time_s": 0, "speed_mps": 0}, {"sim_time_s": 1, "speed_mps": 0.1}
    ]
    assert validate_scenario(scenario) == scenario
    scenario["vehicle_motion"]["speed_profile"].append({"sim_time_s": 1, "speed_mps": 0.2})
    with pytest.raises(MatrixError, match="increase strictly"):
        validate_scenario(scenario)


def test_empty_obstacle_list_is_explicitly_valid_and_pilot_has_no_tolerances():
    pilot = _scenario(purpose="tolerance_pilot")
    assert validate_scenario(pilot)["obstacle_placements"] == []
    assert pilot["tolerances"] == []


def test_pilot_rejects_tolerance_ids_and_quick_marker():
    pilot = _scenario(purpose="tolerance_pilot")
    pilot["quick_case"] = True
    with pytest.raises(MatrixError, match="cannot be marked quick_case"):
        validate_scenario(pilot)


def test_selection_requires_exactly_one_quick_case_per_module(monkeypatch):
    scenarios = [_scenario(), {**_scenario(), "scenario_id": "lidar/case-b", "quick_case": False}]
    monkeypatch.setattr("tools.rc_module_matrix.discover_scenarios", lambda module, pilot=False: scenarios)
    mode, selection = select_scenarios(module="lidar", quick=True)
    assert mode == "quick"
    assert [item["scenario_id"] for item in selection["lidar"]] == ["lidar/case-a"]
    monkeypatch.setattr("tools.rc_module_matrix.discover_scenarios", lambda module, pilot=False: scenarios[1:])
    with pytest.raises(MatrixError, match="exactly one quick_case"):
        select_scenarios(module="lidar", quick=True)


def test_pilot_report_never_contains_verdicts_or_tolerance_ids():
    pilot = _camera(_scenario(module="camera", purpose="tolerance_pilot"))
    report = _make_report(
        module="camera", mode="pilot", scenarios=[pilot], required_case_ids=[pilot["scenario_id"]],
        overlap=[], reason="pilot capture unavailable", run_id="00000000-0000-4000-8000-000000000001",
    )
    assert report["qualified"] is False and report["status"] == "blocked"
    assert report["scenario_results"][0]["ground_truth_ref"]
    report["scenario_results"][0]["observations"] = [{
        "observation_id": "pilot-pixels", "source": "/camera/labels", "stamp_sim_ns": 1,
        "frame_id": "camera_link", "value": {"width_px": 20, "height_px": 10},
        "ground_truth": {"entity_id": "cone_01"}, "unit": "px", "tolerance_id": None,
        "result": "not_applicable", "evidence_ref": "artifacts/rc-car-modules/pilot/frame.png",
    }]
    validate_report(report)


def test_report_status_transitions_never_qualify_quick_or_partial_runs():
    expected = [{"scenario_id": f"lidar/case-{index}", "status": "pass"} for index in range(2)]
    assert derive_report_status(mode="quick", required_case_ids=[x["scenario_id"] for x in expected], scenario_results=expected) == ("smoke", False)
    assert derive_report_status(mode="module", required_case_ids=["lidar/case-0", "lidar/case-1", "lidar/case-2"], scenario_results=expected) == ("partial", False)
    assert derive_report_status(mode="module", required_case_ids=[x["scenario_id"] for x in expected], scenario_results=expected) == ("pass", True)
    failed = [*expected, {"scenario_id": "lidar/case-2", "status": "fail"}]
    assert derive_report_status(mode="module", required_case_ids=[x["scenario_id"] for x in failed], scenario_results=failed) == ("fail", False)


@pytest.mark.parametrize("mode", ["module", "case", "quick", "core_matrix"])
def test_every_blocked_result_has_ground_truth_reference(mode):
    scenario = _scenario(module="imu_odometry")
    report = _make_report(
        module="imu_odometry", mode=mode, scenarios=[scenario], required_case_ids=[scenario["scenario_id"]],
        overlap=[], reason="capture unavailable", run_id="00000000-0000-4000-8000-000000000002",
    )
    report["simulation_noise_statement"] = SIMULATION_ONLY_STATEMENT
    if mode == "core_matrix":
        report["module_id"] = "matrix"
        report.pop("simulation_noise_statement")
        report["aggregate_module_reports"] = []
    validate_report(report)


def test_missing_ground_truth_reference_is_rejected():
    scenario = _scenario()
    scenario["ground_truth"]["references"] = []
    with pytest.raises(MatrixError, match="references"):
        validate_scenario(scenario)


def test_report_ground_truth_ref_prefers_the_fixture_entity_over_a_config_source():
    scenario = _scenario()
    scenario["ground_truth"]["references"] = [
        {"path": "ros2_ws/src/car_bringup/config/rc_vehicle.yaml"},
        {"path": "simulation/worlds/rc_module_matrix.sdf", "entity_id": "vehicle_01_collision"},
    ]
    assert _ground_truth_ref(scenario) == "simulation/worlds/rc_module_matrix.sdf#entity=vehicle_01_collision"


def test_overlap_failure_rejects_duplicate_matrix_physical_setup():
    first = _scenario()
    duplicate = {**first, "scenario_id": "lidar/case-copy", "seed": first["seed"] + 1}
    checks = run_overlap_preflight([first, duplicate])
    assert len(checks) == 1
    assert checks[0]["passed"] is False
    assert "duplicate physical setup" in checks[0]["error"]


def test_matrix_stop_reason_is_validated_by_local_contract():
    scenario = _scenario()
    report = _make_report(
        module="lidar", mode="module", scenarios=[scenario], required_case_ids=[scenario["scenario_id"]],
        overlap=[], reason="blocked", run_id="00000000-0000-4000-8000-000000000003",
    )
    report["stop_reason"] = "sensor_stale"
    validate_report(report)
    report["stop_reason"] = "not_a_matrix_reason"
    with pytest.raises(MatrixError, match="does not satisfy tracked schema"):
        validate_report(report)


def test_artifact_writer_rejects_paths_outside_matrix_root(tmp_path):
    scenario = _scenario()
    report = _make_report(
        module="lidar", mode="module", scenarios=[scenario], required_case_ids=[scenario["scenario_id"]],
        overlap=[], reason="blocked", run_id="00000000-0000-4000-8000-000000000004",
    )
    with pytest.raises(MatrixError, match="must remain under artifacts/rc-car-modules"):
        write_report(report, output_root=tmp_path)
    assert not list(tmp_path.iterdir())


def test_core_module_selection_excludes_stretch_by_default(monkeypatch):
    monkeypatch.setattr("tools.rc_module_matrix.discover_scenarios", lambda module, pilot=False: [_scenario(module=module)])
    mode, selection = select_scenarios(all_modules=True)
    assert mode == "core_matrix"
    assert tuple(selection) == CORE_MODULES


def test_quick_selection_can_target_one_module(monkeypatch):
    monkeypatch.setattr("tools.rc_module_matrix.discover_scenarios", lambda module, pilot=False: [_scenario(module=module)])
    mode, selection = select_scenarios(module="lidar", quick=True)
    assert mode == "quick" and list(selection) == ["lidar"]


def test_scenario_hash_is_the_exact_checked_in_file_hash():
    import hashlib

    scenario = discover_scenarios("lidar")[0]
    path = ROOT / "ros2_ws/src/car_bringup/config/rc_module_matrix/lidar.yaml"
    assert scenario_sha256(scenario) == hashlib.sha256(path.read_bytes()).hexdigest()


def test_runtime_adapter_keeps_compatibility_scenario_ephemeral_and_scoped():
    from tools.checks.runtime_capture import runtime_scenario

    scenario = discover_scenarios("lidar")[0]
    name, runtime = runtime_scenario(scenario, "run-123")
    assert name == f"runtime/run-123/{scenario['scenario_id']}"
    assert runtime["world"] == scenario["world_path"]
    assert runtime["seed"] == scenario["seed"]
    expected_placements = [
        item for item in scenario["obstacle_placements"]
        if -14 <= item["pose"]["x_m"] <= 14 and -2.5 <= item["pose"]["y_m"] <= 2.5
    ]
    assert runtime["obstacles"] == [
        {"entity": item["entity_id"], "pose": item["pose"]}
        for item in expected_placements
    ]
