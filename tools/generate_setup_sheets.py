#!/usr/bin/env python3
"""Generate vehicle-relative RC setup sheets for every Obstacle course scenario."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml

from validate_real_car_mapping import REPO, SCENARIO_IDS, validate_mapping

SCENARIOS = Path("ros2_ws/src/car_bringup/config/obstacle_course/scenarios")
DECISION = Path("ros2_ws/src/car_bringup/config/obstacle_course/decision.yaml")
SCENARIO_IDS = ("S0", "S1", "S2", "S3", "S3b", "S4", "S5", "S6-lidar-drop", "S6-camera-drop", "S7")
SETUP_BEGIN = "<!-- BEGIN GENERATED RC SETUP SHEETS -->"
SETUP_END = "<!-- END GENERATED RC SETUP SHEETS -->"


def _measurement(value: dict) -> str:
    return f"{float(value['value']):g} {value['unit']}"


def render_sheet(scenario: dict, decision: dict) -> str:
    anchor = "setup-" + str(scenario["scenario_id"]).lower()
    lines = [
        f'<a id="{anchor}"></a>', f"### {scenario['scenario_id']} — real-car setup", "",
        "Obstacle course demonstration geometry. Place objects using the vehicle-relative course frame. This is a simulator-to-hardware setup guide, not a claim that the physical car was tested.", "",
        "#### Vehicle and route", "",
        f"- Entry speed: {_measurement(scenario['entry_speed_mps'])}.",
        f"- Route length: {_measurement(scenario['route']['length_m'])}.",
        f"- Route frame: `{scenario['route']['frame']}`; forward is +x and left is +y.",
        "- Measure the physical car and scale the layout to its verified wheelbase, width, and turning radius.", "",
        "#### Obstacles", "",
    ]
    if not scenario.get("obstacles"):
        lines.append("- None; use the clear route as the baseline.")
    for obstacle in scenario.get("obstacles", []):
        pose, dims = obstacle["pose"], obstacle["dimensions_m"]
        lines.extend([
            f"##### {obstacle['entity_id']} ({obstacle['class']})", "",
            f"- Color: {obstacle.get('colour', 'unspecified')}.",
            f"- Vehicle-relative center: x={_measurement(pose['x'])}, y={_measurement(pose['y'])}, yaw={_measurement(pose['yaw'])}.",
            f"- Size: length={_measurement(dims['length'])}, width={_measurement(dims['width'])}, height={_measurement(dims['height'])}.",
            f"- Single-plane LiDAR: {'object intersects the scan plane' if obstacle.get('scan_plane_intersection') else 'scan-plane intersection not required'}.",
        ])
        if obstacle.get("initial_pose"):
            initial = obstacle["initial_pose"]
            lines.append(f"- Hidden staging pose before reveal: x={_measurement(initial['x'])}, y={_measurement(initial['y'])}.")
        lines.append("")
    lines.extend([
        "#### Expected demonstration", "",
        f"- Behavior sequence: `{' → '.join(scenario['expected_behavior_sequence'])}`.",
        f"- Expected triggers: `{', '.join(scenario['expected_trigger_components']) or 'none'}`.",
        f"- Contact rule: `{scenario['contact_rule']}`.",
        "- HSV values are simulator classifier references; they are not physical camera thresholds.",
    ])
    classes = sorted({obstacle["class"] for obstacle in scenario.get("obstacles", [])})
    for class_name in classes:
        ranges = decision.get("camera", {}).get("hsv_classes", {}).get(class_name, {}).get("ranges", [])
        for index, hsv in enumerate(ranges, start=1):
            lines.append(f"- Camera class `{class_name}` HSV range {index}: {hsv['lower_hsv']['value']} to {hsv['upper_hsv']['value']}.")
    return "\n".join(lines) + "\n"


def update_combined_setup_document(output_path: Path, scenarios_dir: Path | None = None) -> list[str]:
    """Replace the generated setup section inside the single implementation document."""
    scenario_root = scenarios_dir or (REPO / SCENARIOS)
    decision = yaml.safe_load((REPO / DECISION).read_text(encoding="utf-8"))
    output = output_path if output_path.is_absolute() else REPO / output_path
    document = output.read_text(encoding="utf-8")
    if document.count(SETUP_BEGIN) != 1 or document.count(SETUP_END) != 1:
        raise ValueError(f"{output}: expected one setup-sheet marker pair")
    start = document.index(SETUP_BEGIN) + len(SETUP_BEGIN)
    end = document.index(SETUP_END)
    if end < start:
        raise ValueError(f"{output}: setup-sheet markers are out of order")
    sections = []
    for scenario_id in SCENARIO_IDS:
        scenario = yaml.safe_load((scenario_root / f"{scenario_id}.yaml").read_text(encoding="utf-8"))
        sections.append(render_sheet(scenario, decision).rstrip())
    rendered = "\n\n" + "\n\n".join(sections) + "\n"
    output.write_text(document[:start] + rendered + document[end:], encoding="utf-8")
    return list(SCENARIO_IDS)


def generate_sheets(mapping_path: Path, output_dir: Path, scenarios_dir: Path | None = None) -> list[Path]:
    scenario_root = scenarios_dir or (REPO / SCENARIOS)
    decision = yaml.safe_load((REPO / DECISION).read_text(encoding="utf-8"))
    output = output_dir if output_dir.is_absolute() else REPO / output_dir
    output.mkdir(parents=True, exist_ok=True)
    paths = []
    for scenario_id in SCENARIO_IDS:
        scenario_path = scenario_root / f"{scenario_id}.yaml"
        scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
        destination = output / f"{scenario_id}.md"
        destination.write_text(render_sheet(scenario, decision), encoding="utf-8")
        paths.append(destination)
    return paths


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--all", action="store_true", required=True)
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="implementation document containing the generated-section markers")
    args = parser.parse_args(argv)
    try:
        profile = REPO / "ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml"
        validate_mapping(args.mapping, profile, None, REPO)
        scenarios = update_combined_setup_document(args.output)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f"generate_setup_sheets: {exc}", file=sys.stderr)
        return 2
    print(f"Updated {len(scenarios)} scenario setup sections in {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
