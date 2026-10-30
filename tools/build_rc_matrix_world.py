#!/usr/bin/env python3
"""Resolve a matrix scenario into a deterministic generated Gazebo world."""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOCK = REPO_ROOT / "ros2_ws/src/car_bringup/config/rc_module_matrix/research_assets.lock.yaml"
DEFAULT_LIGHTING = REPO_ROOT / "ros2_ws/src/car_bringup/config/rc_module_matrix/lighting.yaml"
DEFAULT_QUALITY = REPO_ROOT / "ros2_ws/src/car_bringup/config/rc_module_matrix/quality.yaml"
DEFAULT_OUTPUT = REPO_ROOT / "artifacts/rc-car-matrix/worlds/resolved.sdf"


class MatrixWorldError(ValueError):
    pass


def _load_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise MatrixWorldError(f"expected YAML mapping in {path}")
    return data


def _read_map(asset_lock: dict[str, Any], asset_id: str) -> dict[str, Any]:
    for asset in asset_lock.get("assets", []):
        if asset.get("asset_id") == asset_id:
            return asset
    raise MatrixWorldError(f"asset is not present in the lock: {asset_id}")


def _set_text(element: ET.Element | None, text: str, name: str) -> None:
    if element is None:
        raise MatrixWorldError(f"matrix world template is missing {name}")
    element.text = text


def _add_distractor(world: ET.Element, name: str) -> None:
    if name != "roadside_props":
        return
    for index, y in enumerate((2.12, -2.12)):
        model = ET.SubElement(world, "model", {"name": f"matrix_roadside_prop_{index}"})
        ET.SubElement(model, "static").text = "true"
        link = ET.SubElement(model, "link", {"name": "prop_link"})
        visual = ET.SubElement(link, "visual", {"name": "prop_visual"})
        ET.SubElement(visual, "pose").text = f"0 {y} 0.08 0 0 0"
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "box").append(ET.Element("size"))
        geometry.find("./box/size").text = "0.22 0.18 0.16"
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "ambient").text = "0.18 0.42 0.56 1"
        ET.SubElement(material, "diffuse").text = "0.18 0.42 0.56 1"


def build_world(
    scenario_path: Path,
    lock_path: Path,
    output_path: Path,
    *,
    template_path: Path | None = None,
    lighting_path: Path = DEFAULT_LIGHTING,
    quality_path: Path = DEFAULT_QUALITY,
    quality_preset: str | None = None,
) -> str:
    scenario_path = scenario_path.resolve()
    lock_path = lock_path.resolve()
    output_path = output_path.resolve()
    scenario_document = _load_yaml(scenario_path)
    scenario = scenario_document.get("scenario", scenario_document)
    if scenario.get("world") != "simulation/worlds/rc_module_matrix.sdf":
        raise MatrixWorldError("scenario does not select the matrix world")
    if scenario.get("split") not in {"qualification", "pilot", "runtime"}:
        raise MatrixWorldError("scenario purpose is invalid")
    template = template_path or (REPO_ROOT / scenario["world"])
    asset_lock = _load_yaml(lock_path)
    lighting = _load_yaml(lighting_path)["presets"]
    quality = _load_yaml(quality_path)["presets"]
    setup = scenario.get("setup", {})
    try:
        light = lighting[scenario.get("lighting_preset", setup.get("lighting_preset", "neutral_day"))]
        floor_id = scenario.get("floor_texture", setup.get("floor_texture", "floor_asphalt_001"))
        floor = _read_map(asset_lock, floor_id)
        profile_id = quality_preset or scenario.get("quality_preset", setup.get("quality_preset", "high"))
        profile = quality[profile_id]
    except KeyError as exc:
        raise MatrixWorldError(f"scenario selects an unknown matrix preset or texture: {exc}") from exc

    tree = ET.parse(template)
    world = tree.getroot().find("world")
    if world is None:
        raise MatrixWorldError("SDF template has no world element")
    world.set("name", "rc_module_matrix")

    matrix_scenario = "world_id" in scenario
    if matrix_scenario:
        class_defaults = {
            "cone": "cone_clean_01",
            "pedestrian": "pedestrian_cross_step_01",
            "vehicle": "vehicle_compact_suv_01",
        }
        obstacle_by_entity = {
            row["entity_id"]: row for row in scenario.get("obstacle_placements", [])
        }
        instances = {
            row["entity"]: row for row in scenario.get("object_instances", [])
        }
        includes = {node.findtext("name"): node for node in world.findall("include")}
        vehicle_include = includes.get("gazebo_truck")
        for include in list(world.findall("include")):
            if include is not vehicle_include:
                world.remove(include)
        includes = {"gazebo_truck": vehicle_include} if vehicle_include is not None else {}
        for entity, obstacle in obstacle_by_entity.items():
            class_id = next((name for name in class_defaults if entity.startswith(name)), None)
            if class_id is None:
                raise MatrixWorldError(f"no locked research model class matches fixture {entity}")
            instance = instances.get(entity, {})
            asset_id = instance.get("asset_id", class_defaults[class_id])
            asset = _read_map(asset_lock, asset_id)
            if asset.get("class_id") != class_id:
                raise MatrixWorldError(f"asset class does not match scenario target {entity}")
            include = ET.SubElement(world, "include")
            ET.SubElement(include, "uri").text = f"model://research/{class_id}/{asset_id}"
            ET.SubElement(include, "name").text = entity
            pose = obstacle["pose"]
            ET.SubElement(include, "pose").text = (
                f"{pose['x_m']} {pose['y_m']} {pose.get('z_m', 0)} "
                f"{pose.get('roll_rad', 0)} {pose.get('pitch_rad', 0)} {pose.get('yaw_rad', 0)}"
            )
            includes[entity] = include
        floor_texture = floor_id
    else:
        obstacle_by_entity = {row["entity"]: row for row in scenario.get("obstacles", [])}
        instances = {row["entity"]: row for row in scenario.get("object_instances", [])}
        includes = {node.findtext("name"): node for node in world.findall("include")}
        for entity, obstacle in obstacle_by_entity.items():
            include = includes.get(entity)
            instance = instances.get(entity)
            if include is None or instance is None:
                raise MatrixWorldError(f"template lacks a target include or scenario asset for {entity}")
            asset = _read_map(asset_lock, instance["asset_id"])
            if asset.get("class_id") != obstacle.get("class_id"):
                raise MatrixWorldError(f"asset class does not match scenario target {entity}")
            _set_text(include.find("uri"), f"model://research/{asset['class_id']}/{asset['asset_id']}", f"{entity} URI")
            pose = obstacle["pose"]
            _set_text(
                include.find("pose"),
                f"{pose['x_m']} {pose['y_m']} {pose.get('z_m', 0)} "
                f"{pose.get('roll_rad', 0)} {pose.get('pitch_rad', 0)} {pose.get('yaw_rad', 0)}",
                f"{entity} pose",
            )

        for entity in set(includes) - set(obstacle_by_entity):
            if entity != "gazebo_truck":
                world.remove(includes[entity])
        floor_texture = scenario["floor_texture"]

    vehicle_include = includes.get("gazebo_truck")
    start_pose = scenario.get("start_pose")
    if vehicle_include is not None and start_pose is not None:
        existing_pose = [float(value) for value in (vehicle_include.findtext("pose") or "0 0 0 0 0 0").split()]
        if len(existing_pose) != 6:
            raise MatrixWorldError("template vehicle include pose must have six components")
        pose_values = [
            start_pose["x_m"], start_pose["y_m"], existing_pose[2],
            start_pose.get("roll_rad", 0), start_pose.get("pitch_rad", 0),
            start_pose.get("yaw_rad", 0),
        ]
        _set_text(vehicle_include.find("pose"), " ".join(str(value) for value in pose_values), "vehicle pose")

    scene = world.find("scene")
    _set_text(scene.find("ambient") if scene is not None else None,
              " ".join(str(v) for v in light["ambient"]), "scene ambient")
    _set_text(scene.find("shadows") if scene is not None else None,
              "true" if profile["shadows"] else "false", "scene shadows")
    sun = world.find('./light[@name="sun"]')
    _set_text(sun.find("diffuse") if sun is not None else None,
              " ".join(str(v) for v in light["diffuse"]), "sun diffuse")
    _set_text(sun.find("direction") if sun is not None else None,
              " ".join(str(v) for v in light["direction"]), "sun direction")

    road = next((model for model in world.findall("model") if model.get("name") == "asphalt_road"), None)
    visual = road.find("./link/visual") if road is not None else None
    material = visual.find("material") if visual is not None else None
    pbr = material.find("./pbr/metal") if material is not None else None
    if pbr is None:
        raise MatrixWorldError("matrix road material must define PBR metal")
    base_name = {
        "floor_asphalt_001": "Asphalt001_1K-JPG",
        "floor_paving_001": "PavingStones001_1K-JPG",
        "floor_concrete_017": "Concrete017_1K-JPG",
    }[floor_texture]
    floor_uri = f"model://research/floors/{floor_texture}/{base_name}"
    _set_text(pbr.find("albedo_map"), floor_uri + "_Color.jpg", "road albedo map")
    _set_text(pbr.find("normal_map"), floor_uri + "_NormalGL.jpg", "road normal map")
    _set_text(pbr.find("roughness_map"), floor_uri + "_Roughness.jpg", "road roughness map")

    distractor_names = {item.get("asset_id") for item in scenario.get("distractors", [])}
    _add_distractor(world, "roadside_props" if "roadside_props" in distractor_names else "none")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return hashlib.sha256(output_path.read_bytes()).hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--lock", type=Path, default=DEFAULT_LOCK)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--template", type=Path)
    parser.add_argument("--lighting", type=Path, default=DEFAULT_LIGHTING)
    parser.add_argument("--quality", type=Path, default=DEFAULT_QUALITY)
    parser.add_argument("--quality-preset", choices=["high", "low"])
    args = parser.parse_args(argv)
    try:
        digest = build_world(args.scenario, args.lock, args.output, template_path=args.template,
                             lighting_path=args.lighting, quality_path=args.quality,
                             quality_preset=args.quality_preset)
    except (OSError, yaml.YAMLError, ET.ParseError, MatrixWorldError, KeyError) as exc:
        print(f"matrix world generation failed: {exc}", file=sys.stderr)
        return 2
    print(f"Resolved world: {args.output}")
    print(f"Resolved world SHA-256: {digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
