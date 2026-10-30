"""Temporary model overlay for matrix camera settings and the research truck visual."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET


RESEARCH_VEHICLE_VISUAL_FLAGS = 2
# Reserve bit 2 for the controlled truck and expose research fixture meshes on
# bit 1.  Keep masks within Ogre2's user-visible range (bits 30 and 31 are
# reserved internally by Gazebo).
RESEARCH_VEHICLE_CAMERA_MASK = 1
RESEARCH_VEHICLE_Z_OFFSET_M = -0.0675
RESEARCH_VEHICLE_RPY = "1.57079632679 0 1.57079632679"


@dataclass
class RuntimeModelOverlay:
    """Owns a temporary model copy until its Gazebo process has stopped."""

    temporary_directory: tempfile.TemporaryDirectory
    models_dir: Path

    def cleanup(self) -> None:
        self.temporary_directory.cleanup()


def create_matrix_vehicle_overlay(
    source_model: Path | str,
    quality_profile: dict[str, int | float | str | bool],
    *,
    research_vehicle_mesh_uri: str | None = None,
    research_vehicle_scale: float = 0.08,
) -> RuntimeModelOverlay:
    """Apply a matrix camera profile and locked research vehicle visual."""
    source = Path(source_model).resolve()
    source_sdf = source / "model.sdf"
    if not source_sdf.is_file():
        raise RuntimeError(f"RC car model source is missing {source_sdf}")
    required = {"camera_width_px", "camera_height_px", "camera_rate_hz"}
    if not required.issubset(quality_profile):
        raise RuntimeError(f"matrix camera quality profile is missing {sorted(required - set(quality_profile))}")
    width = int(quality_profile["camera_width_px"])
    height = int(quality_profile["camera_height_px"])
    rate = float(quality_profile["camera_rate_hz"])
    if width <= 0 or height <= 0 or rate <= 0:
        raise RuntimeError("matrix camera dimensions and rate must be positive")

    temporary = tempfile.TemporaryDirectory(prefix="rc-matrix-model-")
    models_dir = Path(temporary.name) / "models"
    models_dir.mkdir()
    copied_model = models_dir / "rc_car"
    shutil.copytree(source, copied_model)
    copied_sdf = copied_model / "model.sdf"
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(copied_sdf, parser=parser)
    sensors = {
        sensor.get("name"): sensor
        for sensor in tree.getroot().iter("sensor")
        if sensor.get("name") in {"front_camera", "front_camera_segmentation"}
    }
    if set(sensors) != {"front_camera", "front_camera_segmentation"}:
        temporary.cleanup()
        raise RuntimeError("matrix vehicle model must contain the RGB and segmentation cameras")
    for sensor in sensors.values():
        image = sensor.find("./camera/image")
        if image is None:
            temporary.cleanup()
            raise RuntimeError(f"camera image settings are missing in {sensor.get('name')}")
        image_width = image.find("width")
        image_height = image.find("height")
        update_rate = sensor.find("update_rate")
        if image_width is None or image_height is None or update_rate is None:
            temporary.cleanup()
            raise RuntimeError(f"camera profile fields are incomplete in {sensor.get('name')}")
        image_width.text = str(width)
        image_height.text = str(height)
        update_rate.text = f"{rate:g}"
    if research_vehicle_mesh_uri is not None:
        _replace_vehicle_visual(
            tree.getroot(),
            mesh_uri=research_vehicle_mesh_uri,
            scale=research_vehicle_scale,
        )
    tree.write(copied_sdf, encoding="utf-8", xml_declaration=True)
    return RuntimeModelOverlay(temporary_directory=temporary, models_dir=models_dir)


def _replace_vehicle_visual(root: ET.Element, *, mesh_uri: str, scale: float) -> None:
    """Use the research truck mesh while retaining RC collisions and sensor poses."""
    if not mesh_uri.startswith("model://"):
        raise RuntimeError("research vehicle visual URI must use model://")
    if not math.isfinite(scale) or scale <= 0:
        raise RuntimeError("research vehicle visual scale must be finite and positive")
    model = root.find("model")
    if model is None:
        raise RuntimeError("RC vehicle model overlay has no model element")
    base_link = next((link for link in model.findall("link") if link.get("name") == "base_link"), None)
    if base_link is None:
        raise RuntimeError("RC vehicle model overlay has no base_link")

    old_visuals = [visual for link in model.findall("link") for visual in link.findall("visual")]
    if old_visuals:
        raise RuntimeError(
            "RC physics model must not contain generated visuals; use the locked research vehicle mesh"
        )

    visual = ET.SubElement(base_link, "visual", {"name": "research_truck_visual"})
    ET.SubElement(visual, "pose").text = (
        f"0 0 {RESEARCH_VEHICLE_Z_OFFSET_M:g} {RESEARCH_VEHICLE_RPY}"
    )
    geometry = ET.SubElement(visual, "geometry")
    mesh = ET.SubElement(geometry, "mesh")
    ET.SubElement(mesh, "uri").text = mesh_uri
    ET.SubElement(mesh, "scale").text = f"{scale:g} {scale:g} {scale:g}"
    ET.SubElement(visual, "visibility_flags").text = str(RESEARCH_VEHICLE_VISUAL_FLAGS)

    camera_names = {"front_camera", "front_camera_segmentation"}
    updated_cameras = set()
    updated_lidar = False
    for sensor in model.iter("sensor"):
        sensor_name = sensor.get("name")
        if sensor_name in camera_names:
            sensor_geometry = sensor.find("camera")
            if sensor_geometry is None:
                raise RuntimeError(f"research vehicle visual isolation requires {sensor_name} camera data")
            updated_cameras.add(sensor_name)
        elif sensor_name == "front_lidar":
            sensor_geometry = sensor.find("lidar")
            if sensor_geometry is None:
                raise RuntimeError("research vehicle visual isolation requires front_lidar data")
            updated_lidar = True
        else:
            continue
        visibility_mask = sensor_geometry.find("visibility_mask")
        if visibility_mask is None:
            visibility_mask = ET.SubElement(sensor_geometry, "visibility_mask")
        visibility_mask.text = str(RESEARCH_VEHICLE_CAMERA_MASK)
    if updated_cameras != camera_names or not updated_lidar:
        raise RuntimeError(
            "research vehicle visual isolation requires both cameras and the front LiDAR; "
            f"found cameras={sorted(updated_cameras)}, lidar={updated_lidar}"
        )
