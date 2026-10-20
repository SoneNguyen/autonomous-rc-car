"""Temporary model resource overlay for autonomous command-authority safety."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET


TRIGGERED_PUBLISHER_NAME = "gz::sim::systems::TriggeredPublisher"
EXPECTED_FILTERED_PLUGIN_COUNT = 5


@dataclass
class RuntimeModelOverlay:
    """Owns a temporary model copy until its Gazebo process has stopped."""

    temporary_directory: tempfile.TemporaryDirectory
    models_dir: Path

    def cleanup(self) -> None:
        self.temporary_directory.cleanup()


def create_filtered_model_overlay(source_model: Path | str) -> RuntimeModelOverlay:
    """Copy a car model and remove only its five keyboard-trigger plugins."""
    source = Path(source_model).resolve()
    source_sdf = source / "model.sdf"
    if not source_sdf.is_file():
        raise RuntimeError(f"car model source is missing {source_sdf}")

    temporary = tempfile.TemporaryDirectory(prefix="gz-ws-autonomous-model-")
    root = Path(temporary.name)
    models_dir = root / "models"
    models_dir.mkdir()
    copied_model = models_dir / "car"
    shutil.copytree(source, copied_model)
    copied_sdf = copied_model / "model.sdf"
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(copied_sdf, parser=parser)
    document_root = tree.getroot()
    removed = 0
    for parent in document_root.iter():
        for child in list(parent):
            if child.tag == "plugin" and child.get("name") == TRIGGERED_PUBLISHER_NAME:
                parent.remove(child)
                removed += 1
    if removed != EXPECTED_FILTERED_PLUGIN_COUNT:
        temporary.cleanup()
        raise RuntimeError(
            "runtime-filtered model safety check failed: expected 5 "
            f"{TRIGGERED_PUBLISHER_NAME} plugins, found {removed}; refusing autonomous launch"
        )
    tree.write(copied_sdf, encoding="utf-8", xml_declaration=True)
    return RuntimeModelOverlay(temporary_directory=temporary, models_dir=models_dir)
