"""The autonomous runtime overlay filters dormant keyboard trigger plugins."""

from pathlib import Path
import shutil

import pytest

from car_bringup.runtime_overlay import create_filtered_model_overlay


MODEL = Path(__file__).resolve().parents[4] / "simulation" / "models" / "car"


def _count_trigger_publishers(path: Path) -> int:
    import xml.etree.ElementTree as ET

    root = ET.parse(path / "model.sdf").getroot()
    return sum(
        plugin.get("name") == "gz::sim::systems::TriggeredPublisher"
        for plugin in root.iter("plugin")
    )


def test_overlay_removes_exact_five_trigger_plugins_without_touching_source():
    if not MODEL.is_dir():
        pytest.skip("repository model source is unavailable in installed test layout")
    source_text = (MODEL / "model.sdf").read_text(encoding="utf-8")
    overlay = create_filtered_model_overlay(MODEL)
    try:
        copied_model = overlay.models_dir / "car"
        assert _count_trigger_publishers(copied_model) == 0
        assert (copied_model / "meshes").is_dir()
        assert (MODEL / "model.sdf").read_text(encoding="utf-8") == source_text
    finally:
        overlay.cleanup()


def test_overlay_fails_closed_if_trigger_plugin_count_changes(tmp_path):
    source = tmp_path / "car"
    source.mkdir()
    (source / "model.sdf").write_text(
        '<sdf><model><plugin name="gz::sim::systems::TriggeredPublisher" '
        'filename="gz-sim-triggered-publisher-system" /></model></sdf>',
        encoding="utf-8",
    )
    with pytest.raises(RuntimeError, match="expected 5"):
        create_filtered_model_overlay(source)
