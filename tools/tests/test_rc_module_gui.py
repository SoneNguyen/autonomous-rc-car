from __future__ import annotations

from collections import Counter
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from rc_module_gui import CONFIG_ROOT, _scenarios, _scene_camera_pose  # noqa: E402


def test_lidar_view_frames_car_and_target_from_the_side() -> None:
    scenario = _scenarios(CONFIG_ROOT / "lidar.yaml")["lidar/sector-front"]
    camera_x, camera_y, _z, _roll, _pitch, yaw = map(
        float, _scene_camera_pose(scenario).split(),
    )
    start = scenario["start_pose"]
    target = next(
        fixture["pose"] for fixture in scenario["obstacle_placements"]
        if fixture["entity_id"] == scenario["setup"]["target_entity"]
    )

    # From the lateral view, neither the truck nor its nearby target is hidden
    # at the far end of the road, and both fit inside the displayed world.
    assert -14.0 <= camera_x <= 14.0
    assert -14.0 <= start["x_m"] <= 14.0
    assert -14.0 <= target["x_m"] <= 14.0
    assert camera_y < min(start["y_m"], target["y_m"])
    assert 1.4 < yaw < 1.8


def test_matrix_gui_config_has_one_of_each_scene_plugin() -> None:
    config = (ROOT / "simulation/gui/rc_module_matrix.config.in").read_text(encoding="utf-8")
    plugin_names = re.findall(r'<plugin\s+filename="([^"]+)"\s+name="[^"]+"', config)
    counts = Counter(plugin_names)

    assert "__CAMERA_POSE__" in config
    assert all(count == 1 for count in counts.values())
    assert {
        "MinimalScene", "GzSceneManager", "VisualizationCapabilities",
        "WorldControl", "WorldStats", "EntityTree",
    } <= counts.keys()
