"""Live headless acceptance test for the lidar-only stop and run record."""

from __future__ import annotations

import csv
import tempfile
import time
import unittest
from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest


SCENARIO = "training/stop_smoke"
_OUTPUT_DIR = Path(tempfile.mkdtemp(prefix="car-lidar-stop-"))
_OUTPUT = _OUTPUT_DIR / "trial.csv"


def generate_test_description():
    package_share = Path(get_package_share_directory("car_bringup"))
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "obstacle_course.launch.py")
        ),
        launch_arguments={
            "mode": "lidar",
            "scenario": SCENARIO,
            "headless": "true",
            "output": str(_OUTPUT),
        }.items(),
    )
    return launch.LaunchDescription([bringup, ReadyToTest()])


class TestLidarStop(unittest.TestCase):
    def test_training_smoke_stops_before_contact_and_flushes_record(self):
        deadline = time.monotonic() + 120.0
        while time.monotonic() < deadline:
            if _OUTPUT.is_file() and _OUTPUT.stat().st_size > 0:
                break
            time.sleep(0.1)
        self.assertTrue(_OUTPUT.is_file(), "run logger did not persist a trial record")
        with _OUTPUT.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual(row["scenario_id"], SCENARIO)
        self.assertEqual(row["trial_status"], "completed")
        self.assertEqual(row["completion_reason"], "rest_and_flush")
        self.assertEqual(row["detected"], "true")
        self.assertEqual(row["detection_source"], "lidar")
        self.assertEqual(row["stopped_before_contact"], "true")
        self.assertEqual(row["collision_status"], "no_collision")
        self.assertEqual(row["collision_coverage"], "verified")
        self.assertGreaterEqual(float(row["stopping_distance_m"]), 0.0)
