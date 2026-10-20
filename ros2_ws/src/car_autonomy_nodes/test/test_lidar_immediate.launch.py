"""An obstacle already within the threshold must not receive a drive pulse."""

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


SCENARIO = "training/inside_stop_distance"
_OUTPUT_DIR = Path(tempfile.mkdtemp(prefix="car-lidar-immediate-"))
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


class TestImmediateLidarStop(unittest.TestCase):
    def test_inside_threshold_is_detected_and_stopped_without_contact(self):
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline:
            if _OUTPUT.is_file() and _OUTPUT.stat().st_size > 0:
                break
            time.sleep(0.1)
        self.assertTrue(_OUTPUT.is_file(), "run logger did not persist the immediate-stop trial")
        with _OUTPUT.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual(row["scenario_id"], SCENARIO)
        self.assertEqual(row["trial_status"], "completed")
        self.assertEqual(row["detected"], "true")
        self.assertTrue(row["stop_request_stamp_ns"])
        self.assertEqual(row["collision_status"], "no_collision")
        self.assertEqual(row["stopped_before_contact"], "true")
        self.assertLessEqual(float(row["stopping_distance_m"]), 0.1)
