"""A live malformed PointCloud2 must latch the autonomous controller to stop."""

from __future__ import annotations

import threading
import tempfile
import time
import unittest
from pathlib import Path

import launch
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2


SCENARIO = "training/stop_smoke"
COMMAND_TOPIC = "/model/gazebo_truck/cmd_vel"
LIDAR_TOPIC = "/model/gazebo_truck/sensors/front_lidar/scan/points"
_OUTPUT_DIR = Path(tempfile.mkdtemp(prefix="car-lidar-invalid-cloud-"))
_OUTPUT = _OUTPUT_DIR / "trial.csv"


def generate_test_description():
    package_share = get_package_share_directory("car_bringup")
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{package_share}/launch/obstacle_course.launch.py"
        ),
        launch_arguments={
            "mode": "lidar",
            "scenario": SCENARIO,
            "headless": "true",
            "output": str(_OUTPUT),
        }.items(),
    )
    return launch.LaunchDescription([bringup, ReadyToTest()])


class TestInvalidLidarFailsSafe(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node(
            "p3_invalid_lidar_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        cls.clock_ns = 0
        cls.commands = []
        cls.lock = threading.Lock()
        cls.node.create_subscription(Clock, "/clock", cls._on_clock, qos_profile_sensor_data)
        cls.node.create_subscription(Twist, COMMAND_TOPIC, cls._on_command, 10)
        cls.cloud_publisher = cls.node.create_publisher(
            PointCloud2, LIDAR_TOPIC, qos_profile_sensor_data
        )
        cls.executor = MultiThreadedExecutor(num_threads=2)
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def _on_clock(cls, message):
        cls.clock_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec

    @classmethod
    def _on_command(cls, message):
        with cls.lock:
            cls.commands.append((time.monotonic(), float(message.linear.x)))

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.spin_thread.join(timeout=2.0)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_invalid_lidar_frame_stops_and_remains_latched(self):
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            with self.__class__.lock:
                if any(speed > 0.1 for _, speed in self.__class__.commands):
                    break
            time.sleep(0.05)
        with self.__class__.lock:
            self.assertTrue(
                any(speed > 0.1 for _, speed in self.__class__.commands),
                "controller never entered forward motion before the invalid sample",
            )

        invalid = PointCloud2()
        invalid.header.stamp.sec, invalid.header.stamp.nanosec = divmod(
            self.__class__.clock_ns, 1_000_000_000
        )
        invalid.header.frame_id = "unverified_integration_frame"
        invalid.height = 1
        invalid.width = 1
        invalid.point_step = 12
        invalid.row_step = 12
        invalid.data = bytes(12)
        injected_at = time.monotonic()
        for _ in range(3):
            self.__class__.cloud_publisher.publish(invalid)
            time.sleep(0.03)

        zero_at = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with self.__class__.lock:
                recent = [
                    (stamp, speed)
                    for stamp, speed in self.__class__.commands
                    if stamp >= injected_at
                ]
            zero_commands = [stamp for stamp, speed in recent if abs(speed) <= 1e-6]
            if zero_commands:
                zero_at = zero_commands[0]
                break
            time.sleep(0.02)
        self.assertIsNotNone(zero_at, "invalid lidar did not result in a zero command")

        time.sleep(0.75)
        with self.__class__.lock:
            after_stop = [
                speed for stamp, speed in self.__class__.commands if stamp >= zero_at
            ]
        self.assertTrue(after_stop, "controller emitted no post-stop samples")
        self.assertTrue(
            all(abs(speed) <= 1e-6 for speed in after_stop),
            f"controller resumed motion with invalid lidar: {after_stop}",
        )
