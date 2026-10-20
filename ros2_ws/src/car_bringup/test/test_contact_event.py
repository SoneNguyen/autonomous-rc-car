"""Controlled low-speed contact test for the existing chassis bumper sensor."""

from __future__ import annotations

import time
import threading
import subprocess
import unittest
from pathlib import Path

import launch
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock

from car_bringup.scenario import load_scenario


CMD_VEL = "/model/gazebo_truck/cmd_vel"
CONTACT = "/model/gazebo_truck/sensors/bumper_contact"
CLOCK = "/clock"
SCENARIO_NAME = "training/contact_smoke"
EXPECTED_OBSTACLE_COLLISION = "post_01_collision"


def generate_test_description():
    package_share = Path(get_package_share_directory("car_bringup"))
    launch_path = package_share / "launch" / "obstacle_course.launch.py"
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_path)),
        launch_arguments={
            "mode": "sensors",
            "scenario": SCENARIO_NAME,
            "headless": "true",
        }.items(),
    )
    return launch.LaunchDescription([bringup, ReadyToTest()])


class TestObstacleContact(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node(
            "p1_contact_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        cls.latest_clock_ns = 0
        cls.obstacle_pair = None
        cls.observed_pairs = []
        cls.callback_group = ReentrantCallbackGroup()
        cls.node.create_subscription(
            Clock,
            CLOCK,
            cls._on_clock,
            qos_profile_sensor_data,
            callback_group=cls.callback_group,
        )
        cls.node.create_subscription(
            Contacts,
            CONTACT,
            cls._on_contact,
            qos_profile_sensor_data,
            callback_group=cls.callback_group,
        )
        cls.command_publisher = cls.node.create_publisher(
            Twist, CMD_VEL, 10
        )
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def _on_clock(cls, message):
        cls.latest_clock_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec

    @classmethod
    def _on_contact(cls, message):
        for contact in message.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if names not in cls.observed_pairs:
                cls.observed_pairs.append(names)
            if (
                any(EXPECTED_OBSTACLE_COLLISION in name for name in names)
                and not any("ground" in name.lower() for name in names)
            ):
                cls.obstacle_pair = names

    @classmethod
    def tearDownClass(cls):
        zero = Twist()
        for _ in range(5):
            cls.command_publisher.publish(zero)
            time.sleep(0.05)
        cls.executor.shutdown()
        cls.spin_thread.join(timeout=2.0)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_bumper_reports_allowlisted_obstacle_pair(self):
        scenario_path = (
            Path(get_package_share_directory("car_bringup"))
            / "config"
            / "scenarios"
            / f"{SCENARIO_NAME}.yaml"
        )
        scenario = load_scenario(scenario_path)
        deadline = time.monotonic() + 30.0
        while self.latest_clock_ns == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertGreater(self.latest_clock_ns, 0, "/clock did not start")

        start = scenario["start_pose"]
        request = (
            'name: "gazebo_truck" '
            f'position {{ x: {start["x_m"]} y: {start["y_m"]} z: {start["z_m"]} }} '
            'orientation { w: 1 }'
        )
        set_pose = subprocess.run(
            [
                "gz",
                "service",
                "-s",
                "/world/sensor_obstacle_course/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                request,
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=8.0,
        )
        self.assertIn("data: true", set_pose.stdout)

        deadline = time.monotonic() + 30.0
        next_command = 0.0
        motion = Twist()
        motion.linear.x = float(scenario["commanded_speed_mps"])

        while time.monotonic() < deadline and self.__class__.obstacle_pair is None:
            now = time.monotonic()
            if now >= next_command:
                self.command_publisher.publish(motion)
                next_command = now + 0.1
            time.sleep(0.05)

        zero = Twist()
        for _ in range(5):
            self.command_publisher.publish(zero)
            time.sleep(0.05)

        self.assertIsNotNone(
            self.__class__.obstacle_pair,
            "no allowlisted chassis-to-post contact was reported within 30 seconds; "
            f"observed pairs={self.__class__.observed_pairs}",
        )
        self.assertTrue(
            any(EXPECTED_OBSTACLE_COLLISION in name for name in self.__class__.obstacle_pair)
        )
        self.assertGreater(self.latest_clock_ns, 0, "/clock did not advance during contact test")
