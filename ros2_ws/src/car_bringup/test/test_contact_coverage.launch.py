"""Verify the selected frozen evaluation obstacle is visible to the chassis sensor."""

from __future__ import annotations

import threading
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
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock

from car_bringup.scenario import load_scenario
from car_bringup.set_model_pose import make_pose_request, set_model_pose


SCENARIO_NAME = "evaluation/eval_01"
EXPECTED_COLLISIONS = ("gate_right_collision", "gate_left_collision")
CMD_VEL = "/model/gazebo_truck/cmd_vel"
CONTACT = "/model/gazebo_truck/sensors/bumper_contact"
_SCENARIO = None


def generate_test_description():
    global _SCENARIO
    package_share = Path(get_package_share_directory("car_bringup"))
    _SCENARIO = load_scenario(
        package_share / "config" / "scenarios" / f"{SCENARIO_NAME}.yaml"
    )
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "obstacle_course.launch.py")
        ),
        launch_arguments={
            "mode": "sensors",
            "scenario": SCENARIO_NAME,
            "headless": "true",
        }.items(),
    )
    return launch.LaunchDescription([bringup, ReadyToTest()])


class TestSelectedObstacleContactCoverage(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node(
            "p3_contact_coverage_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        cls.clock_ns = 0
        cls.obstacle_pairs = {}
        cls.expected_collision = EXPECTED_COLLISIONS[0]
        cls.observed_pairs = []
        cls.node.create_subscription(Clock, "/clock", cls._on_clock, qos_profile_sensor_data)
        cls.node.create_subscription(
            Contacts, CONTACT, cls._on_contacts, qos_profile_sensor_data
        )
        cls.command_publisher = cls.node.create_publisher(Twist, CMD_VEL, 10)
        cls.executor = MultiThreadedExecutor(num_threads=3)
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def _on_clock(cls, message):
        cls.clock_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec

    @classmethod
    def _on_contacts(cls, message):
        for contact in message.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            cls.observed_pairs.append(names)
            if (
                any(cls.expected_collision in name for name in names)
                and not any("ground" in name.lower() for name in names)
            ):
                cls.obstacle_pairs[cls.expected_collision] = names

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

    def test_gate_contacts_are_reported_by_existing_chassis_sensor(self):
        package_share = Path(get_package_share_directory("car_bringup"))
        gate_scenarios = (
            (SCENARIO_NAME, EXPECTED_COLLISIONS[0]),
            ("training/stop_smoke", EXPECTED_COLLISIONS[1]),
        )
        deadline = time.monotonic() + 30.0
        while self.clock_ns == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertGreater(self.clock_ns, 0, "/clock did not start")

        for scenario_name, collision_name in gate_scenarios:
            scenario = load_scenario(
                package_share / "config" / "scenarios" / f"{scenario_name}.yaml"
            )
            self.__class__.expected_collision = collision_name
            pose = scenario["start_pose"]
            request = make_pose_request(
                x=pose["x_m"], y=pose["y_m"], z=pose["z_m"],
                roll=pose["roll_rad"], pitch=pose["pitch_rad"], yaw=pose["yaw_rad"],
            )
            self.assertTrue(set_model_pose(request, world_name="sensor_obstacle_course"))

            motion = Twist()
            motion.linear.x = float(scenario["commanded_speed_mps"])
            next_command = 0.0
            deadline = time.monotonic() + 45.0
            while time.monotonic() < deadline and collision_name not in self.__class__.obstacle_pairs:
                now = time.monotonic()
                if now >= next_command:
                    self.command_publisher.publish(motion)
                    next_command = now + 0.1
                time.sleep(0.05)
            zero = Twist()
            for _ in range(10):
                self.command_publisher.publish(zero)
                time.sleep(0.05)
            self.assertIn(
                collision_name,
                self.__class__.obstacle_pairs,
                "existing chassis-only bumper missed a selected gate obstacle; "
                f"observed pairs={self.__class__.observed_pairs[-10:]}",
            )
