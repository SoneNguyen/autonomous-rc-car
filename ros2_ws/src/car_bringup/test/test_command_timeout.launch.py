"""Live P2 command, motion, single-writer, and expiry acceptance test."""

from __future__ import annotations

import math
import re
import subprocess
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
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2


CMD = "/model/gazebo_truck/cmd_vel"
REQUEST = "/autonomy/cmd_vel_request"
ODOM = "/model/gazebo_truck/odometry"
CLOCK = "/clock"


def generate_test_description():
    launch_path = Path(get_package_share_directory("car_bringup")) / "launch" / "obstacle_course.launch.py"
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_path)),
        launch_arguments={
            "mode": "command_test",
            "scenario": "training/command_smoke",
            "headless": "true",
        }.items(),
    )
    return launch.LaunchDescription([bringup, ReadyToTest()])


def _yaw(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CommandProbe(Node):
    def __init__(self):
        super().__init__("p2_command_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.sim_ns = 0
        self.last_odom = None
        self.odometry = []
        self.commands = []
        self.ready_sensors = set()
        self.last_request_sim_ns = 0
        self.clock_sub = self.create_subscription(Clock, CLOCK, self._on_sim_clock, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, ODOM, self._odom, qos_profile_sensor_data)
        self.command_sub = self.create_subscription(Twist, CMD, self._command, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            "/model/gazebo_truck/sensors/front_lidar/scan/points",
            lambda _m: self.ready_sensors.add("lidar"),
            qos_profile_sensor_data,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            "/model/gazebo_truck/sensors/imu",
            lambda _m: self.ready_sensors.add("imu"),
            qos_profile_sensor_data,
        )
        self.odom_ready_sub = self.create_subscription(
            Odometry,
            ODOM,
            lambda _m: self.ready_sensors.add("odometry"),
            qos_profile_sensor_data,
        )
        self.request_pub = self.create_publisher(Twist, REQUEST, 10)
        self.request_timer = self.create_timer(0.1, self._send_request)

    def _on_sim_clock(self, message):
        self.sim_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec

    def _odom(self, message):
        self.last_odom = message
        self.odometry.append((self.sim_ns, message))

    def _command(self, message):
        self.commands.append((self.sim_ns, message.linear.x, message.angular.z))

    def _send_request(self):
        message = Twist()
        message.linear.x = 0.75
        message.angular.z = 0.1
        self.request_pub.publish(message)
        self.last_request_sim_ns = self.sim_ns


class TestP2CommandTimeout(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = CommandProbe()
        cls.executor = MultiThreadedExecutor(num_threads=6)
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.spin_thread.join(timeout=2.0)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_motion_expires_and_single_writer_is_enforced(self):
        deadline = time.monotonic() + 120
        while time.monotonic() < deadline and (self.node.sim_ns == 0 or len(self.node.ready_sensors) < 3):
            time.sleep(0.05)
        self.assertGreater(self.node.sim_ns, 0, "/clock did not start")
        self.assertEqual(self.node.ready_sensors, {"lidar", "imu", "odometry"}, "required sensors did not arrive")

        publisher_deadline = time.monotonic() + 60
        publishers = self.node.get_publishers_info_by_topic(CMD)
        while (
            (len(publishers) != 1 or publishers[0].node_name == "_NODE_NAME_UNKNOWN_")
            and time.monotonic() < publisher_deadline
        ):
            time.sleep(0.05)
            publishers = self.node.get_publishers_info_by_topic(CMD)
        self.assertEqual(len(publishers), 1, f"expected one ROS publisher, got {publishers}")
        self.assertEqual(publishers[0].node_name, "safe_drive_controller")

        gz_info = subprocess.run(
            ["gz", "topic", "-i", "-t", CMD],
            check=True,
            text=True,
            capture_output=True,
            timeout=15,
        ).stdout
        publisher_section = re.search(
            r"Publishers \[Address, Message Type\]:\s*(.*?)(?:\nSubscribers|\Z)",
            gz_info,
            re.S,
        )
        self.assertIsNotNone(publisher_section, gz_info)
        gz_twist_publishers = re.findall(r"gz\.msgs\.Twist", publisher_section.group(1))
        self.assertEqual(
            len(gz_twist_publishers),
            1,
            f"expected only ros_gz_bridge as Gazebo writer:\n{gz_info}",
        )

        keyboard_info = subprocess.run(
            ["gz", "topic", "-i", "-t", "/keyboard/keypress"],
            check=True,
            text=True,
            capture_output=True,
            timeout=15,
        ).stdout
        self.assertRegex(keyboard_info, r"No publishers on topic", keyboard_info)

        start_ns = self.node.sim_ns
        start_pose = self.node.last_odom.pose.pose
        motion_deadline = time.monotonic() + 120
        while self.node.sim_ns - start_ns < 3_000_000_000 and time.monotonic() < motion_deadline:
            time.sleep(0.05)
        self.assertGreaterEqual(
            self.node.sim_ns - start_ns,
            3_000_000_000,
            "simulation did not advance through motion interval",
        )
        self.node.request_timer.cancel()
        last_request_ns = self.node.last_request_sim_ns

        stop_deadline = time.monotonic() + 30
        zero_ns = None
        while time.monotonic() < stop_deadline:
            zeros = [
                stamp
                for stamp, x, z in self.node.commands
                if stamp >= last_request_ns and abs(x) < 1e-6 and abs(z) < 1e-6
            ]
            if zeros:
                zero_ns = zeros[0]
                break
            time.sleep(0.02)
        self.assertIsNotNone(zero_ns, "controller did not publish a zero command")
        self.assertLessEqual(zero_ns - last_request_ns, 500_000_000, "zero exceeded 0.5 simulation-second lease")

        rest_deadline = zero_ns + 2_000_000_000
        rest_wall_deadline = time.monotonic() + 120
        while self.node.sim_ns < rest_deadline and time.monotonic() < rest_wall_deadline:
            time.sleep(0.05)
        self.assertGreaterEqual(self.node.sim_ns, rest_deadline, "simulation did not advance through rest interval")
        recent = [message for stamp, message in self.node.odometry if stamp >= zero_ns and stamp <= rest_deadline]
        self.assertTrue(recent, "no odometry arrived after stop")
        final = recent[-1]
        speed = math.hypot(final.twist.twist.linear.x, final.twist.twist.linear.y)
        self.assertLess(speed, 0.05, f"vehicle did not reach rest: {speed:.3f} m/s")
        self.assertLess(abs(final.twist.twist.angular.z), 0.02, "vehicle angular rate did not settle")

        end_pose = final.pose.pose
        forward_delta = end_pose.position.x - start_pose.position.x
        self.assertGreater(forward_delta, 0.3, f"vehicle did not move forward: dx={forward_delta:.3f}m")
        start_yaw = math.atan2(
            2.0 * (start_pose.orientation.w * start_pose.orientation.z + start_pose.orientation.x * start_pose.orientation.y),
            1.0 - 2.0 * (start_pose.orientation.y**2 + start_pose.orientation.z**2),
        )
        self.assertGreater(_yaw(final) - start_yaw, 0.02, "vehicle did not turn")
