"""Live launch test for sensor startup, simulation stamps, and rates."""

from __future__ import annotations

import time
import threading
import unittest
from pathlib import Path

import launch
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2


IMAGE = "/model/gazebo_truck/sensors/front_camera/image"
SCAN = "/model/gazebo_truck/sensors/front_lidar/scan"
POINTS = "/model/gazebo_truck/sensors/front_lidar/scan/points"
IMU = "/model/gazebo_truck/sensors/imu"
ODOMETRY = "/model/gazebo_truck/odometry"
CONTACT = "/model/gazebo_truck/sensors/bumper_contact"
CLOCK = "/clock"
SIM_MINUTE_NS = 60_000_000_000
EXPECTED_SAMPLES = {
    IMAGE: 30 * 60,
    POINTS: 15 * 60,
    IMU: 100 * 60,
    ODOMETRY: 50 * 60,
}
# Gazebo's SDF source target remains 30 Hz. WSL headless delivery measured
# about 23 Hz; keep a 20 Hz ROS floor (2x the planned 10 Hz detector) and use
# the declared-rate 90% floor for the other periodic streams.
MINIMUM_SAMPLES = {
    IMAGE: 20 * 60,
    **{
        topic: int(expected * 0.9)
        for topic, expected in EXPECTED_SAMPLES.items()
        if topic != IMAGE
    },
}
EXPECTED_TYPES = {
    IMAGE: "sensor_msgs/msg/Image",
    SCAN: "sensor_msgs/msg/LaserScan",
    POINTS: "sensor_msgs/msg/PointCloud2",
    IMU: "sensor_msgs/msg/Imu",
    ODOMETRY: "nav_msgs/msg/Odometry",
    CONTACT: "ros_gz_interfaces/msg/Contacts",
    CLOCK: "rosgraph_msgs/msg/Clock",
}


def generate_test_description():
    package_share = Path(get_package_share_directory("car_bringup"))
    launch_path = package_share / "launch" / "obstacle_course.launch.py"
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_path)),
        launch_arguments={
            "mode": "sensors",
            "scenario": "training/sensor_smoke",
            "headless": "true",
        }.items(),
    )
    return launch.LaunchDescription([bringup, ReadyToTest()])


class SensorProbe(Node):
    def __init__(self):
        super().__init__(
            "p1_sensor_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.current_clock_ns = 0
        self.first_clock_ns = None
        self.last_clock_ns = 0
        self.interval_start_ns = None
        self.interval_end_ns = None
        self.callback_group = ReentrantCallbackGroup()
        self.samples = {topic: 0 for topic in EXPECTED_SAMPLES}
        self.first_stamps = {}
        self.last_stamps = {}
        self.scan_length = 0
        self.cloud_points = 0
        self.cloud_frame = ""
        self.image_shape = None
        self.topic_subscriptions = []
        self.topic_subscriptions.append(
            self.create_subscription(
                Clock,
                CLOCK,
                self._on_clock,
                qos_profile_sensor_data,
                callback_group=self.callback_group,
            )
        )
        callbacks = {
            IMAGE: (Image, self._on_image),
            POINTS: (PointCloud2, self._on_cloud),
            IMU: (Imu, self._on_periodic),
            ODOMETRY: (Odometry, self._on_periodic),
            SCAN: (LaserScan, self._on_scan),
            CONTACT: (Contacts, lambda _msg: None),
        }
        for topic, (message_type, callback) in callbacks.items():
            self.topic_subscriptions.append(
                self.create_subscription(
                    message_type,
                    topic,
                    callback,
                    qos_profile_sensor_data,
                    callback_group=self.callback_group,
                )
            )

    @staticmethod
    def _stamp_ns(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def _on_clock(self, message):
        stamp_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        if self.first_clock_ns is None:
            self.first_clock_ns = stamp_ns
        self.current_clock_ns = stamp_ns
        self.last_clock_ns = stamp_ns

    def _on_periodic(self, message):
        self._count_message(message, self._topic_for(message))

    def _on_image(self, message):
        if self.image_shape is None:
            self.image_shape = (message.width, message.height, message.encoding)
        self._count_message(message, IMAGE)

    def _topic_for(self, message):
        # Message types in this contract are unique, so this avoids retaining
        # high-volume sensor messages while keeping one callback implementation.
        if isinstance(message, Image):
            return IMAGE
        if isinstance(message, Imu):
            return IMU
        return ODOMETRY

    def _on_cloud(self, message):
        self.cloud_points = max(self.cloud_points, message.width * message.height)
        self.cloud_frame = message.header.frame_id
        self._count_message(message, POINTS)

    def _on_scan(self, message):
        if not self.scan_length:
            self.scan_length = len(message.ranges)

    def _count_message(self, message, topic):
        stamp_ns = self._stamp_ns(message)
        if self.interval_start_ns is None:
            return
        if self.interval_start_ns < stamp_ns <= self.interval_end_ns:
            self.samples[topic] += 1
            self.first_stamps.setdefault(topic, stamp_ns)
            self.last_stamps[topic] = stamp_ns


class TestP1SensorBridge(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = SensorProbe()
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

    def test_topics_clock_and_periodic_rates(self):
        deadline = time.monotonic() + 300.0
        required = set(EXPECTED_TYPES)
        topic_types = {}
        while time.monotonic() < deadline:
            topic_types = dict(self.node.get_topic_names_and_types())
            if all(topic in topic_types for topic in required) and self.node.first_clock_ns is not None:
                break
            time.sleep(0.1)
        missing = required - set(topic_types)
        self.assertFalse(missing, f"topics did not appear within 5 minutes: {sorted(missing)}")
        for topic, expected_type in EXPECTED_TYPES.items():
            self.assertIn(expected_type, topic_types[topic], topic)

        self.assertIsNotNone(self.node.first_clock_ns, "/clock did not publish")
        self.node.interval_start_ns = self.node.current_clock_ns
        self.node.interval_end_ns = self.node.interval_start_ns + SIM_MINUTE_NS
        sim_deadline = time.monotonic() + 300.0
        while (
            self.node.current_clock_ns < self.node.interval_end_ns + 100_000_000
            and time.monotonic() < sim_deadline
        ):
            time.sleep(0.05)

        self.assertGreaterEqual(
            self.node.current_clock_ns - self.node.interval_start_ns,
            SIM_MINUTE_NS,
            "simulation clock did not advance by one simulated minute within 5 minutes",
        )
        self.assertEqual(self.node.scan_length, 720, "LaserScan should expose only the middle row")
        self.assertEqual(self.node.image_shape, (640, 480, "rgb8"))
        self.assertGreaterEqual(
            self.node.cloud_points,
            720 * 16 * 0.9,
            "PointCloud2 does not contain the expected 16-layer lidar data",
        )
        self.assertTrue(self.node.cloud_frame, "PointCloud2 frame_id must be populated")

        failures = []
        for topic in EXPECTED_SAMPLES:
            minimum = MINIMUM_SAMPLES[topic]
            received = self.node.samples[topic]
            if received < minimum:
                failures.append(
                    f"{topic}: got {received} samples, expected at least {minimum}"
                )
            if topic not in self.node.last_stamps or topic not in self.node.first_stamps:
                failures.append(f"{topic}: no stamped samples arrived in the test interval")
                continue
            if self.node.last_stamps[topic] > self.node.last_clock_ns + 1_000_000_000:
                failures.append(f"{topic}: header stamp is ahead of simulation clock")
            if self.node.last_stamps[topic] <= self.node.first_stamps[topic]:
                failures.append(f"{topic}: stamps did not advance with simulation time")
        self.assertFalse(
            failures,
            "P1 periodic-stream acceptance failures:\n- " + "\n- ".join(failures),
        )

        for topic in (IMAGE, POINTS, IMU, ODOMETRY, CONTACT, CLOCK):
            publishers = self.node.get_publishers_info_by_topic(topic)
            self.assertEqual(len(publishers), 1, f"expected one bridge publisher on {topic}")
            self.assertEqual(publishers[0].node_name, "gazebo_ros_bridge")
