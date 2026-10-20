"""Check the committed bridge YAML against the approved interface contract."""

from pathlib import Path

import yaml


BRIDGE_CONFIG = Path(__file__).resolve().parents[1] / "config" / "ros_gz_bridge.yaml"

EXPECTED = {
    "/model/gazebo_truck/cmd_vel": (
        "geometry_msgs/msg/Twist",
        "gz.msgs.Twist",
        "ROS_TO_GZ",
        None,
    ),
    "/model/gazebo_truck/odometry": (
        "nav_msgs/msg/Odometry",
        "gz.msgs.Odometry",
        "GZ_TO_ROS",
        "SENSOR_DATA",
    ),
    "/model/gazebo_truck/sensors/front_camera/image": (
        "sensor_msgs/msg/Image",
        "gz.msgs.Image",
        "GZ_TO_ROS",
        "SENSOR_DATA",
    ),
    "/model/gazebo_truck/sensors/front_lidar/scan": (
        "sensor_msgs/msg/LaserScan",
        "gz.msgs.LaserScan",
        "GZ_TO_ROS",
        "SENSOR_DATA",
    ),
    "/model/gazebo_truck/sensors/front_lidar/scan/points": (
        "sensor_msgs/msg/PointCloud2",
        "gz.msgs.PointCloudPacked",
        "GZ_TO_ROS",
        "SENSOR_DATA",
    ),
    "/model/gazebo_truck/sensors/imu": (
        "sensor_msgs/msg/Imu",
        "gz.msgs.IMU",
        "GZ_TO_ROS",
        "SENSOR_DATA",
    ),
    "/model/gazebo_truck/sensors/bumper_contact": (
        "ros_gz_interfaces/msg/Contacts",
        "gz.msgs.Contacts",
        "GZ_TO_ROS",
        "SENSOR_DATA",
    ),
    "/clock": (
        "rosgraph_msgs/msg/Clock",
        "gz.msgs.Clock",
        "GZ_TO_ROS",
        "CLOCK",
    ),
}


def test_bridge_yaml_matches_topic_names_types_directions_and_qos():
    entries = yaml.safe_load(BRIDGE_CONFIG.read_text(encoding="utf-8"))
    actual = {}
    for entry in entries:
        ros_topic = entry["ros_topic_name"]
        assert entry["gz_topic_name"] == ros_topic
        actual[ros_topic] = (
            entry["ros_type_name"],
            entry["gz_type_name"],
            entry["direction"],
            entry.get("qos_profile"),
        )
    assert actual == EXPECTED


def test_command_is_the_only_ros_to_gazebo_bridge():
    entries = yaml.safe_load(BRIDGE_CONFIG.read_text(encoding="utf-8"))
    ros_to_gz = [entry for entry in entries if entry["direction"] == "ROS_TO_GZ"]
    assert [entry["ros_topic_name"] for entry in ros_to_gz] == [
        "/model/gazebo_truck/cmd_vel"
    ]


def test_laserscan_is_inspection_only_and_cloud_preserves_full_3d_topic():
    entries = {
        entry["ros_topic_name"]: entry
        for entry in yaml.safe_load(BRIDGE_CONFIG.read_text(encoding="utf-8"))
    }
    assert entries["/model/gazebo_truck/sensors/front_lidar/scan"]["ros_type_name"] == (
        "sensor_msgs/msg/LaserScan"
    )
    assert entries[
        "/model/gazebo_truck/sensors/front_lidar/scan/points"
    ]["ros_type_name"] == "sensor_msgs/msg/PointCloud2"
