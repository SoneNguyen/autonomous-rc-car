"""Select Gazebo Harmonic and report the versions in the sourced ROS environment."""

import json
import os
import platform
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def harmonic_version() -> str:
    if os.environ.get("ROS_DISTRO") != "jazzy":
        raise RuntimeError("Source /opt/ros/jazzy/setup.bash before running the obstacle course.")
    result = subprocess.run(
        ["gz", "sim", "--versions"], check=True, capture_output=True, text=True, timeout=10,
    )
    versions = re.findall(r"(?<![\d.])8\.\d+\.\d+(?![\d.])", result.stdout)
    if not versions:
        raise RuntimeError("Gazebo Harmonic (Sim 8) is required by this ROS 2 Jazzy workspace.")
    return max(versions, key=lambda version: tuple(map(int, version.split("."))))


def runtime_identity() -> dict[str, str]:
    gazebo = harmonic_version()
    bridge = Path(get_package_share_directory("ros_gz_bridge")) / "package.xml"
    return {
        "ubuntu": (platform.freedesktop_os_release().get("VERSION_ID") or "unknown"),
        "ros_distro": "jazzy",
        "gazebo_version": gazebo,
        "ros_gz_bridge_version": ET.parse(bridge).getroot().findtext("version") or "unknown",
    }


if __name__ == "__main__":
    print(json.dumps(runtime_identity()))
