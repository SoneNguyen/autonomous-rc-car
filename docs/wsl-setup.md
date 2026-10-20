# WSL2 ROS 2 and Gazebo setup

ROS 2 work for this repository runs inside WSL2 Ubuntu 26.04 (Resolute), from
the Linux filesystem. The Windows-native Pixi/Gazebo path remains separate and
unchanged. This setup uses apt only: Ubuntu packages plus the official ROS apt
repository. Do not use `/mnt/c`, Windows Python, Pixi, Conda, or RoboStack for
ROS work.

## Fresh WSL install

From Windows PowerShell, install WSL2 and the required Ubuntu distro if needed:

```powershell
wsl --list --online
wsl --install Ubuntu-26.04
wsl --set-version Ubuntu-26.04 2
```

Restart if Windows requests it, launch Ubuntu-26.04, and create a Linux user.
Inside Ubuntu, clone the feature branch into the WSL filesystem:

```bash
mkdir -p ~/src
cd ~/src
git clone --branch 001-ros2-ai-obstacle-stop https://github.com/SoneNguyen/autonomous-rc-car.git gz-ws
cd ~/src/gz-ws
```

Install the official ROS apt source by following the [ROS 2 Lyrical Ubuntu
deb guide](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html).
Confirm apt candidates before installing; if the pinned ROS/Gazebo versions
are unavailable, stop rather than switching Ubuntu, ROS, or Gazebo releases.

```bash
sudo apt update
apt-cache policy ros-lyrical-desktop ros-lyrical-ros-gz \
  ros-lyrical-gz-sim-vendor python3-torch
```

Install runtime and test dependencies through apt:

```bash
sudo apt install -y \
  ros-lyrical-desktop \
  ros-lyrical-ros-gz \
  ros-lyrical-launch-testing \
  ros-lyrical-launch-testing-ros \
  ros-lyrical-launch-testing-ament-cmake \
  ros-lyrical-ament-cmake-pytest \
  ros-lyrical-vision-msgs \
  ros-lyrical-sensor-msgs-py \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-pytest \
  python3-torch
```

Verify the runtime before building or launching:

```bash
source /opt/ros/lyrical/setup.bash
printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
ros2 pkg prefix ros_gz_bridge
ros2 pkg xml ros_gz_bridge | grep -m1 '<version>'
gz sim --versions
python3 -c 'import torch; print(torch.__version__)'
```

Expected: `ROS_DISTRO=lyrical`, bridge prefix `/opt/ros/lyrical`, bridge
manifest version `3.0.10`, Gazebo Sim version `10.5.0`, and an apt-managed
PyTorch 2.9.1 build. Record/compare full Debian package revisions with:

```bash
dpkg-query -W \
  ros2-apt-source ros-lyrical-desktop ros-lyrical-ros-gz \
  ros-lyrical-ros-gz-bridge ros-lyrical-ros-gz-sim \
  ros-lyrical-gz-sim-vendor ros-lyrical-gz-tools-vendor \
  ros-lyrical-gz-msgs-vendor ros-lyrical-gz-sensors-vendor \
  ros-lyrical-gz-rendering-vendor ros-lyrical-gz-physics-vendor \
  ros-lyrical-gz-common-vendor ros-lyrical-launch-testing \
  ros-lyrical-launch-testing-ros ros-lyrical-launch-testing-ament-cmake \
  ros-lyrical-ament-cmake-pytest ros-lyrical-vision-msgs \
  ros-lyrical-sensor-msgs-py ros-dev-tools \
  python3-colcon-common-extensions python3-pytest python3-torch
```

### Verified package baseline (2026-10-20)

This baseline was installed and checked in WSL2 Ubuntu 26.04.1 LTS (codename
`resolute`). Package versions below are Debian revisions from apt; the bridge
manifest version is also checked independently. A version mismatch is a setup
failure and must be resolved in `research.md` before implementation continues.

| Package / check | Verified version |
|---|---|
| Ubuntu | 26.04.1 LTS, Resolute, WSL2 |
| `ros2-apt-source` | `1.3.0~resolute` |
| `ros-lyrical-desktop` | `0.13.0-3resolute.20260919.194812` |
| `ros-lyrical-ros-gz` | `3.0.10-1resolute.20260930.020431` |
| `ros-lyrical-ros-gz-bridge` | `3.0.10-1resolute.20260915.142616` (manifest `3.0.10`) |
| `ros-lyrical-ros-gz-sim` | `3.0.10-1resolute.20260930.014144` |
| `ros-lyrical-gz-sim-vendor` | `0.4.6-1resolute.20260930.002648` (provides Gazebo Sim 10.5.0) |
| `ros-lyrical-gz-tools-vendor` | `0.2.2-1resolute.20260820.132934` |
| `ros-lyrical-gz-msgs-vendor` | `0.3.4-2resolute.20260827.150529` |
| `ros-lyrical-gz-sensors-vendor` | `0.3.5-1resolute.20260914.175731` |
| `ros-lyrical-gz-rendering-vendor` | `0.4.4-1resolute.20260827.145029` |
| `ros-lyrical-gz-physics-vendor` | `0.4.8-1resolute.20260929.222801` |
| `ros-lyrical-gz-common-vendor` | `0.3.8-1resolute.20260827.143639` |
| `ros-lyrical-launch-testing` | `3.9.8-1resolute.20260728.175423` |
| `ros-lyrical-launch-testing-ros` | `0.29.9-1resolute.20260915.073741` |
| `ros-lyrical-launch-testing-ament-cmake` | `3.9.8-1resolute.20260728.205136` |
| `ros-lyrical-ament-cmake-pytest` | `2.8.8-1resolute.20260728.175653` |
| `ros-lyrical-vision-msgs` | `4.2.0-3resolute.20260915.050641` |
| `ros-lyrical-sensor-msgs-py` | `5.9.3-1resolute.20260915.052909` |
| `ros-dev-tools` | `1.0.3` |
| `python3-colcon-common-extensions` | `0.3.0-100` |
| `python3-pytest` | `9.0.2-4` |
| `python3-torch` | `2.9.1+dfsg-1~exp1ubuntu2` |
| `gz sim --versions` | `10.5.0` |

## Build the repository workspace

In each WSL terminal:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/src/gz-ws/ros2_ws
colcon build --symlink-install --base-paths src
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The independent P1 sensor smoke launch is scenario-backed. Use WSLg GUI only
for inspection; all autonomous modes must be server-only/headless so the GUI
keyboard publisher cannot compete for command authority.

```bash
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=sensors scenario:=training/sensor_smoke headless:=false
```

For rendered sensor testing without a GUI, use the same checked-in scenario:

```bash
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=sensors scenario:=training/sensor_smoke headless:=true
```

The launch wrapper sets `GZ_SIM_RESOURCE_PATH`, loads the scenario seed and
vehicle/environment parameters, and forces Gazebo 10.5.0. Its server-only
mapping is `-s -r --headless-rendering --force-version 10.5.0`; raw Gazebo
flags are an implementation detail and feature runs should not bypass the
scenario-backed wrapper.

If WSLg/GPU EGL cannot initialize, retry the same scenario with software
rendering:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch car_bringup obstacle_course.launch.py \
  mode:=sensors scenario:=training/sensor_smoke headless:=true
```

Accept software rendering only after camera and multi-layer point-cloud data
and rates have been verified. It may run substantially slower. See the
[feature quickstart](../specs/001-ros2-ai-obstacle-stop/quickstart.md) for the
P1 topic checks and expected sensor rates.
