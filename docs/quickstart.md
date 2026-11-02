# Install and run

Use Ubuntu 26.04 on a native Ubuntu installation or Ubuntu 26.04 under WSL2. The validated stack is ROS 2 Lyrical, Gazebo Sim 10.5.0, and `ros_gz_bridge` 3.0.10. WSL1 and other Ubuntu/ROS/Gazebo combinations are not currently qualified. Headless runs need no desktop; visible runs need Ubuntu's desktop or WSLg.

ROS 2 provides the official [Ubuntu installation steps](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html). Gazebo's compatibility table pairs ROS 2 Lyrical with Gazebo Jetty and ROS 2 Jazzy with Gazebo Harmonic ([official ROS/Gazebo guidance](https://gazebosim.org/docs/jetty/ros_installation/)). This project is measured and tested against the first pairing.

## 1. Install the runtime

Open a terminal in Ubuntu or WSL2. These commands add the official ROS apt source, install ROS and build tools, then initialize `rosdep` if needed:

```bash
set -e
. /etc/os-release
if [ "$ID" != ubuntu ] || [ "$VERSION_ID" != 26.04 ]; then
  echo "This checkout is validated on Ubuntu 26.04; found $ID $VERSION_ID." >&2
  exit 1
fi

sudo apt update
sudo apt install -y software-properties-common curl ca-certificates git
sudo add-apt-repository -y universe
sudo apt update

ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F 'tag_name' | awk -F'"' '{print $4}')"
UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"
curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update

sudo apt install -y \
  ros-lyrical-desktop ros-lyrical-ros-gz ros-lyrical-ros-gz-bridge \
  ros-lyrical-ros-gz-sim ros-lyrical-ackermann-msgs ros-lyrical-cv-bridge \
  ros-lyrical-tf2-ros ros-dev-tools build-essential \
  python3-colcon-common-extensions python3-rosdep \
  python3-pytest python3-yaml python3-jsonschema python3-opencv

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
```

## 2. Clone and build

Clone the repository into the Linux filesystem. Avoid building from `/mnt/c` under WSL:

```bash
mkdir -p ~/src
git clone https://github.com/SoneNguyen/autonomous-rc-car.git ~/src/gz-ws
cd ~/src/gz-ws
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
colcon --log-base ros2_ws/log build --symlink-install \
  --base-paths ros2_ws/src --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --packages-up-to car_navigation car_decision car_bringup car_autonomy_nodes
source ros2_ws/install/setup.bash
```

For an existing checkout, skip `git clone` and `cd` to that checkout before building. Rebuild after moving the checkout because the symlink install records its location.

Check the installed versions before running:

```bash
set -e
. /opt/ros/lyrical/setup.bash
. /etc/os-release && printf 'Ubuntu %s (%s)\n' "$VERSION_ID" "$UBUNTU_CODENAME"
printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
test "$VERSION_ID" = 26.04
test "$ROS_DISTRO" = lyrical
gz sim --versions | grep -F '10.5.0'
ros2 pkg xml ros_gz_bridge | grep -F '<version>3.0.10</version>'
```

Expected baseline: Ubuntu `26.04`, ROS `lyrical`, Gazebo Sim `10.5.0`, and `ros_gz_bridge` `3.0.10`.

## 3. Fetch and verify the visual assets

Asset downloads need network access once. The simulation reads the local locked assets afterward and does not fetch models at run time.

```bash
cd ~/src/gz-ws
python3 tools/fetch_obstacle_course_assets.py
python3 tools/fetch_obstacle_course_assets.py \
  --lock ros2_ws/src/car_bringup/config/obstacle_course/research_assets.lock.yaml
python3 tools/fetch_obstacle_course_assets.py \
  --lock ros2_ws/src/car_bringup/config/obstacle_course/research_vehicle.lock.yaml

python3 tools/fetch_obstacle_course_assets.py --verify-only
python3 tools/fetch_obstacle_course_assets.py \
  --lock ros2_ws/src/car_bringup/config/obstacle_course/research_assets.lock.yaml --verify-only
python3 tools/fetch_obstacle_course_assets.py \
  --lock ros2_ws/src/car_bringup/config/obstacle_course/research_vehicle.lock.yaml --verify-only
```

The verified pickup, cone, SUV, barrier, and gate meshes are cached under ignored `artifacts/obstacle-course/assets/`.

## 4. Validate and run headless

These checks do not need a display and work on Ubuntu Server, desktop Ubuntu, or WSL2:

```bash
cd ~/src/gz-ws
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
python3 tools/validate_obstacle_course_config.py --all
python3 tools/check_obstacle_course_sizing.py \
  --profile ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml \
  --geometry ros2_ws/src/car_bringup/config/obstacle_course/vehicle_geometry.yaml \
  --scenarios ros2_ws/src/car_bringup/config/obstacle_course/scenarios
python3 tools/validate_real_car_mapping.py \
  --mapping ros2_ws/src/car_bringup/config/obstacle_course/real_car_mapping.yaml \
  --profile ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml \
  --scenarios ros2_ws/src/car_bringup/config/obstacle_course/scenarios \
  --sheets docs/implementation.md
python3 -m pytest tools/tests ros2_ws/src/car_decision/test -q
```

Each validator should report `PASS`. The sizing check derives and enforces the course bound from the measured vehicle profile and configured 25% margin. The mapping check confirms every scenario has a real-car setup sheet. The tests should end with a passing summary.

## 5. Run the visible demo

Use a native Ubuntu desktop terminal or a WSL2 terminal with WSLg enabled. Keep this terminal open while the simulation runs:

```bash
cd ~/src/gz-ws
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=78
export GZ_PARTITION=obstacle_course_demo
export GZ_SIM_RESOURCE_PATH="$PWD/artifacts/obstacle-course/assets/models:$PWD/simulation/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="$PWD/artifacts/obstacle-course/assets/models:$PWD/simulation/models${SDF_PATH:+:$SDF_PATH}"

python3 tools/run_obstacle_course.py \
  --scenario-config ros2_ws/src/car_bringup/config/obstacle_course/development/scenarios/barrier_tuning.yaml \
  --decision-config ros2_ws/src/car_bringup/config/obstacle_course/development/decision.yaml \
  --run-purpose development --quick --no-headless --wall-timeout-s 300
```

Gazebo opens the condensed road. Open <http://localhost:8765/> in a browser. The truck follows the route through the configured objects; click a dashboard component card to inspect its live evidence and select Camera to see the camera preview. Decision history shows the behavior, reason, and component attribution. The dashboard is read-only. Press Ctrl-C in the launch terminal to stop the run.

After changing ROS nodes or launch files, stop an active run, rebuild using section 2, and start a new run. Running nodes keep the code they loaded at startup.

For systems without a graphical desktop, run the same command with `--headless` instead of `--no-headless`. The dashboard remains available at localhost if a browser is available.

## 6. Run one scenario or the full demonstration

Single scenario, headless:

```bash
cd ~/src/gz-ws
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=78
export GZ_PARTITION=obstacle_course_demo
python3 tools/run_obstacle_course.py --scenario S3b --quick --headless
```

`--quick` is a one-repeat smoke check and cannot qualify a scenario. The available scenarios are `S0`, `S1`, `S2`, `S3`, `S3b`, `S4`, `S5`, `S6-lidar-drop`, `S6-camera-drop`, and `S7`.

The runner also sources `/opt/ros/$ROS_DISTRO/setup.bash` (defaulting to `lyrical` if `ROS_DISTRO` is unset) and this checkout's install overlay for each launch subprocess. It checks that `ros2`, `gz`, and `car_bringup` are available before creating a run; if setup is missing, it prints the corresponding install/build step rather than a Python `FileNotFoundError`.

Freeze every input before the ten-repeat demonstration:

```bash
cd ~/src/gz-ws
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=78
export GZ_PARTITION=obstacle_course_demo
python3 tools/freeze_obstacle_course_inputs.py \
  --output artifacts/obstacle-course/frozen-inputs/input-manifest.json
python3 tools/verify_obstacle_course_inputs.py \
  --manifest artifacts/obstacle-course/frozen-inputs/input-manifest.json
python3 tools/run_obstacle_course.py --all --repeats 10 \
  --run-purpose demonstration \
  --manifest artifacts/obstacle-course/frozen-inputs/input-manifest.json \
  --wall-timeout-s 300
```

The qualification rules require at least 8/10 behavior-sequence and trigger-set matches per scenario, zero non-ground contact in S0–S4, configured STOP/contact handling for S5, and LiDAR-dropout STOP in 10/10 S6-lidar-drop runs. Current results do not meet the full qualification gate. Generated worlds and reports are written below ignored `artifacts/obstacle-course/`.

For the decision logic, ROS interfaces, asset provenance, geometry, and measured limits, see the [technical implementation report](implementation.md).
