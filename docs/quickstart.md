# Install and run

Use Ubuntu 24.04 LTS (Noble), ROS 2 Jazzy, and Gazebo Harmonic (Sim 8)
on native Ubuntu or WSL2. Install ROS and Gazebo from the official ROS apt
repository. A visible run needs Ubuntu Desktop or WSLg.

If you have an Ubuntu 26.04 WSL checkout, open a separate Ubuntu 24.04 distro
and clone or copy your source into its Linux filesystem. Build a fresh ROS
workspace there. Use a new clone or move the old workspace's generated
`ros2_ws/build`, `ros2_ws/install`, and `ros2_ws/log` directories before rebuilding.
In Windows PowerShell:

```powershell
wsl --install -d Ubuntu-24.04
wsl -d Ubuntu-24.04
```

See the [ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
and [Gazebo/ROS compatibility table](https://gazebosim.org/docs/latest/ros_installation/).

## 1. Install the runtime

Open a terminal in Ubuntu or WSL2. These commands add the official ROS apt source, install ROS and build tools, then initialize `rosdep` if needed:

```bash
set -e
. /etc/os-release
if [ "$ID" != ubuntu ] || [ "$VERSION_ID" != 24.04 ]; then
  echo "This checkout requires Ubuntu 24.04; found $ID $VERSION_ID." >&2
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
  ros-jazzy-ros-base ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim ros-jazzy-ackermann-msgs ros-jazzy-cv-bridge \
  ros-jazzy-tf2-ros ros-jazzy-vision-msgs build-essential \
  python3-colcon-common-extensions python3-rosdep \
  python3-pytest python3-yaml python3-jsonschema python3-opencv

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --rosdistro jazzy
```

## 2. Clone and build

Clone the repository into the Linux filesystem. Avoid building from `/mnt/c` under WSL:

```bash
mkdir -p ~/src
git clone https://github.com/SoneNguyen/autonomous-rc-car.git ~/src/gz-ws
cd ~/src/gz-ws
source /opt/ros/jazzy/setup.bash
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
. /opt/ros/jazzy/setup.bash
. /etc/os-release && printf 'Ubuntu %s (%s)\n' "$VERSION_ID" "$UBUNTU_CODENAME"
printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
test "$VERSION_ID" = 24.04
test "$ROS_DISTRO" = jazzy
source ros2_ws/install/setup.bash
python3 -m car_bringup.runtime
```

Expect Ubuntu `24.04`, ROS `jazzy`, Gazebo Sim `8.x`, and the installed bridge version.
The WSL2 migration was checked with Sim `8.15.0` and `ros_gz_bridge` `1.0.25`.
The runner records exact package versions with each result.

## 3. Fetch and verify the visual assets

The cone, barrier, gate, and asphalt textures need network access on the first fetch. The
checkout includes the verified original pickup and SUV. Fetching restores those
two assets if the local cache has different bytes. Every lock still checks the
original SHA-256.

```bash
cd ~/src/gz-ws
asset_config=ros2_ws/src/car_bringup/config/obstacle_course
(
  set -e
  for lock in assets.lock.yaml research_assets.lock.yaml research_vehicle.lock.yaml; do
    python3 tools/fetch_obstacle_course_assets.py --lock "$asset_config/$lock"
    python3 tools/fetch_obstacle_course_assets.py --lock "$asset_config/$lock" --verify-only
  done
)
```

Expect six `Locked assets: PASS` lines. Files are installed under
`artifacts/obstacle-course/assets/`. Simulation uses this cache offline.

## 4. Validate and run headless

These checks do not need a display and work on Ubuntu Server, desktop Ubuntu, or WSL2:

```bash
cd ~/src/gz-ws
source /opt/ros/jazzy/setup.bash
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
source /opt/ros/jazzy/setup.bash
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

On a slower machine or one without a desktop, replace `--no-headless` with
`--headless --dashboard`. Open <http://localhost:8765/> for the camera,
LiDAR and decision status. For the smallest workload, use
`--headless --no-dashboard`. Both use the same scenario, sensors and controller.

Gazebo uses the graphics driver selected by the operating system. On WSLg with
a working Mesa D3D12 driver, you can opt into GPU rendering by prefixing the
visible launch command with `GALLIUM_DRIVER=d3d12`. If a visible window fails
to open, use the headless dashboard first. For a driver check on Mesa,
try prefixing the command with `LIBGL_ALWAYS_SOFTWARE=1`. Software rendering
can be slower. Allow more wall time with `--wall-timeout-s 120` when the
simulation clock advances slowly. A larger timeout gives the same scenario more
time to finish on the host.

The setup target is Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic on both
native Ubuntu and WSL2. On a different Ubuntu release, run the project in an
Ubuntu 24.04 environment. Run the same S0 smoke check on each device before a demo:

```bash
python3 tools/run_obstacle_course.py --scenario S0 --quick --headless --dashboard --wall-timeout-s 120
```

## 6. Run one scenario or the full demonstration

Single scenario, headless:

```bash
cd ~/src/gz-ws
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=78
export GZ_PARTITION=obstacle_course_demo
python3 tools/run_obstacle_course.py --scenario S3b --quick --headless
```

`--quick` is a one-repeat smoke check and cannot qualify a scenario. The available scenarios are `S0`, `S1`, `S2`, `S3`, `S3b`, `S4`, `S5`, `S6-lidar-drop`, `S6-camera-drop`, and `S7`.

The runner sources `/opt/ros/jazzy/setup.bash` and this checkout's install overlay for each launch subprocess. It checks that `ros2`, `gz`, and `car_bringup` are available before creating a run; if setup is missing, it prints the corresponding install/build step rather than a Python `FileNotFoundError`.

Before qualifying the full demonstration on Harmonic, measure braking again
on this stack and review the resulting profile. The tracked values and earlier
qualification records came from Jetty. A quick run can exercise the migrated
stack with the existing profile.

Freeze every input before the ten-repeat demonstration:

```bash
cd ~/src/gz-ws
source /opt/ros/jazzy/setup.bash
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
