# Quickstart

Use WSL2 Ubuntu 26.04 on the Linux filesystem with ROS 2 Lyrical and Gazebo
Sim 10.5.0 installed from apt. Setup and asset downloads need a network
connection; simulation runs use local assets.

## 1. Install and build

Configure the official [ROS 2 Lyrical apt repository](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html).
If both ROS apt source files exist, disable the older duplicate before
updating apt:

~~~bash
set -e
if [ -f /etc/apt/sources.list.d/ros2.sources ] && [ -f /etc/apt/sources.list.d/ros2-latest.list ]; then
  sudo mv /etc/apt/sources.list.d/ros2-latest.list \
    /etc/apt/sources.list.d/ros2-latest.list.disabled
fi
sudo apt update
sudo apt install -y ros-lyrical-desktop \
  ros-lyrical-ros-gz ros-lyrical-ros-gz-bridge ros-lyrical-ros-gz-sim \
  ros-lyrical-ackermann-msgs ros-lyrical-vision-msgs ros-lyrical-tf2-ros \
  ros-lyrical-rqt-image-view ros-lyrical-rqt-plot ros-lyrical-rqt-topic \
  python3-colcon-common-extensions python3-rosdep python3-pytest \
  python3-yaml python3-jsonschema python3-numpy python3-pil \
  python3-opencv python3-onnxruntime
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
~~~

Clone the repository, or reuse the existing checkout at `~/src/gz-ws`:

~~~bash
mkdir -p ~/src
if [ -d ~/src/gz-ws/.git ]; then
  echo "Using existing checkout at ~/src/gz-ws"
elif [ ! -e ~/src/gz-ws ]; then
  git clone https://github.com/SoneNguyen/autonomous-rc-car.git ~/src/gz-ws
else
  echo "~/src/gz-ws exists but is not a Git checkout; move it or choose another path" >&2
  exit 1
fi
cd ~/src/gz-ws
source /opt/ros/lyrical/setup.bash
unset COLCON_PREFIX_PATH
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
colcon --log-base ros2_ws/log build --symlink-install \
  --base-paths ros2_ws/src --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --packages-up-to car_navigation car_bringup car_autonomy_nodes
source ros2_ws/install/setup.bash
~~~

If you have already initialized rosdep, the install block skips `rosdep init`.
Rebuild after moving the checkout because the symlink install uses absolute
paths.

## 2. Fetch the locked research assets

The controlled vehicle, road, cones, pedestrians, and other vehicles use the
locked high-fidelity assets. Download and verify both the environment/object
assets and the research pickup mesh:

~~~bash
cd ~/src/gz-ws
python3 tools/fetch_matrix_assets.py
python3 tools/fetch_matrix_assets.py \
  --lock ros2_ws/src/car_bringup/config/rc_module_matrix/research_vehicle.lock.yaml
python3 tools/fetch_matrix_assets.py --verify-only
python3 tools/fetch_matrix_assets.py \
  --lock ros2_ws/src/car_bringup/config/rc_module_matrix/research_vehicle.lock.yaml \
  --verify-only
~~~

Assets are stored under the ignored `artifacts/rc-car-matrix/assets/`
directory. Fetch them before running offline.
The controlled vehicle retains the RC model's collision, sensor, and drive
geometry, with the locked research pickup mesh as its visible body. Displayed
vehicles, obstacles, and road materials come from the checked-in research
asset locks.

## 3. Open a visible module demo

In a WSLg terminal, initialize ROS and this checkout in every new terminal:

~~~bash
cd ~/src/gz-ws
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
~~~

Then launch the component you want to inspect:

| Component | Command | What you will see |
|---|---|---|
| LiDAR | `python3 tools/rc_module_gui.py lidar` | Gazebo scene and live front-beam range plot. |
| Camera | `python3 tools/rc_module_gui.py camera` | Gazebo scene, RGB and segmentation windows, and detection topics. |
| IMU/odometry | `python3 tools/rc_module_gui.py imu_odometry` | Gazebo scene and live IMU/odometry plots during a short turn. |
| Contact/TF | `python3 tools/rc_module_gui.py contact_tf` | Gazebo contact scene and contact/TF topics. |
| Drive actuator | `python3 tools/rc_module_gui.py actuator` | Gazebo scene and odometry plots during a short drive pulse. |

These viewer commands start only the selected sensor and bridge only its topics;
the unrelated camera, segmentation, and LiDAR workloads stay disabled. They use
the `low` preview profile by default (320 × 240 camera at 15 Hz, reduced shadows)
to keep WSLg responsive. Use `--quality high` when you need the qualification
camera resolution and rate:

~~~bash
python3 tools/rc_module_gui.py camera --quality high
~~~

The preview profile is for visual inspection and does not produce qualification
results. The full matrix keeps its configured sensors and quality settings.

The viewer uses a project-owned Gazebo layout, so saved panels in `~/.gz` do
not add duplicate UI elements. It opens one 3D scene, one world-control panel,
one stats panel, and one entity tree. The camera is framed from the selected
scenario's car and in-world fixtures; each scenario only displays the actors
that belong to that test.

The IMU/odometry and actuator demos briefly command the simulated truck at
0.2 m/s, then publish a zero command. The actuator demo exercises the Gazebo
drive plugin through its bridge; it does not test the safe-drive controller.
Press Ctrl-C in the launching terminal to close the viewer. If Gazebo exits or
crashes, the launch now stops its bridge and ROS nodes, and the viewer reports
that it did not receive live sensor data instead of waiting on a topic name
created by the bridge. These demos are for visual inspection and do not create
qualification reports.

If the terminal says `ros2: command not found` or exits with status `127`, run
the setup commands again in that same WSLg terminal before starting the demo:

~~~bash
cd ~/src/gz-ws
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
command -v ros2 gz
~~~

The demo script now checks these commands and its selected `rqt` viewer before
launching Gazebo, and prints a setup or package-install hint if one is missing.

## 4. Run the qualification matrix

Keep the ROS and workspace setup from step 3 active. List available scenarios,
then run a pilot, one quick case, one module, or the full core matrix:

~~~bash
python3 tools/rc_module_matrix.py --list

# Camera label pilot: measures visible label pixels; it does not qualify camera.
python3 tools/rc_module_matrix.py --module camera --pilot

# One smoke case, then the complete camera set.
python3 tools/rc_module_matrix.py --module camera --quick
python3 tools/rc_module_matrix.py --module camera

# Other modules.
python3 tools/rc_module_matrix.py --module lidar --quick
python3 tools/rc_module_matrix.py --module lidar
python3 tools/rc_module_matrix.py --module imu_odometry --pilot
python3 tools/rc_module_matrix.py --module imu_odometry
python3 tools/rc_module_matrix.py --module contact_tf

# Full core matrix.
python3 tools/rc_module_matrix.py --all

# Optional command-path set.
python3 tools/rc_module_matrix.py --module command_path --include-stretch
python3 tools/rc_module_matrix.py --all --include-stretch
~~~

The camera pilot reads segmentation labels directly; it does not wait for the
independently scheduled RGB camera or start the detector. Scored camera cases
require exact RGB, label, and detector timestamp matches. Before running, the
CLI checks that `car_bringup` resolves from this checkout so a stale sourced
workspace cannot launch the wrong test sequence.

`--quick` runs one marked smoke case and never qualifies a module. Full module
runs require every declared case and observation to pass. A blocked or partial
report is not a pass. Reports, logs, and captures are written to the ignored
`artifacts/rc-car-modules/<run-id>/` directory. Current results and measured
limits are summarized in `docs/evaluation/rc-car-modules/`. LiDAR passes all
14 cases on the configured GPU sensor (report
`788604d0-4b41-4eb3-b6bd-ebb092e6731e`). IMU/odometry, contact/TF, and the
optional command-path set also pass. Camera remains unqualified until its
complete scored set passes; see the per-module report.

Qualification runs keep the Gazebo viewer closed and write scored reports. On
WSLg, sensor captures use server-only GLX and select Mesa D3D12 on the NVIDIA
adapter automatically. Keep `DISPLAY` or `WAYLAND_DISPLAY` available.

## 5. Optional offline startup check

After fetching and verifying both asset locks, run a quick LiDAR case with
network access disabled:

~~~bash
unshare --user --map-root-user --net bash -lc '
  set -e
  ip link set lo up
  source /opt/ros/lyrical/setup.bash
  cd ~/src/gz-ws
  source ros2_ws/install/setup.bash
  export ROS_LOCALHOST_ONLY=1
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  python3 tools/rc_module_matrix.py --module lidar --quick
'
~~~

The report records scenario status, overlap checks, config hashes, and evidence
references.
