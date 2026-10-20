# Autonomous Vehicle Simulation

Gazebo Sim project for developing and evaluating a four-wheel vehicle model,
obstacle-course perception, and a future ROS 2 autonomy stack. The current
vehicle is a truck-scale prototype with starter physics parameters; it is not
a calibrated digital twin of a passenger car.

## Quick start

Install the Pixi environment once:

```powershell
pixi install
```

From this repository root, launch the obstacle course:

```powershell
$env:GZ_SIM_RESOURCE_PATH = Join-Path (Get-Location).Path 'simulation\models'
pixi run gz sim -r -v 4 simulation/worlds/obstacle.sdf
```

`-r` starts the simulation unpaused. Click the Gazebo 3D view to control the
vehicle with `W` forward, `A`/`D` steer, `X` reverse, and `S` stop.

These Windows GUI launches are visual smoke tests, not autonomy-evaluation
results. Evaluation runs must use the reproducible scenario and seed rules in
the project constitution.

To launch the simple vehicle demo instead:

```powershell
$env:GZ_SIM_RESOURCE_PATH = Join-Path (Get-Location).Path 'simulation\models'
pixi run gz sim -r -v 4 simulation/worlds/car_demo.sdf
```

## ROS 2 quick start (WSL2)

ROS 2 and Gazebo run together in WSL2 Ubuntu 26.04 from apt. Keep the clone and
workspace in the WSL Linux filesystem (for example `~/src/gz-ws`), never under
`/mnt/c`; the existing Windows Pixi environment remains untouched and is not
used for ROS. Follow the [fresh WSL setup and version checks](docs/wsl-setup.md)
before building.

After setup, build and test from Ubuntu:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/src/gz-ws/ros2_ws
colcon build --symlink-install --base-paths src
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The scenario-backed P1 sensor launch and exact expected rates are documented
in the [feature quickstart](specs/001-ros2-ai-obstacle-stop/quickstart.md).
To start P1 sensor inspection from the WSL clone root, use:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/src/gz-ws/ros2_ws
source install/setup.bash
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=sensors scenario:=training/sensor_smoke headless:=false
```

In a second sourced WSL terminal, inspect the camera topic (SDF source target
30 Hz), multi-layer lidar point cloud (15 Hz), IMU (100 Hz), odometry (50 Hz),
contact topic, and `/clock`; see the linked quickstart for exact topic commands
and measured WSL rates. P1's camera delivery floor is 20 Hz averaged over one
simulated minute in server-only headless mode; point cloud, IMU, and odometry
retain a 90%-of-declared-rate floor. Contact is event-only; the dedicated test
deliberately bumps an obstacle to verify the event. WSLg GUI is for visual
inspection only; autonomous runs use headless rendering to keep keyboard
control out of the command path.

To test P2 ROS driving, keep the ROS setup sourced and launch the headless
command scenario from `~/src/gz-ws/ros2_ws`:

```bash
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=command_test scenario:=training/command_smoke headless:=true
```

In a second sourced terminal, publish a slow forward-and-turn request:

```bash
ros2 topic pub --rate 10 /autonomy/cmd_vel_request \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.75}, angular: {z: 0.1}}"
```

The vehicle should move and turn. `safe_drive_controller` is the only ROS
publisher to `/model/gazebo_truck/cmd_vel`; the configured bridge is the only
Gazebo Transport writer. Stop the request publisher with Ctrl+C: the controller
sends zero within 0.5 simulation seconds and odometry settles within two more
simulation seconds. The headless launch filters the five dormant keyboard
trigger plugins from a temporary copy of the model; tracked model/world files
are unchanged. See the [P2 quickstart](specs/001-ros2-ai-obstacle-stop/quickstart.md#4-p2-command-via-the-single-controller)
for both-topic authority checks and automated test commands.

## P3 lidar obstacle-stop baseline (WSL2)

Run the training-only stop smoke scenario from `~/src/gz-ws` after sourcing
`/opt/ros/lyrical/setup.bash` and `ros2_ws/install/setup.bash`:

```bash
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=lidar scenario:=training/stop_smoke headless:=true
```

The same single controller uses the 16-layer point cloud, applies the checked-in
start pose, and stops in the configured forward corridor. It fails safe on
invalid/stale lidar, odometry, IMU, or simulation clock. The passive logger
writes a per-run CSV under ignored `artifacts/runs/`. The P3 target is a tall
existing gate: the roof lidar cannot reliably observe the short posts at the
5 m trigger range. No vehicle/world source files are edited.

After calibration, freeze the three parameter-disjoint scenario sets and run
the ten read-only lidar outcomes once:

```bash
python3 ml/collect_dataset.py validate-splits --freeze-evaluation
python3 tools/run_evaluation.py --split evaluation --behavior lidar \
  --output docs/evaluation/lidar-baseline.csv
```

The runner refuses to overwrite results, checks the sole ROS/Gazebo command
writer, preserves timed-out/aborted attempts, and writes a Markdown summary.
See the [P3 procedure](specs/001-ros2-ai-obstacle-stop/quickstart.md#5-p3-verify-lidar-only-obstacle-stopping)
and [evaluation record semantics](docs/evaluation/README.md). Evaluation
outcomes are read-only and must not feed tuning.

## Repository layout

```text
assets/source/              Blender source and preserved legacy exports
artifacts/                  Generated datasets, weights, and run data (gitignored)
docs/                       WSL setup, implementation, and evaluation reports
ml/                         Dataset collection, training, and evaluation scripts
ros2_ws/src/                car_bringup and car_autonomy_nodes ROS packages
simulation/models/car/       Gazebo model, meshes, and model documentation
simulation/worlds/           Gazebo demo and obstacle-course worlds
specs/                       Spec Kit feature specifications
.specify/                    Spec Kit templates, scripts, and project memory
.agents/skills/              Spec Kit commands for Codex
tools/                       Blender tools and evaluation orchestration
tools/blender/               Blender mesh-export tooling
pixi.toml, pixi.lock         Reproducible Gazebo Sim 10 environment
```

Gazebo resolves `model://car` from `simulation/models`, which is why the
launch commands set `GZ_SIM_RESOURCE_PATH` to that directory.

## Vehicle and sensor interfaces

- Velocity command: `/model/gazebo_truck/cmd_vel` (`gz.msgs.Twist`)
- Odometry: `/model/gazebo_truck/odometry`
- Camera image: `/model/gazebo_truck/sensors/front_camera/image`
- Multi-layer lidar scan and point cloud:
  `/model/gazebo_truck/sensors/front_lidar/scan` and
  `/model/gazebo_truck/sensors/front_lidar/scan/points`
- IMU: `/model/gazebo_truck/sensors/imu`
- Body contact events: `/model/gazebo_truck/sensors/bumper_contact`
- Simulation clock: `/clock` (`gz.msgs.Clock`; bridged as `rosgraph_msgs/msg/Clock`)

## Spec Kit workflow

This repository includes the Spec Kit Codex integration and its project
constitution. Install or update the CLI using the official instructions; the
current pinned release can be installed with:

```powershell
uv tool install specify-cli --from git+https://github.com/github/spec-kit.git@v1.2.1
```

Then open this repository in Codex and invoke the installed skills in order:

```text
$speckit-constitution
$speckit-specify
$speckit-clarify
$speckit-plan
$speckit-tasks
$speckit-analyze
$speckit-implement
```

These are Codex skills, not PowerShell commands. Implement one story at a time;
run its independent test and commit the passing story before starting the next.
For ROS work, keep the repository and workspace in the WSL2 filesystem and use
the apt-installed ROS 2/Gazebo environment; the Windows Pixi environment is
only the Windows-native Gazebo path. See
[`docs/spec-driven-development.md`](docs/spec-driven-development.md).

## Documentation

- [Vehicle model guide](simulation/models/car/README.md)
- [Implementation report](docs/implementation-report.md)
- [Spec-driven development guide](docs/spec-driven-development.md)
- [WSL2 ROS/Gazebo setup](docs/wsl-setup.md)
- [Obstacle-stop evaluation guide](docs/evaluation/README.md)
- [Mesh export tool](tools/blender/export_gazebo_parts.py)
