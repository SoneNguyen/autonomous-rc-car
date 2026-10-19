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

## ROS 2 development

ROS 2 integration is not part of the current Pixi demo yet. For ROS work, keep
the repository and ROS workspace inside WSL2 Ubuntu and install ROS 2 and Gazebo
from apt; do not use the Windows Pixi environment from WSL. Each ROS feature's
`research.md` will pin the required versions, and its `quickstart.md` will give
fresh-WSL setup and run commands with expected results.

## Repository layout

```text
assets/source/              Blender source and preserved legacy exports
artifacts/                  Generated datasets, weights, and results (gitignored)
docs/                       Setup, implementation, and research notes
ml/                         ML training and inference code
ros2_ws/                    ROS 2 packages (WSL2 filesystem only)
simulation/models/car/       Gazebo model, meshes, and model documentation
simulation/worlds/           Gazebo demo and obstacle-course worlds
specs/                       Spec Kit feature specifications
.specify/                    Spec Kit templates, scripts, and project memory
.agents/skills/              Spec Kit commands for Codex
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
- [Mesh export tool](tools/blender/export_gazebo_parts.py)
