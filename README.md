# RC Sensor and Actuation Matrix

A fixed-seed characterization suite for the RC simulation's LiDAR, camera,
IMU/odometry, contact/TF, and command path. The matrix uses the locked research
meshes for the controlled vehicle, obstacles, and road. The RC vehicle SDF
supplies collision, sensor, and drive physics only.

Start with [the quickstart](docs/quickstart.md). It covers the apt-based ROS 2
Lyrical/Gazebo setup, research asset locks, workspace build, and the available
matrix commands.

The matrix records simulator observations with scenario identity, seed, ground
truth, units, frames, timestamps, and tolerances. Quick runs are diagnostic
only. Simulator values and sensor noise are not hardware accuracy claims.

Current qualification: LiDAR 14/14, IMU/odometry 4/4, contact/TF 3/3, and the
optional command-path set 3/3 pass. Camera/detection remains unqualified until
its complete scored set passes. See the [module status](docs/evaluation/rc-car-modules/matrix.md).

## Project map

- `ros2_ws/`: runtime packages and matrix scenario configuration.
- `simulation/`: RC physics/sensor body and the research-asset test world.
- `models/rc_module_matrix/`: frozen detector used by camera checks.
- `tools/rc_module_matrix.py`: scenario selection, separation checks, reports.
- `tools/rc_module_gui.py`: live Gazebo and ROS viewers for individual modules.
- `tools/checks/`: module observation logic and simulator capture.
- `docs/contracts/rc-car-modules/`: local scenario, report, and stop contracts.
- `docs/evaluation/rc-car-modules/`: implementation and qualification status.
- `artifacts/rc-car-matrix/assets/`: fetched research assets; ignored by Git.
- `artifacts/rc-car-modules/`: run reports and evidence; ignored by Git.

Supported runtime: WSL2 Ubuntu 26.04, ROS 2 Lyrical, Gazebo Sim 10.5.0,
installed from apt. Fetch assets before running; simulation runs use local
assets and do not need network access.
