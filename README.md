# RC Obstacle Course and Multi-Sensor Behavior Demo

Feature 007 adds a compact course for the existing RC-scale Gazebo vehicle, a
rule-based sensor decision layer, an attributed live dashboard, and real-car
setup sheets. **The runtime gate, measured vehicle profile, shared contracts,
and protected-input checks are complete. Course behavior and qualification are
in progress; no Feature 007 pass rates are claimed yet.**

Start at the existing [quickstart entry point](docs/quickstart.md). The
Feature 007 implementation sequence and acceptance checks are documented in
[its run guide](docs/features/007-multi-sensor-obstacle-course/quickstart.md).

## Project map

- `simulation/models/rc_car/` and `simulation/models/research/`: protected
  Feature 003 physics and high-detail visual anchors; Feature 007 reads them
  without changing their files.
- `simulation/worlds/`: existing worlds remain protected; the condensed course
  will be added as a separate world.
- `ros2_ws/src/`: existing ROS 2 packages plus the new `car_decision` package
  and additive course wiring as it is implemented.
- `tools/`: fixed-seed scenario generation, dynamics measurement,
  qualification, and mapping tools as they are implemented.
- `docs/features/007-multi-sensor-obstacle-course/`: implementation contract,
  runtime findings, data model, task sequence, and quickstart.
- `docs/real-car-mapping/` and `docs/evaluation/obstacle-course/`: planned
  operator setup sheets and final results.
- `artifacts/obstacle-course/`: ignored generated worlds, measurements, and
  per-run records.

## Protected history

Feature 001 and 002 frozen scenarios, model assets, manifests, and results are
historical inputs and remain unchanged. Feature 003 RC physics and locked
research assets are read-only anchors. New Feature 007 scenarios and results
use separate paths; demonstration runs do not write to prior metric roots.

## Runtime

The required runtime gate passed on WSL2 Ubuntu 26.04, ROS 2 Lyrical,
`ros_gz_bridge` 3.0.10, and Gazebo Sim 10.5.0. The recorded output is in
[runtime-gate.md](docs/features/007-multi-sensor-obstacle-course/runtime-gate.md).
