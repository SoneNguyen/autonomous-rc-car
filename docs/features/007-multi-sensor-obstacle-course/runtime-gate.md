# Feature 007 Runtime Gate

**Result: PASS**. Recorded before Feature 007 implementation on base commit `42b3b12bc77d893231f1fad3999db4b8196e9ee3`.

## Current environment

The exact T001 command block in `tasks.md` was run from the repository root after sourcing `/opt/ros/lyrical/setup.bash`.

```text
OS_NAME=Ubuntu
OS_VERSION_ID=26.04
WSL2=PASS
ROS_DISTRO=lyrical
ROS_GZ_BRIDGE_VERSION=3.0.10
GAZEBO_SIM_VERSION_OUTPUT=10.5.0
APT_PACKAGE_VERSIONS:
ros-lyrical-ros-gz 3.0.10-1resolute.20260930.020431
ros-lyrical-ros-gz-bridge 3.0.10-1resolute.20260915.142616
ros-lyrical-ros-gz-sim 3.0.10-1resolute.20260930.014144
RUNTIME PIN GATE=PASS
```

All required runtime pins match: WSL2 Ubuntu 26.04, ROS 2 Lyrical, `ros_gz_bridge` 3.0.10, and Gazebo Sim 10.5.0. Runtime package versions were provided by apt.

## Prior Feature 002 evidence

The prior runtime/controller evidence is `docs/evaluation/rc-car/p2-route-check.md` from Feature 002 ref `3a0a835179c7366f2921d181436a019b28ba7dca`, SHA-256 `58bfc36e8130115db724132abdd621f4e92acaa2caeb1621421f82c59524b6cb`. It records the matching runtime pins and 10/10 route completion for its own fixed-seed route scenario. It is cited only as prior runtime/controller evidence; its route behavior and placeholder values are not Feature 007 braking or stopping-distance measurements.

## Canonical path inventory

- Inventory: `docs/features/007-multi-sensor-obstacle-course/canonical-paths.md`
- SHA-256: `cad5bd9b8feeb1da220d20910c5c1caadce917e98348fd49c0ad084f8fe8bcd6`
- Verified at base commit above. The task and quickstart paths use this inventory as their canonical names.

## Foundational implementation gate (T017)

The foundation check was run after the measured simulation profile, shared
configuration, JSON contracts, read-only overlap snapshot, protected-input
checker, and `car_decision` package metadata were in place.

```text
python3 -m pytest tools/tests/test_dynamics_measurement.py tools/tests/test_vehicle_geometry.py tools/tests/test_obstacle_course_sizing.py tools/tests/test_protected_paths.py tools/tests/test_scenario_overlap_snapshot.py ros2_ws/src/car_decision/test/test_config_contracts.py -q
36 passed

python3 tools/validate_obstacle_course_config.py --all
OBSTACLE COURSE CONFIG VALIDATION PASS (0 production scenarios)
profile_sha256=886c30481d02054ecbe6f3bf04eff196d06603e9f57efdef656cd2f371b31381
PROTECTED PATH CHECK PASS

source /opt/ros/lyrical/setup.bash
colcon --log-base ros2_ws/log build --symlink-install --base-paths ros2_ws/src --build-base ros2_ws/build --install-base ros2_ws/install --packages-up-to car_decision
Summary: 1 package finished
```

The sizing tests verify geometry derivation and reject routes above the 25%
derived bound or at/above 280 m. The CLI requires tracked scenario inputs; no
production scenarios are authored until P1. The validator's zero-scenario
result is therefore limited to shared config/schema and protected-history
checks and does not claim a course qualification.
