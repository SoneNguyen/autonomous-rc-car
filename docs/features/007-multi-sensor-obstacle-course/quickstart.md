# Feature 007 Quickstart

This guide describes the planned commands after Feature 007 implementation is merged. Canonical tool, config, and output paths are listed in [canonical-paths.md](canonical-paths.md). Run the full simulation on WSL2 Ubuntu 26.04 with ROS 2 Lyrical, `ros_gz` 3.0.10, and Gazebo Sim 10.5.0. The controller is the sole Gazebo command publisher. Every simulation command selects a tracked scenario config with a fixed seed. Runtime scenarios use local assets and do not need network access.

## 1. Install and build on fresh WSL

Install ROS/Gazebo and Python tooling from apt, then build from the WSL filesystem:

```bash
sudo apt update
sudo apt install -y \
  ros-lyrical-desktop ros-lyrical-ros-gz ros-lyrical-ros-gz-bridge \
  ros-lyrical-ros-gz-sim ros-lyrical-ackermann-msgs \
  ros-lyrical-vision-msgs python3-colcon-common-extensions \
  python3-rosdep python3-pytest python3-yaml python3-jsonschema \
  python3-opencv ros-lyrical-teleop-twist-keyboard

source /opt/ros/lyrical/setup.bash
cd ~/src/gz-ws
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
colcon --log-base ros2_ws/log build --symlink-install \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --packages-up-to car_decision car_navigation car_bringup car_autonomy_nodes
source ros2_ws/install/setup.bash
```

Expected: all four packages finish building. `rosdep` resolves ROS message and `cv_bridge` dependencies from apt. The current Feature 003 RC model and safe-drive controller are used directly; no removed Feature 001 large-truck or planner files need to be restored.

## 2. Fetch the highest-quality Feature 003 research assets

Fetch and verify the locked research meshes before going offline:

```bash
python3 tools/fetch_matrix_assets.py
python3 tools/fetch_matrix_assets.py --verify-only
python3 tools/fetch_matrix_assets.py \
  --lock ros2_ws/src/car_bringup/config/obstacle_course/assets.lock.yaml
python3 tools/fetch_matrix_assets.py \
  --lock ros2_ws/src/car_bringup/config/obstacle_course/assets.lock.yaml \
  --verify-only
```

Expected: both locked asset checks pass. Assets remain under the ignored `artifacts/rc-car-matrix/assets/` directory. Feature 007 selects the original Feature 003 research GLBs (including the compact-SUV vehicle visual, cones, pedestrians, and miniature vehicles) plus only separately pinned, quality-reviewed additions in its own lock. Keep the original mesh and material detail; do not load lower-quality or generated visual substitutes. The controlled vehicle visual copies only the pinned mesh URI and scale into a collisionless companion; it does not include the research model's collision geometry. Primitive collision proxies are used only for physics/contact. Export the Gazebo search path in every launch terminal:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/artifacts/rc-car-matrix/assets/models:$PWD/simulation/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
```

Validate the vehicle, decision, trigger-matrix, and scenario configs before running a simulation:

```bash
python3 tools/validate_obstacle_course_config.py --all
```

Expected: all tracked configs and JSON schemas pass; the validator prints the fixed scenario seeds, rejects missing unit/provenance fields, and reports no overlap with frozen scenario inputs using the pinned overlap checker in read-only mode.

## 3. Measure RC vehicle dynamics once for this simulation profile

Do not substitute the RC placeholder deceleration or an old latency report. Run the scripted RC brake test and write only accepted simulation measurements to the new Feature 007 `rc_vehicle_reference` overlay:

```bash
python3 tools/measure_rc_vehicle_dynamics.py \
  --scenario-config ros2_ws/src/car_bringup/config/obstacle_course/diagnostics/rc_vehicle_brake.yaml \
  --profile ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml \
  --repeats 10 \
  --output artifacts/obstacle-course/dynamics/rc-vehicle-reference.json \
  --write-profile
```

Expected: a measurement report with raw simulation-time speed/command traces, `a_brake_mps2`, `t_latency_s`, trial count, Gazebo/ROS identity, and config hashes. The profile records accepted values with status `measured_sim`. Review this report and freeze the profile/config hash before demonstration runs. These numbers describe this simulation only.

Recompute the geometry-derived course bounds before creating or running the new world:

```bash
python3 tools/check_obstacle_course_sizing.py \
  --profile ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml \
  --geometry ros2_ws/src/car_bringup/config/obstacle_course/vehicle_geometry.yaml \
  --scenarios ros2_ws/src/car_bringup/config/obstacle_course/scenarios
```

Expected: each route is at most 1.25 times its recomputed minimum, and every route remains below 280 m. The command fails before world generation if either condition is false.

## 4. Generate and inspect one scenario world

```bash
python3 tools/build_obstacle_course_profile.py \
  --scenario-config ros2_ws/src/car_bringup/config/obstacle_course/scenarios/S1.yaml
python3 tools/generate_obstacle_course_world.py \
  --scenario-config ros2_ws/src/car_bringup/config/obstacle_course/scenarios/S1.yaml
```

Expected: the generated world is `artifacts/obstacle-course/worlds/S1.sdf` and its SHA-256 is printed. `gz sdf -k simulation/worlds/obstacle_condensed.sdf` and `gz sdf -k artifacts/obstacle-course/worlds/S1.sdf` both pass. The collisionless vehicle visual follows the RC model in straight and turning motion using the highest-quality pinned Feature 003 mesh; the Feature 003 RC physics model and every existing world remain unchanged.

The launch runner generates the per-scenario runtime profile under `artifacts/obstacle-course/config/` from the tracked `rc_vehicle_reference` profile and selected scenario. The profile uses the existing RC geometry and contains no lower-quality asset fallback.

## 5. Tune development variants, then freeze production inputs

Tune only the tracked development variant. It has its own fixed seed and is reported as `development`/`qualified=false`:

```bash
python3 tools/run_obstacle_course.py \
  --scenario-config ros2_ws/src/car_bringup/config/obstacle_course/development/scenarios/barrier_tuning.yaml \
  --decision-config ros2_ws/src/car_bringup/config/obstacle_course/development/decision.yaml \
  --quick
```

After tuning, freeze the protected-input baseline, production profile, geometry, decision/matrix configs, all production scenarios, base world, and world fragments. Verify the manifest and protected baseline before any demonstration run:

```bash
python3 tools/freeze_obstacle_course_inputs.py \
  --output artifacts/obstacle-course/frozen-inputs/input-manifest.json
python3 tools/verify_obstacle_course_inputs.py \
  --manifest artifacts/obstacle-course/frozen-inputs/input-manifest.json
python3 tools/check_feature_007_protected_paths.py \
  --baseline docs/features/007-multi-sensor-obstacle-course/protected-inputs.json
```

Expected: the manifest lists every input path and content hash and prints one deterministic root SHA-256. Verification reports the same root. Any changed input fails verification and requires a new freeze and a complete rerun.

## 6. Run a visible behavior demo

Start the autonomous demo headless:

```bash
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=behavior_demo control:=autonomous scenario:=S1 run_purpose:=demonstration headless:=true \
  input_manifest:=artifacts/obstacle-course/frozen-inputs/input-manifest.json
```

In a second terminal, start the dashboard as a separate, read-only process:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/src/gz-ws
source ros2_ws/install/setup.bash
ros2 run car_decision dashboard_node
```

Open `http://localhost:8765/` in a Windows browser. The dashboard shows the active behavior, reason, component decision roles (trigger, support, veto, overruled, idle) separately from freshness (fresh, stale, unavailable), and event history. It has no command endpoint. A view-only Gazebo viewer can be attached with the documented launch option; keyboard driving is disabled in this autonomous mode.

The launch creates one run ID shared with the run logger. The logger records transition and attribution-changing evaluation events under `artifacts/obstacle-course/runs/<run-id>/`; the dashboard displays those same events. Stopping the dashboard does not stop logging or control.

Expected S1 evidence: LiDAR reports a route-corridor return; camera classifies the red barrier; decisions progress through CAUTION, a feasible AVOID side, RESUME, and FOLLOW_ROUTE. The event record names trigger/support components and their values. No non-ground contact is expected.

## 7. Inspect each component's work case

Run one representative case and inspect the normalized topic and corresponding event. The quick case is diagnostic only; it never qualifies the full scenario.

| Component | Work case | Expected evidence |
|---|---|---|
| LiDAR | S1 barrier, then S3b object inside gate opening | Five-sector ranges and corridor occupancy update; S3b records the in-opening return and LiDAR override of the camera gate class |
| Camera | S1/S2 red barrier and orange posts; S3 clear blue gate | HSV class, blob area, confidence/colour score, apparent size, and bearing update in simulation time under world-default lighting |
| IMU | S7 sharp avoidance turn | Yaw rate and lateral acceleration update; configured acceleration limit causes CAUTION/speed cap |
| Odometry | S0 straight route and S7 turn | Simulation-time speed, pose, spawn-relative progress, and `d_stop(v)` are present |
| Contact | S1 no-contact pass and S5 late-reveal case | Ground-only contacts are excluded; non-ground entity pair is identified and latches STOP if contact occurs under S5's allowed condition |

Production and qualification runs use the configured HSV camera path for red barriers, orange posts, and blue gates. The frozen Feature 001 detector is diagnostic-only because its `cone`/`pedestrian`/`vehicle` labels cannot identify barriers or gates. If its local model artifact is present and its SHA-256 verifies, a diagnostic configuration may preserve those source labels (coarse `cone` to `post` only); it cannot qualify a scenario. If the artifact is absent or its hash does not match, use HSV and do not run the detector path.

For example, run the short inspection case and echo the component event:

```bash
python3 tools/run_obstacle_course.py --scenario S3b --repeats 1 --quick --run-purpose diagnostic \
  --manifest artifacts/obstacle-course/frozen-inputs/input-manifest.json
ros2 topic echo /decision/evidence/lidar
ros2 topic echo /decision/event
```

Expected: the S3b case reports the gate as detected and overruled by LiDAR, includes the in-corridor range, then follows the configured normal LiDAR AVOID or STOP branch. `--quick` reports `smoke`/`qualified=false`.

## 8. Drive a course manually through the safety controller

Manual mode is separate from autonomous decision mode. The world keyboard plugin is disabled; keyboard teleoperation sends a `Twist` intent to `manual_twist_adapter.py`, which converts it to the safe-drive controller's bounded Ackermann input. The autonomous controller-request adapter is disabled in manual mode, so only one adapter publishes `/ackermann_cmd`.

Terminal 1:

```bash
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=behavior_demo control:=manual scenario:=S2 run_purpose:=demonstration headless:=false \
  input_manifest:=artifacts/obstacle-course/frozen-inputs/input-manifest.json
```

Terminal 2:

```bash
source /opt/ros/lyrical/setup.bash
source ~/src/gz-ws/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/manual/cmd_vel_intent
```

Expected: the RC vehicle responds to keyboard input, but `/model/gazebo_truck/cmd_vel` still has exactly one ROS publisher: the safe-drive controller. Do not run the teleop process during `control:=autonomous`.

To verify the new world's contact bridge, use the tracked S1 diagnostic config, echo `/contacts`, and gently touch the barrier once. The topic should report the contacting pair. Relaunch S1 before its no-contact traversal; keep this deliberate contact probe out of demonstration scoring.

```bash
# Terminal 1
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=behavior_demo control:=manual scenario:=S1 \
  run_purpose:=diagnostic headless:=false

# Terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/manual/cmd_vel_intent

# Terminal 3; then touch the barrier using the keyboard in Terminal 2
ros2 topic echo --once /contacts
```

## 9. Run qualification repeats

Run one complete scenario or the complete catalogue. Each scenario YAML has a fixed base seed. Repeat 1 uses that seed; repeat `n` uses `base_seed + n - 1`. The runner records all seeds, and the two S6 fault profiles are scored separately.

```bash
python3 tools/verify_obstacle_course_inputs.py \
  --manifest artifacts/obstacle-course/frozen-inputs/input-manifest.json
python3 tools/check_feature_007_protected_paths.py \
  --baseline docs/features/007-multi-sensor-obstacle-course/protected-inputs.json
python3 tools/run_obstacle_course.py --scenario S3 --repeats 10 \
  --manifest artifacts/obstacle-course/frozen-inputs/input-manifest.json
python3 tools/run_obstacle_course.py --scenario S6-lidar-drop --repeats 10 \
  --manifest artifacts/obstacle-course/frozen-inputs/input-manifest.json
python3 tools/run_obstacle_course.py --all --repeats 10 \
  --manifest artifacts/obstacle-course/frozen-inputs/input-manifest.json
```

Expected aggregate: each S0-S7/S3b configuration, with S6 dropout profiles counted separately, has at least 8/10 exact collapsed behavior-sequence and exact trigger-set matches; each repeat records independent behavior and trigger-set match booleans, and any missing or extra trigger fails the latter. S6 LiDAR-dropout STOP is 10/10; passing S0-S4 including S3b has zero non-ground contact. Before freezing, S5 pins a contact-free or unavoidable-contact branch from measured stopping feasibility. Fault events record reveal time, route progress, and range; any permitted S5 contact is inside `d_stop`, is an expected trigger for its pinned branch, is recorded, and latches STOP. Reports are labeled `demonstration`, stored under `artifacts/obstacle-course/runs/<run-id>/`, and each evidence/event/fault-event/intent/report output records its run purpose and the same frozen-input root hash. Nothing is written to Feature 001/002 result roots.

## 10. Run the test suite and Jazzy compatibility check

On the Lyrical/Gazebo runtime:

```bash
python3 -m pytest \
  ros2_ws/src/car_decision/test \
  tools/tests/test_dynamics_measurement.py \
  tools/tests/test_vehicle_geometry.py \
  tools/tests/test_obstacle_course_sizing.py \
  tools/tests/test_obstacle_course_world.py \
  tools/tests/test_obstacle_course_inputs.py \
  tools/tests/test_real_car_mapping.py \
  tools/tests/test_protected_paths.py \
  tools/tests/test_scenario_overlap_snapshot.py -q
```

Expected: decision-rule, schema/config, planner-intent, dynamics, world-bound, protected-input, and dashboard tests pass. Headless integration checks are run separately per scenario by the scenario runner.

In the ROS 2 Jazzy build job, build and unit-test the feature package using the Jazzy apt repository and common standard ROS APIs:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/src/gz-ws
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
colcon --log-base ros2_ws/log --build-base ros2_ws/build --install-base ros2_ws/install build --symlink-install --base-paths ros2_ws/src --packages-up-to car_decision
colcon --build-base ros2_ws/build --install-base ros2_ws/install test --packages-select car_decision
colcon --build-base ros2_ws/build test-result --verbose
```

Expected: `car_decision` builds and unit tests pass on Jazzy. Full Gazebo integration remains on the pinned Lyrical/Gazebo 10.5.0 runtime.

## 11. Generate and validate the real-car setup sheets

```bash
python3 tools/generate_setup_sheets.py \
  --all \
  --mapping docs/real-car-mapping/mapping.yaml \
  --output docs/real-car-mapping/setup-sheets/
python3 tools/validate_real_car_mapping.py \
  --mapping docs/real-car-mapping/mapping.yaml \
  --scenarios ros2_ws/src/car_bringup/config/obstacle_course/scenarios \
  --sheets docs/real-car-mapping/setup-sheets/
```

Expected: one setup sheet exists for every tracked scenario config. Every mapped RC value has a unit, provenance, and `scaled`, `measured`, or `placeholder` status; simulation-only measured dynamics remain `measured_sim` in the RC vehicle profile and map to RC placeholders.
