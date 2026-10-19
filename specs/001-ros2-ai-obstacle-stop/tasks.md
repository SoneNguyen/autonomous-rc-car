---

description: "Implementation tasks for ROS 2 sensor integration and AI-assisted obstacle stopping"

---

# Tasks: ROS 2 Sensor Integration and AI-Assisted Obstacle Stop

**Input**: Design documents in `specs/001-ros2-ai-obstacle-stop/` (`plan.md`, `spec.md`, `research.md`, `data-model.md`, `contracts/`, and `quickstart.md`).

**Prerequisites**: The current workspace is not a Git checkout and has no remote URL. Before executing these tasks, publish/configure the repository remote and clone it into the WSL2 Linux filesystem (for example `~/src/gz-ws`). Do not do ROS work from the Windows checkout or `/mnt/c`. The pinned runtime is WSL2 Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Sim 10 (Jetty), installed with apt. If the pinned apt packages are unavailable, stop; do not substitute a different distro or Gazebo major version.

**Execution rule**: Complete Setup, Foundational, and P1 through P5 in order. Each phase ends with its listed independent test passing and a Git commit before the next phase starts. `[P]` means only that the task can be done in parallel with other `[P]` tasks in the same phase; it does not permit parallel story delivery.

**Protected paths**: Do not change `simulation/models/car/`, `simulation/worlds/`, `assets/source/`, `tools/blender/`, or `pixi.toml`. If collision-coverage testing indicates those assets must change, stop and revise the spec/plan before proceeding.

## Phase 1: Setup

**Purpose**: Document and prepare the supported WSL workflow, repository layout, and generated-file exclusions.

- [ ] T001 [P] Write the fresh-install steps, apt preflight/version checks for Ubuntu 26.04, ROS 2 Lyrical/ros_gz, and Gazebo Sim 10.5.0, scenario-backed ROS launch-wrapper commands for WSLg GUI inspection and headless rendering with `--force-version 10.5.0`, and a scenario-backed software-rendering fallback in `docs/wsl-setup.md`; document raw Gazebo flags only as the wrapper's internal mapping.
- [X] T002 [P] Verify and add only missing ignore rules for generated datasets/model weights, ROS `build/`, `install/`, `log/`, and local Python caches in `.gitignore`; preserve the existing Pixi ignore behavior. Added root/nested `.env` ignores with an `.env.example` exception before the public push.
- [ ] T003 [P] Add workspace orientation files and create the planned empty layout in `ros2_ws/src/README.md` and `ml/README.md`; do not track generated files under `artifacts/`.
- [ ] T004 Update the root `README.md` layout and quick start to distinguish the existing Windows Pixi/Gazebo path from the new WSL ROS path and link `docs/wsl-setup.md`.
- [ ] T005 In the WSL checkout, verify/install the pinned Ubuntu 26.04, ROS 2 Lyrical, `ros_gz` 3.0.11, Gazebo Sim 10.5.0, and apt Torch packages; confirm `gz sim --versions` includes the pinned version, record full apt package versions in `docs/wsl-setup.md`, and stop on an unavailable/mismatched runtime.
- [ ] T006 Setup checkpoint: run `test -f docs/wsl-setup.md && test -d ros2_ws/src && test -d ml && git check-ignore artifacts/datasets/probe.png artifacts/models/probe/weights.pt ros2_ws/build/probe && git diff --check`; expected: all path checks pass, `git check-ignore` lists each generated path, and `git diff --check` is silent. Then commit with `git add docs/wsl-setup.md .gitignore ros2_ws/src/README.md ml/README.md README.md specs/001-ros2-ai-obstacle-stop/tasks.md && git commit -m "docs: add ROS 2 WSL project setup"`.

## Phase 2: Foundational

**Purpose**: Create the two buildable ROS packages and a shared tested scenario-config foundation before story work.

- [ ] T007 [P] Scaffold the Python `car_bringup` ROS package, ament metadata, resource marker, and launch/config/test directories under `ros2_ws/src/car_bringup/`.
- [ ] T008 [P] Scaffold the Python `car_autonomy_nodes` ROS package, ament metadata, resource marker, node/test directories, and `pytest` configuration under `ros2_ws/src/car_autonomy_nodes/`.
- [ ] T009 Implement the shared YAML scenario loader and schema checks for world path, fixed seed, start pose, obstacle identity/pose, speed, stopping distance, corridor, and simulation-time timeout in `ros2_ws/src/car_bringup/car_bringup/scenario.py`, `ros2_ws/src/car_bringup/config/scenario.schema.yaml`, and `ros2_ws/src/car_bringup/test/test_scenario.py`.
- [ ] T010 Declare only the pinned runtime and test dependencies in `ros2_ws/src/car_bringup/package.xml` and `ros2_ws/src/car_autonomy_nodes/package.xml`; register `pytest`/`launch_testing` with the package test targets.
- [ ] T011 Foundational checkpoint: run `source /opt/ros/lyrical/setup.bash && colcon build --symlink-install --base-paths ros2_ws/src && source ros2_ws/install/setup.bash && colcon test --event-handlers console_direct+ && colcon test-result --verbose`; expected: both packages build, scenario-schema tests pass, and the test summary has zero failures. Then commit with `git add ros2_ws/src/car_bringup ros2_ws/src/car_autonomy_nodes specs/001-ros2-ai-obstacle-stop/tasks.md && git commit -m "build: scaffold ROS 2 workspace packages"`.

## Phase 3: User Story 1 - Inspect vehicle sensors through ROS 2 (Priority: P1)

**Goal**: Expose the camera, multi-layer lidar, IMU, odometry, contact events, and simulation clock to ROS 2 from a separately inspectable process.

**Independent Test**:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/src/gz-ws
source ros2_ws/install/setup.bash
ros2 launch car_bringup obstacle_course.launch.py mode:=sensors scenario:=training/sensor_smoke headless:=false
```

In a second sourced WSL terminal, run:

```bash
ros2 topic list
ros2 topic hz /model/gazebo_truck/sensors/front_camera/image
ros2 topic hz /model/gazebo_truck/sensors/front_lidar/scan/points
ros2 topic hz /model/gazebo_truck/sensors/imu
ros2 topic hz /model/gazebo_truck/odometry
ros2 topic echo --once /clock
ros2 topic info -v /model/gazebo_truck/sensors/bumper_contact
```

Expected: the WSLg Gazebo GUI opens; all five sensor interfaces plus `/clock` are available within five wall-clock minutes; camera is about 30 Hz, point cloud 15 Hz with multiple vertical layers, IMU 100 Hz, and odometry about 50 Hz; periodic stamps advance with `/clock`; bridge topic types/QoS match `contracts/topics.md`. Contact is event-only, not a continuous idle stream: the automated contact test or a deliberate low-speed obstacle bump must produce an obstacle collision pair.

- [ ] T012 [US1] Add a controlled non-ground obstacle-contact launch test that verifies `/model/gazebo_truck/sensors/bumper_contact` delivers the allowlisted collision pair in `ros2_ws/src/car_bringup/test/test_contact_event.py`.
- [ ] T013 [P] [US1] Add the complete contract-driven `ros_gz_bridge` YAML mapping, including command direction, camera, scan, point cloud, IMU, odometry, contact, `/clock`, and sensor/clock QoS in `ros2_ws/src/car_bringup/config/ros_gz_bridge.yaml`.
- [ ] T014 [P] [US1] Implement the GUI sensor-inspection and server-only autonomous bringup launch, set `GZ_SIM_RESOURCE_PATH` from the repository root, load the named scenario and seed, force Gazebo 10.5.0, start Gazebo before the bridge, set `use_sim_time=true`, and reject `headless:=false` for autonomous modes in `ros2_ws/src/car_bringup/launch/obstacle_course.launch.py`; add the fixed-seed sensor smoke config in `ros2_ws/src/car_bringup/config/scenarios/training/sensor_smoke.yaml`.
- [ ] T015 [P] [US1] Add bridge-contract and startup/rate tests in `ros2_ws/src/car_bringup/test/test_sensor_bridge.py`; assert all five sensor interfaces and `/clock` become available within five wall-clock minutes, at least 90% of declared camera, point-cloud, IMU, and odometry samples arrive over one simulated minute, `/clock` advances, LaserScan is inspection-only, and PointCloud2 retains all layers.
- [ ] T016 [US1] Update the P1 sensor launch instructions, expected rates, event-only contact note, WSLg/headless safety distinction, and `/clock` Gazebo topic in `specs/001-ros2-ai-obstacle-stop/quickstart.md` and the root `README.md`.
- [ ] T017 [US1] P1 checkpoint: pass the independent test above and `colcon test --packages-select car_bringup --event-handlers console_direct+`; expected: camera, lidar, IMU, odometry, contact topic, and `/clock` are available within five wall-clock minutes; four periodic streams deliver at least 90% of declared samples over one simulated minute; sim stamps follow `/clock`; and a deliberate obstacle contact is reported. Then commit with `git add ros2_ws/src/car_bringup README.md specs/001-ros2-ai-obstacle-stop/spec.md specs/001-ros2-ai-obstacle-stop/quickstart.md specs/001-ros2-ai-obstacle-stop/tasks.md && git commit -m "feat: bridge Gazebo sensors to ROS 2"`.

## Phase 4: User Story 2 - Drive the vehicle from ROS 2 (Priority: P2)

**Goal**: Accept bounded velocity requests through one controller and stop safely when commands or required data expire.

**Independent Test**:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/src/gz-ws
source ros2_ws/install/setup.bash
ros2 launch car_bringup obstacle_course.launch.py mode:=command_test scenario:=training/command_smoke headless:=true
```

In a second sourced terminal, run `ros2 topic pub --rate 10 /autonomy/cmd_vel_request geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.1}}"`, then press Ctrl+C. Expected: the vehicle moves forward and turns; `safe_drive_controller` is the only ROS publisher to the bridged Gazebo command topic, and the configured bridge is its only Gazebo transport writer; a zero command appears within 0.5 simulated seconds of request expiry and odometry reaches rest within two simulated seconds after expiry. The launch test confirms server-only mode has no GUI keyboard writer.

- [ ] T018 [US2] Implement `safe_drive_controller` as the sole ROS publisher to the bridged Gazebo command topic and subscribe to `/autonomy/cmd_vel_request` in `ros2_ws/src/car_autonomy_nodes/car_autonomy_nodes/safe_drive_controller.py`.
- [ ] T019 [US2] Add finite-value validation, configured speed/turn clamps, a 0.5-simulation-second request lease, simulation-time freshness checks for `/clock` and required sensors, and zero-command behavior on timeout/reset in `ros2_ws/src/car_autonomy_nodes/car_autonomy_nodes/safe_drive_controller.py`.
- [ ] T020 [US2] Add controller unit and launch tests for forward/turn response, command expiry/rest, sole ROS publisher on the bridged command topic, sole Gazebo transport writer, GUI-keyboard exclusion, and rejection of invalid/non-finite requests in `ros2_ws/src/car_autonomy_nodes/test/test_safe_drive_controller.py` and `ros2_ws/src/car_autonomy_nodes/test/test_command_timeout.launch.py`; add the fixed-seed command smoke config in `ros2_ws/src/car_bringup/config/scenarios/training/command_smoke.yaml`.
- [ ] T021 [US2] Update the P2 command and timeout instructions and observed limits in `specs/001-ros2-ai-obstacle-stop/quickstart.md` and `README.md`.
- [ ] T022 [US2] P2 checkpoint: pass the independent test above and `colcon test --packages-select car_autonomy_nodes --event-handlers console_direct+`; expected: motion follows the bounded request, `safe_drive_controller` is the only ROS publisher to the bridged command topic, the bridge is its only Gazebo transport writer, and timeout/rest limits pass in simulation time. Then commit with `git add ros2_ws/src/car_autonomy_nodes README.md specs/001-ros2-ai-obstacle-stop/quickstart.md specs/001-ros2-ai-obstacle-stop/tasks.md && git commit -m "feat: add bounded ROS 2 drive controller"`.

## Phase 5: User Story 3 - Stop for obstacles using lidar (Priority: P3)

**Goal**: Use the full point cloud for a deterministic forward-corridor stop, verify contact coverage, and establish the frozen scenario sets before training.

**Independent Test**:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/src/gz-ws
source ros2_ws/install/setup.bash
ros2 launch car_bringup obstacle_course.launch.py mode:=lidar scenario:=training/stop_smoke headless:=true
```

After calibrating only on this training scenario, freeze and validate the sets, then run the baseline:

```bash
python3 ml/collect_dataset.py validate-splits --freeze-evaluation
python3 tools/run_evaluation.py --split evaluation --behavior lidar --output docs/evaluation/lidar-baseline.csv
```

Expected: full `(start pose, obstacle placement, speed)` tuples are disjoint across training, validation, and exactly ten evaluation configs; evaluation hashes are frozen before training; the CSV has ten planned rows with those hashes, at least nine pre-contact stops, and `collision_status=no_collision` with verified coverage in every row. Timed-out/aborted trials remain recorded and count as failures. If contact tests expose a wheel-only coverage gap, collision status is `unknown` and the acceptance criterion fails; stop before changing protected assets.

- [ ] T023 [P] [US3] Implement PointCloud2 decoding, verified frame transform, forward-corridor filtering, and ground rejection with unit tests in `ros2_ws/src/car_autonomy_nodes/car_autonomy_nodes/lidar_corridor.py` and `ros2_ws/src/car_autonomy_nodes/test/test_lidar_corridor.py`.
- [ ] T024 [US3] Add the deterministic lidar stop policy and scenario-configured stopping threshold to `ros2_ws/src/car_autonomy_nodes/car_autonomy_nodes/safe_drive_controller.py`; lidar stop must dominate every non-stop request.
- [ ] T025 [US3] Add the fixed-seed `training/calibration` and `training/stop_smoke` YAML scenarios in `ros2_ws/src/car_bringup/config/scenarios/training/`; these are members of the training split and record the controller threshold and relevant geometry.
- [ ] T026 [US3] Add the remaining training scenarios, validation scenarios, and exactly ten evaluation YAML scenarios with fixed seeds and disjoint full parameter tuples in `ros2_ws/src/car_bringup/config/scenarios/`; do not recreate T025's smoke configs. Set every `lighting_id` to `obstacle_world_default`, and tag evaluation obstacle placements absent from training without changing the protected world.
- [ ] T027 [US3] Implement the full-tuple split validator and evaluation-freeze/hash command in `ml/collect_dataset.py`; reject duplicate tuples, changed frozen evaluation configs, non-default lighting, or anything other than ten evaluation scenarios; derive and validate unseen-obstacle-placement tags; test it in `ml/tests/test_scenario_splits.py`.
- [ ] T028 [US3] Implement passive session logging for detection source, stop request, measured stopping travel, `trial_status`, `completion_reason`, `lighting_id`, tri-state `collision_status`, `collision_coverage`, allowlisted obstacle contact pairs, ground exclusion, freshness events, condition tags, seed, and scenario hash in `ros2_ws/src/car_autonomy_nodes/car_autonomy_nodes/run_logger.py` and `ros2_ws/src/car_autonomy_nodes/test/test_run_logger.py`; test `completed` on rest-and-flush, `timed_out` on simulation-time limit, and `aborted` on reset, shutdown, competing writer, or lighting mismatch.
- [ ] T029 [US3] Implement deterministic repeated lidar-run orchestration in `tools/run_evaluation.py` and `tools/tests/test_lidar_evaluation.py`; launch each run through the scenario-backed bringup wrapper in server-only mode (which maps to `-s --headless-rendering --force-version 10.5.0`), verify the bridge is the sole Gazebo command-topic writer before and during each trial, refuse launch or abort/persist the trial and stop Gazebo if another writer appears, and summarize unseen-obstacle-placement subgroups.
- [ ] T030 [US3] Add integration tests for stale/invalid lidar, odometry, IMU, and clock; pause/resume stale-command rejection; simulation-time timeout versus reset/abort status and fresh run ID; an obstacle already inside `stop_distance_m`; pre-contact stop; and existing chassis-contact coverage in `ros2_ws/src/car_autonomy_nodes/test/test_lidar_stop.launch.py` and `ros2_ws/src/car_bringup/test/test_contact_coverage.launch.py`.
- [ ] T031 [US3] Document lidar calibration restrictions, split freezing, tri-state collision semantics/coverage, timeout handling, condition subgroups, baseline run, and expected metrics in `specs/001-ros2-ai-obstacle-stop/quickstart.md` and `docs/evaluation/README.md`.
- [ ] T032 [US3] P3 checkpoint: pass the independent test above and `colcon test --packages-select car_autonomy_nodes car_bringup --event-handlers console_direct+`; expected: split validation passes and the frozen ten-run baseline yields at least nine pre-contact stops, verified collision coverage, and `collision_status=no_collision` in all ten rows. P3 evaluation outcomes are recorded read-only and never feed tuning. Any unknown or incomplete row fails acceptance. Then commit with `git add ros2_ws/src/car_autonomy_nodes ros2_ws/src/car_bringup ml tools/run_evaluation.py tools/tests docs/evaluation specs/001-ros2-ai-obstacle-stop/spec.md specs/001-ros2-ai-obstacle-stop/quickstart.md specs/001-ros2-ai-obstacle-stop/tasks.md && git commit -m "feat: add lidar obstacle stop baseline"`.

## Phase 6: User Story 4 - Add camera-based obstacle detection (Priority: P4)

**Goal**: Collect lidar-labeled camera data, train and assess a binary detector, then use its output only as an advisory input to the existing controller.

**Independent Test**:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/src/gz-ws
source ros2_ws/install/setup.bash
python3 ml/collect_dataset.py validate-splits --require-frozen
python3 ml/collect_dataset.py collect --split training
python3 ml/collect_dataset.py collect --split validation
python3 ml/train_detector.py --train-split training --validation-split validation
python3 ml/collect_dataset.py collect --split evaluation --frozen-only
python3 ml/evaluate_detector.py --split evaluation --model artifacts/models/latest --read-only
```

Then launch live inference with `ros2 launch car_bringup obstacle_course.launch.py mode:=ai scenario:=evaluation/eval_01 headless:=true model:=artifacts/models/latest`. Expected: artifacts remain under ignored `artifacts/`; evaluation samples/outcomes are read-only and never feed controller, feature, model/checkpoint, threshold, or scenario tuning; the report records held-out sample count, threshold, precision/recall, and at least 90% `too_close` recall; live `too_close` may request a stop, `clear` cannot override lidar, and stale AI falls back to lidar while the controller remains the sole drive publisher.

- [ ] T033 [US4] Implement synchronized camera/PointCloud2 collection, the forward-corridor stopping-distance label rule, invalid-frame handling, and dataset metadata in `ml/collect_dataset.py` and `ml/tests/test_dataset_labels.py`.
- [ ] T034 [US4] Implement the small CNN training, validation-only checkpoint/threshold selection, and safe `state_dict` plus provenance metadata output in `ml/train_detector.py` and `ml/tests/test_train_detector.py`.
- [ ] T035 [US4] Implement the 10-Hz latest-frame ROS camera detector and publish the contracted `vision_msgs/msg/Classification` result with the source image header in `ros2_ws/src/car_autonomy_nodes/car_autonomy_nodes/camera_obstacle_detector.py`.
- [ ] T036 [US4] Connect perception output to the controller as stop-only advice; enforce freshness/current-run checks, reject pre-reset model output, and use lidar fallback without adding any drive-command publisher in `ros2_ws/src/car_autonomy_nodes/car_autonomy_nodes/safe_drive_controller.py`.
- [ ] T037 [US4] Add tests for label provenance, rejection of held-out scoring when either class has fewer than 100 valid samples overall or fewer than five per evaluation scenario, evaluation sample/outcome exclusion from controller, feature, model, threshold, and scenario tuning, validation-only threshold selection, live message contract, stale-model fallback, rejection of pre-reset model output, and sole command authority in `ml/tests/test_data_split_isolation.py`, `ros2_ws/src/car_autonomy_nodes/test/test_camera_obstacle_detector.py`, and `ros2_ws/src/car_autonomy_nodes/test/test_ai_fallback.launch.py`.
- [ ] T038 [US4] Add reproducible data collection, training, read-only evaluation, and live-inference steps with expected outputs to `specs/001-ros2-ai-obstacle-stop/quickstart.md` and `docs/evaluation/README.md`.
- [ ] T039 [US4] P4 checkpoint: pass the independent test above and `colcon test --packages-select car_autonomy_nodes --event-handlers console_direct+`; expected: held-out data meets both-class support quotas, evaluation recall meets 90%, artifacts are ignored, no evaluation samples/outcomes affect any tuning, and stale inference leaves lidar safety in control. Then commit with `git add ml ros2_ws/src/car_autonomy_nodes docs/evaluation specs/001-ros2-ai-obstacle-stop/quickstart.md specs/001-ros2-ai-obstacle-stop/tasks.md && git commit -m "feat: add camera obstacle detector and lidar fallback"`.

## Phase 7: User Story 5 - Compare baseline and AI-assisted stopping (Priority: P5)

**Goal**: Run both modes against the same frozen scenarios and produce a reproducible paired results table and summary.

**Independent Test**:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/src/gz-ws
source ros2_ws/install/setup.bash
python3 tools/run_evaluation.py --split evaluation --behaviors lidar,ai --model artifacts/models/latest --paired
```

Expected: `docs/evaluation/results.csv` contains one planned record for each of ten scenarios and each behavior (20 planned rows) and `docs/evaluation/summary.md` exists; each pair has matching scenario hash, seed, pose, obstacle, and speed; rows include trial status, condition tags, measured stopping travel, tri-state collision status, and coverage. An uninterrupted passing run reports 9/10 lidar and 8/10 AI pre-contact stops, with verified coverage and no collision in all rows; any timeout, abort, or unknown collision status prevents an acceptance pass. Results are scoped to the truck-scale prototype.

- [ ] T040 [US5] Extend `tools/run_evaluation.py` to run each frozen scenario once per behavior with identical config hash, seed, pose, obstacle, and speed; launch through the scenario-backed bringup wrapper in server-only mode (which maps to `-s --headless-rendering --force-version 10.5.0`), reject GUI mode, verify the bridge is the sole Gazebo command-topic writer before and during each trial, refuse launch or abort/persist the trial and stop Gazebo if another writer appears, and persist timeout/abort records in `docs/evaluation/results.csv`.
- [ ] T041 [US5] Generate the concise threshold/outcome summary, including trial status, `lighting_id`, tri-state collision coverage, and unseen-obstacle-placement subgroup results, in `tools/summarize_evaluation.py` and `docs/evaluation/summary.md`; unknown coverage or incomplete trials cannot pass zero-collision criteria.
- [ ] T042 [US5] Add pairing, row-schema, reproducibility, headless/single-authority and competing-writer rejection with simulation-stop verification, non-default-config rejection and runtime-lighting-mismatch abort persistence, ground-contact exclusion, timeout/abort, unseen-placement subgroup, tri-state collision, and acceptance-threshold tests in `tools/tests/test_paired_evaluation.py`.
- [ ] T043 [US5] Update the root `README.md` and feature `specs/001-ros2-ai-obstacle-stop/quickstart.md` with the fresh-WSL end-to-end paired command and output locations.
- [ ] T044 [US5] P5 checkpoint: pass the independent test above and `python3 -m pytest tools/tests/test_paired_evaluation.py`; expected: all ten scenario pairs validate, 20 planned rows are emitted for an uninterrupted evaluation, any retries/aborts remain visible, and the summary reports every acceptance threshold without overstating prototype results. Then commit with `git add tools docs/evaluation README.md specs/001-ros2-ai-obstacle-stop/spec.md specs/001-ros2-ai-obstacle-stop/quickstart.md specs/001-ros2-ai-obstacle-stop/tasks.md && git commit -m "feat: report paired lidar and AI evaluation"`.

## Phase 8: Polish and Cross-Cutting Verification

**Purpose**: Check the fresh-install path, docs consistency, protected assets, and the complete staged test suite.

- [ ] T045 Run the complete fresh-WSL quickstart from the root README and reconcile exact commands, topic names, package versions, and expected outputs in `README.md`, `docs/wsl-setup.md`, and `specs/001-ros2-ai-obstacle-stop/quickstart.md`.
- [ ] T046 Run `source /opt/ros/lyrical/setup.bash && colcon build --symlink-install --base-paths ros2_ws/src && source ros2_ws/install/setup.bash && colcon test --event-handlers console_direct+ && colcon test-result --verbose && python3 -m pytest ml/tests tools/tests`; expected: all ROS, launch, ML, and orchestration tests pass with zero failures.
- [ ] T047 Verify no protected simulation/assets/Pixi paths changed and no generated data/weights are tracked using `git diff -- simulation/models/car simulation/worlds assets/source tools/blender pixi.toml`, `git ls-files artifacts`, and `git status --short --ignored artifacts`; record the check in `docs/evaluation/README.md`.
- [ ] T048 Final checkpoint: run `git diff --check`; expected: no whitespace errors and the README quickstart reproduces the results table from the frozen scenario configs. Then commit with `git add README.md docs specs/001-ros2-ai-obstacle-stop/tasks.md && git commit -m "docs: verify ROS 2 obstacle evaluation workflow"`.

## Dependencies and Execution Order

### Phase Dependencies

- Setup has no code prerequisites, but execution requires a Git remote and the supported WSL checkout; it blocks all later phases.
- Foundational depends on Setup and blocks all stories.
- P1 → P2 → P3 → P4 → P5 are sequential. Each story's independent test and checkpoint commit must pass before the next story starts.
- Polish depends on all five stories and their passing commits.

### Key Cross-Story Dependencies

- P2 depends on P1's bridge, clock, and sensor inputs.
- P3 depends on P1 sensor bridges and P2's sole-authority controller; P3 calibrates only on training data and freezes all three scenario sets and ten evaluation hashes before any P4 training.
- P4 depends on P3's frozen split, lidar labels, logger, and fail-safe controller. Validation alone selects the model/threshold; evaluation stays read-only.
- P5 depends on frozen model choices and the frozen P3 scenarios; it pairs both behaviors on identical config hashes and seeds.

## Implementation Strategy

Deliver the sensor bridge first as the MVP. Then establish a command path, prove a deterministic lidar stop and freeze scenarios, and only then collect/train/evaluate the camera model. The final comparison is the last story; do not tune behavior or model thresholds from its frozen evaluation outcomes.

## Notes

- `[P]` tasks are limited to independent files within one phase; story phases remain sequential.
- All autonomous and evaluation launches must be server-only/headless with active sensor rendering; the GUI path is for inspection only.
- Contact is event-based and the existing sensor covers chassis collision only. Verify selected obstacle coverage and stop if wheel-only impacts are unobserved.
- Keep generated datasets, model weights, and transient evaluation artifacts under ignored `artifacts/`; tracked evaluation reports belong under `docs/`.
- Every reported result applies to the current starter-physics truck-scale simulation, not to real vehicles.
