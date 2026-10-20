# Implementation Plan: ROS 2 Sensor Integration and AI-Assisted Obstacle Stop

**Branch**: `001-ros2-ai-obstacle-stop`
**Date**: 2026-10-20
**Spec**: [spec.md](spec.md)

**Input**: Feature specification for staged ROS 2 integration, a lidar stopping baseline, camera classification, and paired evaluation.

## Summary

Use ROS 2 Lyrical and Gazebo Sim 10.5.0 (Jetty) in WSL2 Ubuntu 26.04, installed through apt. A bringup package will set `GZ_SIM_RESOURCE_PATH`, launch the existing obstacle world and a YAML-configured `ros_gz_bridge`; a separate Python package will contain the sole drive-command controller, camera perception node, and passive run logger. A point-cloud lidar stop is implemented and tested before the CNN is trained or enabled. Three parameter-disjoint scenario sets are checked in; the ten evaluation scenarios are frozen before training/data collection and used only for prespecified read-only P3 baseline outcome recording, P4 detector scoring/live-inference verification, and P5 paired comparison. Evaluation samples and outcomes never feed controller, feature, model, threshold, or scenario tuning.

No tracked model, world, source asset, Blender tooling, or Pixi file is changed by this planning work. The tracked world already has a chassis contact sensor. Because that sensor does not monitor wheel collisions, collision coverage is a required early test; if the planned course scenarios can produce unobserved wheel-only impacts, stop and resolve scope before claiming complete vehicle collision counts or changing protected SDF files.

## Technical Context

**Language/Version**: Python 3.14 from Ubuntu 26.04; SDF 1.10 world/model remain unchanged.  
**Primary Dependencies**: ROS 2 Lyrical, `ros_gz` 3.0.10 / `ros_gz_bridge`, Gazebo Sim 10.5.0 (Jetty), `rclpy`, `launch_testing`, `pytest`, `sensor_msgs_py`, `vision_msgs`, and Ubuntu apt `python3-torch` 2.9.1. The bridge version was amended from 3.0.11 after WSL Resolute apt verification found only 3.0.10; ROS, Ubuntu, and Gazebo pins remain unchanged.
**Storage**: YAML scenario configs and split manifest in `car_bringup`; lossless frame images plus CSV/JSONL metadata and PyTorch `state_dict` under git-ignored `artifacts/`; evaluation CSV and Markdown under `docs/evaluation/`.  
**Testing**: `colcon test` with pytest and `launch_testing`; pure unit tests for corridor filtering, freshness, arbitration, labels, and metrics; repeatable headless simulation runs for command response, timeout, and stopping.  
**Target Platform**: WSL2 Ubuntu 26.04 (Resolute), ROS 2 Lyrical, Gazebo Sim 10 (Jetty); WSLg GUI for visualization and OGRE2 EGL headless rendering for autonomous runs.  
**Project Type**: Two-package ROS 2 workspace plus separated ML scripts and run orchestration.  
**Performance Goals**: Controller tick 20 Hz; camera source 30 Hz; lidar scan/point cloud 15 Hz; IMU 100 Hz; default Ackermann odometry 50 Hz. CNN target inference 10 Hz on CPU after resizing frames to 160x120, using latest-frame processing rather than a growing queue.  
**Constraints**: All ROS nodes use sim time. Exactly one node publishes the Gazebo drive topic. Commands are bounded to 2.0 m/s and 0.15 rad/s by default; command expiry is 0.5 simulated seconds. Camera-model output is stale after three expected 10 Hz periods and can only request a stop. The existing 16-layer PointCloud2 is authoritative for obstacle geometry; the LaserScan conversion keeps only the middle vertical row.  
**Scale/Scope**: Five sequential stories; the ten frozen evaluation scenarios have a P3 read-only lidar-baseline outcome pass, P4 read-only detector scoring/live-inference verification, and ten planned P5 pairs (10 lidar-only + 10 AI-assisted records). Additional repeats/retries retain the frozen scenario parameters and seed, receive a new unique `run_id`, and are appended rather than replacing earlier records; no `run_index` field is required.

## Constitution Check

*Gate before Phase 0; checked again after Phase 1 below.*

| Principle | Status | Design evidence / gate |
|---|---|---|
| I. Reproducible Simulation | Pass | Every launch names a checked-in scenario and fixed seed; logs contain scenario ID/hash, seed, package versions, mode, sim-time clock, and outcome. Evaluation reuses identical scenario config and seed for both behaviors. |
| II. Existing Assets Are Stable | Pass with verification gate | No protected path or `pixi.toml` is changed by this plan. Use current model/world topics. Test contact coverage before final evaluation; if it requires another sensor topic, first update the spec to authorize the minimum protected-file change and record the reason. |
| III. Single Control Authority, Safe Fallback | Pass | `safe_drive_controller` is the sole ROS publisher to the bridged command topic and `ros_gz_bridge` is the sole Gazebo Transport writer; launch and evaluator refuse/abort on competing writers. Watchdogs are inside the controller, not a second zero-command publisher. Perception and logger are non-commanding. Auto/eval launch is headless. |
| IV. Incremental, Verified Delivery | Pass | The WSL Git checkout is on `001-ros2-ai-obstacle-stop` and has the configured GitHub remote. Setup, foundational, and story checkpoints each have independent tests and commits. |
| V. Evidence-Based Claims | Pass | Auto-labels come from synchronized lidar point clouds; train/validation/evaluation tuples are disjoint; the evaluation set is frozen before training and its prespecified outputs are read-only, never feeding tuning; metrics and all run records are retained. Data/weights remain ignored and regenerable. Claims are prototype-only. |
| VI. Simplicity | Pass | Only two ROS packages, one controller, one perception node, and one passive logger. A small CNN avoids pretrained downloads and extra frameworks; no custom ROS message package is needed. PyTorch and ROS 2 remain separate behind a standard message contract. |

## Phase 0: Research Decisions

Resolved technical questions and cited evidence are in [research.md](research.md). The selected platform remains Lyrical + Jetty on Ubuntu 26.04; no fallback to Windows, RoboStack, Jazzy, or Harmonic is in this plan. ROS package metadata lists Lyrical/Jetty binaries from `packages.ros.org`, and the Ubuntu WSL guide names `wsl --install Ubuntu-26.04`. The WSL runtime has now been installed and checked against the package baseline recorded in [docs/wsl-setup.md](../../docs/wsl-setup.md); P1 implementation can proceed in that WSL checkout.

## Phase 1: Design and Contracts

### Data and interfaces

- Scenario configs identify the existing world, vehicle start pose, a named existing obstacle collision plus its world placement, commanded speed, stop threshold, and fixed seed. The obstacle world itself is not randomized or edited. Uniqueness is enforced across the full canonical tuple `(start_pose, obstacle_placement, commanded_speed)`, not the seed or run ID.
- `data-model.md` defines the config, split manifest, synchronized image/point-cloud sample, label, model metadata, run, and evaluation record.
- [contracts/topics.md](contracts/topics.md) is the canonical ROS/Gazebo topic, type, direction, rate, frame, and QoS table. The implementation bridge YAML must match it exactly.
- [contracts/perception.md](contracts/perception.md) defines `/autonomy/obstacle_detection` as `vision_msgs/msg/Classification`, with `clear` and `too_close` class IDs, source-image header, and probability scores.

### Runtime architecture

1. `car_bringup/obstacle_course.launch.py` computes the repository root from the installed bringup package, sets `GZ_SIM_RESOURCE_PATH=<repo>/simulation/models`, then starts Gazebo with `--force-version 10.5.0` and the YAML bridge. A launch argument chooses GUI visualization or server-only headless rendering. GUI mode is sensor inspection only; autonomous modes reject GUI launch because the world includes keyboard publishing.
2. `safe_drive_controller` is the only ROS publisher on `/model/gazebo_truck/cmd_vel`. In P2 it accepts bounded requests on `/autonomy/cmd_vel_request`; in P3 it produces the deterministic forward request from its own lidar policy; in P4 it adds a one-way perception stop advisory. It arbitrates all requests, checks `/clock`, lidar, odometry, IMU, camera when AI mode is selected, command lease, and model freshness, publishes a safe zero when required, and never allows `clear` to suppress a lidar stop. Stale camera/model output in AI mode falls back to lidar; stale lidar, odometry, IMU, or clock stops.
3. Point-cloud points are transformed/filtered into vehicle coordinates using the sensor pose verified from the model and observed ROS `frame_id`. The corridor excludes ground-height returns and includes the vehicle collision width plus a documented margin. At 2.0 m/s, the initial `stop_distance_m` trigger is 5.0 m from the front plane; calibrate or adjust it only with a `training/stop_smoke` scenario, never from evaluation outcomes. It is a conservative test threshold, not measured stopping travel or a calibrated real-vehicle stopping distance. Freeze it and all scenario configs before any evaluation run or model training.
4. The perception node consumes 30 Hz 640x480 RGB frames, uses a latest-frame queue, resizes to 160x120, and publishes up to 10 Hz. The collection script pairs each image with the nearest non-future point cloud no more than one 15 Hz lidar period old; otherwise it records/drops the sample as unlabelable. The label boundary is the `stop_distance_m` trigger. Training uses only training scenarios; epoch/checkpoint/model selection and threshold selection use validation only. Read-only evaluation scoring requires at least 100 valid samples of each class overall and five of each class per evaluation scenario; insufficient support fails the recall criterion.
5. `run_logger` is passive: it records simulated timestamps, odometry, selected behavior detection, the stop request, chassis contact events, obstacle identity, pre-contact-stop outcome, measured stopping travel, trial status, and collision status/coverage. A collision is a reported contact pair naming the allowlisted non-ground obstacle. `no_collision` requires verified coverage; otherwise the result is `unknown`. The sensor's wheel-contact coverage is explicitly tested; do not claim counts for unobserved wheel-only contacts. Rest-and-flush means `completed`; simulation-time limit expiry means `timed_out`; reset, shutdown, competing writer, or lighting mismatch before completion means `aborted`, with the cause recorded.
6. `tools/run_evaluation.py` freezes/validates config hashes, launches each evaluation scenario once in lidar mode and once in AI mode with identical parameters and seed in server-only `-s --headless-rendering --force-version 10.5.0` mode, rejects GUI launch for autonomous modes, and verifies that the only Gazebo command-topic writer is the configured `ros_gz_bridge` path before each trial. It aborts and stops the simulation if another writer appears, preserves timeout/abort records, and writes a paired CSV and short Markdown summary to `docs/evaluation/`. It fails closed if an evaluation config hash appears in training/validation metadata or changes after the freeze manifest is generated.
7. Evaluation scenario metadata identifies `unseen_obstacle_placement` separately from the full-tuple split rule. Every scenario validates `lighting_id=obstacle_world_default`; lighting remains unchanged and is not claimed as a tested generalization condition.

### Spec alignment resolution

SC-001 now applies the 90% sample-rate check to the four periodic streams (camera, point cloud, IMU, and odometry). Body contact is event-based: P1 verifies topic availability and a delivered allowlisted obstacle-contact event rather than requiring idle messages. Do not synthesize messages or count a quiet contact topic as a failed sensor.

### Planned repository structure

```text
ros2_ws/src/car_bringup/
  launch/obstacle_course.launch.py
  config/ros_gz_bridge.yaml
  scenarios/{training,validation,evaluation}/*.yaml
  test/test_bringup.launch.py
ros2_ws/src/car_autonomy_nodes/
  car_autonomy_nodes/{controller,perception,run_logger}.py
  test/{test_controller,test_perception,test_logger}.py
ml/{collect_dataset.py,train_detector.py,evaluate_detector.py,README.md}
artifacts/                         # git-ignored images, labels, weights, manifests
tools/run_evaluation.py
docs/wsl-setup.md
docs/evaluation/{results.csv,summary.md}
specs/001-ros2-ai-obstacle-stop/   # plan, research, model, contracts, quickstart
```

`README.md` is updated during implementation to include the WSL quickstart and this layout while retaining the existing Windows Pixi path. The implementation checkout is the Git branch `001-ros2-ai-obstacle-stop` under `~/src/gz-ws`, with the configured repository remote. ROS work remains confined to that WSL Linux filesystem, not `/mnt/c`.

### Test sequence

| Story | Independent evidence required before next story |
|---|---|
| P1 | From launch, verify all five sensor interfaces and `/clock` are available within five wall-clock minutes; verify at least 90% of declared samples for the four periodic streams over one simulated minute; induce a controlled contact to verify event delivery. |
| P2 | Publish bounded request through the controller; confirm forward/turn response; stop request publisher and confirm final zero within 0.5 sim seconds and rest within 2 sim seconds. Confirm the controller is the sole ROS command-topic publisher and bridge the sole Gazebo writer. |
| P3 | Calibrate only on the training `stop_smoke` scenario, freeze config/threshold hashes, then run fixed evaluation scenarios headless without using any evaluation samples/outcomes to tune; record measured stopping travel, tri-state collision status/coverage, stale-lidar, immediate-stop, pause, and reset results. |
| P4 | Generate training/validation data only for training/model selection; tune on validation; freeze weights and classifier threshold; collect/read evaluation data in read-only mode and record held-out recall/precision/sample count; test model-stale fallback and `clear` vs lidar stop arbitration. No evaluation samples or outcomes feed any tuning. |
| P5 | Execute paired lidar/AI runs headless on the same ten frozen eval configs and seeds; verify the sole Gazebo command-topic writer before and during trials; produce the complete CSV + Markdown table with trial status, collision status/coverage, and condition subgroup; test deterministic table schema and collision filter. |

### Re-check Constitution Check after Phase 1

| Principle | Status | Evidence / remaining gate |
|---|---|---|
| I. Reproducible Simulation | Pass | Checked-in scenario configs, split manifest/hash, fixed seed, sim-time logging, and paired orchestration are defined. |
| II. Existing Assets Are Stable | Pass with explicit scope gate | No model/world/assets/Pixi edits are planned. Verify chassis-contact coverage in P1/P3. Any extra topic must first be authorized in the feature spec and documented here; never modify silently. |
| III. Single Control Authority, Safe Fallback | Pass | Controller owns final output and watchdog; inference is advisory; autonomous/evaluation mode is server-only headless; stale lidar/odom stop and stale AI falls back to lidar. |
| IV. Incremental, Verified Delivery | Pass after Git prerequisite | Story tests are independent and ordered. A WSL Git checkout must exist before implementation/commit checkpoints. |
| V. Evidence-Based Claims | Pass | Three disjoint sets, frozen ten-scenario evaluation, deterministic label rule, recorded metrics and prototype-only report are designed. |
| VI. Simplicity | Pass | Two ROS packages and one small CNN; no additional runtime framework, custom messages, or downloaded pretrained model. |

## Complexity Tracking

No constitution violations are accepted. The two-package split is required by the project layout; dataset/model code is kept out of ROS packages as required.
