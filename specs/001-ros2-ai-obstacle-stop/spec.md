# Feature Specification: ROS 2 Sensor Integration and AI-Assisted Obstacle Stop

**Feature Branch**: `001-ros2-ai-obstacle-stop`

**Created**: 2026-10-19

**Status**: Draft

**Input**: User description: Integrate ROS 2 with the existing Gazebo obstacle-course simulation, then incrementally build and evaluate ROS control, a lidar stopping baseline, camera-based obstacle detection, and a paired comparison.

## Clarifications

### Session 2026-10-19

- Q: Must the live comparison scenarios also be held out from all training and threshold tuning? → A: Maintain three disjoint scenario sets: training scenarios for data collection, validation scenarios for model selection and detection-threshold choice, and ten evaluation scenarios frozen before training. Define disjointness by the full scenario-parameter tuple (start pose, obstacle placement, and speed), not merely by run or seed.
- Q: Should the AI-assisted evaluation require zero obstacle collisions, in addition to stopping before contact in at least 8 of 10 runs? → A: Require zero obstacle collisions across all ten AI-assisted evaluation runs.

### Session 2026-10-20

- Q: Should P1 require 90% of the camera's 30 Hz SDF target on the tested WSL2 host? → A: Keep 30 Hz as the configured Gazebo source target, but set the P1 ROS-delivery floor to 20 Hz average over one simulated minute in server-only headless mode; keep the 90%-of-declared-rate requirement for point cloud, IMU, and odometry. Rationale: measured camera delivery was about 22-25 Hz headless and 14-16 Hz with WSLg open, and the planned live detector processes at 10 Hz. GUI mode remains for visual inspection, not rate acceptance.
- Q: How should P2 preserve the protected model while preventing its dormant keyboard-trigger plugins from competing with ROS? → A: The user authorized a temporary runtime-filtered model copy for autonomous launches. The copy removes exactly the five `TriggeredPublisher` command plugins; the tracked `simulation/models/car/` and `simulation/worlds/` remain unchanged. GUI sensor inspection continues to use the original model. The launch fails closed if the expected plugin count changes.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Inspect vehicle sensors through ROS 2 (Priority: P1)

As a developer, I can launch the obstacle-course simulation and inspect camera,
multi-layer lidar, IMU, odometry, and body-contact data from a separate ROS 2
process. IMU means inertial measurement unit, which reports vehicle motion and
rotation.

**Why this priority**: The autonomy stack cannot be built or debugged until the
simulation data is available outside Gazebo.

**Independent Test**: Follow the fresh-WSL setup and sensor-bridge launch and
inspection commands in the feature quickstart. Expected: all five sensor
interfaces are available within five wall-clock minutes; camera delivery
averages at least 20 Hz in headless mode over one simulated minute, while the
point cloud, IMU, and odometry each deliver at least 90% of declared samples;
timestamps advance with simulation time. The contact topic is
available and delivers an allowlisted obstacle-contact event during a
controlled test; idle contact samples are not required.

**Acceptance Scenarios**:

1. **Given** a fresh supported WSL setup, **When** I run the single documented
   sensor-stack launch command, **Then** all five sensor interfaces (camera,
   lidar, IMU, odometry, and contact topic) are available to ROS 2 within five
   wall-clock minutes.
2. **Given** the simulation is running, **When** I inspect periodic streams
   over one simulated minute, **Then** the camera delivers at least 20 Hz on
   average in headless mode, point cloud, IMU, and odometry each deliver at
   least 90% of declared samples, and timestamps follow simulation time; an
   induced non-ground obstacle contact is delivered on the contact
   topic.

---

### User Story 2 - Drive the vehicle from ROS 2 (Priority: P2)

As a developer, I can send a slow forward-and-turn command through ROS 2 and see
the vehicle respond, without Gazebo keyboard input competing for control.

**Why this priority**: The baseline and later AI-assisted behavior need a safe,
observable ROS control path.

**Independent Test**: Run the ROS command and command-timeout checks in the
quickstart. Expected: the vehicle moves forward and turns in the commanded
direction; when commands stop, a zero command is issued within 0.5 simulated
seconds and the vehicle comes to rest within two simulated seconds after the
timeout.

**Acceptance Scenarios**:

1. **Given** the autonomous stack is running with one active command authority,
   **When** I send a bounded slow forward-and-turn command, **Then** the vehicle
   moves and turns in the requested direction.
2. **Given** a valid command was being sent, **When** commands stop arriving,
   **Then** the command expires after 0.5 simulated seconds, a zero command is
   issued, and the vehicle comes to rest within two simulated seconds after
   expiry.

---

### User Story 3 - Stop for obstacles using lidar (Priority: P3)

As a developer, I can run a deterministic lidar baseline that drives toward an
obstacle and stops before contact in the existing obstacle course.

**Why this priority**: This gives the project a simple safety baseline and a
fallback before adding a learned detector.

**Independent Test**: Run the documented lidar-baseline procedure for the ten
fixed scenario runs. Expected: the vehicle stops before obstacle contact in at
least nine runs, and no obstacle collision is recorded in the acceptance set.

**Acceptance Scenarios**:

1. **Given** a checked-in scenario configuration and a headless autonomous run,
   **When** the lidar baseline approaches the obstacle, **Then** it commands a
   stop before the vehicle contacts the obstacle.
2. **Given** the ten fixed-seed baseline runs, **When** results are summarized,
   **Then** at least nine runs stop before contact, all ten runs have verified
   collision coverage, and no run records an obstacle collision.

---

### User Story 4 - Add camera-based obstacle detection (Priority: P4)

As a developer, I can create a labeled image dataset, train and score a
camera-based `too close` / `clear` detector, then use its output as an advisory
input to the existing drive controller.

**Why this priority**: The detector is meaningful only after the sensor and
control paths and a deterministic lidar fallback exist.

**Independent Test**: Run the dataset, training, validation, frozen-evaluation,
live-inference, and stale-output checks documented in the quickstart. Expected:
the labeling rule and three scenario-disjoint sets are recorded; evaluation
recall for `too close` is at least 90%; live detections can request a stop; and
missing or stale model output leaves the lidar baseline in control.

**Acceptance Scenarios**:

1. **Given** the training, validation, and ten frozen evaluation scenario
   sets, **When** labeled images are collected, **Then** training images come
   only from training scenarios; validation scenarios are used for model
   selection and detection-threshold choice; and evaluation scenarios are not
   used for training or tuning. No two sets contain scenarios with the same
   full parameter tuple (start pose, obstacle placement, and commanded speed);
   changing only a run or seed does not make scenarios disjoint. Every label
   follows the documented stopping-distance rule.
2. **Given** the training and validation scenarios and the untouched evaluation
   scenarios, **When** the model is evaluated, **Then** the report records the
   evaluation sample count, selected decision threshold, and `too close` recall
   of at least 90%, without feeding evaluation results back into tuning.
3. **Given** live camera inference and the drive controller, **When** the model
   reports `too close`, **Then** the controller may request a stop but remains
   the sole drive-command authority; a `clear` result cannot override a lidar
   stop.
4. **Given** model output is missing or stale, **When** the controller evaluates
   the run, **Then** it falls back to the deterministic lidar baseline.

---

### User Story 5 - Compare baseline and AI-assisted stopping (Priority: P5)

As a developer, I can compare lidar-only and AI-assisted stopping using the
same fixed scenarios and review a repeatable results table.

**Why this priority**: The comparison tests whether the camera detector adds
useful evidence without hiding the baseline's safety behavior.

**Independent Test**: Follow the README evaluation procedure for the same ten
scenario configurations in both modes. Expected: the results table contains a
paired row for each mode and scenario, reports all required outcomes and
distances, and can be regenerated from the documented WSL setup.

**Acceptance Scenarios**:

1. **Given** ten checked-in evaluation scenario configurations frozen before
   training and disjoint by full parameter tuple (start pose, obstacle
   placement, and commanded speed) from training and validation scenarios,
   **When** I run both modes with matching start pose, obstacle placement,
   speed, and seed, **Then** each pair has comparable recorded results.
2. **Given** the paired AI-assisted runs, **When** outcomes are summarized,
   **Then** at least eight of ten runs stop before obstacle contact, all ten
   runs have verified collision coverage, and no run records an obstacle
   collision. The report includes detection, measured stopping travel, and
   collision outcomes.
3. **Given** a teammate with a fresh WSL install, **When** they follow the root
   README, **Then** they can reproduce the complete evaluation and produce the
   same results-table structure from the repository.

---

### Edge Cases

- A sensor or model stream is missing, invalid, or stale; the vehicle stops or
  falls back to the lidar baseline, and the event is recorded.
- The simulation is paused or reset mid-run; no stale command or model result
  may cause movement after resume. Reset aborts and preserves the current run
  record and starts a new record with a new run ID; it never silently replaces
  an evaluation attempt.
- Gazebo keyboard control or another command source is active; an autonomous
  run does not start with multiple command authorities and aborts if another
  command-topic writer appears. If a competing writer appears during a run,
  abort the attempt, persist it as `aborted`, and stop the simulation.
  Autonomous runs use headless mode or disable keyboard control.
- An obstacle is already inside the configured stop-trigger distance; the
  controller requests zero on its next control tick and records whether
  contact occurred.
- Ground contact is reported by contact sensing; it is excluded from obstacle
  collision counts.
- An evaluation obstacle placement was not represented in training; tag and
  report that subgroup separately. Lighting stays at the protected world's
  default. If a run's lighting differs from that default, retain the planned
  row as `aborted` with `completion_reason=lighting_mismatch`; do not claim
  lighting generalization.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The feature MUST make camera, multi-layer lidar, IMU, odometry,
  and body-contact data available to a separate ROS 2 process.
- **FR-002**: Each exposed stream MUST retain simulation-time timestamps and
  document its message meaning, update rate, units, and coordinate frame where
  applicable. Existing Gazebo topic names in the root README remain the
  simulation-side source of truth. ROS-side names and message types MUST be
  recorded in the feature contracts and bridge configuration.
- **FR-003**: A single documented launch command MUST start the obstacle-course
  simulation and the selected ROS integration/control mode from the supported
  WSL2 environment.
- **FR-004**: Every run MUST use simulation time and a checked-in scenario
  configuration recording start pose, obstacle placement, commanded speed, and
  a fixed random seed. The run record MUST identify that configuration and seed.
- **FR-005**: An autonomous run MUST have exactly one ROS node
  (`safe_drive_controller`) publish to the bridged drive-command topic, and
  exactly one Gazebo Transport writer (`ros_gz_bridge`) for the corresponding
  Gazebo topic. The run MUST refuse to start or abort if a competing writer is
  detected. Perception and AI components MUST NOT publish drive commands.
  Autonomous runs MUST be headless or have Gazebo keyboard control disabled.
- **FR-006**: The ROS control path MUST accept bounded slow forward-and-turn
  commands. If valid commands stop arriving, it MUST issue a zero command no
  later than 0.5 simulated seconds after the last valid command and bring the
  vehicle to rest within two simulated seconds after expiry.
- **FR-007**: The lidar baseline MUST request a stop when an obstacle enters
  its configured stopping region, and MUST record whether the vehicle stopped
  before obstacle contact.
- **FR-008**: The image dataset MUST document its labeling rule. An image is
  `too_close` when a non-ground obstacle in the forward driving corridor is at
  or inside the configured `stop_distance_m` trigger threshold; it is `clear`
  otherwise. This trigger threshold is distinct from measured stopping travel.
- **FR-009**: The dataset MUST use three scenario sets: training scenarios for
  data collection, validation scenarios for model selection and detection-
  threshold choice, and ten evaluation scenarios frozen before training.
  These sets MUST NOT contain scenarios with an identical full parameter tuple
  (start pose, obstacle placement, and commanded speed); changing only a run or
  seed MUST NOT make scenarios disjoint. Evaluation scenarios and their
  outcomes MUST NOT be used to tune the controller, feature pipeline, model,
  detection threshold, or scenario parameters. Their permitted read-only uses
  are P3 baseline outcome recording, P4 detector scoring and live-inference
  verification, and P5 paired comparison. Model reports MUST include sample
  count, decision threshold, and `too close` recall measured on untouched
  evaluation data.
- **FR-010**: The vision model MUST provide only perception results. The single
  controller retains final command authority; a `too close` result may request
  a stop, and a `clear` result MUST NOT override a lidar stop.
- **FR-011**: Missing, invalid, or stale sensor, command, or model data MUST
  cause a safe stop or deterministic lidar-baseline fallback. Model output is
  stale after three expected update periods without a fresh result.
- **FR-012**: Evaluation MUST run both behaviors on the same scenario
  configurations, start poses, obstacle placements, speeds, and seeds.
- **FR-013**: Each evaluation record MUST include behavior, scenario and seed,
  detection, pre-contact-stop outcome, measured stopping travel, trial status,
  and collision status. `collision_status` MUST be `collision` only when the
  contact sensor reports a pair naming the scenario's allowlisted obstacle
  collision; ground contact MUST be excluded. It MUST be `no_collision` only
  when collision coverage for the relevant vehicle contact surfaces is
  verified and no such event occurred; otherwise it MUST be `unknown`. An
  unknown result cannot satisfy a zero-collision success criterion.
- **FR-014**: The root README and feature quickstart MUST provide fresh-WSL
  setup, launch, test, and expected-result instructions. Evaluation write-ups
  MUST be stored under `docs/`; generated data and model weights MUST stay in
  git-ignored `artifacts/` with a regeneration script or note.
- **FR-015**: The feature MUST NOT change the vehicle model or existing world.
  It MUST leave the Windows-native Pixi environment intact and use Gazebo Sim
  10 (Jetty) unless an explicit future spec authorizes otherwise.

### Key Entities *(include if data is involved)*

- **Scenario configuration**: A named repeatable run definition containing
  world reference, start pose, obstacle placement, commanded speed, and seed.
- **Sensor stream**: A camera, lidar, IMU, odometry, or contact signal with a
  documented ROS-side name/type, timestamp, rate, units, and frame as applicable.
- **ROS 2**: The robotics middleware through which separate software processes
  receive sensor data and send control requests.
- **Lidar**: A distance sensor that measures ranges to nearby surfaces and
  obstacles.
- **IMU (inertial measurement unit)**: A sensor that reports vehicle motion and
  rotation.
- **Control session**: A run with one command authority, a selected behavior,
  command-freshness state, and a stop outcome.
- **Obstacle label**: `too close` or `clear`, assigned by the documented
  forward-corridor and stopping-distance rule.
- **Dataset split**: Training, validation, and evaluation membership is
  assigned by the full scenario-parameter tuple (start pose, obstacle
  placement, and commanded speed); a tuple cannot appear in more than one set.
  Validation scenarios are used for model selection and threshold choice; the
  ten evaluation scenarios are frozen before training. Their permitted
  read-only uses are P3 baseline outcome recording, P4 detector scoring and
  live-inference verification, and P5 paired comparison; no evaluation samples
  or outcomes are used for tuning. Run or seed changes alone do not make
  scenarios disjoint.
- **Evaluation record**: The per-run behavior, scenario, seed, detection,
  pre-contact stop, measured stopping travel, trial status, collision status,
  and coverage status. `collision_status` is `collision`, `no_collision`, or
  `unknown`; ground contacts are never obstacle collisions.
- **Simulation time**: The Gazebo-controlled clock used by run logic and sensor
  timestamps; it pauses with the simulation.
- **Stop trigger distance**: The configured obstacle-to-front-plane threshold
  (`stop_distance_m`) that triggers a stop and defines the image-label boundary.
- **Measured stopping travel**: Odometry path length from the first final
  zero-command output until rest, reported in meters as `stopping_distance_m`.
- **Trial completion**: A trial is `completed` when the vehicle reaches the
  configured rest condition (speed below the rest threshold for the settling
  interval) and its run record is flushed. This is an execution status, not a
  successful-stop result. Expiry of the scenario's simulation-time limit before
  that point is `timed_out`; reset, shutdown, a competing command writer, or a
  runtime lighting mismatch before that point is `aborted`. Each retains its
  specific `completion_reason`, remains in the audit data, and counts as a
  failed planned trial, never as a dropped sample. Reset starts a fresh run ID.
  Retries cannot replace or erase the original attempt.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: From a fresh supported WSL setup, all five sensor interfaces
  (camera, lidar, IMU, odometry, and contact topic) are available in ROS 2
  within five wall-clock minutes. Over one simulated minute in server-only
  headless mode, the camera averages at least 20 ROS-delivered frames per
  second; point cloud, IMU, and odometry each produce at least 90% of their
  declared samples with simulation-time timestamps. The Gazebo camera source
  remains configured for 30 Hz. The contact topic
  is available and delivers a named allowlisted obstacle event during a
  controlled contact test; idle periodic contact samples are not required.
- **SC-002**: After command loss, a zero command is issued within 0.5 simulated
  seconds and the vehicle reaches rest within two simulated seconds after
  expiry.
- **SC-003**: The lidar baseline stops before obstacle contact in at least 9
  of 10 fixed-seed runs, all ten have verified collision coverage, and all ten
  have `collision_status=no_collision`.
- **SC-004**: The camera detector achieves at least 90% recall for `too close`
  on images from the ten frozen evaluation scenarios, whose full parameter
  tuples do not appear in training or validation. The read-only evaluation set
  MUST contain at least 100 valid `too_close` and 100 valid `clear` images in
  total, with at least five images of each class from every evaluation scenario;
  otherwise SC-004 fails for insufficient support. Sample counts and threshold
  are recorded, and evaluation results are not used for tuning.
- **SC-005**: AI-assisted behavior stops before obstacle contact in at least 8
  of the same 10 scenarios used for lidar-baseline comparison; all ten have
  verified collision coverage and `collision_status=no_collision`.
- **SC-006**: The evaluation procedure produces one results table containing
  one planned record for each of ten scenarios and each behavior (20 planned
  rows), including `trial_status` and `collision_status`; an interrupted run
  is retained and cannot be silently replaced. A teammate can reproduce the
  procedure by following the root README from a fresh WSL install.

## Assumptions

- The AI assignment is camera-based binary obstacle detection (`too close` vs
  `clear`). If the assignment specifies another task, revise Story 4 and its
  dependent requirements before planning that story.
- The initial command-loss targets are 0.5 simulated seconds to issue zero and
  two simulated seconds to reach rest. These are prototype acceptance targets,
  not physical-vehicle safety limits.
- The configured stopping distance and relevant sensor rates will be pinned in
  the plan and research after checking the existing model, Gazebo behavior,
  and ROS bridge.
  Gazebo topic names in the root README remain authoritative; ROS-side names and
  types will be selected in the plan's contracts and bridge configuration.
- Scenario files will reference the existing obstacle-course world and live
  outside protected vehicle/world directories. Package versions, apt source,
  and supported WSL setup will be pinned in `research.md`.
- Results apply only to the current truck-scale prototype with starter physics;
  they do not predict passenger-car or real-road performance.
- Out of scope: steering or full-course navigation, reinforcement learning,
  real hardware, tracked vehicle-model or world-file changes, physics calibration, the
  simple car-demo world, and multi-vehicle scenarios.
