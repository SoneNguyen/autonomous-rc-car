# RC obstacle course: implementation and status

This document describes the working multi-sensor vehicle demo, its behavior rules, run inputs, asset sources, and verification status. Fresh setup and commands are in the [Quickstart](quickstart.md). The supported OS is Ubuntu 26.04, either native or under WSL2. The recorded dynamics measurements were collected under WSL2; native Ubuntu uses the same ROS/Gazebo packages and source configuration but has not had an independent GUI run recorded here.

## What runs

A condensed 28 m Gazebo road hosts ten fixed-seed scenarios and one development showcase. A generated world places the current scenario's objects on the road. The existing RC vehicle SDF remains the physics and sensor source; a pinned research pickup mesh follows the same truck pose as a visual body. Objects use detailed meshes for rendering and separate simple collision shapes for repeatable sensor and contact checks.

ROS 2 nodes normalize LiDAR, camera, IMU, odometry, and contact measurements into timestamped, freshness-tagged evidence. A config-driven decision node attributes each state change to the evidence that caused it. Planner intent passes through the existing safe-drive controller, which remains the only Gazebo command authority. A localhost-only dashboard displays evidence and history; it has no vehicle command path.

## Behavior rules

The state priority is STOP, then AVOID, then CAUTION, then FOLLOW_ROUTE. RESUME appears during recovery from a detour.

| Input | Role |
|---|---|
| LiDAR | Detects the route corridor and five scan sectors; can trigger CAUTION, AVOID, STOP, and veto RESUME. For a gate, post-edge returns are outside the opening corridor. A return inside the opening overrides the camera's gate classification and is handled as a hazard. |
| Camera | HSV color classes identify red barriers, orange posts, and blue gates. A class sets clearance margin and a clear gate can veto avoidance; camera alone cannot trigger AVOID or STOP. |
| IMU | Limits speed through high lateral acceleration and can trigger CAUTION on abnormal acceleration. |
| Odometry | Supplies speed, route progress, vehicle pose, and stopping distance. Stale critical pose or speed causes STOP. |
| Contact | Ground-only contact is excluded. A non-ground collision is recorded and latches STOP. |

STOP outranks avoidance. The configured stopping distance is d_stop(v)=v^2/(2*a_brake)+v*t_latency. The current 10-trial simulation brake probe measured conservative deceleration of 0.3999 m/s^2 and command latency of 0.141 s. At the 0.35 m/s entry speed, that gives d_stop=0.20252 m and a minimum turning radius of 0.55894 m. These are Gazebo measurements and model-derived geometry, not RC hardware claims.

Production and development decision configs are separate. The development barrier threshold is widened to show a clear CAUTION phase before a detour; those development values are not silently promoted into the demonstration configuration. The tested development route currently produces FOLLOW_ROUTE → CAUTION → AVOID_LEFT → RESUME → FOLLOW_ROUTE with LiDAR and odometry as the triggers.

## Scenario catalogue

All scenario files are in ros2_ws/src/car_bringup/config/obstacle_course/scenarios/. Geometry uses the vehicle-relative frame: forward is +x, left is +y. Each file has a fixed seed, expected behavior sequence, trigger set, contact rule, and dimensions with units and provenance.

| Scenario | Demonstration work case | Expected behavior |
|---|---|---|
| S0 | Clear route | FOLLOW_ROUTE |
| S1 | Single red lane blocker | FOLLOW_ROUTE → CAUTION → AVOID_LEFT → RESUME → FOLLOW_ROUTE |
| S2 | Alternating posts | FOLLOW_ROUTE → CAUTION → AVOID_LEFT → RESUME → AVOID_RIGHT → RESUME → FOLLOW_ROUTE |
| S3 | Clear gate opening | FOLLOW_ROUTE |
| S3b | Object blocks the gate opening; LiDAR overrules the camera | FOLLOW_ROUTE → CAUTION → STOP |
| S4 | Both detours blocked | FOLLOW_ROUTE → STOP |
| S5 | Obstacle revealed inside stopping distance | FOLLOW_ROUTE → STOP; contact is allowed only on the configured unavoidable branch and must latch |
| S6-lidar-drop | Critical LiDAR dropout | FOLLOW_ROUTE → STOP |
| S6-camera-drop | Camera dropout with LiDAR available | FOLLOW_ROUTE → CAUTION → AVOID_LEFT → RESUME → FOLLOW_ROUTE |
| S7 | IMU-limited turn | FOLLOW_ROUTE → CAUTION → AVOID_LEFT → RESUME → FOLLOW_ROUTE |

The demonstration threshold is at least 8/10 behavior and trigger matches per scenario. Contact requirements are separate: zero non-ground contact in S0–S4; in S5, contact is permitted only after the inside-stopping-distance reveal and must latch STOP; LiDAR dropout in S6 must stop in 10/10 repeats. A quick or single run is only a smoke check.

## Camera preview and dashboard

The dashboard stays on a stable page while values update. Selecting Camera mounts one persistent canvas, with classifications in adjacent rows. Preview image polling runs independently at 125 ms; evidence text updates at 500 ms. HSV classification runs on alternate 15 Hz source frames to limit CPU load while retaining full-resolution input. Each processed frame produces a 360 px wide JPEG at up to 7.5 Hz. Repeated JPEGs return HTTP 304.

The flicker/freeze bug occurred when a new run reset the server's preview sequence to zero while an already-open browser retained the previous run's larger sequence. Its sequence request then waited for the new counter to catch up. The client now resets its sequence when the run ID changes, clears the previous image, and ignores late responses from an old tab or run. Camera requests have a two-second timeout so a stalled request cannot block later frames; transient failures keep the last frame visible and show a reconnect status. The dashboard binds to loopback and accepts only GET requests.

## Visual assets

The checked-in lock files pin each downloaded mesh by source revision and SHA-256. Files are fetched into ignored artifacts/obstacle-course/assets/ and verified before use.

| Visual | Source | License | Use |
|---|---|---|---|
| Controlled pickup | Innerscene Free Parts Library | CC0-1.0 | High-detail visual attached to the existing RC truck; no collision geometry |
| Reflective cone | OpenGameArt, MilkAndBanana | CC0-1.0 | Orange post visual; collision uses a cylinder |
| Compact SUV | Innerscene Free Parts Library | CC0-1.0 | Vehicle obstacle visual; collision uses a box |
| Red road barrier | Innerscene Free Parts Library | CC0-1.0 | Course barrier visual; collision uses a box |
| Security grille gate | Innerscene Free Parts Library | CC0-1.0 | Gate visual; collision uses simple geometry |

The pickup, cone, and SUV descriptors are retained research models. The barrier and gate use separate higher-detail visual sources because their shapes match the course obstacles. The gate source is graphite/bronze rather than blue, so its visual appearance does not demonstrate the blue HSV class. S3 and S3b camera-class acceptance remains unverified until a blue-compatible high-detail gate asset is pinned; the classifier is not retuned to hide that mismatch.

## Real-car setup mapping

ros2_ws/src/car_bringup/config/obstacle_course/real_car_mapping.yaml tags vehicle constants as scaled, measured, or placeholder and records units and provenance. The same scenario geometry can guide a physical layout after measuring the actual vehicle's length, width, wheelbase, turning radius, braking, and latency. The current setup sheets are generated by tools/generate_setup_sheets.py. No physical RC run is claimed.

The following setup sheets are generated from the scenario files and mapping configuration.

<!-- BEGIN GENERATED RC SETUP SHEETS -->

<a id="setup-s0"></a>
### S0 — real-car setup

Obstacle course demonstration geometry. Place objects using the vehicle-relative course frame. This is a simulator-to-hardware setup guide, not a claim that the physical car was tested.

#### Vehicle and route

- Entry speed: 0.35 m/s.
- Route length: 0.85 m.
- Route frame: `vehicle_relative`; forward is +x and left is +y.
- Measure the physical car and scale the layout to its verified wheelbase, width, and turning radius.

#### Obstacles

- None; use the clear route as the baseline.
#### Expected demonstration

- Behavior sequence: `FOLLOW_ROUTE`.
- Expected triggers: `none`.
- Contact rule: `no_non_ground_contact`.
- HSV values are simulator classifier references; they are not physical camera thresholds.

<a id="setup-s1"></a>
### S1 — real-car setup

Obstacle course demonstration geometry. Place objects using the vehicle-relative course frame. This is a simulator-to-hardware setup guide, not a claim that the physical car was tested.

#### Vehicle and route

- Entry speed: 0.35 m/s.
- Route length: 1.8 m.
- Route frame: `vehicle_relative`; forward is +x and left is +y.
- Measure the physical car and scale the layout to its verified wheelbase, width, and turning radius.

#### Obstacles

##### barrier_01 (barrier)

- Color: red.
- Vehicle-relative center: x=0.75 m, y=0 m, yaw=0 rad.
- Size: length=0.5511 m, width=0.2167 m, height=0.1331 m.
- Single-plane LiDAR: object intersects the scan plane.

#### Expected demonstration

- Behavior sequence: `FOLLOW_ROUTE → CAUTION → AVOID_LEFT → RESUME → FOLLOW_ROUTE`.
- Expected triggers: `lidar, camera`.
- Contact rule: `no_non_ground_contact`.
- HSV values are simulator classifier references; they are not physical camera thresholds.
- Camera class `barrier` HSV range 1: [0, 110, 75] to [12, 255, 255].
- Camera class `barrier` HSV range 2: [168, 110, 75] to [179, 255, 255].

<a id="setup-s2"></a>
### S2 — real-car setup

Obstacle course demonstration geometry. Place objects using the vehicle-relative course frame. This is a simulator-to-hardware setup guide, not a claim that the physical car was tested.

#### Vehicle and route

- Entry speed: 0.35 m/s.
- Route length: 2.6 m.
- Route frame: `vehicle_relative`; forward is +x and left is +y.
- Measure the physical car and scale the layout to its verified wheelbase, width, and turning radius.

#### Obstacles

##### post_01 (post)

- Color: orange.
- Vehicle-relative center: x=0.65 m, y=-0.24 m, yaw=0 rad.
- Size: length=0.1068 m, width=0.1068 m, height=0.14 m.
- Single-plane LiDAR: object intersects the scan plane.

##### post_02 (post)

- Color: orange.
- Vehicle-relative center: x=1.55 m, y=0.24 m, yaw=0 rad.
- Size: length=0.1068 m, width=0.1068 m, height=0.14 m.
- Single-plane LiDAR: object intersects the scan plane.

#### Expected demonstration

- Behavior sequence: `FOLLOW_ROUTE → CAUTION → AVOID_LEFT → RESUME → AVOID_RIGHT → RESUME → FOLLOW_ROUTE`.
- Expected triggers: `lidar, camera`.
- Contact rule: `no_non_ground_contact`.
- HSV values are simulator classifier references; they are not physical camera thresholds.
- Camera class `post` HSV range 1: [7, 120, 90] to [24, 255, 255].

<a id="setup-s3"></a>
### S3 — real-car setup

Obstacle course demonstration geometry. Place objects using the vehicle-relative course frame. This is a simulator-to-hardware setup guide, not a claim that the physical car was tested.

#### Vehicle and route

- Entry speed: 0.35 m/s.
- Route length: 0.85 m.
- Route frame: `vehicle_relative`; forward is +x and left is +y.
- Measure the physical car and scale the layout to its verified wheelbase, width, and turning radius.

#### Obstacles

##### gate_left (gate)

- Color: blue.
- Vehicle-relative center: x=0.65 m, y=-0.34 m, yaw=0 rad.
- Size: length=0.27 m, width=0.036 m, height=0.54 m.
- Single-plane LiDAR: object intersects the scan plane.

##### gate_right (gate)

- Color: blue.
- Vehicle-relative center: x=0.65 m, y=0.34 m, yaw=0 rad.
- Size: length=0.27 m, width=0.036 m, height=0.54 m.
- Single-plane LiDAR: object intersects the scan plane.

#### Expected demonstration

- Behavior sequence: `FOLLOW_ROUTE`.
- Expected triggers: `none`.
- Contact rule: `no_non_ground_contact`.
- HSV values are simulator classifier references; they are not physical camera thresholds.
- Camera class `gate` HSV range 1: [92, 90, 65] to [132, 255, 255].

<a id="setup-s3b"></a>
### S3b — real-car setup

Obstacle course demonstration geometry. Place objects using the vehicle-relative course frame. This is a simulator-to-hardware setup guide, not a claim that the physical car was tested.

#### Vehicle and route

- Entry speed: 0.35 m/s.
- Route length: 1.8 m.
- Route frame: `vehicle_relative`; forward is +x and left is +y.
- Measure the physical car and scale the layout to its verified wheelbase, width, and turning radius.

#### Obstacles

##### gate_left (gate)

- Color: blue.
- Vehicle-relative center: x=0.65 m, y=-0.34 m, yaw=0 rad.
- Size: length=0.27 m, width=0.036 m, height=0.54 m.
- Single-plane LiDAR: object intersects the scan plane.

##### gate_right (gate)

- Color: blue.
- Vehicle-relative center: x=0.65 m, y=0.34 m, yaw=0 rad.
- Size: length=0.27 m, width=0.036 m, height=0.54 m.
- Single-plane LiDAR: object intersects the scan plane.

##### opening_blocker (barrier)

- Color: red.
- Vehicle-relative center: x=0.65 m, y=0 m, yaw=0 rad.
- Size: length=0.5511 m, width=0.2167 m, height=0.1331 m.
- Single-plane LiDAR: object intersects the scan plane.

#### Expected demonstration

- Behavior sequence: `FOLLOW_ROUTE → CAUTION → STOP`.
- Expected triggers: `lidar`.
- Contact rule: `no_non_ground_contact`.
- HSV values are simulator classifier references; they are not physical camera thresholds.
- Camera class `barrier` HSV range 1: [0, 110, 75] to [12, 255, 255].
- Camera class `barrier` HSV range 2: [168, 110, 75] to [179, 255, 255].
- Camera class `gate` HSV range 1: [92, 90, 65] to [132, 255, 255].

<a id="setup-s4"></a>
### S4 — real-car setup

Obstacle course demonstration geometry. Place objects using the vehicle-relative course frame. This is a simulator-to-hardware setup guide, not a claim that the physical car was tested.

#### Vehicle and route

- Entry speed: 0.35 m/s.
- Route length: 0.85 m.
- Route frame: `vehicle_relative`; forward is +x and left is +y.
- Measure the physical car and scale the layout to its verified wheelbase, width, and turning radius.

#### Obstacles

##### barrier_left (barrier)

- Color: red.
- Vehicle-relative center: x=0.48 m, y=-0.15 m, yaw=0 rad.
- Size: length=0.5511 m, width=0.2167 m, height=0.1331 m.
- Single-plane LiDAR: object intersects the scan plane.

##### barrier_right (barrier)

- Color: red.
- Vehicle-relative center: x=0.48 m, y=0.15 m, yaw=0 rad.
- Size: length=0.5511 m, width=0.2167 m, height=0.1331 m.
- Single-plane LiDAR: object intersects the scan plane.

#### Expected demonstration

- Behavior sequence: `FOLLOW_ROUTE → STOP`.
- Expected triggers: `lidar`.
- Contact rule: `no_non_ground_contact`.
- HSV values are simulator classifier references; they are not physical camera thresholds.
- Camera class `barrier` HSV range 1: [0, 110, 75] to [12, 255, 255].
- Camera class `barrier` HSV range 2: [168, 110, 75] to [179, 255, 255].

<a id="setup-s5"></a>
### S5 — real-car setup

Obstacle course demonstration geometry. Place objects using the vehicle-relative course frame. This is a simulator-to-hardware setup guide, not a claim that the physical car was tested.

#### Vehicle and route

- Entry speed: 0.35 m/s.
- Route length: 0.85 m.
- Route frame: `vehicle_relative`; forward is +x and left is +y.
- Measure the physical car and scale the layout to its verified wheelbase, width, and turning radius.

#### Obstacles

##### late_barrier (barrier)

- Color: red.
- Vehicle-relative center: x=0.9 m, y=0 m, yaw=0 rad.
- Size: length=0.5511 m, width=0.2167 m, height=0.1331 m.
- Single-plane LiDAR: object intersects the scan plane.
- Hidden staging pose before reveal: x=0.5 m, y=3 m.

#### Expected demonstration

- Behavior sequence: `FOLLOW_ROUTE → STOP`.
- Expected triggers: `lidar, contact`.
- Contact rule: `contact_expected_and_latched`.
- HSV values are simulator classifier references; they are not physical camera thresholds.
- Camera class `barrier` HSV range 1: [0, 110, 75] to [12, 255, 255].
- Camera class `barrier` HSV range 2: [168, 110, 75] to [179, 255, 255].

<a id="setup-s6-lidar-drop"></a>
### S6-lidar-drop — real-car setup

Obstacle course demonstration geometry. Place objects using the vehicle-relative course frame. This is a simulator-to-hardware setup guide, not a claim that the physical car was tested.

#### Vehicle and route

- Entry speed: 0.35 m/s.
- Route length: 0.85 m.
- Route frame: `vehicle_relative`; forward is +x and left is +y.
- Measure the physical car and scale the layout to its verified wheelbase, width, and turning radius.

#### Obstacles

- None; use the clear route as the baseline.
#### Expected demonstration

- Behavior sequence: `FOLLOW_ROUTE → STOP`.
- Expected triggers: `lidar`.
- Contact rule: `no_non_ground_contact`.
- HSV values are simulator classifier references; they are not physical camera thresholds.

<a id="setup-s6-camera-drop"></a>
### S6-camera-drop — real-car setup

Obstacle course demonstration geometry. Place objects using the vehicle-relative course frame. This is a simulator-to-hardware setup guide, not a claim that the physical car was tested.

#### Vehicle and route

- Entry speed: 0.35 m/s.
- Route length: 1.8 m.
- Route frame: `vehicle_relative`; forward is +x and left is +y.
- Measure the physical car and scale the layout to its verified wheelbase, width, and turning radius.

#### Obstacles

##### barrier_01 (barrier)

- Color: red.
- Vehicle-relative center: x=0.75 m, y=0 m, yaw=0 rad.
- Size: length=0.5511 m, width=0.2167 m, height=0.1331 m.
- Single-plane LiDAR: object intersects the scan plane.

#### Expected demonstration

- Behavior sequence: `FOLLOW_ROUTE → CAUTION → AVOID_LEFT → RESUME → FOLLOW_ROUTE`.
- Expected triggers: `lidar`.
- Contact rule: `no_non_ground_contact`.
- HSV values are simulator classifier references; they are not physical camera thresholds.
- Camera class `barrier` HSV range 1: [0, 110, 75] to [12, 255, 255].
- Camera class `barrier` HSV range 2: [168, 110, 75] to [179, 255, 255].

<a id="setup-s7"></a>
### S7 — real-car setup

Obstacle course demonstration geometry. Place objects using the vehicle-relative course frame. This is a simulator-to-hardware setup guide, not a claim that the physical car was tested.

#### Vehicle and route

- Entry speed: 0.35 m/s.
- Route length: 1.8 m.
- Route frame: `vehicle_relative`; forward is +x and left is +y.
- Measure the physical car and scale the layout to its verified wheelbase, width, and turning radius.

#### Obstacles

##### barrier_01 (barrier)

- Color: red.
- Vehicle-relative center: x=0.75 m, y=0 m, yaw=0 rad.
- Size: length=0.5511 m, width=0.2167 m, height=0.1331 m.
- Single-plane LiDAR: object intersects the scan plane.

#### Expected demonstration

- Behavior sequence: `FOLLOW_ROUTE → CAUTION → AVOID_LEFT → RESUME → FOLLOW_ROUTE`.
- Expected triggers: `lidar, imu`.
- Contact rule: `no_non_ground_contact`.
- HSV values are simulator classifier references; they are not physical camera thresholds.
- Camera class `barrier` HSV range 1: [0, 110, 75] to [12, 255, 255].
- Camera class `barrier` HSV range 2: [168, 110, 75] to [179, 255, 255].
<!-- END GENERATED RC SETUP SHEETS -->

## Main implementation paths

- ros2_ws/src/car_decision/car_decision/: evidence adapters, decision engine, planner adapter, run logger, visualization overlay, and dashboard.
- ros2_ws/src/car_bringup/config/obstacle_course/: vehicle geometry, decision and trigger configuration, scenario files, asset locks, and runtime schemas.
- ros2_ws/src/car_bringup/launch/obstacle_course.launch.py: composes the scenario world, ROS/Gazebo bridge, nodes, and optional GUI.
- simulation/worlds/obstacle_condensed.sdf and simulation/worlds/obstacle_course/fragments/: the course and detailed obstacle visuals.
- tools/run_obstacle_course.py: fixed-seed smoke and repeated-run orchestrator.
- tools/freeze_obstacle_course_inputs.py and tools/verify_obstacle_course_inputs.py: reproducible-input manifest creation and verification.

Simulation reports, generated worlds, and manifests live under ignored artifacts/obstacle-course/. The schemas shipped with the source are in ros2_ws/src/car_bringup/config/obstacle_course/contracts/.

## Verification status

The fixed-seed GUI development run completed the 14.94 m route with the expected behavior order, the expected LiDAR/odometry trigger set, and no non-ground contact. A live preview sample returned 12 monotonically advancing JPEG sequence numbers over 18 seconds. Camera evidence was stale at run shutdown, so this confirms preview delivery but does not qualify the camera component. The post-cleanup colcon build completed all four requested packages and the full Python suite passed 123 tests; after the later camera request-timeout change, the focused dashboard suite passed 11 tests. Runtime configuration, course sizing, real-car mapping, setup-sheet, and all three asset-lock validations also passed locally.

This does not qualify the full scenario matrix. The ten-repeat demonstration runs, S5 contact branch, all-scenario collision rules, and S6 10/10 dropout requirement still need their full runs. See the [Quickstart](quickstart.md) for the commands.
