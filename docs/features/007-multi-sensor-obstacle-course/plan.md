# Implementation Plan: Multi-Sensor Obstacle Course and Behavior Dashboard

**Branch**: No Feature 007 branch created; plan prepared on `publish/rc-sensor-actuation-matrix`
**Date**: 2026-11-01
**Spec**: [spec.md](spec.md)

**Input**: Feature 007 specification and the implementation-planning constraints supplied with `$speckit-plan`.

## Summary

Add a compact RC-scale course and S0-S7/S3b scenario set around the unchanged Feature 003 RC physics anchor, using only the highest-quality pinned Feature 003 research visuals available. The new world's internal name is `obstacle_course`; an additive bridge config maps its world-scoped contact topic without changing the existing bridge. Since the protected RC model has no chassis visual, the world includes a collisionless research-mesh companion whose pose is synchronized from `/odom` through Gazebo's `SetEntityPose` service. Five evidence producers normalize LiDAR, camera, IMU, odometry, and contact data. A deterministic rule-based decision node emits attributed behavior events and motion intent; the current safe-drive controller remains the sole publisher to Gazebo. A loopback-only, read-only dashboard consumes the same event stream. Feature-owned scenario generation, reports, mapping, and tests use new paths and leave prior frozen artifacts untouched.

The working checkout already contains the RC-scale vehicle, current safe-drive controller, and Feature 003-derived research model set. Do not restore the removed Feature 001 large vehicle or planner code. Add only opt-in Feature 007 integration wiring and new course/scenario files. The Feature 003 vehicle, research meshes, asset lock, existing worlds, and prior evaluation artifacts remain read-only.

## Technical Context

**Language/Version**: Python 3 using the ROS distribution's system Python; this machine uses Python 3.14.4. Core logic must also build and unit-test on ROS 2 Jazzy.

**Primary Dependencies**: `rclpy`; standard ROS messages (`sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `vision_msgs`, `ackermann_msgs`, `ros_gz_interfaces`); PyYAML, JSON Schema, OpenCV/cv_bridge, pytest, and Python standard-library `http.server`. Install ROS and simulation packages from apt only.

**Storage**: Checked-in Feature 007 YAML configs and JSON Schemas; ignored generated worlds and per-run records under `artifacts/obstacle-course/`; concise tracked summaries under `docs/evaluation/obstacle-course/`.

**Testing**: Pure rule and adapter unit tests with pytest; JSON Schema/config validation; scenario geometry and protected-input checks; Gazebo SDF validation; headless S0-S7/S3b integrations on the pinned runtime; Jazzy build and unit-test CI without Gazebo integration.

**Target Platform**: WSL2 Ubuntu 26.04, ROS 2 Lyrical, Gazebo Sim 10.5.0 at runtime. ROS 2 Jazzy is the secondary build/unit-test target. Runtime assets and apt dependencies are local; no network is required during a run.

**Project Type**: ROS 2 ament_python package plus Gazebo world/scenario generator, CLI runner, mapping documents, and a small local web dashboard.

**Performance Goals**: Decision and motion-intent updates stay within the existing controller cycle. The dashboard emits each decision within 200 ms. Evidence ages and timeouts are simulation-time config values; wall-time is recorded only for dashboard delivery latency.

**Constraints**: Do not modify `simulation/models/rc_car/`, `simulation/models/research/`, the research asset lock, existing worlds, frozen split scenarios, model weights, manifests, registries, or prior results. Add one world file, new scenario fragments/configs, a new `car_decision` package, and a new `behavior_demo` mode in the existing launch wrapper. Do not modify Pixi or add Lyrical-only ROS APIs. Only the safe-drive controller may publish `/model/gazebo_truck/cmd_vel`. The decision node publishes bounded motion intent to the controller request interface, never to Gazebo. All records use simulation time and include config hashes.

**Scale/Scope**: Five normalized sensor evidence streams; six behavior states; eight named cases S0-S7 plus S3b; two S6 fault profiles; ten seeded repeats per configuration from a fixed base-seed schedule; one base world and generated scenario variants; one local read-only dashboard; simulation-to-real-car mapping and setup sheets.

## Constitution Check (before research)

| Principle | Status | Evidence / plan rule |
|---|---|---|
| I. Reproducible Simulation | PASS | Every scenario stores seed, start/obstacle geometry, entry speed, fault windows, and expected sequence. Use ROS simulation time and hash configs/worlds in reports. |
| II. Existing Assets Are Stable | PASS | Keep the Feature 003 RC model, Feature 003-derived pinned research assets, asset lock, pre-existing worlds, and historical frozen data unchanged. Add only new Feature 007 paths and opt-in launch wiring. No Pixi changes. |
| III. Single Control Authority, Safe Fallback | PASS | Decision and UI never publish drive commands. The existing safe-drive controller alone publishes Gazebo Twist; stale LiDAR/odometry, contact, and invalid command inputs take the defined stop path. Autonomous runs are headless. |
| IV. Incremental, Verified Delivery | PASS | Implement stories in priority order, with an independent runnable test and checkpoint before each later story. Report only tests actually run. |
| V. Evidence-Based Claims | PASS | Braking and latency are measured in simulation and labeled `measured (sim)`. RC values remain scaled/placeholder until physically measured. Demo runs do not enter frozen evaluation metrics. |
| VI. Simplicity | PASS | One feature-owned Python package, standard messages/JSON contracts, no custom ROS message package, and a standard-library read-only dashboard. Added launch/intent wiring is opt-in. |

No pre-research constitution deviations are required. The requested new launch mode is an additive integration path; existing launch defaults remain unchanged.

## Phase 0: Research Decisions

The observed source paths, baseline commit IDs, geometry values, current-checkout gaps, topic shapes, measurement status, and alternatives are recorded in [research.md](research.md). In particular:

- Use Feature 003's pinned high-quality RC research meshes (`vehicle_compact_suv_01`, cones, pedestrians, and other selected Feature 003 assets) over the unchanged `simulation/models/rc_car/model.sdf` physics model.
- Derive the RC vehicle envelope from chassis and wheel extents; use `R_min = wheelbase / tan(effective_center_steering_angle)`.
- Measure `a_brake` and `t_latency` through the active safe-drive-controller Ackermann input path; never use placeholder values as measured evidence.
- Normalize the active RC `LaserScan` stream into the common LiDAR evidence contract.
- Preserve the controller command gate by publishing bounded `AckermannDriveStamped` requests to `/ackermann_cmd`. The safe-drive controller remains the only Gazebo command publisher.
- Keep Feature 007 scenarios, generated worlds, and report outputs separate from the frozen Feature 001/002 roots; use a byte-for-byte Feature007-owned snapshot of the pinned Feature 002 split-overlap checker read-only as a preflight. Its source revision and hash are recorded in `canonical-paths.md`; never modify the historical source.

There are no unresolved product-policy questions. Numeric braking and latency values are deliberately produced by the implementation measurement run, rather than invented during planning.

## Phase 1: Design

### Architecture and data flow

1. **Scenario selection and world generation**: `tools/run_obstacle_course.py` selects a Feature 007 registry entry and fixed seed. `tools/generate_obstacle_course_world.py` combines the tracked `simulation/worlds/obstacle_condensed.sdf` with scenario-only obstacle fragments, writes `artifacts/obstacle-course/worlds/<scenario-id>.sdf`, and validates it before launch. Direct `behavior_demo` launch uses this same generator when no generated-world path is supplied, preventing a launch from silently using the empty base world.
2. **Sensor evidence and visuals**: `car_decision` starts one normalizer per component. The LiDAR adapter consumes the active RC `LaserScan`; production and qualification camera evidence uses configured HSV classes. `/detections` is selectable only for explicitly diagnostic, unqualified runs after verifying the frozen model hash; its labels `cone`, `pedestrian`, and `vehicle` are preserved, with only coarse `cone -> post` mapping and no barrier/gate inference. IMU, odometry, and contact normalizers publish timestamped records with freshness and validity. Odometry records spawn-relative pose/progress. Since the protected RC model has no chassis visual, a collisionless Feature003 mesh companion is added to the new world. `vehicle_visual_overlay.py` captures the initial odometry pose and configured world spawn pose, transforms later `/odom` poses into world coordinates, and updates only the companion through `/world/obstacle_course/set_pose`; no low-poly visual fallback is permitted.
3. **Decision and motion intent**: `decision_node` consumes normalized evidence and versioned config. It implements priority, hysteresis, dwell, gate corridor, stale fallback, and attribution rules. It publishes `/decision/state`, `/decision/event`, and `/decision/planner_intent` as versioned JSON in `std_msgs/String`. Scenario waypoints are defined relative to the vehicle at the configured start pose; the waypoint follower transforms them into the configured `odom` frame using the captured odometry origin, advances a waypoint index from `/odom`, and applies the intent's preferred side and speed cap before converting the resulting target to bounded `AckermannDriveStamped` requests on `/ackermann_cmd`. Manual `Twist` input is converted to the same Ackermann interface.
4. **Profile and command authority**: The `rc_vehicle_reference` profile overlays measured simulation dynamics and Feature 007 thresholds on the active `presets.rc_car` geometry. In P1 manual mode, `manual_twist_adapter.py` converts keyboard Twist input to bounded Ackermann requests; in P3 autonomous mode, `controller_request_adapter.py` converts route/behavior targets to the same request type. Launch enables exactly one adapter at a time. The existing safe-drive controller validates freshness and limits in `rc_navigation` mode, then publishes the only Gazebo command. The opt-in `behavior_demo` mode defaults to server-only/headless autonomous operation; no direct Gazebo keyboard publisher runs alongside it.
5. **Logging and dashboard**: A new Feature 007 run logger records scenario/config/world hashes, seed, transition and attribution-changing evaluation events, sensor evidence, contact result, actual/expected triggers, and run purpose under a unique run ID. Qualification outputs are labeled `demonstration`; diagnostics and tuning runs retain their `diagnostic` and `development` labels. It reuses the existing ground exclusion and contact-allowlist semantics without writing to the historical result directories. The separate dashboard process consumes state/event/evidence topics and exposes only GET `/` and GET `/events` (SSE) on loopback; loss of that process cannot affect the controller. Decision events carry same-host monotonic publish timestamps; evaluation events preserve gate-veto and overrule history, while only transitions affect qualification scoring.
6. **Development and demonstration**: Tune only Feature 007 rule/config thresholds on separate development variants. Freeze the config hash before the ten demonstration repeats. The scored sequence starts with the initial `/decision/state` and appends transition events; evaluation events are retained for attribution but excluded from sequence and trigger-set scoring. The sequence and exact trigger-component set must both match; additional triggers remain in failed-run reports for diagnosis. S0-S4 including S3b require zero non-ground contact; S5 follows its conditional contact rule; S6 LiDAR dropout STOP is 10/10.

### Configuration and output layout

- `ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml`: Feature 007 profile overlay referencing `ros2_ws/src/car_bringup/config/rc_vehicle.yaml` → `presets.rc_car`; it contains the RC envelope and explicitly measured simulation dynamics.
- `ros2_ws/src/car_bringup/config/obstacle_course/decision.yaml`: behavior thresholds, class margins, sensor timeouts, scan streak, hysteresis/dwell, turn-speed limit, planner-intent lookahead/side offset, parameter units, and provenance.
- `ros2_ws/src/car_bringup/config/obstacle_course/trigger_matrix.yaml`: single source for component/behavior/role/rule attribution; both decision and dashboard read it, and its hash is part of every run identity.
- `ros2_ws/src/car_bringup/config/obstacle_course/bridge.yaml`: Feature007 topic mappings for the internal `obstacle_course` world, including contacts; the existing RC module matrix bridge stays unchanged.
- `ros2_ws/src/car_bringup/config/obstacle_course/scenarios/{S0,S1,S2,S3,S3b,S4,S5,S6-lidar-drop,S6-camera-drop,S7}.yaml` plus a separate Feature 007 `registry.yaml`; no Feature 002 manifest or registry changes.
- `simulation/worlds/obstacle_condensed.sdf` and `simulation/worlds/obstacle_course/fragments/*.sdf`: new tracked RC-scale course baseline and scenario fixtures. Existing worlds and the Feature 003 RC/research models are not edited. Visible meshes resolve from Feature 003's pinned high-quality asset inventory or `ros2_ws/src/car_bringup/config/obstacle_course/assets.lock.yaml` for separately pinned high-quality semantic additions.
- `artifacts/obstacle-course/{worlds,runs,dynamics,config}/<id>/`: ignored generated worlds, raw trial records, logs, captures, measurement evidence, and hashed runtime compatibility profiles.
- `docs/real-car-mapping/mapping.yaml` and generated setup sheets: each simulation-to-real-car mapping value records unit, provenance, and `scaled`, `measured`, or `placeholder` status. Simulation dynamics are labeled `measured_sim` and map to real-car placeholders until physically measured.
- `docs/evaluation/obstacle-course/`: tracked concise demonstration summaries only; no frozen evaluation metrics are added or overwritten.

### Planned package and tool modules

- `ros2_ws/src/car_decision/car_decision/{lidar_evidence_node.py,camera_evidence_node.py,imu_evidence_node.py,odometry_evidence_node.py,contact_evidence_node.py,decision_node.py,planner_intent_adapter.py,controller_request_adapter.py,manual_twist_adapter.py,vehicle_visual_overlay.py,fault_gate_node.py,dashboard_node.py,run_logger.py}`.
- `tools/measure_rc_vehicle_dynamics.py`, `tools/build_obstacle_course_profile.py`, `tools/generate_obstacle_course_world.py`, `tools/run_obstacle_course.py`, and `tools/generate_setup_sheets.py`.
- `car_bringup` gains only the new opt-in `behavior_demo` launch mode, its mutually exclusive `control:=autonomous|manual` option, new scenario/config install paths, the Feature007 `obstacle_course/bridge.yaml`, and RC profile/asset-path wiring. Direct launch and the run orchestrator share scenario-to-world generation. Existing launch modes, the existing bridge config, and old scenarios retain their defaults.

### Verification plan

- Unit-test every behavior rule, precedence, deterministic replay, scan-streak/hysteresis, gate corridor edge-vs-interior returns, stop-distance branches, freshness/clock epoch handling, contact filtering, course length/turn envelope, and controller-request conversion.
- Validate scenario/profile YAML against the new JSON Schemas. Require fixed seeds, units/provenance, unique scenario IDs, and a read-only overlap pass before first scenario use.
- Test trigger/support/veto roles against the trigger matrix. Validate `overruled` evidence separately against explicit decision rules and overrule pairs; it is an event annotation, not a matrix role.
- Test that SDF and compatibility-profile generator output hashes are stable for identical source/config and that the protected Feature 001/002 assets, scenarios, manifests, registries, and results match their pinned source hashes.
- Test dashboard GET-only behavior and measure decision-publish-to-SSE-emission latency against 200 ms. No request path may publish a command.
- Run one headless integration per scenario, then ten seeded repeats per configuration using the deterministic base-seed schedule. Verify all pass thresholds from the spec, label qualification outputs `demonstration`, and keep diagnostic/development outputs out of the scored set.
- On ROS 2 Jazzy, build and unit-test `car_decision` using standard shared APIs. On Lyrical/Gazebo 10.5.0, run full launch/world integration and the fresh-WSL quickstart.

### Deliverables

The plan produces [data-model.md](data-model.md), [contracts/](contracts/), and [quickstart.md](quickstart.md), in addition to `research.md` and this plan. Phase 2 `tasks.md` is intentionally left to `$speckit-tasks`.

## Constitution Check (after design)

| Principle | Status | Design evidence |
|---|---|---|
| I. Reproducible Simulation | PASS | Scenario seeds, world/config hashes, simulation timestamps, and append-only run IDs are required by scenario/report schemas. |
| II. Existing Assets Are Stable | PASS | New course is a new world file; the Feature 003 RC vehicle, research model descriptions, pinned asset lock, existing worlds, and frozen historical data remain byte-identical. Generated worlds and logs are ignored. |
| III. Single Control Authority, Safe Fallback | PASS | Only the existing safe-drive controller publishes Gazebo Twist. Decision, dashboard, planner intent, and keyboard adapter publish only requests/status. Autonomous runs are headless; stale safety-critical evidence maps to STOP. |
| IV. Incremental, Verified Delivery | PASS | Verification is organized by independently runnable stories and headless scenario cases; report only executed checks. |
| V. Evidence-Based Claims | PASS | Dynamics are explicitly simulation measurements; results are demonstration-only and excluded from frozen evaluation; RC mapping labels unmeasured hardware values as placeholders. |
| VI. Simplicity | PASS | One ament_python feature package, no custom ROS messages, no dashboard framework/CDN, and only opt-in launch/planner integration. Each added process has a directly specified role. |

No post-design constitution deviations remain.

## Project Structure

```text
docs/features/007-multi-sensor-obstacle-course/
├── plan.md
├── research.md
├── data-model.md
├── quickstart.md
└── contracts/
    ├── component-evidence.schema.json
    ├── decision-config.schema.json
    ├── decision-event.schema.json
    ├── fault-event.schema.json
    ├── planner-intent.schema.json
    ├── trigger-matrix.schema.json
    ├── scenario.schema.json
    ├── frozen-inputs.schema.json
    ├── run-report.schema.json
    ├── ros-topics.md
    └── dashboard.md

docs/real-car-mapping/mapping.yaml
docs/evaluation/obstacle-course/
simulation/worlds/obstacle_condensed.sdf
simulation/worlds/obstacle_course/fragments/
ros2_ws/src/car_decision/
ros2_ws/src/car_bringup/config/obstacle_course/
ros2_ws/src/car_bringup/launch/obstacle_course.launch.py  # additive behavior_demo mode
tools/measure_rc_vehicle_dynamics.py
tools/build_obstacle_course_profile.py
tools/generate_obstacle_course_world.py
tools/run_obstacle_course.py
tools/generate_setup_sheets.py
tools/tests/ and ros2_ws/src/car_decision/test/
artifacts/obstacle-course/  # ignored generated data
```

**Structure Decision**: Keep ROS runtime logic in the new `car_decision` package; scenario and rc-vehicle-reference configs alongside `car_bringup`; SDF inputs in `simulation`; deterministic CLI/tooling under `tools`; generated worlds and run data in ignored `artifacts`; contracts and operator documents under `docs`. Feature 001/002 frozen data remains at its existing paths.

## Complexity Tracking

No constitution exceptions are requested. The added package, launch mode, adapters, and dashboard directly implement the specified responsibilities; no new framework, custom message package, database, or external runtime service is introduced.
