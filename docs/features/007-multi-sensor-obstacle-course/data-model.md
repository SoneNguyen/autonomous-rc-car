# Feature 007 Data Model

The schemas in `contracts/` are authoritative for serialized JSON records. YAML configs use the same field constraints after parsing.

## Coordinate and time conventions

- World axes: forward `+x`, left `+y`, up `+z`; angles are radians and distances are metres.
- `L_vehicle_m` and `W_vehicle_m` are conservative full collision/swept envelopes including wheels. `wheelbase_m` is axle-center distance and is used for `R_min_m = wheelbase_m / tan(center_steering_limit_rad)`.
- Sensor, decision, route-progress, and freshness timestamps use ROS/Gazebo simulation time. `clock_epoch` increments whenever `/clock` moves backward or resets so an old sample cannot appear fresh after reset.
- Wall-clock monotonic time is used only to measure dashboard delivery latency and process health. It is not used for vehicle dynamics or stale-data decisions.
- LiDAR evidence is transformed into `base_link` before sector/corridor calculations. Camera bearing is positive left in `base_link`. IMU axes and signs follow the sensor frame documented in the RC vehicle profile and TF.

## Entities

### Vehicle Profile

The Feature 007 `rc_vehicle_reference` overlay references `ros2_ws/src/car_bringup/config/rc_vehicle.yaml` → `presets.rc_car` and stores only Feature 007 derived/measurement fields. Its visual anchor is Feature 003's highest-quality pinned research vehicle mesh; collision, sensor, joint, and drive physics remain from `simulation/models/rc_car/model.sdf`.

| Field | Type | Meaning |
|---|---|---|
| `profile_id` | string | Stable identifier for the resolved profile. |
| `base_profile_ref` | path + source revision | Read-only Feature 003 RC profile source. |
| `preset_id` | enum | `rc_vehicle_reference` for the simulator; `real_rc_car` for physical setup mapping. |
| `geometry.wheelbase_m` | positive number | Axle-center distance. |
| `geometry.vehicle_length_m`, `vehicle_width_m` | positive numbers | Full conservative vehicle envelope, wheels included. |
| `geometry.footprint_radius_m` | positive number | Conservative circle used by the planner/clearance checks. |
| `steering.center_limit_rad` | number in `(0, pi/2)` | Effective center-wheel steering limit. |
| `derived.min_turn_radius_m` | positive number | Recomputed from wheelbase and center steering limit. |
| `dynamics.braking_deceleration_mps2` | positive number | Simulation-measured deceleration selected from the scripted trial set. |
| `dynamics.command_latency_s` | nonnegative number | Simulation-time delay between safe-controller request acceptance and measured deceleration onset. |
| `dynamics.measurement_ref` | path + SHA-256 | Raw measurement report and exact simulator/config identity. |
| `parameters[]` | list | Every source value carries a value, unit, provenance, status, and source ref. Status is `scaled`, `measured`, `measured_sim`, or `placeholder`; `measured_sim` is never a real-car measurement. |

The profile computes `d_stop_m(v) = v² / (2*a_brake) + v*t_latency`. It cannot be qualified if either measurement is absent, nonpositive where required, or associated with a different profile/config hash.

### Component Evidence

One normalized record per component sample. Evidence producers publish versioned JSON via `std_msgs/String` and expose their component status even when a source is stale or unavailable.

| Common field | Meaning |
|---|---|
| `schema_version` | Contract version, initially `1.0`. |
| `component` | `lidar`, `camera`, `imu`, `odometry`, or `contact`. |
| `scenario_id`, `run_id` | Active scenario and isolated run. |
| `run_purpose` | `demonstration`, `development`, or `diagnostic`; demonstration records require the verified frozen-input root. |
| `sequence` | Monotonic per-component counter within a run. |
| `sim_time_ns`, `clock_epoch` | Sample/reference simulation time and reset epoch. |
| `frame_id` | Source/normalized frame where applicable. |
| `freshness.state` | `fresh`, `stale`, or `unavailable`. |
| `freshness.age_s`, `timeout_s` | Simulation-time age and configured freshness bound. |
| `valid`, `invalid_reason` | Whether the observation can be used by the decision rules. |
| `source` | ROS topic, message type, and adapter/profile identity. |
| `config_sha256`, `profile_sha256` | Exact Feature 007 evidence/decision configuration and vehicle-profile bytes used to normalize this sample. |
| `frozen_inputs_root_sha256` | Present on demonstration records after the final input manifest is frozen; identifies the verified inputs. Earlier diagnostic records may omit it and retain their normal config or adapter identity metadata. |

Component payloads:

- **LiDAR**: five-sector nearest returns, route-corridor occupancy, nearest in-corridor range, invalid/ground returns rejected as applicable, usable sample count, and the active RC `LaserScan` adapter identity.
- **Camera**: production/qualification objects use HSV class `barrier`, `post`, or `gate`; each has colour score/confidence, pixel blob area, bearing in radians, apparent size in pixels, and source. A frozen Feature 001 detector is diagnostic-only and records its verified model hash and original label. Its `cone` label maps only to coarse `post`; `pedestrian` and `vehicle` are non-actionable and never receive a semantic clearance margin. It cannot label a barrier or gate or qualify a scenario.
- **IMU**: yaw rate (rad/s), lateral acceleration (m/s²), longitudinal acceleration/deceleration (m/s²), source frame, and freshness.
- **Odometry**: world pose, start-relative pose, speed (m/s), route progress (m), `d_stop_m`, and measured profile identity.
- **Contact**: contact pairs with entity/collision IDs, `ground_excluded`, allowlist match, and non-ground collision flag. Ground-only contact is not a collision.

### Scenario

An independently selectable case stored under `ros2_ws/src/car_bringup/config/obstacle_course/`. IDs are `S0`, `S1`, `S2`, `S3`, `S3b`, `S4`, `S5`, `S6-lidar-drop`, `S6-camera-drop`, and `S7`.

Each scenario includes:

- fixed integer seed and configuration hash;
- base-world reference, entry pose/speed, vehicle preset, route waypoints, and time limit;
- route waypoints are written in the vehicle-relative frame at the configured start pose and transformed into the run's `odom` frame using the captured odometry origin;
- obstacle fragment references with vehicle-relative target geometry, class/colour, dimensions, and LiDAR scan-height intersection; a late-revealed obstacle also has a distinct off-course `initial_pose`;
- optional simulation-time sensor-drop windows and route-progress-triggered reveal windows, with units, provenance, and status on all numeric timing/progress values;
- `lighting_profile=world_default`; camera qualification scenarios do not vary the world's lighting;
- expected collapsed behavior sequence and exact trigger component set;
- contact rule, run purpose (`demonstration`, `development`, or `diagnostic`), and setup-sheet reference. Public demos and the ten-repeat frozen qualification runs use `demonstration`; tuning variants use `development`, and pre-freeze probes use `diagnostic`.

`S6-lidar-drop` and `S6-camera-drop` are separate profiles for repeat counting. `S3b` is a distinct selectable gate-opening obstruction case. A repeat matches only when both the collapsed behavior sequence and trigger-component set exactly equal their expected values. Additional actual triggers are retained in the report and make the repeat a mismatch.

### Decision Event and State

The active behavior is one of `FOLLOW_ROUTE`, `CAUTION`, `AVOID_LEFT`, `AVOID_RIGHT`, `STOP`, or `RESUME`. Priority is `STOP > AVOID > CAUTION > FOLLOW_ROUTE`. `STOP` is latched until configured path-clear and dwell conditions pass. `RESUME` records regained lateral clearance before `FOLLOW_ROUTE`.

Every decision event has `event_kind=transition` or `event_kind=evaluation`. A transition changes the behavior and carries the transition attribution. The decision node also emits an evaluation event whenever veto/overruled attribution changes without a behavior transition; it records the unchanged active behavior and non-null `candidate_behavior` that was vetoed/overruled. Evaluation events preserve such cases as a clear S3 gate veto in the dashboard and run log. The scored collapsed behavior sequence starts with the initial `/decision/state` behavior and appends the `behavior` of each transition event in order; evaluation events do not alter it. The scored trigger set is the union of transition-event trigger components only; evaluation veto/overrule evidence never counts as a trigger. Both event kinds contain:

- event/scenario/run IDs, run purpose, and simulation timestamp/clock epoch;
- previous and active behavior, event kind, optional candidate behavior, and rule ID;
- trigger, supporting, vetoing, overruled, and stale component lists;
- evidence values, units, freshness, and reason used by the rule;
- optional speed cap, selected avoidance side, class margin, and gate return range;
- decision-config hash, trigger-matrix hash, optional frozen-input root hash, deterministic sequence number, and `published_monotonic_ns` from the same host OS clock used by the dashboard. This monotonic value is delivery metadata, excluded from deterministic replay comparison and scoring. Every demonstration event includes the verified final root; diagnostic events before freeze may omit it.

The behavior dashboard and run logger consume the same `Decision Event`; neither reconstructs attribution independently.

### Scenario Fault Event

The tracked fault gate publishes a versioned record for each S6 sensor-drop start/end and each S5 obstacle reveal. Records contain run/scenario identity, fixed seed, run purpose, event ID/type, simulation timestamp, affected component or obstacle entity, route progress and range when applicable, config hash, and the frozen-input root on demonstration runs. The run logger copies these events into each repeat report so the S5 rule can verify whether the obstacle was revealed inside `d_stop`. Before freezing, S5 selects one deterministic contact branch from measured stopping feasibility and its tracked reveal geometry. If contact is expected, the configured expected trigger set includes `contact`; if stopping is possible, contact is forbidden and excluded. The scorer never changes expected triggers based on the observed outcome.

### Decision Configuration and Trigger Matrix

`decision.yaml` is the single source for behavior thresholds, camera source/HSV ranges/minimum blob area, class clearance margins, gate scan streak/hysteresis, sensor timeouts, dwell values, and planner-intent geometry. Class margins are configured as `class_clearance_vehicle_width_multiple.*`; runtime derives each `class_margin_m = W * configured_multiple` from the measured/anchored vehicle width. They are not literal meter values in configuration. Every numeric config parameter has an explicit `parameter_units` entry and a separate `parameter_provenance` entry. Distance rules use configured multiples of profile-derived `d_stop_m`; they do not copy braking/latency numbers into scenario files. `trigger_matrix.yaml` is the single source for component/behavior/role/rule attribution and is loaded by both the decision and dashboard.

The same config carries `planner_intent.goal_lookahead_vehicle_lengths` and `planner_intent.side_offset_vehicle_widths`. The planner-intent adapter loads route waypoints from the active tracked scenario selected by `scenario_id`, transforms them from the vehicle-relative start frame into the configured `odom` frame using the captured initial pose, tracks the waypoint index using `/odom`, and constructs the next subgoal at the configured lookahead. It shifts that subgoal by the configured signed vehicle-width offset for `left` or `right`, and follows the unshifted route for `none`. The controller-request adapter converts the target to bounded Ackermann speed/steering and rejects expired intents or offsets whose implied turn violates the profile's `R_min`. Each numeric config value is paired with its unit and a provenance string in `parameter_provenance`; unit-bearing field names do not replace provenance.

The trigger matrix is a list of `(component, behavior, role, rule_ids)` rows. `role` is `trigger`, `support`, or `veto`. The state machine uses this table to authorize those three event roles, and the dashboard reads the same file to label cards. `overruled` is a separate evidence annotation, not a matrix role; tests validate its `overruled_by` pair against explicit decision rules such as the gate/LiDAR override.

### Planner Intent

Short-lived Feature 007 intent emitted by the decision node for its bounded Ackermann request adapter. It includes:

- `behavior`, `preferred_side` (`left`, `right`, or `none`), `class_margin_m`, and `speed_cap_mps`;
- scenario/run IDs, run purpose, decision event ID, simulation timestamp/clock epoch, and validity deadline;
- decision-config, trigger-matrix, and vehicle-profile hashes;
- selected class and rule ID.

The Feature 007 intent adapter consumes this message only in `behavior_demo` mode. `scenario_id` selects the tracked route config; the adapter combines its route target, current `/odom` pose/progress, preferred side, and speed cap to produce a bounded `AckermannDriveStamped` on `/ackermann_cmd`. The existing safe-drive controller validates it and remains the sole Gazebo command publisher. `STOP` produces a zero speed cap. The intent is not a drive command and is rejected when expired, from a prior clock epoch, or inconsistent with the scenario/profile.

The world contains the collisionless model entity `gazebo_truck_visual`, which uses the pinned Feature 003 vehicle mesh because the protected RC model has no chassis visual. Since `SetEntityPose` expects world coordinates, `vehicle_visual_overlay.py` captures the initial odometry pose and configured world spawn pose, then applies `T_world_base(t) = T_world_base(0) · inverse(T_odom_base(0)) · T_odom_base(t)` before updating only this companion through `/world/obstacle_course/set_pose`; it never changes the RC entity or its physics.

### Run Report

One append-only report per run ID, written beneath `artifacts/obstacle-course/runs/<run-id>/`. Required fields include scenario/profile/decision-config/trigger-matrix/world hashes, seed, environment identity, run purpose, expected and observed collapsed behavior sequences, expected and actual triggers, per-event attribution refs, contact outcome, status, and pass criteria. Every demonstration evidence, decision, fault, intent, and report record carries the same verified frozen-input root. Its run-purpose field makes this conditional requirement independently schema-checkable. Pre-freeze diagnostic and development records may omit it, are never qualified, and are not counted toward demonstration thresholds.

The comparison passes a repeat only when the collapsed observed sequence and actual trigger-component set exactly equal the expected sequence and trigger set. Each repeat records `behavior_sequence_matches` and `trigger_set_matches`; the latter is true only for exact set equality. Any extra or missing trigger is recorded and fails that repeat's trigger match. Contact and special fault rules are scored separately. A quick or partial run cannot mark the full scenario qualified. Reports are never stored under frozen historical evaluation roots.

### Real-Car Mapping and Setup Sheet

`docs/real-car-mapping/mapping.yaml` maps each simulation RC vehicle preset key to its physical-car equivalent and records unit, source/provenance, status (`scaled`, `measured`, or `placeholder`), and derivation. Simulation-measured dynamics are labeled `measured_sim` in vehicle reports and map to physical-car placeholders until measured. Generated sheets give scenario-specific object class, dimensions, HSV starting ranges, spacing, scan-height relation, entry speed, and vehicle-relative placement.

## State and validation rules

1. Evidence age is computed only within its `clock_epoch`. A clock reset invalidates earlier samples until new samples arrive.
2. LiDAR or odometry stale selects `STOP`; camera stale retains LiDAR-only conservative margins; IMU stale selects `CAUTION` with the configured conservative turn-speed cap and `STOP` if a safe turn cannot be established.
3. A gate opening is the corridor between posts. It is passable only if clear width is at least `vehicle_width_m + gate_margin_multiple * vehicle_width_m`. Post-edge returns are not inside-opening returns. The gate veto requires the configured number of consecutive clear scans; an in-corridor return overrides the camera and follows normal LiDAR distance rules. A narrow opening never gets a gate veto.
4. Contact is a collision only when a non-ground entity pair matches the allowlist rule. Any such contact triggers and latches `STOP`.
5. The full run comparison uses ten seeded repeats per configuration. Repeat 1 uses the scenario's tracked base seed; repeat `n` uses `base_seed + n - 1`, and every seed is recorded. At least eight must exactly match both the expected collapsed sequence and trigger set; the S6 LiDAR-dropout STOP requirement is ten of ten. Passing S0-S4 including S3b has no non-ground contact. S5 contact is allowed only for an obstacle first revealed inside `d_stop`, must be recorded and latch STOP, and is forbidden when the configured physics says stopping is possible.
6. The course-bound check independently derives every scenario minimum route length from the vehicle profile and scenario layout, applies the 25% margin, and requires every route to be below both its derived bound and 280 m.
7. The world command topic has exactly one ROS publisher: the existing safe-drive controller. Any additional publisher is a launch failure and must stop the run.
