# Feature Specification: Multi-Sensor Obstacle Course and Behavior Dashboard

- **Feature Branch**: Not created; specification only
- **Created**: 2026-11-01
- **Status**: Draft
**Input**: User description: Add a short obstacle course around the existing RC-scale Gazebo vehicle, rule-based decisions using all vehicle sensors, a live read-only behavior-attribution dashboard, and a real-RC scenario mapping. The vehicle and scene visuals are anchored to Feature 003's highest-quality pinned research assets.

## Clarifications

### Session 2026-11-01

- Q: What course-length cap should the acceptance criteria use? → A: Use a testable geometry-derived bound: derive each scenario's minimum feasible route length from `R_min`, `d_stop(v)`, and its layout; apply a 25% margin; the longest scenario route must remain below 280 m and within that bound. Record the derivation in `research.md` and recompute it in a test from the vehicle profile.
- Q: Should the 8-of-10 demonstration threshold apply to S5-S7? → A: Behavior-sequence and trigger-set matches must meet 8 of 10 runs for every scenario S0-S7. A trigger-set match means exact set equality; extra actual triggers are recorded and make that repeat a mismatch. Contact rules are separate: S0-S4 require zero non-ground contact; S5 permits contact only when the obstacle is revealed inside `d_stop`, and any contact must be recorded and latch STOP; S6 LiDAR dropout must produce STOP in 10 of 10 runs.
- Q: How should the decision layer resolve a camera-recognized gate when LiDAR reports a return in its opening? → A: A passable opening is the corridor between the posts and must be at least vehicle width plus the configured gate clearance margin, expressed in vehicle widths. Edge returns outside the corridor do not count. Gate veto requires no returns inside the corridor for the configured number of consecutive approach scans and is re-evaluated with hysteresis. A return inside the corridor overrules the camera; normal LiDAR distance rules apply, and attribution records the camera gate classification, LiDAR override, and return range. Stale LiDAR means STOP. Add S3b for an object in the opening, expecting CAUTION followed by LiDAR-triggered AVOID or STOP with the camera shown as overruled.

## User Scenarios & Testing

### User Story 1 - Drive the condensed scenario course (Priority: P1)

As a demonstrator, I can select a short, RC-scale scenario and drive it with the existing controller so I can show how the vehicle approaches and passes obstacles without replaying the long legacy course. The visual vehicle and scene objects use the highest-quality Feature 003 research assets available for each object class.

**Why this priority**: Every later story needs a repeatable course and selectable scenarios.

**Independent Test**: Open the new course in manual keyboard mode and traverse each selectable scenario. Confirm the vehicle fits the course, completes every route that has a feasible path, and avoids non-ground contact where avoidance is physically possible.

**Acceptance Scenarios**:

1. **Given** the new course and S0 clear-run configuration, **When** the RC vehicle follows the configured route, **Then** it completes the course without activating CAUTION, AVOID, or STOP.
2. **Given** any S0-S7 or S3b configuration, **When** a demonstrator selects it, **Then** only that scenario's configured objects and route conditions are active in the new world.
3. **Given** S1, S2, or S3 in manual keyboard mode, **When** the RC vehicle follows the course, **Then** it can pass every obstacle for which a feasible path exists without non-ground contact.
4. **Given** an existing world or vehicle model, **When** this feature is added, **Then** that file remains unchanged and the new course is provided as a separate world.
5. **Given** the vehicle profile and all scenario layouts, **When** the route-bound check runs, **Then** it independently derives the minimum feasible route lengths, applies the 25% margin, and fails if the longest configured route is at least 280 m or exceeds the derived bound.

### User Story 2 - Inspect live evidence from every component (Priority: P2)

As a demonstrator or developer, I can see fresh, normalized evidence from LiDAR, camera, IMU, odometry, and contact sensing so I can tell which parts of the vehicle are contributing to the current decision.

**Why this priority**: The decision rules must use observable, time-aligned inputs rather than hidden assumptions.

**Independent Test**: Run S0, then run S6 with the configured sensor dropouts. Confirm that each active component displays evidence with simulation time and freshness, and that the selected dropout is visibly stale.

**Acceptance Scenarios**:

1. **Given** a running scenario with all sensors active, **When** evidence is displayed, **Then** all five components provide updating records stamped in simulation time and marked fresh or stale.
2. **Given** a controlled LiDAR or camera dropout in S6, **When** the dropout begins, **Then** the corresponding evidence becomes stale within its configured timeout and the other component records continue updating.
3. **Given** a simulation pause or clock reset, **When** evidence timestamps stop advancing or enter a new clock epoch, **Then** the evidence view exposes that condition and does not present old records as fresh.

### User Story 3 - See attributed avoidance decisions (Priority: P3)

As a demonstrator, I can see the rule-based decision layer select a behavior and list the component evidence that triggered or supported it, so I can explain why the vehicle slows, avoids, stops, or resumes.

**Why this priority**: The central value is demonstrating how the sensors combine, with a traceable reason for each behavior.

**Independent Test**: Replay the same scenario input and seed twice. Compare behavior transitions and their trigger/support records; the same input must produce the same ordered decisions and evidence-derived reasons.

**Acceptance Scenarios**:

1. **Given** a route with a clear corridor, **When** the decision layer evaluates fresh evidence, **Then** it selects FOLLOW_ROUTE.
2. **Given** a LiDAR return inside the configured caution distance or excessive lateral acceleration, **When** the evidence is evaluated, **Then** CAUTION is selected and its trigger and supporting component values are recorded.
3. **Given** a blocked route inside the configured avoidance distance, **When** LiDAR confirms the blockage and a class-appropriate detour is feasible, **Then** AVOID_LEFT or AVOID_RIGHT is selected; camera classification supplies the applicable clearance margin but cannot trigger AVOID by itself.
4. **Given** a camera-recognized gate whose opening is at least the vehicle width plus its configured clearance margin, **When** no LiDAR return is inside the opening corridor for the configured number of consecutive approach scans, **Then** the gate vetoes avoidance and the RC vehicle passes through; returns on the post edges outside the corridor do not count.
5. **Given** S3b places an object inside the gate opening within the configured caution trigger range, **When** LiDAR reports a return inside the opening corridor, **Then** LiDAR overrules the camera gate classification, CAUTION is followed by AVOID or STOP under the normal LiDAR rules, and attribution records the gate classification, override, and return range.
6. **Given** a camera-recognized gate whose opening is narrower than vehicle width plus its clearance margin, **When** the decision layer evaluates the gate, **Then** the camera does not veto avoidance and any LiDAR returns on the posts remain eligible as ordinary route-hazard evidence.
7. **Given** an infeasible detour, excessive required deceleration, a critical stale component, or non-ground contact, **When** the decision is evaluated, **Then** STOP supersedes other behaviors and remains latched until the path is clear for the configured dwell time.
8. **Given** the corridor is clear and lateral clearance has been regained for the configured dwell time, **When** RESUME conditions are met, **Then** the layer records RESUME before returning to FOLLOW_ROUTE.
9. **Given** identical component evidence and scenario configuration, **When** a decision is repeated, **Then** state, direction, trigger set, and reason are identical.

### User Story 4 - Run and compare the scenario catalogue (Priority: P4)

As an evaluator, I can run each scenario independently or as a batch and compare the observed behavior sequence and trigger set with its expected catalogue so the demonstration is repeatable and discrepancies are visible.

**Why this priority**: Individual component evidence becomes meaningful only when expected behavior can be checked across repeatable runs.

**Independent Test**: Run each S0-S7 configuration, including S3b and both S6 fault profiles, headless with the deterministic repeat seed schedule. Compare the ordered behavior changes and trigger sets with the scenario's expected sequence and confirm the qualification run is labeled demonstration.

**Acceptance Scenarios**:

1. **Given** any one scenario, **When** it is selected for a headless run, **Then** its result can be checked independently without requiring another scenario to pass.
2. **Given** a batch request, **When** all configured scenarios run, **Then** the batch reports each scenario's expected and observed sequences, trigger sets, contact outcome, seed, and pass/fail result separately.
3. **Given** S0-S4, including S3b, **When** a run passes, **Then** it has no non-ground contact.
4. **Given** S4, **When** both sides are blocked, **Then** the RC vehicle selects STOP without alternating avoidance directions and stays stopped until the configured clear dwell completes.
5. **Given** S5 reveals an obstacle inside `d_stop`, **When** contact occurs, **Then** the contact is recorded and STOP latches; contact is permitted only for an obstacle revealed inside `d_stop`, and is avoided when stopping is physically possible.
6. **Given** the S6 LiDAR-dropout profile, **When** ten demonstration repeats using the deterministic seed schedule complete, **Then** STOP is selected in all ten; when camera evidence drops instead, **Then** the RC vehicle may continue using LiDAR-only margins and the camera staleness remains visible in attribution.
7. **Given** S7 exceeds the configured lateral-acceleration limit, **When** the RC vehicle enters a sharp detour, **Then** the configured turn-speed restriction is applied and the behavior and IMU evidence are recorded.
8. **Given** a P4 qualification run, **When** results are stored or summarized, **Then** the run is labeled `demonstration` and is not counted toward a frozen-set evaluation metric; pre-freeze probes and development tuning runs are labeled `diagnostic` and `development` respectively and are not qualification runs.
9. **Given** the pinned WSL runtime, apt-installed dependencies, and local assets, **When** a scenario is run with network access unavailable, **Then** it completes with the configured seed and produces its normal result record.
10. **Given** S3b reports an object inside the gate opening, **When** its run result is compared with the catalogue, **Then** CAUTION is followed by the configured LiDAR-rule branch, LiDAR is the trigger, and camera evidence is recorded as overruled.

### User Story 5 - Follow the decision dashboard live (Priority: P5)

As an observer, I can see the active behavior, its reason, component status, and recent decision history in a read-only dashboard while the RC vehicle runs, even when a component is stale.

**Why this priority**: The dashboard makes the sensor combination and decision history understandable during a live demonstration.

**Independent Test**: Run S1 and S6 while watching the dashboard. Compare its status and event history with the run log and verify an update appears within 200 ms of each decision.

**Acceptance Scenarios**:

1. **Given** an active run, **When** a component is neither triggering nor supporting the current behavior, **Then** its card reads idle and shows its latest freshness status.
2. **Given** a component contributes evidence to the current behavior without triggering a transition, **When** the dashboard updates, **Then** its card reads supporting and exposes its evidence summary.
3. **Given** a component causes a behavior transition, **When** the dashboard updates, **Then** its card reads trigger, and the active behavior, reason, and history match the run log.
4. **Given** a sensor's evidence is overruled by another component, **When** the dashboard updates, **Then** its card identifies the overruled evidence and the log preserves the reason and values.
5. **Given** a clear gate opening allows the camera to veto avoidance, **When** the dashboard updates, **Then** the camera card shows `veto` and the event history preserves the veto evidence.
6. **Given** a component becomes stale, **When** a user views the dashboard, **Then** stale status is visible and the dashboard remains usable.
7. **Given** the dashboard is unavailable, **When** a scenario runs, **Then** vehicle behavior and command handling are unchanged.

### User Story 6 - Recreate scenarios on the real RC car (Priority: P6)

As an RC-car operator, I can use a setup sheet for each scenario and map simulation values to measured or explicitly provisional vehicle values, so I can reproduce the relative geometry and expected behavior sequence on the real car.

**Why this priority**: Vehicle-relative scenarios are useful only when operators can set them up consistently and understand which values are measured versus assumed.

**Independent Test**: Review the setup sheets for S0-S7 and S3b. For each, verify that the object class, colour, size, spacing, height, entry speed, route relation, and conversion from vehicle-relative units are specified and each physical constant is marked scaled, measured, or placeholder.

**Acceptance Scenarios**:

1. **Given** any S0-S7 scenario or S3b, **When** its setup sheet is opened, **Then** it provides object size and colour, placement and spacing, relevant height for the single-plane LiDAR, entry speed, and route-relative placement.
2. **Given** any vehicle or sensor constant in the RC mapping, **When** it is used for a setup or threshold, **Then** it is marked scaled, measured, or placeholder and includes units and provenance.
3. **Given** an RC-scale scenario definition, **When** its real-car setup is derived, **Then** the scenario's vehicle-relative geometry and expected behavior sequence are preserved; equivalence is judged by behavior sequence and trigger set, not identical absolute distances.
4. **Given** a placeholder value, **When** the setup sheet is used, **Then** it is clearly identified as unvalidated and is not described as a real-car measurement.

### User Story 7 - Compare component ablations (Priority: P7, stretch)

As a developer, I can run a scenario with one component deliberately ablated and compare its behavior and attribution with the full-input run, so I can inspect the contribution of each component without changing the normal scenario result.

**Independent Test**: Select one component ablation for S1 or S6 and confirm the run is separately labeled, reports the disabled component, and does not replace the full-input result.

**Acceptance Scenarios**:

1. **Given** a full-input and an ablated run with the same scenario and seed, **When** they are compared, **Then** the reports identify the removed evidence and show both behavior sequences separately.
2. **Given** an ablation run, **When** the dashboard displays it, **Then** the ablated status is explicit and cannot be mistaken for a normal run.

### User Story 8 - Inspect frozen detector compatibility (Priority: P8, stretch)

As a developer, I can inspect the read-only Feature 001 detector's outputs against this feature's colour classes without changing scenario geometry or decision rules, so its compatibility limits are explicit.

**Independent Test**: In a diagnostic run only, select the frozen detector and verify the frozen model hash, its source labels, and the explicit class mapping in normalized camera evidence. Confirm this source cannot qualify or score an S0-S7/S3b demonstration.

**Acceptance Scenarios**:

1. **Given** the frozen Feature 001 classifier is selected for diagnostics, **When** it returns `cone`, `pedestrian`, or `vehicle`, **Then** `cone` maps only to the coarse `post` class and `pedestrian`/`vehicle` remain non-actionable; none is interpreted as `barrier` or `gate`.
2. **Given** a diagnostic detector run, **When** records are written, **Then** the frozen model hash and original source label are retained and the run is marked unqualified; production demonstration scenarios continue to use HSV classification.

### Edge Cases

- LiDAR detects an obstacle while the camera classifies the location as a gate, or camera and LiDAR evidence otherwise disagree.
- Multiple obstacles occupy the route corridor at different distances or have different classes and clearance margins.
- Evidence arrives out of timestamp order, stops advancing, or becomes stale during an avoidance.
- A clock pause or reset occurs while the vehicle is moving or STOP is latched.
- A detour alternates between feasible and infeasible because clearance fluctuates near its boundary.
- Contact is reported against ground-only geometry, or a contact event identifies multiple fixtures.
- Contact occurs while stopped or while a scenario is already latched in STOP.
- The dashboard disconnects or closes during a run.
- S5 contact is physically unavoidable at the configured entry speed; the result must distinguish this from avoidable contact.
- The RC car's measured dimensions or braking values are not yet available and therefore remain placeholders.

### Scenario Catalogue

Each row defines the expected behavior and trigger evidence. Numeric placements,
entry speeds, thresholds, and object dimensions belong in the scenario
configuration with units and provenance.

| Scenario | Case | Expected behavior sequence | Expected triggers and evidence |
|---|---|---|---|
| S0 | Clear run | FOLLOW_ROUTE only; no activations | No trigger components |
| S1 | Single lane-blocking red barrier | FOLLOW_ROUTE → CAUTION → AVOID_LEFT or AVOID_RIGHT → RESUME → FOLLOW_ROUTE; no contact | LiDAR triggers CAUTION; LiDAR plus camera class `barrier` trigger the selected AVOID; clearance and route goal select the side |
| S2 | Orange-post slalom | FOLLOW_ROUTE, alternating AVOID_LEFT and AVOID_RIGHT at successive posts, then RESUME → FOLLOW_ROUTE | LiDAR blockage plus camera class `post`; post clearance margin is smaller than the barrier margin |
| S3 | Blue gate pass-through | FOLLOW_ROUTE through the opening; no avoidance | No transition trigger components are expected. A clear-gate camera veto is retained in an `evaluation` event only when the opening is passable and the configured clear-scan count is reached; post-edge returns are excluded |
| S3b | Object in blue gate opening | FOLLOW_ROUTE → CAUTION → AVOID_LEFT or AVOID_RIGHT, or STOP when required by normal LiDAR rules | An in-corridor LiDAR return overrules camera class `gate`; LiDAR triggers the hazard response and the camera classification is marked overruled with the return range |
| S4 | Both sides blocked | CAUTION as applicable → STOP; remain STOP until clear dwell completes, then RESUME | LiDAR confirms the blocked corridor and no feasible detour; no alternating AVOID direction |
| S5 | Obstacle revealed inside stopping distance | STOP; the tracked config pins either contact-free stopping or unavoidable contact → latched STOP | LiDAR return and required-deceleration test trigger STOP; if measured feasibility predicts unavoidable contact, `contact` is an expected trigger and the event is recorded |
| S6 | Degraded sensing; separate LiDAR-dropout and camera-dropout run profiles | LiDAR dropout: STOP. Camera dropout: continue with conservative LiDAR-only margins and the behavior required by LiDAR | The stale component is identified in evidence and decision attribution; camera loss alone does not trigger STOP |
| S7 | Speed-limited sharp detour | CAUTION, AVOID_LEFT or AVOID_RIGHT at the configured turn-speed limit, RESUME, FOLLOW_ROUTE | IMU lateral acceleration triggers CAUTION and the speed limit; LiDAR and camera provide obstacle and class evidence for the detour |

## Requirements

### Functional Requirements

- **FR-001**: The feature MUST provide a new RC-scale course in a separate world file around the existing `simulation/models/rc_car/model.sdf` physics model and MUST leave every existing world and the existing vehicle model unchanged.
- **FR-002**: Every scenario route MUST be shorter than the existing 280 m course and no longer than 1.25 times the geometry-derived minimum feasible route length. The derivation MUST account for `R_min`, `d_stop(v)`, and the scenario layout, be recorded in `research.md`, and be independently recomputed by a check using the vehicle profile.
- **FR-003**: The course MUST provide separately selectable configurations S0-S7 and S3b, each with a fixed base seed, entry speed, route and obstacle geometry, expected behavior sequence, expected trigger set, and real-car setup reference. For repeat index `n`, the runner MUST use and record `base_seed + n - 1`. S6 MUST have separate LiDAR-dropout and camera-dropout run profiles.
- **FR-004**: Scenario geometry and decision thresholds MUST be expressed relative to vehicle length `L`, vehicle width `W`, minimum turning radius `R_min`, and stopping distance `d_stop(v) = v²/(2*a_brake) + v*t_latency`; every configured quantity MUST include a numeric value, unit, and source or provenance. Gate clearance margins MUST be expressed as configured multiples of `W`.
- **FR-005**: Configuration MUST identify whether each real-car value is scaled, measured, or a placeholder. Unmeasured real-car braking, latency, geometry, and sensor values MUST NOT be presented as hardware measurements.
- **FR-006**: A manual keyboard mode MUST allow course traversal through the existing controller. Manual control and autonomous scenario execution MUST not publish competing actuator commands.
- **FR-007**: The feature MUST represent LiDAR, camera, IMU, odometry, and contact observations as normalized evidence with simulation timestamps and explicit freshness. Odometry evidence MUST supply vehicle speed and pose for route-corridor and stopping-distance calculations.
- **FR-008**: The camera's production and qualification classification MUST use the configured HSV colour classes: red barrier, orange post, and blue gate. The frozen Feature 001 classifier MAY be selected only for an explicitly diagnostic, unqualified run and MUST remain read-only. Its labels are `cone`, `pedestrian`, and `vehicle`: `cone` MAY map to the coarse, non-gate `post` class; `pedestrian` and `vehicle` remain non-actionable. No frozen label may be mapped to `barrier` or `gate`, set a class-specific avoidance margin except the `post` margin for `cone`, create a gate veto, or alter scenario qualification. Detector diagnostics MUST record the verified frozen-model SHA-256 and original classifier label. This feature MUST NOT retrain, re-threshold, or modify the classifier.
- **FR-009**: The decision layer MUST implement FOLLOW_ROUTE, CAUTION, AVOID_LEFT, AVOID_RIGHT, STOP, and RESUME with priority STOP over AVOID over CAUTION over FOLLOW_ROUTE. Hysteresis and minimum dwell times MUST be configuration values.
- **FR-010**: CAUTION MUST be eligible when LiDAR reports an object in the route corridor within the configured multiple of `d_stop`, or when IMU evidence exceeds the configured lateral-acceleration or abnormal-deceleration limit. Camera evidence MAY trigger CAUTION alone, but MUST NOT trigger AVOID or STOP alone.
- **FR-011**: AVOID_LEFT or AVOID_RIGHT MUST require LiDAR evidence of a blocked route corridor within the configured avoidance multiple of `d_stop` and a feasible detour that satisfies `R_min` and the configured class clearance margin, expressed as a multiple of `W` and converted to meters as `W × multiple` at runtime. Camera classification MUST set the applicable margin. A camera-recognized gate MAY veto avoidance only under FR-028; camera evidence alone MUST NOT trigger AVOID.
- **FR-012**: The feasible avoidance side MUST be selected using configured clearance and route-goal rules. If neither side is feasible, the system MUST select STOP.
- **FR-013**: STOP MUST be selected when no detour is feasible, required deceleration exceeds the configured fraction of braking capability, contact occurs, or a critical component is stale. LiDAR MUST be able to trigger STOP and veto RESUME; odometry staleness MUST trigger STOP.
- **FR-014**: Camera staleness MUST remain visible and MAY permit LiDAR-only avoidance using the configured conservative margin. LiDAR staleness MUST trigger STOP. IMU staleness MUST apply the configured conservative turn-speed fallback and MUST stop the vehicle if a safe turn cannot be established.
- **FR-015**: STOP MUST remain latched until the configured corridor-clear and dwell conditions are met. RESUME MUST require regained lateral clearance and a clear route corridor for the configured dwell time.
- **FR-016**: Ground-only contact MUST NOT be reported as a collision. Non-ground contact MUST identify the contacted entity when available, trigger STOP, and remain latched.
- **FR-017**: The decision layer MUST send behavior intent to the existing controller; the existing controller MUST remain the sole node issuing vehicle actuation commands. The dashboard MUST be read-only and its availability MUST NOT affect vehicle behavior.
- **FR-018**: Every behavior transition MUST have a deterministic record containing simulation time, previous and next behavior, trigger components, supporting components, overruled evidence and the component that overruled it where applicable, values used, freshness, and reason. The decision node MUST also emit an `evaluation` event whenever veto or overruled attribution changes without a behavior transition; it records the unchanged active behavior and the candidate behavior that was vetoed or overruled. Identical evidence, configuration, and seed MUST produce the same ordered transitions and attribution. A same-host monotonic publish timestamp is delivery metadata and is excluded from deterministic replay comparison and qualification scoring.
- **FR-019**: Each scenario MUST be runnable alone or in a batch, headless by default, and compared against its configured expected behavior sequence and trigger set. The observed collapsed behavior sequence MUST start with the run's initial `/decision/state` behavior and append behaviors from `transition` events only; `evaluation` events MUST NOT alter it. The actual trigger-component set MUST be the union of components in `attribution.triggers` on `transition` events only; evaluation veto/overrule components MUST NOT count as triggers. Both values MUST exactly equal their expected values for a repeat to match. Additional actual triggers MUST remain in the report and make that repeat a trigger-set mismatch. Every report MUST include seed, scenario, expected and observed sequences, expected and actual trigger sets, contact outcome, and run purpose; P4 qualification reports MUST be labeled `demonstration`. Every demonstration evidence, decision-event, fault-event, planner-intent, and run-report record MUST include the same verified frozen-input root hash; each record MUST identify its run purpose so its hash requirement is independently checkable.
- **FR-020**: Demonstration runs MUST NOT count toward Feature 001 or another frozen-set success, collision, or accuracy metric. Any reported evaluation metric MUST follow the applicable frozen-set rules.
- **FR-021**: A dashboard MUST show each component's decision role (`idle`, `supporting`, `trigger`, `veto`, or `overruled`) separately from freshness (`fresh`, `stale`, or `unavailable`), plus the active behavior and reason and an event history that matches the run log. The history MUST include both behavior `transition` and attribution-changing `evaluation` events. A veto MUST be distinguished from a trigger and identify the evidence that prevented a candidate behavior.
- **FR-022**: The dashboard MUST emit each behavior transition as a flushed SSE frame within 200 ms, measured from the decision node's same-host monotonic publish timestamp using the shared OS monotonic clock, and remain usable when any noncritical component is stale. Dashboard failure MUST NOT affect driving.
- **FR-023**: Each S0-S7 scenario and S3b MUST have a real-car setup sheet specifying object dimensions and colours, spacing, height relative to the single-plane LiDAR, entry speed, and placement relative to the route and vehicle.
- **FR-024**: Real-car setup values MUST be derivable from the same RC-scale, vehicle-relative scenario definitions. Simulation-to-real-car equivalence MUST be judged by behavior sequence and trigger set, not by identical absolute distances.
- **FR-025**: The feature MUST preserve WSL2 Ubuntu 26.04, ROS 2 Lyrical, and Gazebo Sim 10.5.0, use apt as the only package source, make no Pixi changes, and run without network access after local assets and dependencies are present.
- **FR-026**: Future AEBS integration MAY add a behavior through the decision model; AEBS thresholds and behavior are out of scope for this feature.
- **FR-027**: Component ablation runs (P7) and learned camera classification (P8) are stretch scope and MUST be reported separately from the normal S0-S7 and S3b results.
- **FR-028**: A gate opening MUST be modeled as the corridor between its posts and MUST be passable only when its width is at least vehicle width `W` plus the configured gate-class clearance margin, expressed as a multiple of `W`. LiDAR returns outside that corridor, including returns on post edges, MUST NOT count as blockage within the opening, but remain eligible as ordinary route-hazard evidence. A camera-recognized gate MAY veto avoidance only after the configured minimum number of consecutive approach scans contain no LiDAR return inside the opening corridor; the veto MUST be re-evaluated each decision cycle with configured hysteresis. An opening below the passable width MUST NOT veto avoidance. A return inside the corridor MUST overrule the gate classification and MUST be evaluated using the normal LiDAR caution, avoidance, stopping-distance, and STOP rules. Stale LiDAR MUST result in STOP. Decision attribution MUST retain the camera gate classification, the LiDAR override status, and the in-corridor return range when present.
- **FR-029**: All visible vehicle and scene assets MUST use the highest-detail compatible pinned Feature 003 research mesh available for the object class, preserving its original locked bytes, mesh detail, and materials. Implementation MUST NOT decimate, simplify, downsample, or substitute a lower-detail variant; low-poly, generated, unpinned, or lower-quality fallback visuals MUST NOT appear in the default course or demonstrated runs. Primitive collision proxies MAY be used for stable physics and contact truth. Where Feature 003 has no matching semantic visual, implementation MUST select the most detailed compatible asset available from a pinned source and add it to the Feature 007 lock with source, revision, license, hash, dimensions, mesh/texture quality evidence, and a brief comparison to alternatives before using it. If the protected RC model has no visible chassis, the new world MUST add a collisionless vehicle-visual companion using the pinned Feature 003 mesh and synchronize it to the RC model pose without changing the physics model.
- **FR-030**: The new world MUST use the internal name `obstacle_course` and a Feature 007-owned bridge config for its world-scoped contact topic. Existing bridge configs MUST remain unchanged. The contact integration test MUST confirm contact messages arrive from the new world.
- **FR-031**: The route-waypoint follower MUST load the selected tracked scenario route, use `/odom` to advance route progress, and combine the current behavior intent with a route target before producing bounded Ackermann requests. The route source, waypoint progression, coordinate frame, and turn-radius limit MUST be defined and tested; the final Gazebo command remains exclusive to the existing safe-drive controller.
- **FR-032**: Before freezing demonstration inputs, the S5 scenario MUST select exactly one contact branch from the measured vehicle profile and its fixed obstacle reveal geometry. Its expected trigger set MUST include `contact` only when the selected branch expects physically unavoidable contact. A reveal event MUST record route progress and obstacle range and MUST prove the obstacle was first made visible inside `d_stop`; observed outcomes MUST NOT alter expected triggers or qualification rules.

### Key Entities

- **Scenario**: A selectable course case containing vehicle-relative route geometry, obstacles, entry conditions, seed, expected behavior sequence, trigger set, and a real-car setup reference.
- **Component Evidence**: A normalized observation from one of the five vehicle components, with value, unit, frame or object reference where applicable, simulation timestamp, freshness, and validity.
- **Decision Event**: A `transition` or attribution-changing `evaluation` with the previous and active behavior, optional candidate behavior, trigger/support/veto/overruled evidence, reason, simulation time, scenario identity, and non-deterministic same-host monotonic publish metadata. Qualification sequence and trigger-set scoring use `transition` events only.
- **Vehicle Parameter**: A dimension, threshold, speed, deceleration, latency, clearance, or sensor value with unit, provenance, and scaled/measured/placeholder status.
- **Real-Car Setup Sheet**: Human-readable instructions to place and size scenario objects and configure entry speed using RC vehicle-relative units.
- **Dashboard View**: A read-only presentation of component status, active behavior, reason, and decision history associated with one scenario run.

## Success Criteria

### Measurable Outcomes

- **SC-001**: The longest scenario route is under 280 m and no longer than the geometry-derived minimum feasible length plus a 25% margin; an independent check recomputes the bound from the vehicle profile and scenario layouts.
- **SC-002**: Each S0-S7 and S3b configuration, counting the two S6 dropout profiles separately, exactly matches its expected collapsed behavior sequence and expected trigger-component set in at least 8 of 10 demonstration runs using the recorded deterministic seed schedule; passing S0-S4 runs, including S3b, have zero non-ground contacts.
- **SC-003**: S5's tracked config pins one deterministic contact outcome before freezing: if measured simulation dynamics show stopping is physically possible, non-ground contact is absent; if the obstacle is physically unavoidable after reveal inside `d_stop`, contact is expected, recorded, and latches STOP. The expected trigger set includes `contact` only in that latter branch, so exact trigger-set comparison never changes based on an observed result.
- **SC-004**: The S6 LiDAR-dropout profile selects STOP in 10 of 10 demonstration repeats using the recorded deterministic seed schedule.
- **SC-005**: Every behavior transition in every completed run has at least one trigger component, its evidence values and freshness, and a reason; no transition is missing attribution. Evaluation events have a valid candidate/veto or overrule record and do not affect sequence/trigger scoring.
- **SC-006**: All five component evidence records show simulation timestamps and freshness, and a deliberate S6 dropout is identified as stale within its configured timeout.
- **SC-007**: The dashboard emits every behavior transition as a flushed SSE frame within 200 ms of the decision node's monotonic publish timestamp; its active behavior and transition/evaluation history, trigger set, and overruled evidence match the corresponding run log.
- **SC-008**: S0-S7 and S3b each have setup sheets, and every value used in the scenario geometry or threshold calculations has units, provenance, and a scaled/measured/placeholder status.
- **SC-009**: Repeated runs with identical inputs and the same recorded seed produce identical ordered behavior sequences and trigger sets; scored runs are labeled demonstration and excluded from frozen-set metrics, while development and diagnostic runs are separately labeled and never scored.
- **SC-010**: Every visible mesh in the default course resolves to a pinned highest-quality Feature 003 asset or a separately pinned high-quality addition; the controlled vehicle uses a Feature 003 research vehicle visual synchronized to the unchanged RC physics model, and no low-poly or generated visual fallback is loaded.

## Assumptions

- `L`, `R_min`, `a_brake`, `t_latency`, sensor freshness limits, acceleration thresholds, class margins, hysteresis, and dwell times are explicit scenario or vehicle configuration values with provenance. Values without real-car measurement remain placeholders.
- In manual mode, keyboard input reaches the existing controller's manual interface; no second node or GUI publisher directly commands the vehicle. Autonomous scenario runs are headless and use the same controller as their sole actuation authority.
- LiDAR and odometry are critical for safe motion. Camera loss permits LiDAR-only margins; IMU loss limits turning and stops the vehicle when safe-turn constraints cannot be checked.
- S0-S7 and S3b are demonstration scenarios. Their results explain behavior and repeatability for this prototype and do not establish real-vehicle safety or performance.
- The frozen Feature 001 classifier is permitted only for read-only diagnostics because its cone/pedestrian/vehicle labels cannot supply barrier/gate semantics; all production and qualification camera evidence uses the configured colour classes. Feature 003's RC-scale model and highest-quality pinned meshes are the visual anchors.
- Runtime portability: WSL2 Ubuntu 26.04, ROS 2 Lyrical, and Gazebo Sim 10.5.0 via apt only; do not change Pixi. Network access is not required at run time after dependencies and local assets are available.

## Out of Scope

- Training or tuning a detection model, changing a frozen classifier's threshold, or tuning an existing controller or planner to improve a reported metric.
- Implementing AEBS behavior or thresholds, SLAM, or changes to the existing RC vehicle model and existing worlds.
- Testing the real RC car or claiming that simulation values predict real-car behavior.
