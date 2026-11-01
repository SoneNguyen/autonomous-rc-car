# Feature 007 Research

## Evidence and anchor decision

Feature 007 uses the RC-scale vehicle and high fidelity visual assets established by Feature 003 (Realistic RC Domain Transfer), not the large Feature 001 vehicle. The locally available Feature 003 source ref is `003-realistic-domain-transfer` at `a2a315978f8de486bf938f080a1e44ffb7cf8fa8`.

| Anchor | Active checkout path | Feature 003 source / observed role |
|---|---|---|
| Dynamic vehicle, sensors, and collision geometry | `simulation/models/rc_car/model.sdf` | Feature 003 RC model, modified later to remove its primitive visuals while preserving physics, sensors, and joints; it has no chassis `<visual>` element. Gazebo entity name remains `gazebo_truck`. |
| RC vehicle profile | `ros2_ws/src/car_bringup/config/rc_vehicle.yaml`, `presets.rc_car` | Feature 003 vehicle-relative geometry and simulator placeholders; hardware values remain unmeasured. |
| Feature 003 vehicle and obstacle mesh pins | `ros2_ws/src/car_bringup/config/rc_module_matrix/research_assets.lock.yaml` | Active derivative of Feature 003 `ros2_ws/src/car_bringup/config/hifi/assets.lock.yaml`; preserves the Feature 003 mesh identities and hashes. Feature 007 reads it unchanged. |
| High fidelity model descriptions | `simulation/models/research/{cone,pedestrian,vehicle}/` | Feature 003 mesh scales and simple collision proxies, with mesh URIs renamed from `hifi` to `research` in the cleaned project. |
| Asset fetch and hash verification | `tools/fetch_matrix_assets.py` | Fetches locked files into ignored artifacts; runtime uses only local verified meshes. |

The Feature 003 high-fidelity inventory includes miniature cones, pedestrian figures, compact-SUV and convertible vehicles, and floor materials. Only assets present in the Feature 003 lock are Feature 003 anchors; any unpinned mesh in the ignored cache is excluded. Use Feature 003's `vehicle_compact_suv_01` mesh as the Feature 007 controlled-vehicle visual overlay where a detailed vehicle body is needed, while leaving the RC physics model's collisions, sensors, joints, and controller behavior unchanged. Use the original locked Feature 003 GLBs for matching scene objects. Keep their pinned bytes and native mesh/material detail; do not decimate, downsample, simplify, or substitute a lower-detail variant. Where Feature 003 has no matching semantic visual, Feature 007 uses a separately pinned source asset and records the candidate comparison, mesh/texture detail, source, license, revision, hash, and dimensions before use. A low-poly, generated, or unpinned visual is not an acceptable fallback. Primitive collisions remain appropriate and required for stable contact truth.

### Visual detail comparison

The pinned Feature 003 vehicle GLBs were inspected locally before selecting the default RC visual. Mesh primitive counts and file sizes are repeatable detail indicators; the implementation's visual review must still confirm materials and silhouette. The Feature 003 compact SUV has more authored mesh detail than the convertible and is compatible with the RC envelope as a collisionless visual after applying its locked scale.

| Feature 003 asset | Locked SHA-256 | GLB size | Meshes | Primitives | Materials | Selection |
|---|---|---:|---:|---:|---:|---|
| `vehicle_compact_suv_01` | `634f6d8723d3d00a3265416620b2e1da3d229ca003fdbc2412895d55d08850c1` | 4,782,720 bytes | 429 | 435 | 12 | Use for the controlled vehicle visual; highest-detail pinned Feature 003 vehicle option by these observed counts. |
| `vehicle_convertible_01` | `22f7c7321862f4f6e54fadfc53ada73ef6da268eb38f8a22c299b7dac7a0ef27` | 4,218,172 bytes | 384 | 388 | 12 | Retain as an alternative research asset; lower primitive and mesh counts than the selected SUV. |

The orange post uses the more detailed pinned Feature 003 cone variant, `cone_reflector_01` (SHA-256 `a0bf270ab7a170f31f22903a86ebd6bcfeb5768e34a2e3ca58060a5929231149`; 6,564 bytes versus 5,340 bytes for `cone_clean_01`), when the reflective cone appearance fits the scenario. The Feature 003 inventory has no matching red barrier or blue gate model; those additions require a pre-use candidate comparison, pinned source/license/revision/hash/dimensions, mesh and texture detail evidence, and selection of the highest-detail compatible candidate. No generated primitive or low-poly mesh may be used as a visible substitute; primitives remain limited to collision geometry.

Do not require a Feature 001 large-truck model, restore removed historical files, or copy prior feature artifacts into the active tree. Record current anchor hashes and protected paths at implementation start; absence of unrelated historical files is not a blocker. Feature 001/002 frozen evaluation artifacts, existing worlds, model sources, manifests, registries, weights, and results remain read-only.

## Runtime and package source

**Decision**: Keep the simulation target at WSL2 Ubuntu 26.04, ROS 2 Lyrical, and Gazebo Sim 10.5.0 installed from apt. Also build and unit-test the new Python package against ROS 2 Jazzy using APIs shared by both distributions. Do not change Pixi. Asset downloads happen during setup; a scenario run must work offline using verified local files.

**Observation**: The active environment uses Python 3.14.4 and ROS packages under `/opt/ros/lyrical`; the observed Gazebo Sim version is 10.5.0. Jazzy compatibility still requires a separate build check.

## RC vehicle geometry and course bound

**Decision**: Use the active `presets.rc_car` values and `simulation/models/rc_car/model.sdf`. Keep these anchor files unchanged. The model is named `gazebo_truck` for topic compatibility, but its scale and geometry are those of the Feature 003 RC car.

| Quantity | Value | Source / status |
|---|---:|---|
| Chassis length | 0.38 m | RC SDF collision box; simulation geometry anchor. |
| Chassis width | 0.22 m | RC SDF collision box; simulation geometry anchor. |
| Wheelbase | 0.27 m | `presets.rc_car.geometry.wheelbase_m`; labelled placeholder, not a hardware measurement. |
| Effective center steering limit | 0.45 rad | `presets.rc_car.steering.effective_center_wheel_limit_rad`; labelled placeholder. |
| Minimum turn radius | 0.558942 m | Derived as `0.27 / tan(0.45)`; recompute from the accepted vehicle profile. |
| Wheel-inclusive width | 0.225 m | Derived from 0.20 m track and 0.025 m wheel width; verify against SDF wheel extents. |
| Conservative footprint radius | 0.220808 m | Derived as half the diagonal of the 0.38 × 0.225 m envelope. |
| Maximum speed | 1.0 m/s | RC profile placeholder; the test uses an explicitly configured entry speed. |

The current `rc_car` preset marks dimensions, steering, and braking defaults as placeholders. A scripted Feature 007 probe must measure simulation braking deceleration and request-to-deceleration latency before computing course stop bounds. These are simulation measurements, not physical RC-car claims. For each layout, derive its shortest feasible route from the measured `d_stop(v)`, the derived `R_min`, obstacle clearance and approach/recovery segments; then apply the specified 25% margin. The course stays below 280 m and the sizing test independently recomputes the bound.

### Feature 007 brake-probe observation

The tracked fixed-seed probe `rc_vehicle_brake` (seed `78001`) ran ten trials through the existing `safe_drive_controller` Ackermann input path. All trials produced at least 76 simulation-time odometry samples and a measured entry speed near `0.429 m/s`. The conservative minimum fitted deceleration was `0.3999092579 m/s²`; the maximum request-to-deceleration latency was `0.140000 s`; the largest measured stop distance was `0.271954 m`. These values describe the configured Gazebo simulation only. The probe report is `artifacts/obstacle-course/dynamics/rc-vehicle-reference.json` (SHA-256 `95c7bfdeb63e8e77631919d49d02d3fb1445838d49c77e85f030f24941b4911f`).

The profile derives `R_min = 0.27 / tan(0.45) = 0.5589424876 m`. Using the conservative measured deceleration and latency, the stopping-distance values are: `0.026503 m` at `0.1 m/s`, `0.113143 m` at `0.25 m/s`, `0.382571 m` at `0.5 m/s`, `0.808285 m` at `0.75 m/s`, and `1.390284 m` at `1.0 m/s`. The measurement report records hashes for the profile, fixed-seed scenario, source world, RC vehicle profile, RC model, and unchanged safe-drive controller. The accepted Feature 007 profile SHA-256 is `886c30481d02054ecbe6f3bf04eff196d06603e9f57efdef656cd2f371b31381`.

The minimum lane-change path uses two opposite-curvature arcs at `R_min`: for lateral shift `d`, `theta = acos(1 - d/(2 R_min))`, longitudinal projection `2 R_min sin(theta)`, and path length `2 R_min theta`. The route minimum sums the approach distance, `d_stop(entry_speed)`, lane-change path length, and recovery distance, then applies a separate `1.25` layout margin. The independent sizing utility recomputes this value from the tracked profile, geometry, and scenario configs.

## Dynamics and stopping distance

Add `tools/measure_rc_vehicle_dynamics.py`. It runs repeatable straight-line brake trials through the existing safe-drive-controller request path and records simulation-time request acceptance, odometry, speed, stop distance, deceleration, and source hashes. Select a conservative measurement only after the scripted runs pass. Store it in the Feature 007 `rc_vehicle_reference` overlay; never overwrite `rc_vehicle.yaml` or describe simulation results as hardware measurements.

Use `d_stop(v) = v^2 / (2 * a_brake) + v * t_latency`, where `a_brake > 0` and both inputs are measured from the same pinned simulation setup. Development thresholds are tuned only on separate development variants, then frozen and hashed before demonstration repeats.

## Existing ROS interfaces and extension boundary

**Decision**: Reuse the active RC vehicle model, sensor bridge, `safe_drive_controller.py`, lidar relay/corridor safety path, and controller Ackermann input. Do not assume the removed Feature 002 large-truck planner or pure-pursuit files are present. Feature 007 owns its decision and path-intent logic and sends bounded `AckermannDriveStamped` requests to `/ackermann_cmd` in `rc_navigation` mode; the existing safe-drive controller remains the only publisher to `/model/gazebo_truck/cmd_vel`.

The controlled entity keeps the existing `gazebo_truck` topic namespace for compatibility. The Feature 003 RC sensors include a front camera, instance-segmentation camera, GPU LiDAR, IMU, and bumper contact sensor. The existing bridge maps contacts beneath `/world/rc_module_matrix/`; a Feature 007 world named `obstacle_course` therefore needs an additive `ros2_ws/src/car_bringup/config/obstacle_course/bridge.yaml` with the corresponding `/world/obstacle_course/` contact path. Do not edit the existing bridge. Feature 007 normalizes the active RC messages and does not add a legacy point-cloud adapter or duplicate actuator publisher.

Because the protected RC model has no visible chassis, the new world adds a collisionless `gazebo_truck_visual` companion using the pinned `vehicle_compact_suv_01` mesh. The Feature 003 model description also contains a primitive collision box, so Feature 007 copies only its locked mesh URI and visual scale into the new world and does not include its collision geometry. A Feature 007 `vehicle_visual_overlay.py` node captures the initial `/odom` pose and configured world spawn pose, then transforms later odometry poses into world coordinates before updating only that companion through Gazebo's world-scoped `SetEntityPose` service. The P1 integration check must verify that the visual follows straight and turning motion while its collisions remain disabled and the RC physics entity remains untouched.

The decision node publishes behavior state, attributed events, and short-lived motion intent. Its adapter converts intent to the controller request topic with the profile bounds. Neither decision logic nor the dashboard publishes directly to Gazebo. Manual controls, if exposed, must share the same request/controller authority and be mutually exclusive with autonomous scenario control.

## Frozen Feature 001 camera-label compatibility

The frozen classifier's observed source labels are `cone`, `pedestrian`, and `vehicle` (`CLASS_NAMES={1:"cone", 2:"pedestrian", 3:"vehicle"}` in its read-only source). Feature 007's actionable HSV classes are `barrier`, `post`, and `gate`; the detector has no label that can identify a barrier or gate. Do not map a pedestrian or vehicle detection to a course obstacle class. A detector `cone` may be retained as a coarse `post` label only, and must use the post clearance margin. The normalized evidence retains the original detector label and verified model SHA-256.

Therefore, the frozen classifier is available only in diagnostic, unqualified runs; S0-S7/S3b demonstration configs use HSV. The diagnostic source cannot create gate vetoes, select barrier/gate margins, or contribute to qualification trigger/sequence scoring. No model weights, labels, or thresholds are changed. If the frozen model artifact is unavailable or its hash cannot be verified locally, the diagnostic source is unavailable and the HSV path remains active.

## Component-specific observations

- **Visual assets**: The default Feature 007 scene uses the highest-detail pinned Feature 003 research meshes at their locked source detail. Preserve the original GLB hashes and do not simplify or downsample them. Primitive collision proxies are retained because Feature 003 uses them for dependable contact reporting. Any new barrier or gate visual absent from Feature 003 must use the most detailed compatible asset found in a pinned source, with its alternatives and quality evidence (mesh/texture detail), source/license/revision/hash/dimensions recorded in the Feature 007-only `ros2_ws/src/car_bringup/config/obstacle_course/assets.lock.yaml`; never extend or rewrite the protected Feature 003 lock.
- **Camera**: Keep the requested default colour rules (red barrier, orange post, blue gate) as config. Camera evidence reports class, colour score, pixel area, bearing, and freshness. The frozen Feature 001 detector is read-only and diagnostic-only because its labels do not represent barriers or gates; its outputs cannot qualify runs.
- **IMU and odometry**: Use ROS simulation timestamps. The tolerance measures the configured simulated sensor noise and odometry behavior only, not hardware accuracy. Capture the odometry origin at run start and report spawn-relative pose/progress.
- **Faults and late reveal**: Use tracked fixed-seed scenario profiles and simulation-time triggers. Fault gates alter only Feature 007 sensor evidence or scenario state; they never publish drive commands.
- **Scenario isolation**: Feature 007 cases use their own registry, configs, and report root. Existing feature split manifests and results remain immutable. Run diagnostic/development probes outside demonstration scoring; reports always retain run purpose and input hashes.

## Design decisions

1. One `car_decision` ament_python package owns evidence normalizers, the deterministic decision engine, intent adapter, fault injection, run logger, and dashboard. This keeps Feature 007 behavior separate from the protected RC vehicle and safety controller.
2. Use standard ROS messages for sensor and controller interfaces. Versioned evidence, decision, intent, and report payloads use `std_msgs/String` JSON contracts, avoiding a custom message package.
3. Use Python's standard-library HTTP server and server-sent events for the localhost GET-only dashboard. Dashboard failure cannot gate, stop, or redirect vehicle commands.
4. Keep the new world, scenarios, reports, and real-car mapping in Feature 007 paths. Feature 003 asset/model anchors are read-only inputs; Feature007 does not modify existing worlds, vehicle physics, or asset locks.

## Feature 007 semantic visual additions and sizing

The Feature 003 lock contains no traffic barrier or gate mesh. The Feature 007
lock therefore adds two original CC0 GLBs without changing the protected
Feature 003 lock or the original source bytes:

| Candidate | Observed detail | Comparison and decision |
|---|---|---|
| [Innerscene Road barrier set](https://www.innerscene.com/tools/library/3d-parts/road-barrier-set-4d29eb0a) | 6.94 MB local GLB, 2,704 triangles, 8 meshes, 12 embedded texture images; authored red/white reflective barricade and orange cones. | Selected for the barrier visual over a lower-size untextured STEP barrier and a YDR-only concrete barrier; the pinned source includes road-work objects with native PBR materials. |
| [Innerscene Security grille gate](https://www.innerscene.com/tools/library/3d-parts/grille-security-grille-gate-941c0408) | 350.6 KB local GLB, 11,796 triangles, 31 meshes, 3 embedded texture images; graphite powder-coat and bronze hardware. | Selected as the most detailed compatible realistic gate model among the inspected CC0 gate alternatives; the 18 KB / 190-triangle low-poly medieval gate was rejected. Its original materials are preserved. |

**Camera-class limitation**: the selected gate source is graphite/bronze, not
blue. Its geometry and detail are suitable as a gate visual, but it cannot be
counted as blue-HSV gate evidence. Do not tune the HSV rule or recolor the
source GLB to hide this mismatch. Before claiming S3/S3b camera qualification,
replace this candidate with a compatible pinned high-detail blue gate asset
or keep that camera-specific acceptance explicitly unqualified. The red
barrier and orange post retain the requested source colors; the post uses the
Feature 003 `cone_reflector_01` mesh.

The fixed-seed scenario route lengths were checked using `R_min =
0.5589424875513274 m`, `a_brake = 0.3999092578809597 m/s^2`, and `t_latency =
0.14 s`. At each scenario's `0.35 m/s` entry speed, `d_stop(v) =
0.2021597451 m`. The bound is `1.25 × (approach + d_stop + counted lane-change
path lengths + recovery)`.

| Scenario | Route length | Derived minimum | With 25% margin |
|---|---:|---:|---:|
| S0 | 0.850 m | 0.702 m | 0.878 m |
| S1 | 1.800 m | 1.464 m | 1.831 m |
| S2 | 2.600 m | 2.129 m | 2.661 m |
| S3 | 0.850 m | 0.702 m | 0.878 m |
| S3b | 1.800 m | 1.464 m | 1.831 m |
| S4 | 0.850 m | 0.702 m | 0.878 m |
| S5 | 0.850 m | 0.702 m | 0.878 m |
| S6-lidar-drop | 0.850 m | 0.702 m | 0.878 m |
| S6-camera-drop | 1.800 m | 1.464 m | 1.831 m |
| S7 | 1.800 m | 1.464 m | 1.831 m |

The independent checker counts both lane changes in S2. Every listed route is
strictly below 280 m and no longer than its recomputed margin bound. These are
simulation layout values, not measurements of a physical RC car.
