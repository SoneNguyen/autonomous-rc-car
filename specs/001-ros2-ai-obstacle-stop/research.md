# Research: ROS 2 and AI Obstacle Stop

**Research date**: 2026-10-20
**Decision**: Continue with the user-mandated WSL2 Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Sim 10 (Jetty) stack. Official ROS metadata identifies Lyrical/Jetty binary packages at `packages.ros.org`; Ubuntu documents `wsl --install Ubuntu-26.04`. The availability gate passes. No native Windows, RoboStack, Jazzy, or Harmonic fallback is considered.

## Decisions

### 1. WSL distribution and apt package pairing

**Decision**: Use WSL 2 Ubuntu 26.04 (Resolute), ROS 2 Lyrical, and Gazebo Sim 10 (Jetty), installed with apt. Install `ros-lyrical-desktop` and `ros-lyrical-ros-gz` from the official ROS 2 apt repository at `packages.ros.org`; keep the repository and `ros2_ws/` within the Linux filesystem (for example `~/gz-ws`, not `/mnt/c/...`). The ROS Index repository table explicitly lists Lyrical + Jetty binaries from `packages.ros.org`. The official Ubuntu WSL tutorial explicitly uses `wsl --install Ubuntu-26.04`; the ROS apt listing contains a `lyrical-resolute-amd64` package index.

**Version baseline to record in the setup manifest**:

- Ubuntu 26.04 LTS / Resolute, WSL 2.
- ROS 2 Lyrical.
- `ros_gz` / `ros_gz_bridge` 3.0.10 (`ros-lyrical-ros-gz` apt candidate `3.0.10-1resolute.20260930.020431`, verified from `packages.ros.org` on 2026-10-20; amended from 3.0.11 because that pinned version was unavailable).
- Gazebo Sim 10.5.0, Jetty (Gazebo Sim API documents 10.5.0; major version 10 is fixed).
- Use Gazebo Sim's `--seed <integer>` CLI option, documented in the current Gazebo Sim CLI/release documentation; preserve the seed in every scenario and record.
- Ubuntu `python3-torch` 2.9.1+dfsg-1~exp1ubuntu2 for CPU training/inference. The exact Debian revision resolved by apt must be recorded with `dpkg-query` during setup; do not replace it with an untracked pip/Conda package.

Before implementation, `gz sim --versions` must list `10.5.0`, `ros2 pkg xml ros_gz_bridge` must report upstream version `3.0.10`, and setup must record the full apt package revisions. The ROS apt repository is configured by `ros2-apt-source` 1.3.0 for Resolute. All launch invocations force Gazebo Sim `10.5.0`; if that exact version is unavailable, stop rather than selecting another installed version. This amendment changes only the ROS-Gazebo bridge patch version after an actual apt candidate check; it does not change ROS Lyrical, Ubuntu 26.04, or the Gazebo Sim major/minor pin.

The WSL2 Ubuntu 26.04.1 LTS distro is installed and reports codename `resolute`. `ros2-apt-source` 1.3.0 is installed, and apt fetched the Resolute index from `packages.ros.org`. The initial exact check found `ros-lyrical-ros-gz` 3.0.10 (not 3.0.11), which prompted the documented pin amendment. Setup installed the missing `ros-dev-tools` and `python3-colcon-common-extensions` packages from apt. The ROS bridge manifest reports 3.0.10, `gz sim --versions` reports 10.5.0, and apt `python3-torch` 2.9.1 imports under system Python. Full package revisions are recorded in `docs/wsl-setup.md`. A missing or mismatched Gazebo Sim 10.5.0 remains a stop condition; it is not permission to switch distributions or Gazebo major versions.

**Rationale**: This is the requested ROS/Gazebo pairing and preserves compatibility with the existing Jetty model. It also keeps ROS and Gazebo under apt rather than mixing Windows Pixi, source builds, or RoboStack into the runtime.

**Alternatives considered**: Jazzy/Harmonic changes ROS and Gazebo majors and is explicitly out of scope. Windows-native ROS and RoboStack are disallowed. Building `ros_gz` from source is unnecessary when Lyrical/Jetty binaries are published.

### 2. WSLg GUI and autonomous headless rendering

**Decision**: Support two launch modes. GUI inspection uses WSLg and the scenario-backed ROS launch wrapper with `headless:=false`; it is for visualization/sensor checks only because GUI keyboard publishing can compete with autonomy. All autonomous and evaluation runs use the same wrapper with `headless:=true`, server-only mode, and the existing OGRE2 Sensors system. For example:

```bash
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=lidar scenario:=training/stop_smoke headless:=true
```

The wrapper owns the model resource path, loads the seed/start pose/obstacle/speed from the named checked-in scenario, and maps `headless:=true` to the Gazebo flags `--force-version 10.5.0 -v 4 -s -r --headless-rendering`. Do not run feature trials using a standalone `gz sim` command because that bypasses scenario configuration. If WSLg/GPU EGL rendering is unavailable, try the scenario-backed software-rendering fallback:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch car_bringup obstacle_course.launch.py \
  mode:=lidar scenario:=training/stop_smoke headless:=true
```

The fallback is accepted only after camera and lidar rates/data are verified; it may be much slower. Preserve OGRE2 because Gazebo Sim headless EGL rendering supports OGRE2, and the current world already configures it. WSLg is the visualization path; it is not the evaluation path.

**Rationale**: Gazebo Sim 10 supports server-only headless rendering with OGRE2; the wrapper's exact flag mapping is recorded above. WSLg provides Linux GUI application support. Server-only mode removes the GUI KeyPublisher from the autonomous command-authority graph while keeping rendering sensors active.

**Alternatives considered**: GUI-only autonomy risks a second command source. `-s` without `--headless-rendering` does not satisfy camera/lidar rendering. Switching the world render engine is disallowed because the world is protected.

### 3. Bridge protocol and topic map

**Decision**: Use the `ros_gz_bridge` YAML configuration for all mappings. The implementation YAML lives in `ros2_ws/src/car_bringup/config/ros_gz_bridge.yaml`; [contracts/topics.md](contracts/topics.md) is its source of truth. Use `GZ_TO_ROS` for sensors/odometry/contact/clock and `ROS_TO_GZ` only for the drive command. Set ROS-side names to the existing Gazebo names except for the controller request and perception output. Use `SENSOR_DATA` QoS for camera, lidar, IMU, odometry, and contacts; use the clock QoS profile for `/clock`; use the reliable/default command profile for `cmd_vel`.

Conversions verified in the official `ros_gz_bridge` mapping table include:

- `geometry_msgs/msg/Twist` <-> `gz.msgs.Twist`
- `nav_msgs/msg/Odometry` <-> `gz.msgs.Odometry`
- `sensor_msgs/msg/Image` <-> `gz.msgs.Image`
- `sensor_msgs/msg/LaserScan` <-> `gz.msgs.LaserScan`
- `sensor_msgs/msg/PointCloud2` <-> `gz.msgs.PointCloudPacked`
- `sensor_msgs/msg/Imu` <-> `gz.msgs.IMU`
- `ros_gz_interfaces/msg/Contacts` <-> `gz.msgs.Contacts`
- `rosgraph_msgs/msg/Clock` <-> `gz.msgs.Clock`

Bridge `/clock` one-way from Gazebo to ROS and enable `use_sim_time=true` on every ROS node. Ensure no second `/clock` publisher exists; otherwise Gazebo may use its fully-qualified world clock topic instead.

**Rationale**: YAML keeps names, types, direction, and QoS centralized as required. The standard bridge supports each requested message without custom bridge code.

**Alternatives considered**: Per-topic command-line bridges scatter contracts and QoS. A custom plugin adds unnecessary coupling and dependencies.

### 4. Multi-layer lidar choice

**Decision**: Bridge `/model/gazebo_truck/sensors/front_lidar/scan` as `sensor_msgs/msg/LaserScan` for inspection only, and `/model/gazebo_truck/sensors/front_lidar/scan/points` as `sensor_msgs/msg/PointCloud2` for obstacle filtering and labels. Never base the safety stop on the flattened LaserScan.

**Rationale**: The bridge conversion code explicitly selects only the middle vertical row from `gz.msgs.LaserScan` when multiple vertical beams are present. The vehicle sensor is a 16-row multi-layer lidar; the point-cloud conversion preserves the packed 3-D points. The forward corridor can filter obstacle returns by x/y/height after applying the verified sensor-to-vehicle transform.

**Alternatives considered**: Using LaserScan would silently discard 15 of 16 vertical layers. Bridging only PointCloud2 would meet controller needs but hide the existing scan topic from ROS inspection.

### 5. Controller and watchdog ownership

**Decision**: Implement one `safe_drive_controller` node as the sole ROS publisher to the bridged Gazebo command topic. For P2, a client sends a bounded requested Twist to `/autonomy/cmd_vel_request`; for P3/P4, the same controller selects its own baseline/AI-assisted target. Command freshness, sensor freshness, pause/reset detection, arbitration, speed/turn limits, and zero-command output live inside this controller. The logger is a subscriber only; perception only publishes classifications.

Use 20 Hz controller updates, expire external requests after 0.5 simulated seconds, and stop if `/clock`, required lidar, or odometry goes stale. In AI mode, stale/invalid model results switch to lidar-only behavior; a fresh `clear` result never cancels a lidar stop. Detect backward clock jumps on reset and emit zero before requiring fresh sensor samples.

**Rationale**: A separate watchdog publisher would violate the one-authority requirement. Putting the watchdog and all final command arbitration in one node provides a single audited output path.

**Alternatives considered**: Directly bridging external `cmd_vel` requests would permit multiple ROS publishers to control Gazebo. A stand-alone watchdog publishing zero would introduce a second writer. A behavior-tree framework is not justified for this single stop policy.

### 6. Camera model and training dependencies

**Decision**: Use a small CNN trained from scratch in PyTorch; use Ubuntu's apt `python3-torch` 2.9.1 CPU package for both the WSL trainer and live inference. Do not require torchvision, an external pretrained checkpoint, CUDA, or model downloads. Downsample 640x480 RGB images to 160x120 and target 10 Hz inference with a bounded latest-frame queue. Save only a plain model `state_dict` plus JSON metadata under `artifacts/`; use model code in `ml/` so weights are data, not executable pickle code. Training outside WSL is optional, but its weights must load under the pinned WSL PyTorch version and pass a CPU load/inference check.

Use a lidar-derived label: within the non-ground forward vehicle corridor, `too_close` is positive if the nearest obstacle return is at or inside the scenario's frozen trigger distance; otherwise `clear`. Pair images to the latest non-future point cloud no more than 1/15 simulated second old; reject stale/unmatched samples. Select the classification threshold only with validation scenarios. Report class-specific precision, recall, F1, confusion counts, sample count, and chosen threshold on the untouched evaluation set.

**Rationale**: A small custom CNN avoids a separately downloaded checkpoint and preserves the required trained-camera behavior. Ubuntu Resolute publishes Python 3.14 and `python3-torch` 2.9.1, so the model can share the apt-managed ROS runtime without a second Python environment. PyTorch recommends pip for its upstream binaries; this project deliberately chooses the Ubuntu apt build to honor the mandated package-manager boundary.

**Alternatives considered**: A pretrained backbone adds downloads, licenses, and storage that the story does not require. `pip`/Conda Torch creates another dependency channel and can complicate `rclpy` imports. Training on evaluation data or choosing a threshold on it invalidates the held-out claim.

### 7. Scenario splits and paired evaluation

**Decision**: Check in separate training, validation, and evaluation scenario YAMLs. Each config contains the world path, fixed seed, start pose, named existing obstacle/collision and its unchanged world placement, speed, direction, corridor/height filter, and stopping threshold. The world remains fixed; scenarios choose among existing course obstacles and vary start pose/speed/heading. Canonicalize numeric values and compare the full `(start_pose, obstacle_placement, commanded_speed)` tuple to reject cross-split duplicates. Generate and hash a split manifest before any training; freeze the ten evaluation configs and their hashes at that point. Run both behavior modes once per frozen config with the same seed and parameters.

**Rationale**: The current world groups the static course geometry in a fixed `course_obstacles` model. Choosing named existing collision geometries and varying the vehicle pose/speed produces reproducible, disjoint test cases without modifying or regenerating the protected world.

**Alternatives considered**: Different seeds/runs alone do not create scenario-disjoint splits. Randomly moving obstacles would require world edits or runtime scene generation, which is outside this feature. Hand-editing evaluation configs after seeing model metrics is leakage.

### 8. Contact evidence boundary

**Decision**: Initially use the existing bumper topic `/model/gazebo_truck/sensors/bumper_contact` (`gz.msgs.Contacts`) and classify a collision only when a reported pair names an allowlisted obstacle entity/collision; ground contacts are excluded. The existing SDF sensor watches `chassis_collision`, not wheel collision geometry. P1/P3 must deliberately test contact delivery and inspect the collision names. If an evaluated obstacle can be contacted by a wheel without a chassis event, do not claim complete vehicle collision counts. Stop and revise the feature spec/plan before exposing additional contact topics or changing protected model/world files.

**Rationale**: This preserves the current protected model while making the limitation explicit. It avoids silently redefining a vehicle collision as a chassis-only collision.

**Alternatives considered**: Counting all non-empty contact messages would incorrectly count ground contacts. Adding wheel sensors before verifying need would be an unnecessary protected-model edit and contradict current FR-015.

## Evidence and source links

- Ubuntu on WSL installation and Ubuntu 26.04 command: [Ubuntu WSL development tutorial](https://ubuntu.com/wsl/docs/latest/tutorials/develop-with-ubuntu-wsl/).
- Ubuntu 26.04.1 WSL availability: [Ubuntu Download for WSL](https://ubuntu.com/download/wsl).
- Current ROS apt `lyrical-resolute-amd64` index: [packages.ros.org ROS 2 apt listings](https://packages.ros.org/ros2/ubuntu/lists/).
- Lyrical + Jetty binary package pairing and current `ros_gz` version: [ROS Index: ros_gz repository](https://index.ros.org/r/ros_gz/).
- ROS/Gazebo installation documentation: [ROS 2 Lyrical Ubuntu deb install guide](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html) (official page; use its current apt-source setup steps).
- Gazebo Sim API version and exact EGL/headless flags: [Gazebo Sim 10 headless rendering](https://gazebosim.org/api/sim/10/headless_rendering.html).
- Gazebo installed-version inspection and exact-version selection: [Gazebo GUI version-selection guide](https://github.com/gazebosim/docs/blob/master/harmonic/GUI_tutorial.md).
- WSLg Linux GUI support: [Microsoft Learn: Run Linux GUI apps with WSL](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).
- Official ROS/Gazebo conversion mapping, config examples, `/clock` behavior, and YAML QoS support: [ros_gz_bridge README](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md).
- Ackermann command/odometry topic defaults and the 50 Hz default odometry rate: [Gazebo Sim 10 AckermannSteering API](https://gazebosim.org/api/sim/10/classgz_1_1sim_1_1systems_1_1AckermannSteering.html).
- Gazebo Sim 10.4 release documentation calls out command-line seed support: [Gazebo Sim release/API page](https://gazebosim.org/libs/sim/).
- ROS Lyrical test packages for planned `launch_testing` coverage: [ROS Index launch repository](https://index.ros.org/r/launch/) and [ROS Index launch_ros repository](https://index.ros.org/r/launch_ros/).
- Multi-layer LaserScan conversion selecting the middle vertical row: [ros_gz_bridge sensor conversion source](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/src/convert/sensor_msgs.cpp#L2682-L2762).
- Classification message purpose/header convention: [vision_msgs README](https://github.com/ros-perception/vision_msgs/blob/ros2/README.md).
- Ubuntu Resolute `python3` and PyTorch source/package versions: [Ubuntu package search: python3](https://packages.ubuntu.com/en/resolute/python3), [Ubuntu package search: PyTorch source](https://packages.ubuntu.com/search?arch=any&keywords=pytorch&searchon=sourcenames&section=all&suite=all).
- PyTorch upstream Linux install guidance and supported Python range: [PyTorch Get Started Locally](https://pytorch.org/get-started/locally/).

## Local evidence inspected

- `simulation/worlds/obstacle.sdf`: world is `sensor_obstacle_course`; it has Physics, UserCommands, SceneBroadcaster, Contact, IMU, and Sensors systems; Sensors uses OGRE2; the GUI includes keyboard publishing; all static course obstacles live in one `course_obstacles` model; car starts at `(-105, -10, 1.96)`.
- `simulation/models/car/model.sdf`: camera 640x480 RGB at 30 Hz; GPU lidar 720 horizontal x 16 vertical at 15 Hz and a points topic; IMU 100 Hz; chassis contact 100 Hz watches `chassis_collision`; Ackermann steering uses 11.612212 m wheelbase, 0.5 rad steering limit, speed range +/-20 m/s, and acceleration range +/-3 m/s^2. The model parameters remain starter values, not calibrated vehicle data.
- Existing ROS topics in root README are the authoritative Gazebo-side names and will be copied exactly into contracts and bridge YAML.
- P1 test evidence on 2026-10-20: the bridge-contract tests pass and the contact launch test reports the existing chassis-to-`post_01_collision` event after a runtime pose set. At the default start pose, the front wheel reaches `barrier_right_collision` about 2.43 m before the chassis collision, so the current chassis-only bumper reports no event there; P3 must not claim verified collision coverage for that wheel-first barrier.
- P1 rate evidence on the tested WSL2 RTX A2000 laptop: `gz topic -f` measured the camera at about 22-25 Hz headless and 14-16 Hz with the WSLg GUI open. The first ROS one-minute probe received 1,379 camera frames (~23.0 Hz). On 2026-10-20, the P1 acceptance was amended to at least 20 ROS-delivered frames/s averaged over one simulated minute in server-only headless mode, keeping 2x headroom over the planned 10 Hz detector. This does not change the 30 Hz SDF source configuration; the other periodic streams retain a 90%-of-declared-rate floor. GUI rate is not acceptance evidence. The full sensor suite must be rerun before P1 passes.
