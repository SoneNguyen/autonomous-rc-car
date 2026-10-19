# Quickstart and Validation Guide

This is the target workflow for the feature after implementation. It is not yet executable in this workspace: the ROS packages and orchestration scripts are design outputs and have not been built. Keep this guide and the root README synchronized as each story is implemented.

## 1. Install the supported WSL runtime

From Windows PowerShell, install WSL2 Ubuntu 26.04 if it is not already present:

```powershell
wsl --list --online
wsl --install Ubuntu-26.04
```

Restart Windows if prompted, launch the Ubuntu-26.04 distribution, and create its Linux user. Keep the repository on the WSL Linux filesystem (for example under `~/src`), not on `/mnt/c`.

```bash
mkdir -p ~/src
cd ~/src
git clone <project-git-remote> gz-ws
cd ~/src/gz-ws
```

The current workspace is not a Git repository and has no remote URL, so the project owner must replace `<project-git-remote>` after creating/publishing a Git remote. Do not use a Windows absolute path in the permanent README.

Enable the official ROS 2 apt repository by following the current [Lyrical Ubuntu deb installation guide](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html), then check package candidates before installing:

```bash
sudo apt update
apt-cache policy ros-lyrical-desktop ros-lyrical-ros-gz python3-torch
```

Expected: candidates exist from Ubuntu Resolute / `packages.ros.org`; ROS Index lists the Lyrical + Jetty binary pairing. If a pinned package is unavailable, stop; do not substitute Jazzy/Harmonic or source builds.

Install the runtime, build/test tools, and Python model dependency through apt:

```bash
sudo apt install -y \
  ros-lyrical-desktop \
  ros-lyrical-ros-gz \
  ros-lyrical-launch-testing \
  ros-lyrical-launch-testing-ros \
  ros-lyrical-vision-msgs \
  ros-lyrical-sensor-msgs-py \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-pytest \
  python3-torch
```

Expected: ROS 2 Lyrical setup files exist at `/opt/ros/lyrical/setup.bash`; `ros_gz_bridge` and `gz sim` are installed; Ubuntu `python3-torch` imports under the same system Python used by ROS. Record exact apt package versions in `docs/wsl-setup.md` and the run manifest.

```bash
source /opt/ros/lyrical/setup.bash
printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
ros2 pkg prefix ros_gz_bridge
ros2 pkg xml ros_gz_bridge | grep -m1 '<version>'
gz sim --versions
dpkg-query -W -f='${binary:Package} ${Version}\n' ros-lyrical-ros-gz python3-torch
gz sim --help
python3 -c 'import torch; print(torch.__version__)'
```

Expected: `ROS_DISTRO=lyrical`; the bridge prefix is under `/opt/ros/lyrical`; `ros_gz_bridge` reports version `3.0.11`; `gz sim --versions` lists `10.5.0`; the full apt revisions are recorded and match the pinned research; and Torch reports the apt-managed 2.9.1 series. Stop on a mismatch. The launch file forces Gazebo Sim 10.5.0 rather than selecting an arbitrary installed major.

## 2. Build and run checks

In each new WSL terminal:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/src/gz-ws
colcon build --symlink-install --base-paths ros2_ws/src
source ros2_ws/install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Expected: both ROS packages build; unit and `launch_testing` checks pass. A failing build/test is a story checkpoint failure; do not proceed to the next story.

## 3. P1: inspect the five sensor interfaces

Start the WSLg GUI launch for visualization and sensor inspection only:

```bash
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=sensors scenario:=training/sensor_smoke headless:=false
```

Expected: Gazebo opens through WSLg, model resources resolve from `simulation/models`, and ROS topics appear. From a second sourced terminal:

```bash
ros2 topic list
ros2 topic hz /model/gazebo_truck/sensors/front_camera/image
ros2 topic hz /model/gazebo_truck/sensors/front_lidar/scan/points
ros2 topic hz /model/gazebo_truck/sensors/imu
ros2 topic hz /model/gazebo_truck/odometry
ros2 topic echo --once /clock
ros2 topic info -v /model/gazebo_truck/sensors/bumper_contact
```

Expected: all five sensor interfaces (camera, lidar, IMU, odometry, and contact topic) are available within five wall-clock minutes. Camera is about 30 Hz, point cloud about 15 Hz with multiple vertical layers, IMU about 100 Hz, and odometry about 50 Hz; each periodic stream delivers at least 90% of its declared samples over one simulated minute, and timestamps advance with simulation time. Contact is an event signal: inspect its type/QoS here, then induce one low-speed obstacle contact in the dedicated contact test and verify the allowlisted obstacle collision pair appears. Do not require a continuous contact event while the truck is not touching anything.

The bridge also exposes `/model/gazebo_truck/sensors/front_lidar/scan` as `sensor_msgs/msg/LaserScan`; use it for display only. The bridge retains the middle vertical row, so the controller uses the `/scan/points` PointCloud2.

SC-001 measures declared rates only for the four periodic streams. Contact passes through topic availability and delivery of the induced allowlisted obstacle event; idle contact frames are not expected.

## 4. P2: command via the single controller

Launch the controller test in headless mode:

```bash
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=command_test scenario:=training/command_smoke headless:=true
```

From a second sourced terminal, send a slow bounded request:

```bash
ros2 topic pub --rate 10 /autonomy/cmd_vel_request \
  geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.1}}"
```

While it is moving, inspect both sides of the bridge:

```bash
ros2 topic info -v /model/gazebo_truck/cmd_vel
gz topic -i -t /model/gazebo_truck/cmd_vel
```

Expected: the vehicle moves forward and turns; ROS reports exactly one
publisher on the bridged command topic, `safe_drive_controller`, and Gazebo
Transport reports exactly one writer (the launch's bridge path), with no
additional command writer. Stop the
request publisher with Ctrl+C. The final command becomes zero within 0.5
simulated seconds and odometry reaches the configured rest condition within 2
simulated seconds after expiry. The launch test also checks that no GUI
keyboard writer is active.

## 5. P3: verify lidar-only obstacle stopping

Run the frozen scenario smoke test in server-only headless-rendering mode:

```bash
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=lidar scenario:=training/stop_smoke headless:=true
```

Expected: on this training-only calibration scenario, lidar detects the obstacle in the configured forward corridor, requests zero before contact, and the logger records measured stopping distance. Any change to the 5.0 m trigger distance is allowed only here, never from evaluation outcomes.

Freeze the scenario split and ten evaluation config hashes before the first evaluation run and before model training:

```bash
python3 ml/collect_dataset.py validate-splits --freeze-evaluation
```

Expected: all three sets are disjoint by the full `(start pose, obstacle placement, speed)` tuple, and the ten evaluation scenarios are hashed/frozen. Any later change invalidates the frozen evaluation and requires restarting the feature before model training.

Run the ten baseline evaluation trials once as a read-only P3 outcome pass. Do not use evaluation samples or outcomes to tune the controller, feature pipeline, model, threshold, or scenario parameters:

```bash
python3 tools/run_evaluation.py --split evaluation --behavior lidar \
  --output docs/evaluation/lidar-baseline.csv
```

Expected: ten planned rows with the ten frozen config hashes; at least nine stop before contact, and every row has verified collision coverage with `collision_status=no_collision`. Scenario simulation-time limit expiry is `timed_out`; reset, shutdown, competing command writer, or lighting mismatch before completion is `aborted`. Both remain records and count as failures. Run the stale-lidar, already-inside-stop-distance, pause/resume, and reset-mid-run launch tests; a reset aborts the old record, starts a new run ID, and stale command/model data cannot move the car after resume.

The bringup wrapper starts Gazebo server-only with `--force-version 10.5.0 -v 4 -s -r --headless-rendering`, while loading the seed, pose, obstacle, and speed from the named scenario config. Do not start a feature run with standalone `gz sim`, which bypasses the scenario configuration. If EGL cannot initialize in WSL2, retry the same scenario through the wrapper with software rendering:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch car_bringup obstacle_course.launch.py \
  mode:=lidar scenario:=training/stop_smoke headless:=true
```

Verify camera/lidar data and rates before accepting the software-rendering run.

## 6. P4: collect, train, select, and evaluate the camera model

The evaluation split was frozen in P3. Verify its hashes, then collect training and validation data separately; train/select the model checkpoint and classifier threshold using validation only:

```bash
python3 ml/collect_dataset.py validate-splits --require-frozen
python3 ml/collect_dataset.py collect --split training
python3 ml/collect_dataset.py collect --split validation
python3 ml/train_detector.py --train-split training --validation-split validation
```

Expected: generated training/validation images and weights remain under `artifacts/`; every label is linked to a valid synchronized point cloud and the documented `stop_distance_m` trigger rule. No evaluation samples or outcomes are used to tune the controller, feature pipeline, model/checkpoint, threshold, or scenario parameters.

Only after the model and decision threshold are frozen, collect/read the held-out evaluation frames in read-only mode and compute the final detector metrics:

```bash
python3 ml/collect_dataset.py collect --split evaluation --frozen-only
python3 ml/evaluate_detector.py --split evaluation --model artifacts/models/latest --read-only
```

Expected: the first command writes held-out frames/labels under `artifacts/` without changing configs/model; the second reports sample count, precision, recall, confusion counts, and the validation-selected threshold. The held-out set must contain at least 100 valid images of each class overall and at least five of each class from every evaluation scenario; otherwise scoring fails as insufficient support. The `too_close` recall target is at least 90%. These are read-only P4 scoring uses; neither command can update model weights or threshold, and no result feeds tuning.

Run the live AI-assisted behavior:

```bash
ros2 launch car_bringup obstacle_course.launch.py \
  mode:=ai scenario:=evaluation/eval_01 headless:=true \
  model:=artifacts/models/latest
```

Expected: detector publishes `/autonomy/obstacle_detection`; `too_close` can request a stop, `clear` cannot override lidar, and stale/missing or pre-reset AI output switches to lidar-only mode. Perception has no drive-command publisher.

## 7. P5: paired comparison

Run lidar and AI on the exact same ten frozen scenarios, seeds, start poses, obstacle placements, and speeds:

```bash
python3 tools/run_evaluation.py \
  --split evaluation \
  --behaviors lidar,ai \
  --model artifacts/models/latest \
  --paired
```

Expected: one planned row for each of ten scenarios and each behavior (20 planned rows) in `docs/evaluation/results.csv` and a short `docs/evaluation/summary.md`. Each pair has matching config hash and seed and includes detection, pre-contact stop, measured stopping travel, trial status, `lighting_id`, condition tags, collision status, and coverage. An uninterrupted pass requires the lidar 9/10 and AI 8/10 stop goals plus verified coverage and `no_collision` on every row; an unknown collision or incomplete trial fails acceptance. The summary reports unseen-obstacle-placement subgroups separately and makes no lighting-generalization claim. Results apply only to the truck-scale Gazebo prototype.

## Reproducibility and safety checklist

- Use only checked-in scenario YAMLs and fixed seeds. Do not alter the evaluation files after manifest freeze.
- Capture `use_sim_time=true` on every node and record scenario hash, seed, package versions, and run mode.
- Autonomous runs are server-only/headless. Do not launch GUI keyboard control alongside autonomy.
- Autonomous/evaluation launches must use server-only rendering with Gazebo 10.5.0 forced; reject GUI mode for autonomous runs and verify the sole command authority before each trial.
- Generated images, labels, and model weights stay under git-ignored `artifacts/`; regenerated by the scripts above.
- If the existing chassis bumper sensor misses a wheel-only collision, pause evaluation and revise the spec/plan before adding a topic or editing protected assets.
