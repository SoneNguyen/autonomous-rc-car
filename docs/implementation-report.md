# Implementation Report

## Delivered

- Fixed-seed scenario sets and overlap preflight for LiDAR, camera/detection,
  IMU/odometry, contact/TF, and the optional command path.
- Server-only per-scenario Gazebo worlds built from the locked research models,
  research truck visual, and road assets. On WSLg, the sensor capture selects
  Mesa D3D12 on the NVIDIA adapter so the configured GPU LiDAR renders without
  opening a viewer. The RC vehicle model supplies collision, sensor, and drive
  physics.
- Live capture and scoring for IMU/odometry motion and reset, contact identity
  and per-sample TF, controller bounds/watchdog/response, LiDAR range cases,
  moving-target pose injection, and stale-scan stop behavior.
- Append-only JSON reports and raw captures under ignored
  `artifacts/rc-car-modules/`, plus tracked per-module results under
  `docs/evaluation/rc-car-modules/`.

## Qualification status

| Module | Result |
|---|---|
| IMU/odometry | Qualified: 4/4 cases passed. Report `6813ba88-6e7a-4dc0-a430-af336a7c173c`. Noise and drift bounds describe the simulated model, not hardware accuracy. |
| Contact/TF | Qualified: 3/3 cases passed; zero TF lookup gaps. Report `dbfba035-4709-4916-8093-7765b1347403`. |
| Command path (stretch) | Qualified: 3/3 cases passed. Report `a5ef0659-a54e-4e0b-96bd-fee849fe7b09`. |
| LiDAR | Qualified: all 14 fixed-seed cases passed, including min/max range boundaries, all five sectors, moving-target range trend, zero ground-only returns, and stale-scan stop. Report `788604d0-4b41-4eb3-b6bd-ebb092e6731e`. |
| Camera/detection | Not qualified: the camera scenario set has not passed a complete scored run. |

The earlier EGL headless path selected llvmpipe and produced no finite LiDAR
returns. The capture tool now detects WSLg and uses server-only GLX with Mesa
D3D12 on the NVIDIA adapter. The qualified report records this render
configuration for each LiDAR sensor-capture case. The report uses measured
samples from the configured GPU LiDAR.

See [module details](evaluation/rc-car-modules/matrix.md) and the
[quickstart](quickstart.md) for commands and observed limits.

## Runtime and assets

The target is WSL2 Ubuntu 26.04, ROS 2 Lyrical, and Gazebo Sim 10.5.0, installed
with apt. Research mesh attribution and locks are recorded in
[`research-asset-licenses.md`](research-asset-licenses.md). Fetch and verify
assets before running; runtime runs use local files and need no network access.
