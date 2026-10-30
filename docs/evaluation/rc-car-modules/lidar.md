# LiDAR qualification

**Status: qualified — 14/14 fixed-seed cases passed on the configured GPU
LiDAR.** Full-run report: `788604d0-4b41-4eb3-b6bd-ebb092e6731e`. This result
uses WSLg server-only rendering through Mesa D3D12 on the NVIDIA RTX A2000
Laptop GPU. The test reads live `/scan` messages from the configured 361-beam,
10 Hz sensor in `lidar_link`; it does not synthesize ranges or substitute a
CPU-ray sensor.

| Coverage | Observed result |
|---|---|
| Minimum range below / at / inside | No return below 0.05 m; 0.05190 m at the 0.05 m boundary; 0.06200 m at 0.06 m. |
| Maximum range inside / at / beyond | 7.99539 m at 7.99 m; 7.99939 m at 8.00 m; no return at 8.01 m. |
| Five planner sectors | Right, front-right, front, front-left, and left all returned the research target at approximately 3.0 m in the expected sector. |
| Moving obstacle | 21 finite target returns and 16 acknowledged pose updates; measured range decreased during approach and increased after turnaround. |
| Ground-only | Zero finite returns. |
| Stale-sensor stop | 31 scans forwarded before injection; at 0.6 s scan age the controller reported `sensor_lidar_stale`, emitted zero output, and retained its `controller_not_ready` stop reason. |

The rendered target placement is tied to each locked research model's collision
dimensions. Targets are raised to the mounted scan plane so the one-plane LiDAR
tests the research mesh rather than only the ground. Boundary scoring uses the
visible target's angular footprint and records the actual beam index and range.

## WSLg rendering

On WSLg, keep `DISPLAY` or `WAYLAND_DISPLAY` available in the shell. The capture
tool detects WSLg, selects server-only Gazebo rendering (`headless:=glx`), and
sets Mesa's D3D12 driver and NVIDIA adapter for Gazebo. It writes the selected
renderer and adapter to each LiDAR capture case's
`render-configuration.json`. Gazebo's
`--headless-rendering` EGL path used by the earlier attempt selected software
rendering on this machine and produced no target returns. Both paths keep the
viewer closed; only the WSLg GLX path provides the sensor renderer used for this
qualification.

## Reproduce

From the repository root, with ROS sourced and the workspace built:

~~~bash
python3 tools/rc_module_matrix.py --module lidar --quick
python3 tools/rc_module_matrix.py --module lidar
~~~

Quick mode is smoke coverage only and cannot qualify the module. A full run
stores raw scans, pose-service results, relay status, renderer configuration,
and Gazebo logs under ignored `artifacts/rc-car-modules/<run-id>/`.
