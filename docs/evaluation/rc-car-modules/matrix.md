# RC sensor and actuation qualification

Qualification is recorded only when every required fixed-seed case for a module
passes. A passing module does not qualify the complete matrix.

| Module | Current result | Evidence |
|---|---|---|
| LiDAR | Qualified: all 14 fixed-seed cases pass on the configured GPU sensor using WSLg D3D12 rendering. | Report `788604d0-4b41-4eb3-b6bd-ebb092e6731e`; see [LiDAR status](lidar.md). |
| Camera/detection | Not qualified. Scenarios and capture path exist; a complete scored set has not passed. | See [camera status](camera.md). |
| IMU/odometry | Qualified: all four fixed-seed cases pass. | Report `6813ba88-6e7a-4dc0-a430-af336a7c173c`. |
| Contact/TF | Qualified: all three fixed-seed cases pass with zero sampled TF gaps. | Report `dbfba035-4709-4916-8093-7765b1347403`. |
| Command path (stretch) | Qualified: all three fixed-seed cases pass. | Report `a5ef0659-a54e-4e0b-96bd-fee849fe7b09`. |

The IMU result characterizes the configured simulated sensor noise and odometry
model only. It is not a hardware accuracy claim. The command-path stretch set
remains optional and is reported separately from the core set.

Run a module from a sourced ROS 2 Lyrical environment with
`python3 tools/rc_module_matrix.py --module <module>`. Reports and raw evidence
are append-only under the ignored `artifacts/rc-car-modules/<run-id>/` tree.
Quick mode is smoke coverage and never qualifies a module.
