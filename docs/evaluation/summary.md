# Lidar Baseline Results

Results apply only to the truck-scale Gazebo prototype. The evaluation configs are frozen and were recorded read-only; these outcomes must not be used to tune controller or scenario parameters.

Planned rows: 10 / 10. A pass requires at least 9 pre-contact stops, ten verified coverage rows, ten `no_collision` rows, and no timed-out/aborted trial.

| Condition group | Rows | Detected | Pre-contact stops | Completed | Mean stopping travel | Verified no-collision | Unknown/collision |
|---|---:|---:|---:|---:|---:|---:|---:|
| unseen_obstacle_placement | 10 | 10 | 10 | 10 | 0.171 m | 10 | 0 |
| seen_or_unlabelled | 0 | 0 | 0 | 0 | n/a | 0 | 0 |

Trial rows are in [lidar-baseline.csv](lidar-baseline.csv).

Observed P3 checkpoint: **PASS** — 10/10 detected, 10/10 pre-contact stops, 10/10 completed, and 10/10 verified no-collision rows.
Measured odometry stopping travel: mean 0.171 m; range 0.049–0.397 m across 10 rows with a recorded stop.
