# Obstacle-stop evaluation records

Evaluation outputs for the truck-scale Gazebo prototype belong here. They are
tracked evidence; scenario-split manifests, ROS logs, and run scratch files stay
under git-ignored `artifacts/`.

## P3 lidar baseline

1. Calibrate only with `training/calibration` and `training/stop_smoke`, using
   the scenario's 5.0 m stop trigger and checking the live recorded stopping
   travel. Never tune from evaluation runs.
2. Run `colcon test --packages-select car_bringup --ctest-args -R
   test_contact_coverage` to confirm the current chassis contact sensor reports
   the selected gates. It covers both `gate_left_collision` and
   `gate_right_collision`; the existing P1 contact test covers
   `post_01_collision`. Other post/barrier contacts and wheel-only impacts are
   not certified by those checks.
3. Freeze/validate scenarios with
   `python3 ml/collect_dataset.py validate-splits --freeze-evaluation`. This
   writes `artifacts/scenario-splits/split-manifest.json` and hashes all ten
   evaluation configs. Three sets are disjoint by the normalized full tuple
   `(start_pose, obstacle entity/collision/pose, commanded_speed_mps)`; seed or
   filename differences alone do not qualify.
4. Run the ten lidar trials once with the command in the feature quickstart.
   The runner checks for the single ROS controller publisher and single Gazebo
   Transport bridge writer before and during each trial. Results are
   append-only per invocation: it refuses to overwrite an existing output file.

The expected evaluation obstacle is the existing 6 m tall right-side gate,
which is not used in training (training calibrates on the left-side gate).
The existing chassis-only contact sensor was deliberately exercised against
that gate before baseline runs. The roof lidar's vertical field leaves a
near-field blind zone on the 3 m posts, so they are not used as P3 acceptance
targets. No world or vehicle model edits are part of this story.

## Record semantics

- `stop_distance_m` in scenario configs is the sensor trigger distance from the
  chassis front collision plane; `stopping_distance_m` in CSV is measured
  odometry travel from the first final zero command until the rest condition.
- `completed` means the vehicle rested and the record was flushed. It does not
  itself mean a successful obstacle stop. A timeout is `timed_out`; reset,
  shutdown, lighting mismatch, or a competing command writer is `aborted`.
- `collision_status=collision` requires an allowlisted non-ground contact.
  `no_collision` requires verified coverage for the selected obstacle surface;
  otherwise status is `unknown`. Ground/floor contacts are excluded.
- `stopped_before_contact` requires a fresh lidar `too close` detection followed
  by a zero command before any matching contact. Missing detection cannot count
  as a successful baseline stop.
- Every run has a unique UUID. Retries never replace earlier rows; any retry
  needs a fresh output path and must retain the same frozen scenario config.

The runner also writes `summary.md`, including the unseen-obstacle-placement
subgroup. An uninterrupted P3 acceptance requires ten planned rows, at least
nine pre-contact stops, ten verified coverage rows with `no_collision`, and no
timed-out/aborted attempts. Evaluation results are read-only and must not feed
controller, feature-pipeline, model, threshold, or scenario tuning. Any result
statement must explicitly apply to this starter-physics prototype, not to a
real vehicle.

## P3 baseline result (2026-10-20)

The frozen ten-run lidar baseline passed its specified P3 gate:

- 10/10 runs detected the gate with lidar and stopped before contact.
- 10/10 were `completed`; collision coverage was verified in every row and
  `collision_status=no_collision` in all ten.
- Measured stopping travel from the controller's first final zero output to the
  logger's settled-rest condition averaged **0.171 m** (range **0.049–0.397 m**).
- All ten runs carry the `unseen_obstacle_placement` tag because the evaluation
  right gate is not used in training; this is one fixed obstacle placement, not
  broad placement-generalization evidence.

The source CSV and generated summary are
[`lidar-baseline.csv`](lidar-baseline.csv) and [`summary.md`](summary.md). The
controller threshold remained 5.0 m, and evaluation outcomes were not used to
change code or configuration. Metrics apply only to this Gazebo starter-physics
prototype.
