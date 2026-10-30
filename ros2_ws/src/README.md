# ROS workspace packages

This workspace contains the runtime used by the RC sensor and actuation
matrix:

- `car_bringup`: research-world generation launch, bridge configuration, and
  the shared vehicle profile.
- `car_navigation`: static sensor transforms and the matrix's frozen camera
  detector node.
- `car_autonomy_nodes`: the bounded command controller used by command-path
  observations.

Build from the repository root using the commands in `docs/quickstart.md`.
