# Machine-learning workflow

Dataset collection, model training, and read-only detector evaluation belong
in this directory and run under the ROS 2 system Python in WSL2 Ubuntu 26.04.
PyTorch is installed from apt; do not introduce a separate Conda/Pixi runtime.

Generated images, labels, split manifests, and weights are written below the
root `artifacts/` directory, which is git-ignored. Keep scripts and small
configuration/code files in `ml/`; provide commands that regenerate generated
data rather than committing it.

The planned scripts are `collect_dataset.py`, `train_detector.py`, and
`evaluate_detector.py`. Dataset labels come from synchronized lidar point
clouds: `too_close` means an allowlisted non-ground return in the forward
corridor is at or inside the scenario's frozen `stop_distance_m`; stale or
unmatched sensor samples are invalid, never `clear`. Training uses only the
training split, model/threshold selection uses validation, and evaluation is
frozen and read-only.
