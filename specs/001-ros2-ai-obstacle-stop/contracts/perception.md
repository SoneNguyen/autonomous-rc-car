# Perception-to-Controller Contract

## Published message

| Property | Contract |
|---|---|
| Topic | `/autonomy/obstacle_detection` |
| Type | `vision_msgs/msg/Classification` |
| Publisher | `camera_obstacle_detector` only |
| Expected maximum rate | 10 Hz; source camera is 30 Hz. Process the latest image and drop obsolete queued frames. |
| Header | Copy the exact `sensor_msgs/Image.header` (simulation timestamp and source frame ID). Never substitute wall time. |
| Results | Exactly two `vision_msgs/msg/ObjectHypothesis` entries: `class_id="clear"` and `class_id="too_close"`; each score is a finite probability in [0,1], and scores sum to 1 within numeric tolerance. |
| Input | `/model/gazebo_truck/sensors/front_camera/image`, `sensor_msgs/msg/Image`, expected `rgb8`, 640x480. Resize to 160x120 for the small CNN. |
| QoS | Sensor-data profile; subscription compatible with camera bridge. |

## Meaning

- `too_close` means the learned image classifier estimates that a non-ground obstacle lies in the forward driving corridor at or within the scenario's frozen stopping threshold.
- `clear` means the classifier estimates the corridor is clear. It is advisory only: it cannot override a lidar stop, stale lidar, stale odometry, or other safe-stop condition.
- Detection threshold is selected on validation scenarios only and stored in model metadata. Evaluation scenarios cannot change it.
- The classification is a whole-image binary result, not a bounding box or obstacle localization. No claim of steering, avoidance, or real-road detection is made.

## Freshness and arbitration

- A result is usable only when its header is nonzero, not in the future relative to `/clock`, matches the current run, and is no older than three expected model periods (0.3 simulated seconds at 10 Hz).
- `too_close` above the frozen threshold may request a stop from `safe_drive_controller`.
- `clear` never cancels a lidar stop.
- Missing/invalid/stale model output switches the controller to lidar-only mode and records `ai_stale_fallback`.
- If lidar is stale/invalid, controller issues zero regardless of model output.
- Perception has no publisher to `/model/gazebo_truck/cmd_vel` or `/autonomy/cmd_vel_request`.

## Dataset rule

Each output can be traced to one source image, one lidar point cloud within the synchronization tolerance, a scenario ID/hash, and the active label-rule version. Training and validation membership come from scenario configs, never from random frame-level splitting. The ten evaluation scenario IDs and hashes are frozen before training.
