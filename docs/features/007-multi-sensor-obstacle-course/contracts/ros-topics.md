# Feature 007 ROS Topics

The active vehicle is the Feature 003 RC-scale physics model. Its Gazebo entity remains named `gazebo_truck` for compatibility. Feature 007 uses its additive bridge config at `ros2_ws/src/car_bringup/config/obstacle_course/bridge.yaml`; the new world is named `obstacle_course`, so the world-scoped contact endpoint resolves correctly. The existing RC bridge config remains unchanged. No Feature 001 large-truck point-cloud path or removed local-planner/pure-pursuit interface is required.

New normalized evidence and decision records use versioned JSON strings in `std_msgs/msg/String`. Physical sensor/controller edges remain standard ROS messages.

## Inputs

| ROS topic | Type | Source | Consumer |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Feature 003 RC GPU LiDAR through the active bridge | LiDAR evidence producer; safe-drive controller in RC navigation mode |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Feature 003 RGB camera | Colour-class evidence producer |
| `/camera/labels` | `sensor_msgs/msg/Image` | Feature 003 instance-segmentation camera | Optional diagnostics/ground-truth comparison; not the default detector |
| `/detections` | `vision_msgs/msg/Detection2DArray` | Optional frozen Feature 001 classifier only | Camera evidence producer only for `camera.source:=frozen_feature_001_diagnostic`; diagnostic/unqualified runs only, with verified model hash and source-label preservation |
| `/imu/data` | `sensor_msgs/msg/Imu` | Feature 003 IMU | IMU evidence producer and safe-drive controller |
| `/odom` | `nav_msgs/msg/Odometry` | Feature 003 odometry bridge | Odometry evidence producer, route-waypoint follower, vehicle-visual overlay pose source, route progress, and stopping-distance calculation |
| `/contacts` | `ros_gz_interfaces/msg/Contacts` | Feature 003 bumper contact sensor | Contact evidence producer and run logger |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo simulation clock | All nodes using simulation-time freshness and decision stamps |

## Feature 007 outputs

| ROS topic | Type | Publisher | Consumer |
|---|---|---|---|
| `/decision/evidence/lidar` | `std_msgs/msg/String` | LiDAR evidence producer | Decision engine, logger, dashboard |
| `/decision/evidence/camera` | `std_msgs/msg/String` | Camera evidence producer | Decision engine, logger, dashboard |
| `/decision/evidence/imu` | `std_msgs/msg/String` | IMU evidence producer | Decision engine, logger, dashboard |
| `/decision/evidence/odometry` | `std_msgs/msg/String` | Odometry evidence producer | Decision engine, logger, dashboard |
| `/decision/evidence/contact` | `std_msgs/msg/String` | Contact evidence producer | Decision engine, logger, dashboard |
| `/decision/state` | `std_msgs/msg/String` | Decision node | Dashboard and run logger |
| `/decision/event` | `std_msgs/msg/String` | Decision node | Dashboard and run logger |
| `/decision/planner_intent` | `std_msgs/msg/String` | Decision node | Feature 007 controller-request adapter |
| `/scenario/fault_event` | `std_msgs/msg/String` | Tracked scenario fault gate | Run logger and live diagnostic tools; validated by `fault-event.schema.json` |
| `/manual/cmd_vel_intent` | `geometry_msgs/msg/Twist` | Keyboard teleop in manual mode only | Feature 007 manual-Twist adapter |
| `/ackermann_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | Feature 007 manual-Twist adapter in manual mode or controller-request adapter in autonomous mode; never both | Existing safe-drive controller in `rc_navigation` mode |
| `/model/gazebo_truck/cmd_vel` | `geometry_msgs/msg/Twist` | Existing safe-drive controller only | The single ROS-to-Gazebo command bridge |

In autonomous mode, the Feature 007 controller-request adapter converts short-lived route/avoidance intent into bounded `AckermannDriveStamped` requests using the RC profile's wheelbase, steering, speed, and yaw limits. In manual mode, `manual_twist_adapter.py` converts the selected keyboard `Twist` intent to the same bounded Ackermann request; launch starts exactly one of these publishers. Expired autonomous intents, stale critical evidence, and `STOP` produce a zero-speed request. The safe-drive controller remains responsible for command leases, bounds, sensor checks, and the final Gazebo command. No decision, evidence, dashboard, or teleop process publishes `/model/gazebo_truck/cmd_vel` directly.

## Gazebo Service

| ROS service | Type | Purpose |
|---|---|---|
| `/world/obstacle_course/set_pose` | `ros_gz_interfaces/srv/SetEntityPose` | Move the collisionless `gazebo_truck_visual` companion to the transformed `/odom` pose or move the tracked S5 reveal entity from its off-course staging pose to its configured target; neither operation commands or changes the RC vehicle physics entity. |

`vehicle_visual_overlay.py` sends pose updates only for `gazebo_truck_visual`. It captures the initial odometry pose and configured world spawn pose, then transforms subsequent `/odom` poses into world coordinates before calling the service. `fault_gate_node.py` uses the same service only for the S5 entity named in the tracked reveal config. The launch path starts the world-scoped service bridge and these separate consumers together. The Feature003 research mesh remains a separate world entity because the protected RC model has no chassis visual; the end-to-end check confirms the visible companion follows the truck through straight and turning motion.

In autonomous `behavior_demo`, the decision engine is the only intent source. In manual mode, it is disabled and teleop is the only intent source. Exactly one selected adapter publishes `/ackermann_cmd` in either mode. Dashboard and logger subscriptions are read-only; stopping them cannot alter command behavior.
