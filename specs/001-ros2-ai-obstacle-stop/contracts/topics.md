# Gazebo and ROS Topic Contract

The Gazebo topic column is authoritative and must match the root README and installed SDF. The ROS-side names and message types below are the planned public interface. Keep this table, bridge YAML, and ROS launch parameters in sync.

| Purpose | Gazebo topic / type | ROS topic / type | Direction | Expected rate / timing | Frame, units, and QoS |
|---|---|---|---|---|---|
| Drive command | `/model/gazebo_truck/cmd_vel` / `gz.msgs.Twist` | `/model/gazebo_truck/cmd_vel` / `geometry_msgs/msg/Twist` | ROS to Gazebo only | Controller publishes at 20 Hz; command lease 0.5 sim s | SI: m/s and rad/s. Default clamp: `linear.x` +/-2.0, `angular.z` +/-0.15. Reliable/default QoS. Sole ROS publisher is `safe_drive_controller`. |
| Odometry | `/model/gazebo_truck/odometry` / `gz.msgs.Odometry` | same / `nav_msgs/msg/Odometry` | Gazebo to ROS | 50 Hz expected from Ackermann plugin default; verify after launch | Pose in `odom`, child/body frame from actual message; m, m/s, radians, rad/s. Sensor-data QoS. |
| Camera | `/model/gazebo_truck/sensors/front_camera/image` / `gz.msgs.Image` | same / `sensor_msgs/msg/Image` | Gazebo to ROS | 30 Hz, 640x480 RGB | Preserve source header stamp and source frame ID. Encoding `rgb8` expected; validate actual message. Sensor-data QoS. |
| Lidar scan | `/model/gazebo_truck/sensors/front_lidar/scan` / `gz.msgs.LaserScan` | same / `sensor_msgs/msg/LaserScan` | Gazebo to ROS | 15 Hz, source has 720 horizontal x 16 vertical rays | The bridge keeps only the middle vertical row. This topic is for display/inspection, not stopping logic. Sensor-data QoS. |
| Lidar cloud | `/model/gazebo_truck/sensors/front_lidar/scan/points` / `gz.msgs.PointCloudPacked` | same / `sensor_msgs/msg/PointCloud2` | Gazebo to ROS | 15 Hz, retain all 16 vertical layers | SI point coordinates; transform from observed point-cloud frame to `base_link` using the SDF lidar pose `(7.2, 0, 3.5) m`, zero RPY, subject to runtime frame verification. Sensor-data QoS. |
| IMU | `/model/gazebo_truck/sensors/imu` / `gz.msgs.IMU` | same / `sensor_msgs/msg/Imu` | Gazebo to ROS | 100 Hz | Angular velocity rad/s, linear acceleration m/s^2; preserve actual sensor frame and sim timestamp. Sensor-data QoS. |
| Body contact | `/model/gazebo_truck/sensors/bumper_contact` / `gz.msgs.Contacts` | same / `ros_gz_interfaces/msg/Contacts` | Gazebo to ROS | Sensor configured 100 Hz; data may be event-only/empty while no contact | Contact names/IDs identify collision pair; count only configured obstacle collision names and exclude ground. Sensor-data QoS. Existing sensor watches chassis collision only. |
| Simulation clock | `/clock` / `gz.msgs.Clock` | `/clock` / `rosgraph_msgs/msg/Clock` | Gazebo to ROS only | Simulation clock updates; not wall time | Sole clock source. Every ROS node sets `use_sim_time=true`; never create a second `/clock` publisher. Clock QoS profile. |
| ROS command request | N/A | `/autonomy/cmd_vel_request` / `geometry_msgs/msg/Twist` | Client to controller | External P2 requests must refresh faster than the 0.5 sim s lease | Bounded and validated by controller; it is not bridged. A client publishes requests, never to Gazebo command output. |
| Perception result | N/A | `/autonomy/obstacle_detection` / `vision_msgs/msg/Classification` | Detector to controller/logger | Up to 10 Hz; timestamp is the source camera stamp | `clear`/`too_close`, confidence scores in [0,1]. Detection node never publishes drive commands. |

## Planned bridge YAML shape

`ros2_ws/src/car_bringup/config/ros_gz_bridge.yaml` will use the `ros_gz_bridge` parameter-bridge YAML schema; the excerpt below illustrates direction and QoS. Complete entries must cover all rows marked bridgeable above.

```yaml
- ros_topic_name: /model/gazebo_truck/cmd_vel
  gz_topic_name: /model/gazebo_truck/cmd_vel
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: gz.msgs.Twist
  direction: ROS_TO_GZ

- ros_topic_name: /model/gazebo_truck/sensors/front_lidar/scan/points
  gz_topic_name: /model/gazebo_truck/sensors/front_lidar/scan/points
  ros_type_name: sensor_msgs/msg/PointCloud2
  gz_type_name: gz.msgs.PointCloudPacked
  direction: GZ_TO_ROS
  qos_profile: SENSOR_DATA

- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
  qos_profile: CLOCK
```

## Startup and validation requirements

- Start Gazebo before the bridge, then wait for `/clock` and sensor topics before enabling the controller.
- All node parameters and launch actions use simulation time. A wall-clock watchdog is not an acceptable substitute for the sim-time expiry requirement.
- Verify actual rates with `ros2 topic hz` and timestamp advancement while the sim runs; pause the world and confirm simulation timestamps stop advancing.
- Verify QoS using `ros2 topic info -v`; the subscriber QoS must be compatible with the bridge publisher.
- Contact messages are events, not a required continuous stream in an obstacle-free state. Test a deliberate non-ground obstacle contact to prove the topic is active.
- Headless autonomous launch is the only supported evaluation launch. GUI keyboard control remains available only for non-autonomous inspection.
- The autonomous authority invariant is one ROS publisher (`safe_drive_controller`) on the bridged command topic and one Gazebo Transport writer (`ros_gz_bridge`) on the Gazebo topic. Check both graphs before each trial and monitor for additional writers during it; refuse launch or abort, persist an aborted row, and stop Gazebo if one appears.
