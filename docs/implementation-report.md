# Gazebo vehicle and obstacle-course implementation report

## Purpose and scope

This workspace provides a Gazebo Sim 10 prototype for early autonomous-driving
research.  It turns a Blender vehicle asset into an independently movable
four-wheel Gazebo model, places it in a sensor-enabled obstacle course, and
exposes the sensor and control interfaces needed to develop perception,
planning, and basic closed-loop control.

It is a research and integration baseline.  It is **not** currently a
calibrated digital twin from which real-world braking distance, tyre grip, or
handling limits can be predicted.

## 1. Gazebo runtime setup

The workspace is configured through Pixi rather than modifying Gazebo files in
the Pixi environment:

| Item | Implemented configuration |
| --- | --- |
| Package manager | Pixi / conda-forge |
| Target platform | `win-64` |
| Simulator dependency | `gz-sim = "10.*"` |
| Qt setup | Pixi activation sets the Qt platform-plugin and QML import paths |
| Project resource discovery | Launch commands set `GZ_SIM_RESOURCE_PATH` to `simulation/models` |

This makes the local `car` model discoverable as `model://car` while keeping
the Gazebo examples inside `.pixi` untouched.  A Pixi environment rebuild
therefore does not overwrite the project model or worlds.

Launch any project world from the workspace root with:

```powershell
$env:GZ_SIM_RESOURCE_PATH = Join-Path (Get-Location).Path 'simulation\models'
pixi run gz sim -r -v 4 simulation/worlds/obstacle.sdf
```

The `-r` flag starts physics immediately.  Without it, Gazebo opens paused and
velocity commands do not move the vehicle.

## 2. Vehicle asset preparation

### Source asset and mesh export

The source file, `assets/source/chassis.blend`, contains one connected mesh object named
`Truck`, rather than a separate chassis object and four wheel objects.  The
exporter `export_gazebo_parts.py` analyses connected mesh islands and produces
the following Gazebo-ready visual meshes:

| Output | Role | Triangles |
| --- | --- | ---: |
| `meshes/chassis.stl` | Main body and non-wheel geometry | 11,835 |
| `meshes/wheel_fl.stl` | Front-left wheel, hub-centred | 2,392 |
| `meshes/wheel_fr.stl` | Front-right wheel, hub-centred | 2,402 |
| `meshes/wheel_rl.stl` | Rear-left wheel, hub-centred | 2,478 |
| `meshes/wheel_rr.stl` | Rear-right wheel, hub-centred | 2,480 |

The exporter converts Blender axes to Gazebo vehicle axes:

```text
Blender: +Y forward, +X right, +Z up
Gazebo:  +X forward, +Y left,  +Z up
```

The exact component poses, source transform, dimensions, and wheel radii are
recorded in `meshes/mesh_manifest.json`, so the model does not rely on guessed
wheel locations.

### Vehicle model

`simulation/models/car/model.sdf` implements the dynamic vehicle:

- detailed STL visuals for the chassis and each wheel;
- simple box and cylinder collision geometry, chosen for stable simulation;
- front steering links and four wheel links;
- an Ackermann drive system accepting `gz.msgs.Twist` commands on
  `/model/gazebo_truck/cmd_vel`;
- odometry on `/model/gazebo_truck/odometry`;
- a body contact monitor on
  `/model/gazebo_truck/sensors/bumper_contact`; and
- manual keyboard-command publishers for a smoke test in the 3D view.

The asset was not silently rescaled.  It is a large truck-scale model: about
15.36 m long, about 10.7 m wide, with tyres approximately 3.76 m in diameter.
The obstacle course was sized for that vehicle.

## 3. Perception and safety sensors

The chassis carries the following sensor suite:

| Sensor | Configuration | Main topic | Use in the prototype |
| --- | --- | --- | --- |
| RGB camera | 640 x 480, 30 Hz, forward and slightly downward | `/model/gazebo_truck/sensors/front_camera/image` | Lane-marking and visual obstacle research |
| GPU lidar | 180 deg horizontal x 30 deg vertical, 720 x 16 beams, 15 Hz | `/model/gazebo_truck/sensors/front_lidar/scan` | Obstacle range and clearance |
| IMU | 100 Hz | `/model/gazebo_truck/sensors/imu` | State-estimation interface |
| Contact sensor | 100 Hz | `/model/gazebo_truck/sensors/bumper_contact` | Collision event evaluation |

The lidar was deliberately changed from a single horizontal layer to 16
vertical layers.  The original roof-mounted planar ray could pass above the
course barriers, producing non-detections.  The multi-layer scan returns
obstacle data at the barrier and post heights.

## 4. Obstacle-course environment

`simulation/worlds/obstacle.sdf` provides a repeatable, truck-scale driving
scenario:

- a 280 m x 50 m road with a dashed lane-following guide, centre divider, edge
  lines, start line, and finish line;
- two red physical lane-blocking barriers that require a lane change;
- three orange physical marker posts for obstacle/slalom tests;
- a blue finish gate that is visible to the camera and lidar;
- collision and visual geometry for obstacles;
- an Ogre2 overview camera with a long far clip, positioned to show the course
  and vehicle at startup; and
- a docked front-camera display in the Gazebo GUI.

The initial rendering issue was a presentation issue rather than missing
course geometry: Gazebo's generic camera begins near the world origin, while
the vehicle starts at `x = -105 m`.  A world-local Ogre2 GUI view now frames
the course correctly.  The road markings were also changed from overlapping
thin boxes to slightly raised visual planes, removing z-fighting in the camera
image.

### Course route

The vehicle starts on the dashed guide at approximately `x = -105 m`,
`y = -10 m`.

1. Move forward along the guide.
2. Before the first red barrier at `x = -20 m`, move left toward positive Y.
3. After clearing it, move right toward negative Y to pass the second red
   barrier at `x = 40 m`.
4. Pass through the blue gate near `x = 80 m` and reach the finish line at
   `x = 115 m`.

## 5. How the implementation was tested

### Static validation

Both SDF documents were checked with Gazebo's SDF validator:

```powershell
pixi run gz sdf -k simulation/worlds/obstacle.sdf
pixi run gz sdf -k simulation/models/car/model.sdf
```

Both commands returned `Valid.`

### Runtime interface validation

The obstacle course was started in an isolated Gazebo partition and the live
topic list was inspected.  The following required topics were present:

```text
/model/gazebo_truck/cmd_vel
/model/gazebo_truck/odometry
/model/gazebo_truck/sensors/front_camera/image
/model/gazebo_truck/sensors/front_lidar/scan
/model/gazebo_truck/sensors/front_lidar/scan/points
/model/gazebo_truck/sensors/imu
/model/gazebo_truck/sensors/bumper_contact
```

The live lidar message reported `count: 720` and `vertical_count: 16`, with
finite ranges.  The contact publisher reported the `gz.msgs.Contacts` type,
and the IMU published a valid orientation, angular velocity, and acceleration
message.

### Manual drive-through procedure

1. Launch `simulation/worlds/obstacle.sdf` with `-r`.
2. Click the Gazebo **3D View** so it receives key presses.
3. Use `W` to drive forward, `A` for forward-left, `D` for forward-right,
   `X` for reverse, and `S` to stop.
4. First make a short forward/stop test and verify that the front-camera panel
   updates.
5. Then drive the route above at low speed.  Watch the camera and lidar before
   each barrier, and stop before a collision.
6. To confirm that a deliberate low-speed collision is detected, monitor the
   contact topic in a second terminal:

```powershell
pixi run gz topic -e -t /model/gazebo_truck/sensors/bumper_contact
```

For direct scripted commands instead of the keyboard, publish to the same
command topic:

```powershell
pixi run gz topic -t /model/gazebo_truck/cmd_vel -m gz.msgs.Twist -p 'linear { x: 1.0 } angular { z: 0.0 }'
```

Stop with a zero command:

```powershell
pixi run gz topic -t /model/gazebo_truck/cmd_vel -m gz.msgs.Twist -p 'linear { x: 0.0 } angular { z: 0.0 }'
```

## 6. Runtime notes and current limitations

- On this Windows/Pixi installation, `gz topic` can display a
  `DynamicFactory()` descriptor-set warning.  The tested topics still listed,
  subscribed, and delivered messages; treat this as a package warning unless
  the expected topic itself is absent.
- The Gazebo renderer was observed using the integrated Intel GPU rather than
  the installed RTX GPU.  If the GUI is sluggish, set `gz-sim-main.exe` and
  `gz-sim-gui-client.exe` to **High performance** in Windows Graphics settings
  and restart Gazebo.
- The visible road is currently a visual layer above the world's collision
  plane.  This is suitable for perception and route planning, but road/tyre
  friction still needs calibration before dynamics research.
- Current mass, inertia, collision extents, tyre friction, actuator response,
  braking, and suspension parameters are starter values.  Gazebo's current
  physics engine must be calibrated with the selected physical platform rather
  than assumed from mesh appearance.
- The contact sensor is an evaluation and emergency-event signal, not an
  obstacle-avoidance method.  An autonomy controller should stop from lidar or
  camera perception before body contact.

## 7. Recommended next research milestones

1. Choose the intended real platform and measure wheelbase, track, tyre size,
   mass, centre of gravity, steering limits, brakes, and sensor mounting.
2. Rescale or replace the truck asset if the physical target is a passenger
   car; update collision primitives and inertial values from measurements.
3. Add realistic sensor noise, camera distortion, latency, dropped frames,
   lighting changes, and repeatable random seeds.
4. Implement perception for lane markings and obstacle detection, then fuse
   IMU and wheel information for state estimation.
5. Add a safety-stop policy based on lidar distance/time-to-collision before
   attempting automatic lane following.
6. Measure each run: route completion, lateral lane error, minimum clearance,
   detection range, contact count, and controller stability.
7. Validate against low-speed physical tests only after the vehicle and sensor
   model have been calibrated.

## Project files

| File | Purpose |
| --- | --- |
| `pixi.toml` | Windows Pixi/Gazebo runtime dependency and Qt activation |
| `simulation/models/car/model.config` | Gazebo model package metadata |
| `simulation/models/car/model.sdf` | Vehicle links, dynamics, sensors, and control plugins |
| `simulation/worlds/car_demo.sdf` | Minimal vehicle driving demo |
| `simulation/worlds/obstacle.sdf` | Sensor-enabled driving/obstacle scenario |
| `simulation/models/car/meshes/mesh_manifest.json` | Exported component geometry and hub-pose record |
| `assets/source/chassis.blend` | Blender source model |
| `tools/blender/export_gazebo_parts.py` | Blender-to-STL exporter |
| `README.md` | Repository quick-start commands |
| `docs/implementation-report.md` | This implementation, testing, and research record |
