# Gazebo truck mesh package

`chassis.blend` contains one mesh object, `Truck`, rather than five objects.
The exported Gazebo-ready parts are in [`meshes`](meshes):

- `chassis.stl` — 11,835 triangles, all non-wheel geometry
- `wheel_fl.stl` — front-left wheel, 2,392 triangles
- `wheel_fr.stl` — front-right wheel, 2,402 triangles
- `wheel_rl.stl` — rear-left wheel, 2,478 triangles
- `wheel_rr.stl` — rear-right wheel, 2,480 triangles

Each wheel STL is centered at its own hub. `chassis.stl` is centered midway
between the axles, at their average hub height. The mesh convention is Gazebo
vehicle axes: +X forward, +Y left, +Z up. Exact dimensions and link poses are
in [`meshes/mesh_manifest.json`](meshes/mesh_manifest.json).

The source blend uses metric units with a scale of 1. The resulting vehicle is
large: about 15.36 m long, with approximately 3.76 m diameter wheels. No scale
has been guessed or silently applied.

`model.sdf` is a dynamic starter model: visuals use the detailed STL meshes,
while the chassis and wheels use a box and cylinders for collision stability.
It includes front steering and an Ackermann drive plugin. Treat the mass,
inertia, friction, and collision box as sensible starting values to tune for
the intended vehicle behavior.

This is a truck-scale integration model, not yet a calibrated digital twin of
a passenger car. It is appropriate for validating perception, basic planning,
and controller interfaces; measured vehicle dimensions, mass distribution,
tyre data, steering limits, and sensor placement must replace the starter
values before using results to predict real-world handling or stopping.

For the complete setup, implementation, validation, and research handoff, see
[`../../../docs/implementation-report.md`](../../../docs/implementation-report.md).

## Launch the demo

From the repository root in PowerShell:

```powershell
$env:GZ_SIM_RESOURCE_PATH = Join-Path (Get-Location).Path 'simulation\models'
pixi run gz sim -r -v 4 simulation/worlds/car_demo.sdf
```

The model listens to the Ackermann system's default velocity-command topic.
The installed Gazebo sample world under `.pixi` has deliberately not been
modified, so these assets remain safe across Pixi environment rebuilds.

`-r` is important: it starts physics immediately. Without it, Gazebo opens
paused and a velocity command cannot move the truck. If the demo is already
open and paused, click the GUI's play button or run:

```powershell
pixi run gz service -s /world/car_demo/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'
```

For example, after the demo starts, send a gentle forward / left-turn command:

```powershell
pixi run gz topic -t /model/gazebo_truck/cmd_vel -m gz.msgs.Twist -p 'linear: { x: 2.0 } angular: { z: 0.15 }'
```

## Perception obstacle course

[`../../worlds/obstacle.sdf`](../../worlds/obstacle.sdf) adds a local
road-course for perception and autonomy experiments. It provides:

- a dashed white lane-following guide, yellow centre divider, white road edges,
  and start / finish lines;
- two physical lane-blocking barriers, three orange marker posts, and a finish
  gate; and
- camera-visible materials plus matching collision geometry for every obstacle;
- an Ogre2 course overview that opens on the truck rather than the world
  origin; and
- a docked live view from the truck's front camera.

The course is deliberately scaled for this large truck. Start it with:

```powershell
$env:GZ_SIM_RESOURCE_PATH = Join-Path (Get-Location).Path 'simulation\models'
pixi run gz sim -r -v 4 simulation/worlds/obstacle.sdf
```

The obstacle-course world enables the model's sensor suite:

| Sensor | Rate | Topic |
| --- | ---: | --- |
| Downward-pitched 640 × 480 RGB camera | 30 Hz | `/model/gazebo_truck/sensors/front_camera/image` |
| 180° × 30° GPU lidar, 720 × 16 beams | 15 Hz | `/model/gazebo_truck/sensors/front_lidar/scan` |
| IMU | 100 Hz | `/model/gazebo_truck/sensors/imu` |
| Body / obstacle contact monitor | 100 Hz | `/model/gazebo_truck/sensors/bumper_contact` |

For example, inspect the live data types or receive one lidar / IMU sample:

```powershell
pixi run gz topic -i -t /model/gazebo_truck/sensors/front_camera/image
pixi run gz topic -e -n 1 -t /model/gazebo_truck/sensors/front_lidar/scan
pixi run gz topic -e -n 1 -t /model/gazebo_truck/sensors/imu
pixi run gz topic -e -n 1 -t /model/gazebo_truck/sensors/bumper_contact
```

The camera is intended for lane-marking detection; the lidar detects physical
obstacles and publishes a companion point-cloud topic at
`/model/gazebo_truck/sensors/front_lidar/scan/points`. The existing odometry
topic remains `/model/gazebo_truck/odometry`.

For a manual sensor smoke test, click the 3D View and use `W` forward, `X`
reverse, `A` forward-left, `D` forward-right, and `S` stop. These are only a
manual baseline; an autonomy controller should publish its commands to
`/model/gazebo_truck/cmd_vel`.

### Research use and next calibration gate

Use this scene to build the stack in this order: lane / obstacle perception,
state estimation from IMU and wheel data, a safety stop based on lidar range,
then lane-following control. Record route completion, lateral lane error,
minimum obstacle clearance, and body-contact events for each run.

Do not use the current vehicle to claim real-car braking or handling fidelity:
the Blender asset is approximately 15.36 m long and 10.7 m wide, and its
starter collision, tyre friction, and Ackermann dynamics are not calibrated.
Choose the real target platform first, then replace those values and add
sensor noise, latency, lighting / weather variation, and repeatable scenario
seeds before sim-to-real evaluation.

If Gazebo's GUI is sluggish on this laptop, set both `gz-sim-main.exe` and
`gz-sim-gui-client.exe` to **High performance** in Windows Graphics settings,
then relaunch. The renderer was observed selecting the integrated Intel GPU
instead of the installed RTX GPU.

## Re-export after editing the blend

Blender's command-line executable must be available. Run from the repository
root:

```powershell
& 'C:\Path\To\Blender\blender.exe' --background assets/source/chassis.blend --python tools/blender/export_gazebo_parts.py
```

The script detects the four dominant connected wheel islands, attaches their
enclosed hub/rim details, preserves the remaining geometry in the chassis,
transforms Blender axes to Gazebo axes, writes binary STL files, and regenerates
the mesh manifest.
