# Camera and Detection Matrix

## Scope

The fixed-seed set covers cone, pedestrian, and vehicle research meshes at
near/middle/far placements. Its unscored pilot places every class at all three
ranges and records visible pixel dimensions, a pedestrian occlusion scene, three lighting
profiles, a multi-object separation scene, and a detector-omission stale case.
The frozen detector identity and confidence threshold are checked against the
tracked model metadata. The unscored label pilot records instance pixel boxes
and object ranges for later review.

## Status

The runner, capture path, checker, and scenarios are implemented. The WSLg
capture now uses the NVIDIA renderer and waits for aligned RGB, label, and
detector output. Qualification remains pending until a complete scored set
passes with exact simulation-stamp pairs. Do not treat the declared
near/middle/far placements as measured detector performance.

The stale case intentionally starts the sensor bridge without the detector
process. The matrix observer records the no-message interval; this probes
absence observability and does not claim that the detector implements a
watchdog.

## Run

~~~bash
python3 tools/rc_module_matrix.py --module camera --pilot
python3 tools/rc_module_matrix.py --module camera --quick
python3 tools/rc_module_matrix.py --module camera
~~~

Raw frames, capture metadata, and launch logs are written under the ignored
`artifacts/rc-car-modules/` run directory. A pilot is unscored; quick mode is
smoke coverage; neither can qualify the module.
