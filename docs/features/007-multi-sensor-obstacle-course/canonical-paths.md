# Feature 007 Canonical Paths

This is the canonical path inventory for Feature 007. Tasks and quickstart commands use these exact names. T001 records the runtime gate result and confirms this inventory before any implementation task proceeds.

## Feature Design Records

- `docs/features/007-multi-sensor-obstacle-course/spec.md`
- `docs/features/007-multi-sensor-obstacle-course/plan.md`
- `docs/features/007-multi-sensor-obstacle-course/research.md`
- `docs/features/007-multi-sensor-obstacle-course/data-model.md`
- `docs/features/007-multi-sensor-obstacle-course/tasks.md`
- `docs/features/007-multi-sensor-obstacle-course/quickstart.md`
- `docs/features/007-multi-sensor-obstacle-course/contracts/ros-topics.md`

## Tools

- `tools/fetch_matrix_assets.py` (existing read-only asset fetcher)
- `tools/measure_rc_vehicle_dynamics.py`
- `tools/build_obstacle_course_profile.py`
- `tools/generate_obstacle_course_world.py`
- `tools/check_obstacle_course_sizing.py`
- `tools/validate_obstacle_course_config.py`
- `tools/run_obstacle_course.py`
- `tools/freeze_obstacle_course_inputs.py`
- `tools/verify_obstacle_course_inputs.py`
- `tools/generate_setup_sheets.py`
- `tools/validate_real_car_mapping.py`
- `tools/check_feature_007_protected_paths.py`

## Pinned Feature 002 Overlap-Checker Snapshot

- `tools/vendor/scenario_overlap/feature002_overlap.py` (byte-for-byte snapshot from commit `3a0a835179c7366f2921d181436a019b28ba7dca`; expected SHA-256 `d01ede3dbdfc73606cc52cd8e03c3f86da07d80c690ed48c4d860837b7849dd3`)
- `tools/vendor/scenario_overlap/README.md` (source revision, snapshot hash, and read-only use)

## Feature 003 Visual and Vehicle Anchors (Read-Only)

- `simulation/models/rc_car/model.sdf`
- `simulation/models/rc_car/model.config`
- `simulation/models/research/cone/cone_clean_01/model.sdf`
- `simulation/models/research/cone/cone_clean_01/model.config`
- `simulation/models/research/cone/cone_reflector_01/model.sdf`
- `simulation/models/research/cone/cone_reflector_01/model.config`
- `simulation/models/research/pedestrian/pedestrian_cross_step_01/model.sdf`
- `simulation/models/research/pedestrian/pedestrian_cross_step_01/model.config`
- `simulation/models/research/pedestrian/pedestrian_nurse_01/model.sdf`
- `simulation/models/research/pedestrian/pedestrian_nurse_01/model.config`
- `simulation/models/research/vehicle/vehicle_compact_suv_01/model.sdf`
- `simulation/models/research/vehicle/vehicle_compact_suv_01/model.config`
- `simulation/models/research/vehicle/vehicle_convertible_01/model.sdf`
- `simulation/models/research/vehicle/vehicle_convertible_01/model.config`
- `ros2_ws/src/car_bringup/config/rc_vehicle.yaml`
- `ros2_ws/src/car_bringup/config/rc_module_matrix/research_assets.lock.yaml`
- `ros2_ws/src/car_autonomy_nodes/car_autonomy_nodes/safe_drive_controller.py` (read-only final command authority)
- `artifacts/rc-car-matrix/assets/` (ignored verified mesh files)
- `artifacts/rc-car-matrix/assets/models/research/` (locked GLB files used by the course)

## Configurations

- `ros2_ws/src/car_bringup/config/obstacle_course/rc_vehicle_reference.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/bridge.yaml` (Feature 007 bridge; world name is `obstacle_course`)
- `ros2_ws/src/car_bringup/config/obstacle_course/assets.lock.yaml` (Feature 007 high-quality additions only; Feature 003 anchor assets stay in the protected research lock)
- `ros2_ws/src/car_bringup/config/obstacle_course/vehicle_geometry.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/decision.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/trigger_matrix.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/diagnostics/rc_vehicle_brake.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/development/decision.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/development/scenarios/barrier_tuning.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/scenarios/registry.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/scenarios/S0.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/scenarios/S1.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/scenarios/S2.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/scenarios/S3.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/scenarios/S3b.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/scenarios/S4.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/scenarios/S5.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/scenarios/S6-lidar-drop.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/scenarios/S6-camera-drop.yaml`
- `ros2_ws/src/car_bringup/config/obstacle_course/scenarios/S7.yaml`

## World Inputs

- `simulation/worlds/obstacle_condensed.sdf`
- `simulation/worlds/obstacle_course/fragments/barrier_red.sdf`
- `simulation/worlds/obstacle_course/fragments/post_orange.sdf`
- `simulation/worlds/obstacle_course/fragments/gate_blue.sdf`
- `simulation/worlds/obstacle_course/fragments/gate_blocked.sdf`
- `simulation/worlds/obstacle_course/fragments/generic_obstacle.sdf`

## Contracts and Tests

- `docs/features/007-multi-sensor-obstacle-course/contracts/component-evidence.schema.json`
- `docs/features/007-multi-sensor-obstacle-course/contracts/decision-config.schema.json`
- `docs/features/007-multi-sensor-obstacle-course/contracts/decision-event.schema.json`
- `docs/features/007-multi-sensor-obstacle-course/contracts/fault-event.schema.json`
- `docs/features/007-multi-sensor-obstacle-course/contracts/frozen-inputs.schema.json`
- `docs/features/007-multi-sensor-obstacle-course/contracts/planner-intent.schema.json`
- `docs/features/007-multi-sensor-obstacle-course/contracts/trigger-matrix.schema.json`
- `docs/features/007-multi-sensor-obstacle-course/contracts/scenario.schema.json`
- `docs/features/007-multi-sensor-obstacle-course/contracts/run-report.schema.json`
- `docs/features/007-multi-sensor-obstacle-course/contracts/dashboard.md`
- `tools/tests/test_dynamics_measurement.py`
- `tools/tests/test_vehicle_geometry.py`
- `tools/tests/test_obstacle_course_sizing.py`
- `tools/tests/test_obstacle_course_world.py`
- `tools/tests/test_obstacle_course_inputs.py`
- `tools/tests/test_real_car_mapping.py`
- `tools/tests/test_protected_paths.py`
- `tools/tests/test_scenario_overlap_snapshot.py`
- `ros2_ws/src/car_decision/test/test_config_contracts.py`
- `ros2_ws/src/car_decision/test/test_evidence_nodes.py`
- `ros2_ws/src/car_decision/test/test_fault_gate_node.py`
- `ros2_ws/src/car_decision/test/test_decision_engine.py`
- `ros2_ws/src/car_decision/test/test_planner_intent_adapter.py`
- `ros2_ws/src/car_decision/test/test_scenario_runner.py`
- `ros2_ws/src/car_decision/test/test_dashboard.py`
- `ros2_ws/src/car_decision/test/test_vehicle_visual_overlay.py`
- `ros2_ws/src/car_decision/test/test_manual_twist_adapter.py`

## Runtime Package and Integration Files

- `ros2_ws/src/car_decision/package.xml`
- `ros2_ws/src/car_decision/setup.py`
- `ros2_ws/src/car_decision/setup.cfg`
- `ros2_ws/src/car_decision/resource/car_decision`
- `ros2_ws/src/car_decision/car_decision/__init__.py`
- `ros2_ws/src/car_decision/car_decision/lidar_evidence_node.py`
- `ros2_ws/src/car_decision/car_decision/camera_evidence_node.py`
- `ros2_ws/src/car_decision/car_decision/imu_evidence_node.py`
- `ros2_ws/src/car_decision/car_decision/odometry_evidence_node.py`
- `ros2_ws/src/car_decision/car_decision/contact_evidence_node.py`
- `ros2_ws/src/car_decision/car_decision/decision_engine.py`
- `ros2_ws/src/car_decision/car_decision/decision_node.py`
- `ros2_ws/src/car_decision/car_decision/gate_corridor.py`
- `ros2_ws/src/car_decision/car_decision/planner_intent_adapter.py`
- `ros2_ws/src/car_decision/car_decision/controller_request_adapter.py`
- `ros2_ws/src/car_decision/car_decision/vehicle_visual_overlay.py`
- `ros2_ws/src/car_decision/car_decision/manual_twist_adapter.py`
- `ros2_ws/src/car_decision/car_decision/fault_gate_node.py`
- `ros2_ws/src/car_decision/car_decision/dashboard_node.py`
- `ros2_ws/src/car_decision/car_decision/run_logger.py`
- `ros2_ws/src/car_bringup/setup.py`
- `ros2_ws/src/car_bringup/launch/obstacle_course.launch.py`

## Outputs

- `ros2_ws/build/`
- `ros2_ws/install/`
- `ros2_ws/log/`
- `docs/features/007-multi-sensor-obstacle-course/runtime-gate.md`
- `docs/features/007-multi-sensor-obstacle-course/protected-inputs.json`
- `artifacts/obstacle-course/dynamics/rc-vehicle-reference.json`
- `artifacts/obstacle-course/config/`
- `artifacts/obstacle-course/worlds/`
- `artifacts/obstacle-course/frozen-inputs/input-manifest.json`
- `artifacts/obstacle-course/runs/<run-id>/`
- `docs/real-car-mapping/mapping.yaml`
- `docs/real-car-mapping/setup-sheets/`
- `docs/evaluation/obstacle-course/final-report.md`
- `docs/evaluation/obstacle-course/development-tuning.md`
- `docs/evaluation/obstacle-course/dashboard-check.md`
- `docs/evaluation/obstacle-course/jazzy-check.md`
- `docs/evaluation/obstacle-course/constitution-check.md`
- `README.md`
