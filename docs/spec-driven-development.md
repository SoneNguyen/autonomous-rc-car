# Spec-driven development with Codex

This repository uses GitHub's Spec Kit to turn feature requests into reviewed
specifications, technical plans, and implementation tasks before code changes.
Spec Kit's Codex integration installs skills in `.agents/skills/`; invoke them
from Codex chat with `$speckit-<command>`.

## Standard feature flow

1. `$speckit-constitution` — confirm or amend the governing project rules.
2. `$speckit-specify` — define the user problem, scope, requirements, and
   measurable acceptance criteria.
3. `$speckit-clarify` — resolve important unanswered questions.
4. `$speckit-plan` — choose architecture, interfaces, dependencies, and
   validation approach.
5. `$speckit-tasks` — split the plan into reviewable implementation tasks.
6. `$speckit-analyze` — check consistency across the specification, plan, and
   tasks.
7. `$speckit-implement` — implement the approved task list, one story at a time.
8. `$speckit-converge` — optionally identify remaining work and verify completion against
   the artifacts.

Artifacts are stored in `specs/<feature-name>/`. Keep them with the code so
future simulation, ROS 2, and AI work has an explicit record of requirements
and decisions.

## Project-specific expectations

- Keep Gazebo world, vehicle-model, ROS 2-node, and AI-training responsibilities
  distinct and document the message interfaces between them.
- Preserve a runnable simulation path and keep paths portable; do not encode
  one developer's absolute Windows paths in project files.
- State units, coordinate frames, sensor rates, topic names, and message types
  in specifications when they affect vehicle behavior.
- Require measurable acceptance criteria for driving features, such as route
  completion, detection range, minimum clearance, stopping behavior, and
  contact events.
- Distinguish simulation results from real-vehicle claims. The vehicle mass,
  dimensions, tyre behavior, steering, braking, and sensor model require
  calibration before results predict real-world performance.
- Prefer small, reversible changes and preserve source meshes and exported
  assets; protected model, world, source-asset, and Blender-tool directories
  require explicit spec scope and a recorded reason before changes.
- Every story's independent test must state runnable commands and expected
  results. Each feature's `quickstart.md` must provide the fresh-WSL setup and
  run commands. Run the check, record the result, stop at a checkpoint, and
  commit each passing story before starting the next.
- Every plan must include a Constitution Check before research and after design.
  Report only validation checks that were actually performed.
- ROS 2 work uses the WSL2 filesystem with ROS 2 and Gazebo installed through
  apt. Each plan pins Ubuntu, ROS 2, and Gazebo versions in `research.md`; the
  Windows `win-64` Pixi environment remains for Windows-native Gazebo only.
- Keep generated data and weights in git-ignored `artifacts/`, ROS packages in
  `ros2_ws/`, ML code in `ml/`, and evaluation write-ups in `docs/`.
