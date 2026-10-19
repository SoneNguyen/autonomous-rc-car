<!--
Sync Impact Report
- Version: 1.0.0 -> 1.1.0 (minor: materially expanded project rules).
- Modified principles: I. Measurable behavior -> V. Evidence-Based Claims;
  II. Explicit units and frames -> Project Constraints / Interfaces;
  III. Simulation results have a stated scope -> V. Evidence-Based Claims;
  IV. Safe command and failure behavior -> III. Single Control Authority, Safe Fallback;
  V. Reproducible experiments -> I. Reproducible Simulation;
  VI. Portable and modular architecture -> Project Constraints;
  VII. Preserve source assets and explain changes -> II. Existing Assets Are Stable.
- Added section: Project Constraints.
- Modified sections: Development Workflow (formerly Validation and development workflow); Governance.
- Removed sections: None; compatible prior requirements are retained or consolidated.
- Templates updated: ✅ .specify/templates/plan-template.md
  ✅ .specify/templates/spec-template.md
  ✅ .specify/templates/tasks-template.md
- Runtime guidance updated: ✅ README.md; ✅ docs/spec-driven-development.md
- Command skills updated: ✅ .agents/skills/speckit-tasks/SKILL.md
  ✅ .agents/skills/speckit-implement/SKILL.md
- Other supporting file updated: ✅ .gitignore (generated artifacts).
- Follow-up: the deterministic scenario/seed runner is not yet implemented;
  current GUI demos remain visual smoke tests, not evaluation evidence.
-->
# Autonomous Vehicle Simulation Constitution

This constitution governs the Gazebo Sim, ROS 2, perception, AI, evaluation,
and tooling work in this repository. The vehicle is a truck-scale prototype
with starter physics parameters, not a calibrated passenger-car digital twin.

## Core Principles

### I. Reproducible Simulation

Every simulation run MUST use a documented repository command, Gazebo simulation
time (`use_sim_time=true` for ROS nodes), a checked-in scenario configuration
that records start pose, obstacle placement, and speed, and a fixed random seed.
Record the configuration and seed with the results. Results that cannot be
reproduced from the repository MUST NOT be claimed.

### II. Existing Assets Are Stable

`simulation/models/car/`, `simulation/worlds/`, `assets/source/`, and
`tools/blender/` MUST NOT change unless a feature spec explicitly requires the
change and records its reason in the spec or plan. Preserve source assets and
the existing Windows-native Pixi/Gazebo path; do not modify the installed
`.pixi` environment as a project implementation shortcut.

### III. Single Control Authority, Safe Fallback

An autonomous run MUST have exactly one node publishing drive commands. Gazebo
GUI keyboard control counts as a command publisher, so autonomous runs MUST be
headless or have keyboard control disabled. Perception and AI nodes MUST NOT
command the vehicle. Stale, missing, or invalid sensor, command, or model data
MUST stop the vehicle or fall back to the deterministic lidar baseline. Drive
commands MUST be bounded. Contact sensing is an evaluation signal, not the
primary obstacle-avoidance method.

### IV. Incremental, Verified Delivery

Implement one user story at a time. Every story MUST have an independent test
with runnable command(s) and expected result(s); use an automated check when
practical, otherwise document a manual check. The test MUST pass at a checkpoint
before work starts on the next story, and each passing story MUST be committed
to git before proceeding. Report only checks that were actually run. Validate
SDF edits with the Gazebo SDF validator, ROS interfaces with topic inspection
and a minimal end-to-end command, and autonomy behavior in repeatable scenarios.

### V. Evidence-Based Claims

Datasets MUST have a documented labeling rule and a held-out split that is never
used for training. Model and controller claims MUST include recorded metrics,
such as route completion, detection range, lateral error, minimum clearance,
stopping distance, or contact count. Results MUST be described as applying to
this prototype, not to real vehicles. Large datasets and model weights MUST
stay out of git; provide a script or note that regenerates them. Starter mass, inertia,
collision geometry, tyre friction, steering, braking, suspension, and sensor
models are not calibrated for real-vehicle predictions.

### VI. Simplicity

Solutions MUST use the smallest approach that meets the current story. The
implementation MUST NOT add frameworks, services, or abstractions that the
story does not need. Non-obvious dependencies MUST be justified in the plan
with their purpose and compatible versions.

## Project Constraints

- **Runtime:** The current simulation baseline is Gazebo Sim 10 (Jetty). ROS 2
  and Gazebo MUST run together in WSL2 Ubuntu and use apt as their only package
  source. The repository and ROS workspace MUST live in the WSL filesystem for
  ROS work. The existing `win-64` Pixi environment remains the documented
  Windows-native Gazebo path and MUST NOT be used for ROS work. Each plan's
  `research.md` MUST decide and pin the Ubuntu, ROS 2, and Gazebo versions and
  package source. A plan MUST NOT silently change the Gazebo major version;
  such a change touches protected vehicle assets and requires explicit scope
  and rationale in the spec and plan.
- **Layout:** ROS packages belong under `ros2_ws/`, ML code under `ml/`,
  generated data and weights under git-ignored `artifacts/`, and evaluation
  write-ups under `docs/`. The root README layout MUST stay current.
- **Interfaces:** README Gazebo topic names are authoritative for the
  simulation. ROS-side topic names and message types belong in the feature
  plan's contracts and bridge configuration, not scattered through code.
  Specifications and interfaces MUST state relevant units, coordinate
  conventions, frames, message types, and sensor rates, including documented
  transforms between Gazebo and ROS conventions.
- **Separation and portability:** Simulation, ROS nodes, ML training/inference,
  and visualization responsibilities MUST remain separate behind documented
  interfaces. Committed commands and docs MUST NOT depend on a
  developer-specific absolute path.
- **Reproduction docs:** The root README and each feature's `quickstart.md`
  MUST provide the commands and expected results needed to reproduce a run
  from a fresh WSL install. Keep the Windows Pixi/Gazebo path documented too.

## Development Workflow

The Spec Kit workflow MUST follow this order: constitution, specify, clarify,
plan, tasks, analyze, implement. Every plan MUST include a Constitution Check
against Principles I–VI before research and repeat it after design. Plans MUST
record how each relevant principle is met or resolve a conflict before
implementation. Stories MUST be implemented sequentially, with each passing
story tested and committed before starting the next. Automated checks SHOULD be
used when practical; a written manual check is acceptable otherwise.

## Governance

This constitution supersedes ad-hoc decisions and applies to specs, plans,
tasks, and implementation. Resolve and record any conflict before
implementation. Amendments MUST retain the original ratification date, bump the
semantic version, update the last-amended date, and include a one-line rationale.
Constitution compliance MUST be reviewed in every plan and during analyze
before implementation.

Amendment rationale: define WSL2/ROS boundaries and checkable solo, reproducible autonomy rules.

**Version**: 1.1.0 | **Ratified**: 2026-10-18 | **Last Amended**: 2026-10-19
