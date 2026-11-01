"""Parameter-based separation checks for Feature 002 scenario sets."""
from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from typing import Any

GROUP_ORDER = ("start_pose", "obstacle_placement", "speed")


class SplitValidationError(ValueError):
    """A scenario set contains a duplicate or violates minimum separation."""


def _first(mapping: Mapping[str, Any], names: Sequence[str], label: str) -> Any:
    for name in names:
        if name in mapping:
            return mapping[name]
    raise ValueError(f"scenario is missing {label}; expected one of {', '.join(names)}")


def _pose_value(pose: Mapping[str, Any], names: Sequence[str], label: str) -> float:
    return float(_first(pose, names, label))


def _start_pose(scenario: Mapping[str, Any]) -> tuple[float, float, float]:
    pose = scenario.get("start_pose")
    if not isinstance(pose, Mapping):
        raise ValueError("scenario.start_pose must be a mapping")
    return (
        _pose_value(pose, ("x_m", "x"), "start_pose.x_m"),
        _pose_value(pose, ("y_m", "y"), "start_pose.y_m"),
        _pose_value(pose, ("yaw_rad", "yaw"), "start_pose.yaw_rad"),
    )


def _obstacle_map(scenario: Mapping[str, Any]) -> dict[str, tuple[float, float, float]]:
    obstacles = scenario.get("obstacles")
    if obstacles is None and "obstacle" in scenario:
        obstacles = [scenario["obstacle"]]
    if not isinstance(obstacles, list):
        raise ValueError("scenario.obstacles must be a list")
    result: dict[str, tuple[float, float, float]] = {}
    for obstacle in obstacles:
        if not isinstance(obstacle, Mapping):
            raise ValueError("each obstacle must be a mapping")
        identity = str(_first(obstacle, ("entity", "entity_id", "id"), "obstacle identity"))
        if identity in result:
            raise ValueError(f"duplicate obstacle identity: {identity}")
        pose = obstacle.get("pose")
        if not isinstance(pose, Mapping):
            raise ValueError(f"obstacle {identity} pose must be a mapping")
        result[identity] = (
            _pose_value(pose, ("x_m", "x"), f"obstacle {identity} x"),
            _pose_value(pose, ("y_m", "y"), f"obstacle {identity} y"),
            _pose_value(pose, ("z_m", "z"), f"obstacle {identity} z"),
        )
    return result


def _speed(scenario: Mapping[str, Any]) -> float:
    return float(_first(scenario, ("target_speed_mps", "commanded_speed_mps", "speed_mps"), "speed"))


def _wrapped_angle_delta(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def _rounded(value: float) -> float:
    return round(float(value), 6)


def parameter_key(scenario: Mapping[str, Any], footprint_radius_m: float) -> tuple[Any, ...]:
    """Return the six-decimal duplicate key; scenario ID and seed are excluded."""
    x, y, yaw = _start_pose(scenario)
    obstacles = _obstacle_map(scenario)
    obstacle_key = tuple(
        (identity, *(_rounded(component) for component in position))
        for identity, position in sorted(obstacles.items())
    )
    normalized_yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return (
        _rounded(x),
        _rounded(y),
        _rounded(normalized_yaw),
        obstacle_key,
        _rounded(_speed(scenario)),
    )


def exact_duplicate(a: Mapping[str, Any], b: Mapping[str, Any], footprint_radius_m: float) -> bool:
    """Compare scenario parameters at six decimals, ignoring seed and ID."""
    return parameter_key(a, footprint_radius_m) == parameter_key(b, footprint_radius_m)


def group_distances(
    a: Mapping[str, Any], b: Mapping[str, Any], footprint_radius_m: float
) -> dict[str, float]:
    """Compute start-pose (m), obstacle-placement (m), and speed (m/s) distances."""
    ax, ay, ayaw = _start_pose(a)
    bx, by, byaw = _start_pose(b)
    dyaw = _wrapped_angle_delta(ayaw, byaw)
    pose_distance = math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (footprint_radius_m * dyaw) ** 2)

    obstacles_a = _obstacle_map(a)
    obstacles_b = _obstacle_map(b)
    if obstacles_a.keys() != obstacles_b.keys():
        placement_distance = math.inf
    elif not obstacles_a:
        placement_distance = 0.0
    else:
        placement_distance = max(
            math.dist(obstacles_a[identity], obstacles_b[identity])
            for identity in sorted(obstacles_a)
        )

    return {
        "start_pose": pose_distance,
        "obstacle_placement": placement_distance,
        "speed": abs(_speed(a) - _speed(b)),
    }


def normalized_distance(
    a: Mapping[str, Any],
    b: Mapping[str, Any],
    taus: Mapping[str, float],
    footprint_radius_m: float,
) -> dict[str, Any]:
    """Compute max_g(d_g/tau_g), returning the winning separation group."""
    distances = group_distances(a, b, footprint_radius_m)
    ratios: dict[str, float] = {}
    for group in GROUP_ORDER:
        tau = float(taus[group])
        if not math.isfinite(tau) or tau <= 0.0:
            raise ValueError(f"tau for {group} must be finite and positive")
        ratios[group] = distances[group] / tau
    limiting_group = max(GROUP_ORDER, key=lambda group: ratios[group])
    return {
        "distance": ratios[limiting_group],
        "limiting_group": limiting_group,
        "group_distances": distances,
        "normalized_group_distances": ratios,
    }


def nearest_training_distance(
    candidate: Mapping[str, Any],
    training: Sequence[Mapping[str, Any]],
    validation: Sequence[Mapping[str, Any]],
    taus: Mapping[str, float],
    footprint_radius_m: float,
) -> dict[str, Any]:
    """Return the nearest pair score over all training and validation scenarios."""
    references = [*training, *validation]
    if not references:
        return {"nearest_train_distance": math.inf, "limiting_group": None, "nearest_scenario_id": None}
    scored = [
        (normalized_distance(candidate, reference, taus, footprint_radius_m), reference)
        for reference in references
    ]
    nearest, reference = min(scored, key=lambda item: item[0]["distance"])
    return {
        "nearest_train_distance": nearest["distance"],
        "limiting_group": nearest["limiting_group"],
        "nearest_scenario_id": reference.get("scenario_id"),
    }


def _assert_unique_parameters(
    scenarios: Sequence[Mapping[str, Any]], footprint_radius_m: float, label: str
) -> None:
    seen: dict[tuple[Any, ...], str] = {}
    for scenario in scenarios:
        scenario_id = str(scenario.get("scenario_id", "<unknown>"))
        key = parameter_key(scenario, footprint_radius_m)
        if key in seen:
            raise SplitValidationError(
                f"six-decimal duplicate parameters: {seen[key]} and {scenario_id} ({label})"
            )
        seen[key] = scenario_id


def validate_split_separation(
    *,
    training: Sequence[Mapping[str, Any]],
    validation: Sequence[Mapping[str, Any]],
    evaluation: Sequence[Mapping[str, Any]],
    taus: Mapping[str, float],
    footprint_radius_m: float,
    previous_evaluations: Iterable[Sequence[Mapping[str, Any]]] = (),
) -> None:
    """Reject exact duplicates globally and near overlaps across split boundaries."""
    prior_sets = list(previous_evaluations)
    all_sets = [
        ("training", training),
        ("validation", validation),
        ("evaluation", evaluation),
        *[(f"previous_evaluation_{index}", values) for index, values in enumerate(prior_sets)],
    ]
    flattened: list[tuple[str, Mapping[str, Any]]] = []
    for label, values in all_sets:
        _assert_unique_parameters(values, footprint_radius_m, label)
        flattened.extend((label, scenario) for scenario in values)

    seen: dict[tuple[Any, ...], tuple[str, str]] = {}
    for label, scenario in flattened:
        scenario_id = str(scenario.get("scenario_id", "<unknown>"))
        key = parameter_key(scenario, footprint_radius_m)
        if key in seen:
            previous_label, previous_id = seen[key]
            raise SplitValidationError(
                f"six-decimal duplicate parameters: {previous_id} ({previous_label}) and {scenario_id} ({label})"
            )
        seen[key] = (label, scenario_id)

    references = [*training, *validation]
    for previous in prior_sets:
        references.extend(previous)
    comparisons = [
        ("training", a, "validation", b)
        for a in training for b in validation
    ]
    comparisons.extend(
        ("evaluation", a, "training/validation/previous evaluation", b)
        for a in evaluation for b in references
    )
    comparisons.extend(
        ("evaluation", a, "evaluation", b)
        for i, a in enumerate(evaluation) for b in evaluation[i + 1:]
    )

    for label_a, a, label_b, b in comparisons:
        result = normalized_distance(a, b, taus, footprint_radius_m)
        if result["distance"] < 1.0:
            raise SplitValidationError(
                f"minimum separation failed ({result['distance']:.6f} < 1): "
                f"{a.get('scenario_id', '<unknown>')} [{label_a}] vs "
                f"{b.get('scenario_id', '<unknown>')} [{label_b}], "
                f"limiting_group={result['limiting_group']}"
            )