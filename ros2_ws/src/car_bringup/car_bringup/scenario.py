"""Load and validate the checked-in scenario configuration files."""

from __future__ import annotations

import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET
from typing import Any

import jsonschema
import yaml


class ScenarioValidationError(ValueError):
    """Raised when a scenario cannot safely identify a repeatable run."""


def _default_schema_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("car_bringup")) / "config" / "scenario.schema.yaml"
    except (ImportError, LookupError):
        return Path(__file__).resolve().parents[1] / "config" / "scenario.schema.yaml"


SCHEMA_PATH = _default_schema_path()


def _reject_non_finite(value: Any, path: str = "$") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ScenarioValidationError(f"{path}: numbers must be finite")
    if isinstance(value, dict):
        for key, child in value.items():
            _reject_non_finite(child, f"{path}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _reject_non_finite(child, f"{path}[{index}]")


def validate_scenario(data: Any, schema_path: Path | None = None) -> dict[str, Any]:
    """Validate and return one scenario mapping, raising a concise error on failure."""
    if not isinstance(data, dict):
        raise ScenarioValidationError("scenario document must be a YAML mapping")

    _reject_non_finite(data)
    path = schema_path or SCHEMA_PATH
    try:
        schema = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ScenarioValidationError(f"cannot load scenario schema {path}: {exc}") from exc

    try:
        jsonschema.Draft202012Validator.check_schema(schema)
    except jsonschema.SchemaError as exc:
        raise ScenarioValidationError(f"invalid scenario schema {path}: {exc.message}") from exc

    errors = sorted(
        jsonschema.Draft202012Validator(schema).iter_errors(data),
        key=lambda error: tuple(str(part) for part in error.absolute_path),
    )
    if errors:
        details = []
        for error in errors:
            location = ".".join(str(part) for part in error.absolute_path) or "$"
            details.append(f"{location}: {error.message}")
        raise ScenarioValidationError("invalid scenario:\n- " + "\n- ".join(details))
    return data


def load_scenario(path: Path | str) -> dict[str, Any]:
    """Read a YAML scenario and validate its required run parameters."""
    scenario_path = Path(path)
    try:
        data = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ScenarioValidationError(f"cannot load scenario {scenario_path}: {exc}") from exc
    return validate_scenario(data)


def validate_scenario_obstacle(scenario: dict[str, Any], world_path: Path | str) -> None:
    """Ensure the scenario's named static obstacle and pose match the SDF.

    This lets runs select only existing protected-world obstacles; it does not
    create, move, or rewrite world entities.
    """
    source = Path(world_path)
    try:
        # SDF permits Gazebo extension elements with the ``gz:`` prefix but
        # some checked-in worlds omit an XML namespace declaration for it.
        # ElementTree is stricter than libsdformat, so normalize element names
        # only in this in-memory validation copy; never rewrite the world.
        xml_text = source.read_text(encoding="utf-8")
        xml_text = re.sub(r"(<\/?)(gz:)", r"\1", xml_text)
        root = ET.fromstring(xml_text)
    except (OSError, ET.ParseError) as exc:
        raise ScenarioValidationError(f"cannot parse scenario world {source}: {exc}") from exc

    obstacle = scenario["obstacle"]
    target_model = next(
        (model for model in root.iter("model") if model.get("name") == obstacle["entity"]),
        None,
    )
    if target_model is None:
        raise ScenarioValidationError(
            f"world {source} has no model named {obstacle['entity']!r}"
        )
    collision = next(
        (item for item in target_model.iter("collision") if item.get("name") == obstacle["collision"]),
        None,
    )
    if collision is None:
        raise ScenarioValidationError(
            f"model {obstacle['entity']!r} has no collision named {obstacle['collision']!r}"
        )
    pose_element = collision.find("pose")
    try:
        actual = [0.0] * 6 if pose_element is None or not (pose_element.text or "").strip() else [
            float(value) for value in pose_element.text.split()
        ]
    except ValueError as exc:
        raise ScenarioValidationError(
            f"SDF collision {obstacle['collision']!r} pose must contain numeric values"
        ) from exc
    if len(actual) != 6:
        raise ScenarioValidationError(
            f"SDF collision {obstacle['collision']!r} pose must contain six values"
        )
    pose = obstacle["pose"]
    keys = ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")
    mismatches = [
        f"{key}: config={pose[key]} SDF={value}"
        for key, value in zip(keys, actual)
        if not math.isclose(float(pose[key]), value, rel_tol=0.0, abs_tol=1e-6)
    ]
    if mismatches:
        raise ScenarioValidationError(
            f"scenario obstacle pose does not match protected SDF collision {obstacle['collision']!r}: "
            + "; ".join(mismatches)
        )
