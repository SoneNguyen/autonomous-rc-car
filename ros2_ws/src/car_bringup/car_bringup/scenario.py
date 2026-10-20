"""Load and validate the checked-in scenario configuration files."""

from __future__ import annotations

import math
from pathlib import Path
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
