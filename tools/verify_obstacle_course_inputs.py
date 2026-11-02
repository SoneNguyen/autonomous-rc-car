#!/usr/bin/env python3
"""Re-hash the frozen Obstacle course demonstration sources."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

from freeze_obstacle_course_inputs import CONTRACTS, REPO, root_hash_for_inputs


def verify_manifest(manifest_path: Path, *, root: Path = REPO) -> dict:
    source = manifest_path if manifest_path.is_absolute() else root / manifest_path
    manifest = json.loads(source.read_text(encoding="utf-8"))
    import jsonschema
    schema_path = REPO / CONTRACTS / "frozen-inputs.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator(schema, format_checker=jsonschema.FormatChecker()).validate(manifest)
    if root_hash_for_inputs(manifest["inputs"]) != manifest["root_sha256"]:
        raise ValueError("manifest root hash does not match its sorted input entries")
    changed, missing = [], []
    for item in manifest["inputs"]:
        path = root / item["path"]
        if not path.is_file():
            missing.append(item["path"])
        elif hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
            changed.append(item["path"])
    return {
        "valid": not changed and not missing,
        "root_sha256": manifest["root_sha256"],
        "changed": sorted(changed), "missing": sorted(missing),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = verify_manifest(args.manifest)
    except (OSError, ValueError, json.JSONDecodeError, KeyError, TypeError) as exc:
        print(f"verify_obstacle_course_inputs: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
