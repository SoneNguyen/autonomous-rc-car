#!/usr/bin/env python3
"""Fail closed if Feature 007 changes protected history or adds unlisted files."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import re
import subprocess
import sys


REPO = Path(__file__).resolve().parents[1]
CANONICAL = Path("docs/features/007-multi-sensor-obstacle-course/canonical-paths.md")


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _git(*args: str, check=True) -> str:
    result = subprocess.run(
        ["git", *args], cwd=REPO, capture_output=True, check=False,
    )
    if check and result.returncode:
        raise RuntimeError(
            f"git {' '.join(args)} failed: {result.stderr.decode('utf-8', errors='replace')}"
        )
    return result.stdout.decode("utf-8", errors="replace")


def canonical_allowlist(markdown: str) -> set[str]:
    """Extract path literals and directory prefixes from the canonical path inventory."""
    paths: set[str] = {CANONICAL.as_posix()}
    for literal in re.findall(r"`([^`]+)`", markdown):
        value = literal.strip()
        if not value or value.startswith(("<", "http", "#")) or "<" in value or ">" in value:
            continue
        if value.startswith(("docs/", "tools/", "simulation/", "ros2_ws/", "models/", "artifacts/", "README.md")):
            paths.add(value.rstrip("`"))
    return paths


def is_canonical(path: str, allowed: set[str]) -> bool:
    if path in allowed:
        return True
    return any(item.endswith("/") and path.startswith(item) for item in allowed)


def compare_inventory(*, baseline: dict, base_paths: set[str], current_tracked: set[str],
                      current_untracked: set[str], current_files: set[str],
                      current_hashes: dict[str, str], canonical_paths: set[str],
                      base_hashes: dict[str, str]) -> list[str]:
    """Pure baseline comparison used by the command and its unit tests."""
    errors: list[str] = []
    entries = {entry["path"].rstrip("/"): entry for entry in baseline.get("entries", [])}
    if len(entries) != len(baseline.get("entries", [])):
        errors.append("baseline contains duplicate paths")

    for entry in baseline.get("entries", []):
        raw_path = entry["path"]
        path = raw_path.rstrip("/")
        expected = entry["status"]
        if expected == "present":
            expected_hash = entry.get("sha256")
            base_hash = base_hashes.get(path)
            if path not in base_paths or not expected_hash or base_hash != expected_hash:
                errors.append(f"baseline itself does not match base commit: {path}")
            if path not in current_tracked:
                errors.append(f"protected tracked path was deleted or untracked: {path}")
            elif path not in current_files:
                errors.append(f"protected path is missing from worktree: {path}")
            elif current_hashes.get(path) != expected_hash:
                errors.append(f"protected content changed: {path}")
        elif expected == "absent":
            if path in base_paths:
                errors.append(f"baseline marks base-commit path absent but it is tracked: {path}")
            if path in current_tracked or path in current_files:
                errors.append(f"protected historical path was reintroduced: {path}")
            if raw_path.endswith("/") and any(item.startswith(raw_path) for item in current_files):
                errors.append(f"protected historical directory was reintroduced: {raw_path}")
        else:
            errors.append(f"unknown baseline status {expected!r} for {path}")

    for root in baseline.get("protected_roots", []):
        if root.endswith("/"):
            base_under_root = {path for path in base_paths if path.startswith(root)}
            current_under_root = {path for path in current_files if path.startswith(root)}
            allowed_world = set(baseline.get("allowed_feature007_world_additions", []))
            root_additions = current_under_root - base_under_root
            bad_root_additions = sorted(path for path in root_additions if path not in allowed_world)
            if bad_root_additions:
                errors.append(f"protected root {root} has unapproved additions: {', '.join(bad_root_additions)}")
            missing = sorted(base_under_root - current_files)
            if missing:
                errors.append(f"protected root {root} has deletions: {', '.join(missing)}")
        else:
            if root in entries and entries[root]["status"] == "present" and root not in current_files:
                errors.append(f"protected root file missing: {root}")

    new_paths = (current_tracked | current_untracked) - base_paths
    for path in sorted(new_paths):
        if path.startswith("simulation/worlds/"):
            if path not in set(baseline.get("allowed_feature007_world_additions", [])):
                errors.append(f"new world path is not explicitly allowed: {path}")
            continue
        for root in baseline.get("protected_roots", []):
            if root.endswith("/") and path.startswith(root):
                errors.append(f"new path is inside protected root {root}: {path}")
        if not is_canonical(path, canonical_paths):
            errors.append(f"new path is not listed in canonical-paths.md: {path}")

    for path in sorted(current_untracked):
        if path.startswith("simulation/worlds/") and path not in set(baseline.get("allowed_feature007_world_additions", [])):
            errors.append(f"untracked world path is not explicitly allowed: {path}")
    return errors


def _filesystem_files() -> set[str]:
    files: set[str] = set()
    for path in REPO.rglob("*"):
        if not path.is_file() and not path.is_symlink():
            continue
        try:
            relative = path.relative_to(REPO).as_posix()
        except ValueError:
            continue
        if relative.startswith(".git/"):
            continue
        files.add(relative)
    return files


def check(baseline_path: Path) -> tuple[list[str], list[str]]:
    import json

    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    base_commit = baseline["base_commit"]
    _git("cat-file", "-e", f"{base_commit}^{{commit}}")
    _git("merge-base", "--is-ancestor", base_commit, "HEAD")
    base_paths = set(_git("ls-tree", "-r", "--name-only", base_commit).splitlines())
    current_tracked = set(_git("ls-files").splitlines())
    current_untracked = set(_git("ls-files", "--others", "--exclude-standard").splitlines())
    current_files = _filesystem_files()
    base_hashes: dict[str, str] = {}
    current_hashes: dict[str, str] = {}
    for entry in baseline["entries"]:
        path = entry["path"].rstrip("/")
        if entry["status"] == "present":
            try:
                base_bytes = subprocess.check_output(["git", "show", f"{base_commit}:{path}"], cwd=REPO)
                base_hashes[path] = sha256_bytes(base_bytes)
            except subprocess.CalledProcessError:
                pass
            file_path = REPO / path
            if file_path.is_file() or file_path.is_symlink():
                current_bytes = file_path.read_bytes() if not file_path.is_symlink() else file_path.readlink().as_posix().encode()
                current_hashes[path] = sha256_bytes(current_bytes)
    canonical_file = REPO / CANONICAL
    if not canonical_file.is_file():
        raise RuntimeError(f"canonical Feature 007 path inventory is missing: {CANONICAL}")
    allowed = canonical_allowlist(canonical_file.read_text(encoding="utf-8"))
    errors = compare_inventory(
        baseline=baseline,
        base_paths=base_paths,
        current_tracked=current_tracked,
        current_untracked=current_untracked,
        current_files=current_files,
        current_hashes=current_hashes,
        canonical_paths=allowed,
        base_hashes=base_hashes,
    )
    roots = baseline.get("protected_roots", [])
    results = []
    for root in roots:
        if root.endswith("/"):
            expected = sum(path.startswith(root) for path in base_paths)
            current = sum(path.startswith(root) for path in current_files)
            results.append(f"{root}: base={expected} worktree={current} status={'PASS' if not any(root in e for e in errors) else 'FAIL'}")
        else:
            results.append(f"{root}: status={'PASS' if not any(root in e for e in errors) else 'FAIL'}")
    return errors, results


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    args = parser.parse_args(argv)
    baseline_path = args.baseline.resolve()
    try:
        errors, results = check(baseline_path)
        for line in results:
            print(line)
        if errors:
            for error in errors:
                print(f"FAIL: {error}", file=sys.stderr)
            print(f"PROTECTED PATH CHECK FAIL ({len(errors)} violations)")
            return 1
        print("PROTECTED PATH CHECK PASS")
        return 0
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        print(f"check_feature_007_protected_paths: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
