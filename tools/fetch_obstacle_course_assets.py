#!/usr/bin/env python3
"""Fetch and verify the immutable assets used by the obstacle-course simulation."""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path, PurePosixPath
import shutil
import sys
import urllib.request
import zipfile
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOCK = REPO_ROOT / "ros2_ws/src/car_bringup/config/obstacle_course/research_assets.lock.yaml"
DEFAULT_OUTPUT = REPO_ROOT / "artifacts/obstacle-course/assets"
ALLOWED_LICENSES = {"CC0-1.0"}
FLOATING = {"main", "master", "latest", "tip", "head", "trunk"}


class AssetLockError(ValueError):
    pass


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_sha256(path: Path, expected: str) -> None:
    if not path.is_file():
        raise AssetLockError(f"locked file is missing: {path}")
    actual = sha256(path)
    if actual.lower() != str(expected).lower():
        raise AssetLockError(f"SHA-256 mismatch for {path}: expected {expected}, got {actual}")


def validate_lock(lock: dict[str, Any]) -> None:
    if not isinstance(lock, dict) or lock.get("schema_version") != 1:
        raise AssetLockError("asset lock must use schema_version 1")
    sources = lock.get("sources", {})
    if not isinstance(sources, dict):
        raise AssetLockError("asset lock sources must be a mapping")
    for asset in lock.get("assets", []):
        asset_id = asset.get("asset_id", "<unknown>")
        source = sources.get(asset.get("source_id"), {})
        resolved = {field: asset.get(field) or source.get(field) for field in
                    ("source", "author", "revision", "page_url", "license", "license_url")}
        for field, value in resolved.items():
            if not value:
                raise AssetLockError(f"{asset_id}: missing {field}")
        if resolved["license"] not in ALLOWED_LICENSES:
            raise AssetLockError(f"{asset_id}: license is not approved: {resolved['license']}")
        revision = str(resolved["revision"]).strip().lower()
        if revision in FLOATING or any(token in revision for token in ("/main", "/master", "latest")):
            raise AssetLockError(f"{asset_id}: floating revision is not allowed")
        if not asset.get("file_sha256") and not asset.get("archive_sha256"):
            source = sources.get(asset.get("source_id"), {})
            if not source.get("archive_sha256"):
                raise AssetLockError(f"{asset_id}: no immutable file or archive hash")
        if asset.get("file_sha256") and len(asset["file_sha256"]) != 64:
            raise AssetLockError(f"{asset_id}: invalid file SHA-256")


def _download(url: str, destination: Path, *, referer_url: str | None = None) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    headers = {
        "User-Agent": "Mozilla/5.0 (compatible; rc-obstacle-course-asset-fetcher/1)",
        "Accept": "*/*",
    }
    if referer_url:
        headers["Referer"] = referer_url
    request = urllib.request.Request(url, headers=headers)
    with urllib.request.urlopen(request, timeout=60) as response, destination.open("wb") as output:
        shutil.copyfileobj(response, output)


def _safe_destination(root: Path, member: str) -> Path:
    posix = PurePosixPath(member)
    if posix.is_absolute() or ".." in posix.parts:
        raise AssetLockError(f"unsafe archive member path: {member}")
    destination = (root / Path(*posix.parts)).resolve()
    if not destination.is_relative_to(root.resolve()):
        raise AssetLockError(f"archive member escapes target directory: {member}")
    return destination


def _extract_one(archive: Path, member: str, destination: Path, expected_sha: str) -> None:
    with zipfile.ZipFile(archive) as source:
        try:
            payload = source.read(member)
        except KeyError as exc:
            raise AssetLockError(f"locked archive member is missing: {member}") from exc
    digest = hashlib.sha256(payload).hexdigest()
    if digest.lower() != expected_sha.lower():
        raise AssetLockError(f"SHA-256 mismatch for archive member {member}: {digest}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(payload)


def _extract_texture_archive(archive: Path, destination_root: Path) -> None:
    with zipfile.ZipFile(archive) as source:
        for info in source.infolist():
            if info.is_dir():
                continue
            target = _safe_destination(destination_root, info.filename)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(source.read(info.filename))


def _cache_candidate(cache_root: Path, expected: str) -> Path | None:
    if not cache_root.is_dir():
        return None
    for path in cache_root.rglob("*"):
        if path.is_file() and sha256(path) == expected.lower():
            return path
    return None


def fetch_assets(lock_path: Path, output: Path, verify_only: bool = False) -> list[Path]:
    lock = yaml.safe_load(lock_path.read_text(encoding="utf-8"))
    validate_lock(lock)
    output = output.resolve()
    cache = output / "source-review"
    archive_dir = output / "_archives"
    installed: list[Path] = []
    sources = lock.get("sources", {})

    for source_id, source in sources.items():
        archive_name = source.get("archive_file")
        if not archive_name:
            continue
        source_assets = [a for a in lock.get("assets", []) if a.get("source_id") == source_id]
        extracted = []
        for asset in source_assets:
            if asset.get("member_path"):
                destination = output / "models" / "research" / asset["class_id"] / asset["asset_id"] / asset["file_name"]
                if destination.is_file():
                    verify_sha256(destination, asset["file_sha256"])
                    extracted.append((asset, destination))
                elif verify_only:
                    raise AssetLockError(f"locked extracted file is missing: {destination}")
        if len(extracted) == len([asset for asset in source_assets if asset.get("member_path")]):
            installed.extend(destination for _, destination in extracted)
            continue
        archive = cache / archive_name
        if not archive.is_file():
            archive = archive_dir / archive_name
        if not archive.is_file():
            if verify_only:
                raise AssetLockError(f"locked archive is missing: {archive_name}")
            archive = archive_dir / archive_name
            _download(source["download_url"], archive)
        verify_sha256(archive, source["archive_sha256"])

        for asset in source_assets:
            if asset.get("member_path"):
                destination = output / "models" / "research" / asset["class_id"] / asset["asset_id"] / asset["file_name"]
                if not destination.is_file():
                    if verify_only:
                        raise AssetLockError(f"locked extracted file is missing: {destination}")
                    _extract_one(archive, asset["member_path"], destination, asset["file_sha256"])
                verify_sha256(destination, asset["file_sha256"])
                installed.append(destination)
            elif asset.get("class_id") == "floor_texture":
                destination_root = output / "models" / "research" / "floors" / asset["asset_id"]
                if not any(destination_root.rglob("*")) if destination_root.exists() else True:
                    if verify_only:
                        raise AssetLockError(f"locked texture extraction is missing: {destination_root}")
                    _extract_texture_archive(archive, destination_root)
                installed.extend(path for path in destination_root.rglob("*") if path.is_file())

    for asset in lock.get("assets", []):
        if asset.get("class_id") == "floor_texture":
            archive = cache / asset["archive_file"]
            if not archive.is_file():
                archive = archive_dir / asset["archive_file"]
            if not archive.is_file():
                if verify_only:
                    raise AssetLockError(f"locked texture archive is missing: {asset['archive_file']}")
                archive = archive_dir / asset["archive_file"]
                _download(asset["download_url"], archive)
            verify_sha256(archive, asset["archive_sha256"])
            destination_root = output / "models" / "research" / "floors" / asset["asset_id"]
            if not destination_root.exists() or not any(destination_root.rglob("*")):
                if verify_only:
                    raise AssetLockError(f"locked texture extraction is missing: {destination_root}")
                _extract_texture_archive(archive, destination_root)
            installed.extend(path for path in destination_root.rglob("*") if path.is_file())
            continue
        if not asset.get("file_sha256") or asset.get("member_path"):
            continue
        destination = output / "models" / "research" / asset["class_id"] / asset["asset_id"] / asset["file_name"]
        if not destination.is_file():
            if verify_only:
                raise AssetLockError(f"locked extracted file is missing: {destination}")
            cached = _cache_candidate(cache, asset["file_sha256"])
            if cached is not None:
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(cached, destination)
            else:
                _download(
                    asset["download_url"],
                    destination,
                    referer_url=asset.get("page_url"),
                )
        verify_sha256(destination, asset["file_sha256"])
        installed.append(destination)

    return installed


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lock", type=Path, default=DEFAULT_LOCK)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args(argv)
    try:
        paths = fetch_assets(args.lock, args.output, args.verify_only)
    except (OSError, zipfile.BadZipFile, yaml.YAMLError, AssetLockError) as exc:
        print(f"research asset fetch failed: {exc}", file=sys.stderr)
        return 2
    print(f"Locked assets: PASS ({len(paths)} extracted files)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
