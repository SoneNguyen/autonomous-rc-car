#!/usr/bin/env python3
"""Apply a scenario's initial vehicle pose through Gazebo Transport."""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def make_pose_request(
    *, x: float, y: float, z: float, roll: float, pitch: float, yaw: float,
    model_name: str = "gazebo_truck",
) -> str:
    qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
    numbers = (x, y, z, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError("scenario pose must contain only finite values")
    if not model_name.replace("_", "").isalnum():
        raise ValueError("model name contains unsupported characters")
    return (
        f'name: "{model_name}" position {{ x: {x:.9g} y: {y:.9g} z: {z:.9g} }} '
        f'orientation {{ x: {qx:.12g} y: {qy:.12g} z: {qz:.12g} w: {qw:.12g} }}'
    )


def set_model_pose(request: str, *, world_name: str, timeout_s: float = 30.0) -> bool:
    deadline = time.monotonic() + timeout_s
    last_error = "Gazebo set_pose service did not become available"
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                [
                    "gz", "service", "-s", f"/world/{world_name}/set_pose",
                    "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "5000", "--req", request,
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=8.0,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            last_error = str(exc)
            time.sleep(0.25)
            continue
        if result.returncode == 0 and "data: true" in result.stdout:
            print("scenario vehicle start pose applied", flush=True)
            return True
        last_error = (result.stderr or result.stdout or "set_pose returned false").strip()
        time.sleep(0.25)
    print(f"failed to apply scenario start pose: {last_error}", file=sys.stderr, flush=True)
    return False


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="sensor_obstacle_course")
    for name in ("x", "y", "z", "roll", "pitch", "yaw"):
        parser.add_argument(f"--{name}", type=float, required=True)
    args = parser.parse_args(argv)
    request = make_pose_request(
        x=args.x, y=args.y, z=args.z, roll=args.roll, pitch=args.pitch, yaw=args.yaw
    )
    return 0 if set_model_pose(request, world_name=args.world) else 2


if __name__ == "__main__":
    raise SystemExit(main())
