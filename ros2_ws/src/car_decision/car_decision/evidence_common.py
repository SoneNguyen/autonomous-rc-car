"""Shared, simulation-time helpers for Obstacle course evidence records."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml


class ClockEpochTracker:
    """Detect simulation clock rewinds and distinguish samples across resets."""

    def __init__(self) -> None:
        self.last_ns: int | None = None
        self.epoch = 0

    def observe(self, now_ns: int) -> bool:
        now_ns = int(now_ns)
        reset = self.last_ns is not None and now_ns < self.last_ns
        if reset:
            self.epoch += 1
        self.last_ns = now_ns
        return reset


def sha256_file(path: str | Path) -> str:
    """Return a file digest for normalized evidence provenance."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_yaml(path: str | Path) -> dict[str, Any]:
    result = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(result, dict):
        raise ValueError(f"expected a YAML mapping in {path}")
    return result


def value_of(item: Any) -> Any:
    if isinstance(item, dict) and "value" in item:
        return item["value"]
    return item


def message_stamp_ns(header: Any, fallback_ns: int) -> int:
    if header is None or not hasattr(header, "stamp"):
        return int(fallback_ns)
    stamp = header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else int(fallback_ns)


def freshness(sample_ns: int | None, now_ns: int, timeout_s: float) -> dict[str, Any]:
    if timeout_s <= 0 or not math.isfinite(timeout_s):
        raise ValueError("freshness timeout must be finite and positive")
    if sample_ns is None:
        return {"state": "unavailable", "age_s": None, "timeout_s": float(timeout_s)}
    age = max(0.0, (int(now_ns) - int(sample_ns)) / 1_000_000_000)
    return {
        "state": "fresh" if age <= timeout_s else "stale",
        "age_s": age,
        "timeout_s": float(timeout_s),
    }


def fault_active(scenario: dict[str, Any], component: str, sim_time_ns: int) -> bool:
    """Return whether a tracked scenario currently drops the selected source."""
    for item in scenario.get("fault_injections", []):
        if item.get("action") != "drop" or item.get("component") != component:
            continue
        start = float(value_of(item["start_sim_time"])) * 1_000_000_000
        end = float(value_of(item["end_sim_time"])) * 1_000_000_000
        if start <= sim_time_ns < end:
            return True
    return False


def route_progress(route: dict[str, Any], x_m: float, y_m: float) -> float:
    """Project a vehicle-relative pose onto a route polyline and return arc length."""
    points = route.get("waypoints", [])
    if len(points) < 2:
        return 0.0
    points_xy = [(float(value_of(p["x"])), float(value_of(p["y"]))) for p in points]
    traversed = 0.0
    best_distance = float("inf")
    best_progress = 0.0
    for start, end in zip(points_xy, points_xy[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_sq = dx * dx + dy * dy
        length = math.sqrt(length_sq)
        if length_sq <= 1e-12:
            continue
        ratio = max(0.0, min(1.0, ((x_m - start[0]) * dx + (y_m - start[1]) * dy) / length_sq))
        projection = (start[0] + ratio * dx, start[1] + ratio * dy)
        distance = math.hypot(x_m - projection[0], y_m - projection[1])
        if distance < best_distance:
            best_distance = distance
            best_progress = traversed + ratio * length
        traversed += length
    return best_progress


def lidar_payload(
    ranges: list[float] | tuple[float, ...],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    lidar_pose: list[float] | tuple[float, ...],
    sector_boundaries: list[float] | tuple[float, ...],
    corridor_half_width_m: float,
    self_filter_radius_m: float = 0.0,
) -> dict[str, Any]:
    """Transform a planar scan into base_link and reduce it to five sectors."""
    if len(sector_boundaries) != 6:
        raise ValueError("five sectors require six angular boundaries")
    x0, y0, _z0, _roll, _pitch, yaw0 = map(float, lidar_pose)
    limits = [float(v) for v in sector_boundaries]
    nearest: list[float | None] = [None] * 5
    corridor: list[float] = []
    points: list[dict[str, float]] = []
    invalid_rejected = 0
    usable = 0
    for index, raw in enumerate(ranges):
        r = float(raw)
        if not math.isfinite(r) or r < max(float(range_min), float(self_filter_radius_m)) or r > float(range_max):
            invalid_rejected += 1
            continue
        angle = float(angle_min) + index * float(angle_increment) + yaw0
        x = x0 + r * math.cos(angle)
        y = y0 + r * math.sin(angle)
        bearing = math.atan2(y, x)
        base_range = math.hypot(x, y)
        if not limits[0] <= bearing <= limits[-1]:
            continue
        points.append({"x_m": x, "y_m": y, "range_m": base_range, "bearing_rad": bearing})
        sector = next((i for i in range(5) if limits[i] <= bearing < limits[i + 1]), 4 if bearing == limits[-1] else None)
        if sector is not None and (nearest[sector] is None or base_range < nearest[sector]):
            nearest[sector] = base_range
        usable += 1
        if x > 0.0 and abs(y) <= float(corridor_half_width_m):
            corridor.append(base_range)
    names = ("right_m", "front_right_m", "front_m", "front_left_m", "left_m")
    return {
        "input_adapter": "laser_scan",
        "sectors": dict(zip(names, nearest)),
        "route_corridor": {
            "occupied": bool(corridor),
            "nearest_range_m": min(corridor) if corridor else None,
        },
        "points_base_link": points,
        "ground_points_rejected": 0,
        "invalid_returns_rejected": invalid_rejected,
        "usable_points": usable,
    }


def _hsv_ranges(config: dict[str, Any], class_name: str) -> list[tuple[list[int], list[int]]]:
    classes = config["camera"]["hsv_classes"][class_name]["ranges"]
    return [
        (list(item["lower_hsv"]["value"]), list(item["upper_hsv"]["value"]))
        for item in classes
    ]


def classify_hsv_image(image_bgr, config: dict[str, Any], horizontal_fov_rad: float) -> list[dict[str, Any]]:
    """Find connected colored blobs and normalize them to camera evidence objects."""
    import cv2
    import numpy as np

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    height, width = hsv.shape[:2]
    output = []
    minimum_area = int(value_of(config["camera"]["minimum_blob_area_px"]))
    min_confidence = float(value_of(config["camera"]["minimum_class_confidence"]))
    merge_bearing_gap = float(value_of(config["camera"]["same_class_merge_bearing_gap_rad"]))
    merge_gap_px = merge_bearing_gap * width / float(horizontal_fov_rad)
    precedence = ("barrier", "gate", "post")
    for class_name in precedence:
        combined = np.zeros((height, width), dtype=np.uint8)
        for low, high in _hsv_ranges(config, class_name):
            combined = cv2.bitwise_or(
                combined,
                cv2.inRange(hsv, np.asarray(low, dtype=np.uint8), np.asarray(high, dtype=np.uint8)),
            )
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(combined, 8)
        components = []
        for label in range(1, count):
            x, y, blob_w, blob_h, area = (int(v) for v in stats[label])
            if area < minimum_area or blob_w <= 0 or blob_h <= 0:
                continue
            score = min(1.0, area / float(blob_w * blob_h))
            if score < min_confidence:
                continue
            components.append({"x0": x, "y0": y, "x1": x + blob_w, "y1": y + blob_h,
                               "area": area, "score": score})
        components.sort(key=lambda item: (item["x0"], item["y0"]))
        clusters: list[dict[str, Any]] = []
        for component in components:
            target = None
            for cluster in reversed(clusters):
                horizontal_gap = component["x0"] - cluster["x1"]
                vertical_overlap = min(component["y1"], cluster["y1"]) - max(component["y0"], cluster["y0"])
                smaller_height = min(component["y1"] - component["y0"], cluster["y1"] - cluster["y0"])
                if horizontal_gap > merge_gap_px:
                    break
                if vertical_overlap >= 0.25 * smaller_height:
                    target = cluster
                    break
            if target is None:
                clusters.append(dict(component))
            else:
                old_area = target["area"]
                new_area = old_area + component["area"]
                target["score"] = (target["score"] * old_area + component["score"] * component["area"]) / new_area
                target["area"] = new_area
                target["x0"] = min(target["x0"], component["x0"])
                target["y0"] = min(target["y0"], component["y0"])
                target["x1"] = max(target["x1"], component["x1"])
                target["y1"] = max(target["y1"], component["y1"])
        for cluster in clusters:
            blob_w = cluster["x1"] - cluster["x0"]
            blob_h = cluster["y1"] - cluster["y0"]
            area = cluster["area"]
            score = min(1.0, float(cluster["score"]))
            center_x = 0.5 * (cluster["x0"] + cluster["x1"])
            # Camera horizontal projection: left is positive in base_link.
            bearing = ((width / 2.0 - center_x) / width) * float(horizontal_fov_rad)
            output.append({
                "class": class_name,
                "colour_score": score,
                "confidence": score,
                "blob_area_px": int(area),
                "bearing_rad": bearing,
                "apparent_size_px": float(max(blob_w, blob_h)),
                "_bbox": (cluster["x0"], cluster["y0"], cluster["x1"], cluster["y1"]),
            })

    # Multi-colour meshes can produce a red and orange mask over the same
    # physical object (for example, a red barrier with orange reflectors).
    # Keep the stronger color classification when their image boxes overlap
    # substantially, while leaving adjacent objects as separate detections.
    class_precedence = {"barrier": 0, "gate": 1, "post": 2}
    candidates = sorted(
        output,
        key=lambda item: (
            -item["confidence"],
            class_precedence[item["class"]],
            -item["blob_area_px"],
            item["bearing_rad"],
        ),
    )
    deduplicated = []
    for candidate in candidates:
        x0, y0, x1, y1 = candidate["_bbox"]
        candidate_area = max(1, (x1 - x0) * (y1 - y0))
        overlaps_existing = False
        for accepted in deduplicated:
            if {candidate["class"], accepted["class"]} != {"barrier", "post"}:
                continue
            ax0, ay0, ax1, ay1 = accepted["_bbox"]
            intersection = max(0, min(x1, ax1) - max(x0, ax0)) * max(
                0, min(y1, ay1) - max(y0, ay0)
            )
            accepted_area = max(1, (ax1 - ax0) * (ay1 - ay0))
            union = candidate_area + accepted_area - intersection
            if union and intersection / union >= 0.5:
                overlaps_existing = True
                break
        if not overlaps_existing:
            deduplicated.append(candidate)

    for item in deduplicated:
        item.pop("_bbox")
    deduplicated.sort(key=lambda item: (item["bearing_rad"], -item["blob_area_px"], item["class"]))
    return deduplicated


def encode_record(record: dict[str, Any]) -> str:
    return json.dumps(record, sort_keys=True, separators=(",", ":"), allow_nan=False)
