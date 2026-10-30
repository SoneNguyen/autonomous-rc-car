"""Visible-label and frozen-detector observations for the research camera scene."""
from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np


CLASS_LABELS = {"cone": 10, "pedestrian": 20, "vehicle": 30}
CLASS_TO_INDEX = {"cone": 1, "pedestrian": 2, "vehicle": 3}
REPO_ROOT = Path(__file__).resolve().parents[2]


def _evidence_refs(output_dir: Path) -> list[str]:
    return [
        str(path.relative_to(REPO_ROOT))
        for path in sorted(output_dir.rglob("*")) if path.is_file()
    ]


def decode_rgb8(data: bytes, width: int, height: int, step: int, encoding: str) -> np.ndarray:
    """Decode a row-strided ROS rgb8 image without channel reordering."""
    if encoding.lower() not in {"rgb8", "8uc3"}:
        raise ValueError(f"expected RGB8 input, received {encoding!r}")
    if width < 1 or height < 1 or step < width * 3 or len(data) < step * height:
        raise ValueError("RGB image dimensions, row stride, or payload are invalid")
    rows = np.frombuffer(data, dtype=np.uint8, count=step * height).reshape(height, step)
    return rows[:, : width * 3].reshape(height, width, 3).copy()


def visible_instance_boxes(
    label_rgb: np.ndarray,
    entities_by_class: Mapping[str, list[str]],
) -> list[dict[str, Any]]:
    """Extract inclusive pixel boxes from Gazebo's instance/class RGB label map."""
    if label_rgb.ndim != 3 or label_rgb.shape[2] != 3 or label_rgb.dtype != np.uint8:
        raise ValueError("panoptic label map must be an HxWx3 uint8 array")
    instance_ids = label_rgb[:, :, 0].astype(np.uint16) * 256 + label_rgb[:, :, 1].astype(np.uint16)
    class_ids = label_rgb[:, :, 2]
    annotations: list[dict[str, Any]] = []
    for class_name, label_id in CLASS_LABELS.items():
        target = (class_ids == label_id) & (instance_ids != 0)
        visible_ids = np.unique(instance_ids[target])
        entities = sorted(entities_by_class.get(class_name, []))
        if len(visible_ids) > 1 and len(visible_ids) != len(entities):
            raise ValueError(
                f"cannot associate {len(visible_ids)} visible {class_name} instances "
                f"with {len(entities)} scenario entities"
            )
        for position, instance_id in enumerate(visible_ids.tolist()):
            ys, xs = np.nonzero(target & (instance_ids == instance_id))
            entity = entities[position] if position < len(entities) else (entities[0] if len(entities) == 1 else None)
            annotations.append({
                "entity": entity,
                "class_name": class_name,
                "class_id": label_id,
                "instance_id": int(instance_id),
                "bbox_xyxy_inclusive_px": [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
                "visible_pixel_count": int(xs.size),
            })
    return sorted(annotations, key=lambda row: (row["class_name"], row["instance_id"]))


def box_iou_inclusive(a: list[float], b: list[float]) -> float:
    """Intersection-over-union for inclusive pixel-edge XYXY boxes."""
    ax0, ay0, ax1, ay1 = map(float, a)
    bx0, by0, bx1, by1 = map(float, b)
    width = max(0.0, min(ax1, bx1) - max(ax0, bx0) + 1.0)
    height = max(0.0, min(ay1, by1) - max(ay0, by0) + 1.0)
    intersection = width * height
    area_a = max(0.0, ax1 - ax0 + 1.0) * max(0.0, ay1 - ay0 + 1.0)
    area_b = max(0.0, bx1 - bx0 + 1.0) * max(0.0, by1 - by0 + 1.0)
    union = area_a + area_b - intersection
    return intersection / union if union else 0.0


def associate_boxes(
    truth: list[dict[str, Any]], detections: list[dict[str, Any]], *, minimum_iou: float,
) -> list[dict[str, Any]]:
    """Greedily associate class-matched detector boxes to unique visible instances."""
    if not 0.0 <= minimum_iou <= 1.0:
        raise ValueError("minimum IoU must be in [0, 1]")
    candidates = []
    for truth_index, target in enumerate(truth):
        for detection_index, detection in enumerate(detections):
            if target["class_name"] != detection["class_name"]:
                continue
            score = box_iou_inclusive(target["bbox_xyxy_inclusive_px"], detection["bbox_xyxy_inclusive_px"])
            if score >= minimum_iou:
                candidates.append((score, truth_index, detection_index))
    candidates.sort(reverse=True)
    used_truth: set[int] = set()
    used_detections: set[int] = set()
    matches = []
    for score, truth_index, detection_index in candidates:
        if truth_index in used_truth or detection_index in used_detections:
            continue
        used_truth.add(truth_index)
        used_detections.add(detection_index)
        matches.append({"truth_index": truth_index, "detection_index": detection_index, "iou": score})
    return matches


def run_camera_case(scenario: dict[str, Any], *, output_dir: Path) -> dict[str, Any]:
    """Capture one camera case and compare the last aligned RGB/label/detector frame."""
    try:
        from tools.checks.runtime_capture import capture_camera

        capture = capture_camera(scenario, output_dir=output_dir)
        frame = capture["frames"][-1]
        width, height, step = frame["width"], frame["height"], frame["step"]
        label = decode_rgb8(
            (REPO_ROOT / frame["label_raw_path"]).read_bytes(), width, height, step,
            frame["label_encoding"],
        )
        entities_by_class: dict[str, list[str]] = {name: [] for name in CLASS_LABELS}
        for placement in scenario["obstacle_placements"]:
            entity = placement["entity_id"]
            class_name = next((name for name in CLASS_LABELS if entity.startswith(name)), None)
            if class_name is not None:
                entities_by_class[class_name].append(entity)
        truth = visible_instance_boxes(label, entities_by_class)
        detections = frame["detections"]
        iou_limit = float(scenario["setup"].get("minimum_box_iou", 0.5))
        matches = associate_boxes(truth, detections, minimum_iou=iou_limit)
        target = scenario["setup"].get("target_entity")
        if scenario["setup"].get("expect_detector_stale"):
            age_s = max(0, capture["clock_last_sim_ns"] - capture["clock_start_sim_ns"]) / 1e9
            minimum_age_s = float(scenario["tolerances"][0].get("minimum", 0.5))
            stale = capture.get("detector_staleness") or {}
            passed = (
                not detections
                and age_s >= minimum_age_s
                and stale.get("state") == "stale"
                and stale.get("detection_message_count") == 0
            )
            reason = None if passed else "detector output did not remain stale for the declared simulation interval"
            observation = {
                "observation_id": f"{scenario['scenario_id']}/staleness",
                "source": "/detections",
                "stamp_sim_ns": frame["stamp_sim_ns"],
                "frame_id": "camera_link",
                "value": {
                    "detection_count": len(detections), "stale_age_sim_s": age_s,
                    "observer_state": stale.get("state"),
                    "fault_injection": stale.get("fault_injection"),
                },
                "ground_truth": {"expected_no_detection": True, "minimum_stale_age_sim_s": minimum_age_s},
                "unit": "s",
                "tolerance_id": scenario["tolerances"][0]["tolerance_id"],
                "error": reason,
                "result": "pass" if passed else "fail",
                "evidence_ref": frame["evidence_ref"],
            }
            return {
                "status": "pass" if passed else "fail",
                "observations": [observation],
                "evidence_refs": [
                    frame["evidence_ref"], capture["log_ref"],
                    capture["render_configuration_ref"],
                ],
                "reason": reason,
            }
        target_truth = [row for row in truth if row["entity"] == target]
        target_matches = [
            match for match in matches
            if truth[match["truth_index"]]["entity"] == target
        ]
        visible_expected = bool(scenario["setup"].get("target_visible", True))
        pass_case = (bool(target_truth) and bool(target_matches)) if visible_expected else not target_truth
        check_all = bool(scenario["setup"].get("check_all_visible_instances", False))
        unmatched_detection_indices = set(range(len(detections))) - {
            match["detection_index"] for match in matches
        }
        merged_detection_indices = {
            detection_index for detection_index, detection in enumerate(detections)
            if sum(
                item["class_name"] == detection["class_name"]
                and box_iou_inclusive(item["bbox_xyxy_inclusive_px"], detection["bbox_xyxy_inclusive_px"]) >= iou_limit
                for item in truth
            ) > 1
        }
        if check_all and (len(matches) != len(truth) or unmatched_detection_indices or merged_detection_indices):
            pass_case = False
        reason = None
        if visible_expected and not target_truth:
            reason = "target has no visible labeled pixels"
        elif visible_expected and not target_matches:
            reason = "frozen detector has no class-matched box over the declared IoU threshold"
        elif not visible_expected and target_truth:
            reason = "target remained visible in a case declared fully occluded"
        elif check_all and unmatched_detection_indices:
            reason = "one or more detector boxes are duplicate or unmatched"
        elif check_all and merged_detection_indices:
            reason = "a detector box overlaps multiple distinct visible instances"
        elif check_all and len(matches) != len(truth):
            reason = "not every visible scenario instance has a class-matched detector box"
        observation = {
            "observation_id": f"{scenario['scenario_id']}/boxes",
            "source": "/detections",
            "stamp_sim_ns": frame["stamp_sim_ns"],
            "frame_id": "camera_link",
            "value": {
                "visible_truth_boxes": truth,
                "detections": detections,
                "matches": matches,
                "duplicate_or_unmatched_detection_indices": sorted(unmatched_detection_indices),
                "merged_detection_indices": sorted(merged_detection_indices),
                "target_entity": target,
                "target_visible": bool(target_truth),
            },
            "ground_truth": {
                "target_entity": target,
                "target_class": scenario["setup"]["target_class"],
                "target_visible": visible_expected,
                "camera_range_m": scenario["setup"].get("camera_range_m"),
            },
            "unit": "ratio",
            "tolerance_id": scenario["tolerances"][0]["tolerance_id"],
            "error": reason,
            "result": "pass" if pass_case else "fail",
            "evidence_ref": frame["evidence_ref"],
        }
        return {
            "status": "pass" if pass_case else "fail",
            "observations": [observation],
            "evidence_refs": [
                frame["evidence_ref"], capture["log_ref"],
                capture["render_configuration_ref"],
            ],
            "reason": reason,
        }
    except Exception as exc:
        return {
            "status": "blocked", "observations": [],
            "evidence_refs": _evidence_refs(output_dir), "reason": str(exc),
        }


def run_camera_pilot(scenario: dict[str, Any], *, output_dir: Path) -> dict[str, Any]:
    """Record visible pixel boxes and ranges without assigning a score or tolerance."""
    try:
        from tools.checks.runtime_capture import capture_camera

        capture = capture_camera(scenario, output_dir=output_dir, require_aligned_frames=False)
        frame = capture["frames"][-1]
        labels = decode_rgb8(
            (REPO_ROOT / frame["label_raw_path"]).read_bytes(),
            frame["width"], frame["height"], frame["step"], frame["label_encoding"],
        )
        entities_by_class: dict[str, list[str]] = {name: [] for name in CLASS_LABELS}
        for placement in scenario["obstacle_placements"]:
            entity = placement["entity_id"]
            class_name = next((name for name in CLASS_LABELS if entity.startswith(name)), None)
            if class_name is not None:
                entities_by_class[class_name].append(entity)
        boxes = visible_instance_boxes(labels, entities_by_class)
        expected_ranges = scenario["setup"].get("camera_ranges_m", {})
        distance_bands = scenario["setup"].get("distance_band_by_entity", {})
        rows = []
        for box in boxes:
            x0, y0, x1, y1 = box["bbox_xyxy_inclusive_px"]
            rows.append({
                **box,
                "width_px": x1 - x0 + 1,
                "height_px": y1 - y0 + 1,
                "area_px": box["visible_pixel_count"],
                "camera_range_m": expected_ranges.get(box["entity"]),
                "distance_band": distance_bands.get(box["entity"]),
            })
        observation = {
            "observation_id": f"{scenario['scenario_id']}/visible-pixel-boxes",
            "source": "/camera/labels",
            "stamp_sim_ns": frame["stamp_sim_ns"],
            "frame_id": "camera_link",
            "value": {"boxes": rows, "detector_outputs_observed": len(frame["detections"])},
            "ground_truth": {"entities_by_class": entities_by_class},
            "unit": "px",
            "tolerance_id": None,
            "result": "not_applicable",
            "evidence_ref": frame["evidence_ref"],
        }
        return {
            "status": "unscored", "observations": [observation],
            "evidence_refs": [frame["evidence_ref"], capture["log_ref"]],
            "reason": "unscored label pilot; freeze distance bands before qualification",
        }
    except Exception as exc:
        return {
            "status": "blocked", "observations": [],
            "evidence_refs": _evidence_refs(output_dir), "reason": str(exc),
        }
