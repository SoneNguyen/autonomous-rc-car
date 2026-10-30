"""Contact identity and sensor-frame transform qualification."""
from __future__ import annotations

from pathlib import Path
from typing import Any


def run_contact_tf_case(scenario: dict[str, Any], *, output_dir: Path) -> dict[str, Any]:
    """Capture one deliberate/no-contact run and qualify entity identity plus TF continuity."""
    try:
        from tools.checks.runtime_capture import capture_contact_tf

        capture = capture_contact_tf(scenario, output_dir=output_dir)
        expected = set(scenario["setup"].get("expected_contact_entities", []))
        events = capture["contacts"]
        names = [f"{row['collision1']['name']} {row['collision2']['name']}" for row in events]
        found = {
            entity for entity in expected
            if any(entity in collision_names for collision_names in names)
        }
        unexpected_contact = bool(events) if not expected else any(
            not any(entity in collision_names for entity in expected)
            for collision_names in names
        )
        contact_pass = found == expected and not unexpected_contact
        tf_pass = (
            capture["sensor_sample_count"] > 0
            and not capture["missing_sensor_frames"]
            and capture["tf_gap_count"] == 0
        )
        observations = [
            {
                "observation_id": f"{scenario['scenario_id']}/contact-identity",
                "source": "/contacts",
                "stamp_sim_ns": int(events[-1]["stamp_sim_ns"] if events else capture["clock_last_sim_ns"]),
                "frame_id": "base_link",
                "value": {"contact_event_count": len(events), "collision_pairs": names,
                          "identified_entities": sorted(found)},
                "ground_truth": {"expected_contact_entities": sorted(expected),
                                 "ground_only_false_events_allowed": 0},
                "unit": "entity-event",
                "tolerance_id": scenario["tolerances"][0]["tolerance_id"],
                "error": None if contact_pass else "contact identity differs from the fixed fixture ground truth",
                "result": "pass" if contact_pass else "fail",
                "evidence_ref": capture["evidence_ref"],
            },
            {
                "observation_id": f"{scenario['scenario_id']}/tf-continuity",
                "source": "/tf_static at sensor sample stamps",
                "stamp_sim_ns": int(capture["clock_last_sim_ns"]),
                "frame_id": "base_link",
                "value": {"sample_count": capture["sensor_sample_count"],
                          "resolved_sensor_frames": capture["resolved_sensor_frames"],
                          "missing_sensor_frames": capture["missing_sensor_frames"],
                          "tf_gap_count": capture["tf_gap_count"]},
                "ground_truth": {"required_sensor_frames": capture["required_sensor_frames"],
                                 "unresolved_sample_limit": 0},
                "unit": "transform-lookup",
                "tolerance_id": scenario["tolerances"][1]["tolerance_id"],
                "error": None if tf_pass else "one or more sensor timestamps did not resolve continuously to base_link",
                "result": "pass" if tf_pass else "fail",
                "evidence_ref": capture["evidence_ref"],
            },
        ]
        passed = contact_pass and tf_pass
        return {
            "status": "pass" if passed else "fail",
            "observations": observations,
            "evidence_refs": [capture["evidence_ref"], capture["log_ref"]],
            "reason": None if passed else next(row["error"] for row in observations if row["result"] == "fail"),
        }
    except Exception as exc:
        return {"status": "blocked", "observations": [], "evidence_refs": [], "reason": str(exc)}
