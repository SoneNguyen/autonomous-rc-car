"""Tests for the isolated, loopback-only Obstacle course dashboard."""

from __future__ import annotations

import json
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from car_decision.dashboard_node import DashboardStore, LatencyMetrics, make_server


RUN_ID = "00000000-0000-4000-8000-000000000007"


def state_record(run_id: str = RUN_ID) -> dict:
    return {
        "schema_version": "1.0", "run_id": run_id, "scenario_id": "S1",
        "run_purpose": "diagnostic", "sim_time_ns": 1230000000, "clock_epoch": 0,
        "behavior": "CAUTION", "reason": "front corridor return",
        "attribution": {
            "triggers": [{"component": "lidar"}],
            "supporters": [{"component": "odometry"}],
            "vetoes": [], "overruled": [], "stale_components": [],
        },
        "freshness": {"lidar": "fresh", "camera": "stale"},
    }


def evidence(component: str, run_id: str = RUN_ID) -> dict:
    return {
        "run_id": run_id, "scenario_id": "S1", "run_purpose": "diagnostic",
        "component": component, "freshness": {"state": "stale" if component == "camera" else "fresh"},
        "evidence": {"route_corridor": {"nearest_range_m": 0.8}} if component == "lidar" else {"objects": []},
    }


def event_record(run_id: str = RUN_ID) -> dict:
    return {
        "run_id": run_id, "scenario_id": "S1", "run_purpose": "diagnostic",
        "event_kind": "transition", "previous_behavior": "FOLLOW_ROUTE", "behavior": "CAUTION",
        "sim_time_ns": 1230000000, "published_monotonic_ns": time.monotonic_ns(),
        "reason": "front corridor return", "attribution": {"triggers": [{"component": "lidar"}]},
    }


def test_dashboard_binds_only_to_loopback():
    store = DashboardStore()
    with pytest.raises(ValueError, match="loopback"):
        make_server(store, LatencyMetrics(), "0.0.0.0", 0)


def test_get_ui_and_post_returns_405():
    server = make_server(DashboardStore(), LatencyMetrics(), port=0)
    import threading
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urlopen(base + "/") as response:
            body = response.read().decode()
            assert response.status == 200
            assert "RC obstacle course" in body
            assert "Pause display" in body
            assert "History component" in body
            assert "Resume and reset view" in body
            assert "select a card for its live preview and history" in body
            assert "camera.jpg" in body
            assert "updateCameraPreview" in body
            assert "cameraFrameSequence" in body
            assert "cameraFrameTimer=setInterval" in body
            assert "refreshCameraFrame(refs.canvas,refs.status),125" in body
            assert "cameraPreviewRunId" in body
            assert "resetCameraPreviewForRun(record.run_id)" in body
            assert "resetCameraPreviewForRun(m.snapshot.run_id)" in body
            assert "resetCameraPreviewForRun(m.run_id)" in body
            assert "generation!==cameraPreviewGeneration" in body
            assert "createImageBitmap" in body
            assert "node('canvas','camera-frame')" in body
            assert "cameraFrameObjectUrl" not in body
            assert "canvas?.isConnected" in body
            assert "if(activePreview==='camera'){const refs=mountCameraPreview(layout);setText(refs.evidenceStatus,text);return;}" in body
            assert "setText(refs.evidenceStatus,`${freshness} evidence · sim ${simTime} s`)" in body
            assert "refs.objectRows.push({row,label,value})" in body
            assert "setText(row.value" in body
            assert "refs.details.replaceChildren" not in body
            assert "const dataKey=JSON.stringify" not in body
            assert "cameraFrameFailures>=3" in body
            assert "Live frame ${sequence||cameraFrameSequence}" in body
            assert "?frame=${Date.now()}" not in body
            assert "cameraFrameSequence}`" in body
            assert "Top-down forward LiDAR scan" in body
            assert "Stopping distance" in body
            assert "status-trigger" in body
            assert "event-triggered" in body
            assert "eventComponents" in body  # UI renders roles from live attribution.
            assert "linear-gradient" not in body
            assert "radial-gradient" not in body
            assert "box-shadow" not in body
            assert ".card.status-trigger{border:2px solid var(--red)}" in body
            assert "const cardNodes" in body
            assert "card.isConnected" in body
            assert "new MutationObserver" in body
            for component in ("lidar", "camera", "imu", "odometry", "contact"):
                assert f'data-component="{component}"' in body
            assert "historySignature" in body
            assert "grid.replaceChildren()" not in body
        with pytest.raises(HTTPError) as error:
            urlopen(Request(base + "/", data=b"x", method="POST"))
        assert error.value.code == 405
        with pytest.raises(HTTPError) as error:
            urlopen(base + "/mutate")
        assert error.value.code == 404
        with pytest.raises(HTTPError) as error:
            urlopen(Request(base + "/", method="PATCH"))
        assert error.value.code == 405
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_active_run_filter_role_and_freshness_are_independent():
    store = DashboardStore()
    assert store.accept("state", state_record())
    assert store.accept("evidence", evidence("lidar"))
    assert store.accept("evidence", evidence("camera"))
    assert not store.accept("evidence", evidence("imu", "00000000-0000-4000-8000-000000000008"))
    snapshot = store.snapshot()
    assert snapshot["run_id"] == RUN_ID
    assert snapshot["components"]["lidar"]["status"] == "trigger"
    assert snapshot["components"]["lidar"]["freshness"] == "fresh"
    assert snapshot["components"]["camera"]["status"] == "idle"
    assert snapshot["components"]["camera"]["freshness"] == "stale"
    assert "0.80 m" in snapshot["components"]["lidar"]["summary"]


def test_component_trigger_highlight_stays_stable_during_same_behavior():
    store = DashboardStore()
    first = state_record()
    assert store.accept("state", first)
    update = state_record()
    update["sim_time_ns"] += 50_000_000
    update["attribution"] = {
        "triggers": [{"component": "camera"}],
        "supporters": [], "vetoes": [{"component": "camera"}],
        "overruled": [], "stale_components": [],
    }
    assert store.accept("state", update)
    cards = store.snapshot()["components"]
    assert cards["lidar"]["status"] == "trigger"
    assert cards["camera"]["status"] == "veto"

    transition = state_record()
    transition["behavior"] = "AVOID_LEFT"
    transition["attribution"] = {
        "triggers": [{"component": "camera"}],
        "supporters": [], "vetoes": [], "overruled": [], "stale_components": [],
    }
    assert store.accept("state", transition)
    cards = store.snapshot()["components"]
    assert cards["camera"]["status"] == "trigger"
    assert cards["lidar"]["status"] == "idle"


def test_component_evidence_and_camera_preview_are_read_only_endpoints():
    store = DashboardStore()
    assert store.accept("state", state_record())
    assert store.accept("evidence", evidence("lidar"))
    jpeg = b"\xff\xd8small-preview\xff\xd9"
    assert store.accept_camera_preview(jpeg) == 1
    server = make_server(store, LatencyMetrics(), port=0)
    import threading
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urlopen(base + "/evidence/lidar") as response:
            body = json.loads(response.read())
            assert response.status == 200
            assert body["component"] == "lidar"
            assert body["evidence"]["route_corridor"]["nearest_range_m"] == 0.8
        with urlopen(base + "/camera.jpg") as response:
            assert response.status == 200
            assert response.headers["Content-Type"] == "image/jpeg"
            assert response.headers["X-Preview-Sequence"] == "1"
            assert response.read() == jpeg
        with pytest.raises(HTTPError) as error:
            urlopen(base + "/camera.jpg?since=1")
        assert error.value.code == 304
        next_run = "00000000-0000-4000-8000-000000000008"
        assert store.accept("state", state_record(next_run))
        with pytest.raises(HTTPError) as error:
            urlopen(base + "/camera.jpg?since=1")
        assert error.value.code == 503
        next_jpeg = b"\xff\xd8next-run-preview\xff\xd9"
        assert store.accept_camera_preview(next_jpeg) == 1
        with urlopen(base + "/camera.jpg?since=0") as response:
            assert response.status == 200
            assert response.headers["X-Preview-Sequence"] == "1"
            assert response.read() == next_jpeg
        with pytest.raises(HTTPError) as error:
            urlopen(base + "/camera.jpg?since=bad")
        assert error.value.code == 400
        with pytest.raises(HTTPError) as error:
            urlopen(Request(base + "/camera.jpg", data=b"x", method="POST"))
        assert error.value.code == 405
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_trigger_card_keeps_trigger_evidence_visible_after_sensor_sample_changes():
    store = DashboardStore()
    state = state_record()
    state["attribution"]["triggers"] = [{
        "component": "contact",
        "evidence_sequence": 4,
        "value": [{
            "collision_a": "gazebo_truck::base_link::chassis_collision",
            "collision_b": "opening_blocker::obstacle_link::opening_blocker_contact",
        }],
        "unit": "contact_pair",
        "freshness": "fresh",
    }]
    store.accept("state", state)
    assert store.accept("evidence", evidence("contact"))
    card = store.snapshot()["components"]["contact"]
    assert card["status"] == "trigger"
    assert "chassis_collision" in card["summary"]
    assert "opening_blocker_contact" in card["summary"]


def test_decision_payload_is_retained_exactly_and_reconnect_replays():
    store = DashboardStore(history_limit=8)
    store.accept("state", state_record())
    event = event_record()
    store.accept("decision", event)
    snapshot = store.snapshot()
    assert snapshot["history"] == [event]
    ok, replay = store.replay_after(1)
    assert ok
    assert replay[-1]["kind"] == "decision"
    assert replay[-1]["payload"] == event
    assert store.accept("decision", {**event, "event_kind": "evaluation"})
    assert len(store.snapshot()["history"]) == 2


def test_reconnect_outside_bounded_history_gets_snapshot():
    store = DashboardStore(history_limit=2)
    store.accept("state", state_record())
    for _ in range(5):
        store.accept("decision", event_record())
    ok, replay = store.replay_after(0)
    assert not ok
    assert replay == []


def test_sse_event_ids_remain_monotonic_when_a_new_run_replaces_the_snapshot():
    store = DashboardStore(history_limit=8)
    assert store.accept("state", state_record())
    _, old_cursor = store.snapshot_with_cursor()

    next_run = "00000000-0000-4000-8000-000000000008"
    assert store.accept("state", state_record(next_run))
    replay_ok, replay = store.replay_after(old_cursor)

    assert replay_ok
    assert [item["kind"] for item in replay] == ["run", "state"]
    assert [item["id"] for item in replay] == list(range(old_cursor + 1, old_cursor + 3))


def test_sse_starts_with_snapshot_and_uses_monotonic_flush_measurement():
    store, metrics = DashboardStore(), LatencyMetrics()
    store.accept("state", state_record())
    server = make_server(store, metrics, port=0)
    import threading
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with urlopen(f"http://127.0.0.1:{server.server_port}/events", timeout=2) as response:
            first_line = response.readline().decode().strip()
            assert first_line.startswith("data:")
            assert json.loads(first_line[6:])["type"] == "snapshot"
            response.readline()  # blank line after the snapshot frame
            decision = event_record()
            store.accept("decision", decision)
            event_id = response.readline().decode().strip()
            data_line = response.readline().decode().strip()
            assert event_id.startswith("id: ")
            received = json.loads(data_line[6:])
            assert received["type"] == "decision"
            assert received["event"] == decision
        deadline = time.monotonic() + 1.0
        while metrics.summary()["count"] == 0 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert metrics.summary()["count"] == 1
        result = metrics.summary()
        assert result["maximum_ms"] >= 0
        assert result["within_200_ms"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_client_disconnect_does_not_block_ros_side_store_updates():
    store = DashboardStore(history_limit=16)
    started = time.monotonic()
    for _ in range(1000):
        store.accept("state", state_record())
    assert time.monotonic() - started < 1.0
