# Feature 007 Dashboard Contract

## HTTP surface

The dashboard is a separate read-only process using Python's standard-library `http.server`. It serves one self-contained page and one one-way event stream. It has no command, scenario-selection, configuration-update, or mutation endpoint.

| Method | Path | Response | Behavior |
|---|---|---|---|
| `GET` | `/` | `text/html; charset=utf-8` | Self-contained page; no CDN or remote scripts/styles |
| `GET` | `/events` | `text/event-stream` | Server-sent event stream for component snapshots, state changes, decisions, and run status |

Default bind is `127.0.0.1:8765`; port is configurable. Open `http://localhost:8765/` from a Windows browser while the WSL process is running. The server must reject non-GET methods and paths outside these two routes.

## SSE messages

Each SSE `data:` value is one JSON object. `id` is a monotonic event sequence within the active `run_id`; `event` is one of:

- `snapshot`: initial dashboard state after connection;
- `component`: one normalized evidence/status update;
- `decision`: one complete transition or attribution-changing evaluation object validated by `decision-event.schema.json`;
- `run`: scenario ID, run ID, demonstration status, and latest pass/partial state;
- `heartbeat`: keepalive without vehicle state changes.

The snapshot/state envelope is:

```json
{
  "schema_version": "1.0",
  "run_id": "00000000-0000-4000-8000-000000000001",
  "scenario_id": "S1",
  "sim_time_ns": 1230000000,
  "clock_epoch": 0,
  "active_behavior": "CAUTION",
  "reason": "front corridor return at the configured caution range",
  "components": {
    "lidar": {"status": "trigger", "freshness": "fresh", "summary": "front 8.2 m"},
    "camera": {"status": "supporting", "freshness": "fresh", "summary": "red barrier"},
    "imu": {"status": "idle", "freshness": "fresh", "summary": "within configured bounds"},
    "odometry": {"status": "supporting", "freshness": "fresh", "summary": "0.8 m/s"},
    "contact": {"status": "idle", "freshness": "fresh", "summary": "no contact"}
  },
  "history": []
}
```

Component `status` is the decision role `idle`, `supporting`, `trigger`, `veto`, or `overruled`. `freshness` is an independent `fresh`, `stale`, or `unavailable` value, so a stale LiDAR sample that triggers STOP remains visibly both `trigger` and `stale`. A `veto` status means the component's evidence prevented a candidate behavior; for example, a clear gate opening lets the camera veto avoidance. The decision node emits an `evaluation` event when veto/overruled attribution changes without changing behavior, so this evidence appears in `history`. Each history record uses the exact decision-event payload received from ROS; the dashboard does not recompute triggers or reasons. Run scoring uses transition events only.

## Delivery and failure behavior

- The server consumes ROS topics asynchronously and uses a bounded in-memory queue; a slow/disconnected browser cannot block sensor callbacks, decisions, planner inputs, or vehicle commands.
- A reconnect receives a fresh `snapshot`; event IDs allow the page to identify a gap and refresh state.
- The decision event carries `published_monotonic_ns` from the host OS monotonic clock. The SSE writer records its own monotonic timestamp after flushing the frame and subtracts the event timestamp; both timestamps are from the same host clock. The measured delta must remain below the spec's 200 ms target under scenario load. This wall-clock metadata is excluded from deterministic replay comparisons.
- If the dashboard process exits, the ROS decision and control graph continues unchanged. The run report records dashboard unavailable; this does not create or alter a drive command.
- The bind address is loopback by default; no public-network exposure is part of this feature.
