# Drone Server — For Agents

Detailed map of this crate for AI assistants and maintainers: **what lives where**, **how the TUI is threaded**, and **how to extend** without breaking MAVLink or UX semantics.

---

## 1. Role in the workspace

- The parent repo (**Code**) is the SAR drone project; see **[Code/README.md](../README.md)** for the full system diagram.
- **Drone Server** is the component that **terminates MAVLink toward the flight controller** (UDP or serial). Higher layers (Gateway, future LLM tools, frontend backends) should prefer calling this library (or a thin API around it) instead of duplicating MAVLink command knowledge.
- **Not in this crate:** HTTP APIs, web UI, model inference. The Gateway crate may eventually depend on or spawn processes that use `drone_server`.

---

## 2. Library layout (`src/`)

| Module / path | Responsibility |
|---------------|----------------|
| **`lib.rs`** | Declares `mission`, `cmd`, `mission_store`, `mavlink_connect`. Re-exports command helpers and types listed below so callers can `use drone_server::rtl` etc. |
| **`cmd.rs`** | ArduPilotMega-oriented helpers: `VehicleIds`, arm/disarm/force_arm, land, RTL, `set_mode_guided` / `set_mode_auto`, takeoff, `goto_global` / `goto_global_command_int`, `mission_set_current`, `mission_start`. Returns `MavMessage` or sends on `MavConnection` as appropriate. |
| **`mission.rs`** | Protocol-agnostic types: `Waypoint`, `Mission`, `WaypointCommand` with **serde**. Used for JSON/config/mission representation; not a full implementation of MAVLink mission upload/download by itself. |
| **`mission_store.rs`** | **TUI-centric but in the library** so tests and future APIs can reuse it: stores `MISSION_ITEM_INT` payloads from the FC, `MISSION_CURRENT`, optional **snapshot** `(items, seq)` for interrupt/override, **`upload_pending`** for FC-driven mission item upload during resume, and flags like `upload_done`. Methods: `update_from_item`, `update_current_seq`, `ensure_snapshot_for_pause`, `get_snapshot`, `set_upload_pending`, `take_upload_item`, `set_upload_done`, etc. |
| **`mavlink_connect.rs`** | Binary-facing utilities: `DEFAULT_UDP_URL`, serial defaults, `resolve_from_args`, `usage_string`, `open_error_message`, `tune_connection` (e.g. `set_allow_recv_any_version`). Keeps UDP vs serial URL construction and CLI parsing in one place. |

### Crate-root re-exports (prefer these in external code)

`arm`, `disarm`, `force_arm`, `land`, `rtl`, `set_mode_auto`, `set_mode_guided`, `takeoff`, `takeoff_alt`, `goto_global`, `goto_global_command_int`, `mission_set_current`, `mission_start`, `VehicleIds`, `DEFAULT_TAKEOFF_ALTITUDE_M`, `CUSTOM_MODE_GUIDED`, `CUSTOM_MODE_AUTO`, `MissionStore`, `StoredMissionItem`.

---

## 3. Binaries

### `tui` — `src/bin/tui/main.rs` (+ submodules)

**Declared in `Cargo.toml`** as `[[bin]] name = "tui" path = "src/bin/tui/main.rs"`. `default-run = "tui"`.

**`main.rs`** (orchestration only):

- Parse args via `mavlink_connect::resolve_from_args`.
- `mavlink::connect`, `tune_connection`, wrap in `Arc<Mutex<Conn>>`.
- Channels: frames to UI, string log lines from recv thread, stream-retry signal.
- Shared `Arc`s: `MissionStore`, `OverrideState`, optional vehicle ids for watchdog, `NetWatchdogStatus`.
- Spawn **`recv::spawn_recv_thread`**, **`watchdog::spawn_net_watchdog`**, then **`ui_loop::run_ui`**.

| Submodule | Role |
|-----------|------|
| **`consts.rs`** | `TARGET_*`, MAVLink stream message IDs, pending-command timeout, stream auto-retry timing/counts, geofence/RTL thresholds as used by TUI. |
| **`state.rs`** | `TelemetryState` (all displayed fields + `recent_messages` + `pending_feedback`), `PendingFeedback`, `TelemetryCoverage` (recv thread: which message types seen), `Waypoint` (display copy), `OverrideState` (mission running / paused / active override / resuming), `NetWatchdogStatus`, `vehicle_ids_from_state`. |
| **`format.rs`** | Pure formatters: MAV enums short strings, mode names, GPS fix text, `waypoint_line`, etc. No I/O. |
| **`mavlink_cmd.rs`** | TUI-local `COMMAND_LONG` builders (arm, disarm, DO_SET_MODE, takeoff, mission start) and **`refresh_mavlink_streams`** / `request_stream_rates` / `heartbeat_from_autopilot`. Complements `drone_server::cmd` where the TUI uses different entry patterns. |
| **`telemetry.rs`** | **`apply_message`**: fold `MavFrame` into `TelemetryState` (and push human-readable lines). **`log_outgoing` / `log_outgoing_two`**: record `[1] TUI → link` and set `pending_feedback`. **`check_pending_feedback_timeout`**: user-visible hint if FC never ACKs. Order-sensitive: UI loop processes keys before draining frames so `[1]` appears before `[2]` in the same tick when possible. |
| **`geo.rs`** | `parse_waypoint_input` (lat lon alt or alt-only with current position), `horizontal_distance_m` for override “reached waypoint” logic. |
| **`render.rs`** | **`draw_ui`**: ratatui layout, all panels, help popup. Depends on `state` + `format` only at the data layer. |
| **`recv.rs`** | Blocking **`recv_frame`** loop: manual/auto stream refresh, first-heartshake detection, mission download (`MISSION_COUNT` / `MISSION_ITEM_INT` / `MISSION_CURRENT`), upload handshake (`MISSION_REQUEST_INT` → `MISSION_ITEM_INT`), **`MISSION_ACK`** path for resume-after-override, **GLOBAL_POSITION_INT** progression for `OverrideState::OverrideActive`. Forwards every frame to the UI channel. |
| **`watchdog.rs`** | TCP connect probes to well-known IPs; updates `NetWatchdogStatus`; after sustained offline, may send RTL using `drone_server::rtl` when vehicle ids are known. |
| **`ui_loop.rs`** | Terminal setup, **`'ui` / `'keys`** labeled loops, waypoint input mode, all keybindings, drain `rx`/`log_rx`, timeout check, net status mirror, **`terminal.draw`**. |

**Dependency direction (avoid cycles):** shared structs stay in **`state`**. `ui_loop` pulls in telemetry, render, geo, mavlink_cmd, consts. `recv` pulls consts, geo, mavlink_cmd, state. `watchdog` pulls consts, state. `format` stays low-level.

**Gotchas for edits:**

- Preserve **key order vs MAVLink drain** in `ui_loop` if you care about `[1]` before `[2]` ordering in the log.
- **Override / resume** touches `MissionStore`, `OverrideState`, and `recv.rs` MISSION_* handling together—test mentally end-to-end when changing any one piece.
- Stream retry: **auto** loop spawned from recv after handshake; **`s`** sends on a channel consumed in recv the same way.

### `raw` — `src/bin/raw.rs`

- Uses **`drone_server::mavlink_connect`** and **`VehicleIds`** (not a standalone duplicate of connection logic).
- Single thread: read loop, print formatted telemetry. No `MissionStore` or override flow.

---

## 4. Conventions

- **MAVLink crate:** `mavlink` **0.17**, feature **ardupilotmega**, **udp**, **direct-serial**.
- **Working directory:** run `cargo` from **`drone-server/`** unless the workspace root passes `-p drone-server`.
- **Edition:** 2021.
- **TUI crate attribute:** `#![allow(deprecated)]` on the binary root where generated MAVLink types still carry deprecation warnings—do not remove without a coordinated mavlink upgrade.

---

## 5. How to add features (guided)

| Goal | Suggested approach |
|------|---------------------|
| New MAVLink command for library users | Implement in **`cmd.rs`**, export from **`lib.rs`**, add a small test or doc example if non-trivial. |
| New TUI key or command | Add handling in **`ui_loop.rs`**; use **`mavlink_cmd`** or **`drone_server::cmd`** consistently; use **`telemetry::log_outgoing`** so ACK correlation stays correct. |
| Waypoint / mission from Gateway | Prefer **`goto_global_command_int`** or mission protocol helpers; **`mission_store`** patterns in recv show how FC-driven upload works. See **ROADMAP.md** step 2. |
| HTTP/WebSocket API | New binary or module in this crate (or separate crate with path dep) that calls **`drone_server`**; keep **`tui`**/**`raw`** behavior stable unless intentionally changing UX. |

---

## 6. Dependencies (summary)

- **mavlink**, **ratatui**, **crossterm** (TUI), **serde**, **serde_json**.
- No direct dependency on **gateway** or **frontend** crates.

For end-to-end system behavior, always cross-check **Code/README.md**.
