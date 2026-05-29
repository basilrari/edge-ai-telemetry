# Drone Server

**Drone Server** is the MAVLink telemetry and control layer for the SAR (Search and Rescue) drone stack. It speaks to the flight controller over MAVLink (UDP or USB serial), ships as a **Rust library** for reuse by future services, and includes **field binaries**: an interactive TUI and a minimal console dumper.

## Role in the larger project

- In the [workspace architecture](../README.md), **Drone Server** sits between higher-level software (Gateway, LLM, frontend) and the **flight controller**. It turns structured intent—arm, modes, RTL, waypoints, mission control—into MAVLink frames the FC understands.
- **Today:** library + **`drone-http`** + `tui` + `raw`. The Gateway and frontend use **`drone-http`** on port 3001 (proxied at gateway `:3000`).
- Longer-term control story is tracked outside this repo (see git history / issues).

---

## What this crate provides

### Library: `drone_server`

| Area | Purpose |
|------|---------|
| **`cmd`** | Build and send common ArduPilotMega commands: arm/disarm/force_arm, land, RTL, guided/auto modes, takeoff, global goto (`COMMAND_INT` / helpers), mission set-current and mission start. |
| **`mission`** | Small, serde-friendly mission types (`Waypoint`, `Mission`, `WaypointCommand`) for describing missions in JSON or internal APIs—not a full mission-protocol implementation by itself. |
| **`mission_store`** | Runtime state for the TUI: mission items downloaded from the FC, current sequence, snapshot for interrupt/override, and pending upload items when resuming a mission after override. |
| **`mavlink_connect`** | Shared CLI parsing for binaries: default UDP listen URL, `--serial` with optional device and baud, `tune_connection` (e.g. accept MAVLink v1+v2), help text, and friendly open-error messages for serial permission/busy/device issues. |

**Re-exported at the crate root** (see `src/lib.rs`): `arm`, `disarm`, `force_arm`, `land`, `rtl`, `set_mode_auto`, `set_mode_guided`, `takeoff`, `takeoff_alt`, `goto_global`, `goto_global_command_int`, `mission_set_current`, `mission_start`, `VehicleIds`, altitude/mode constants, plus `MissionStore`, `StoredMissionItem`.

### Binary: `tui` (default)

Terminal UI built with **ratatui** + **crossterm**:

- **Panels:** vehicle summary, attitude, GPS, battery, HUD (airspeed, heading, climb, etc.), mission waypoint list, scrolling message log.
- **Threads:** (1) dedicated receive loop: reads MAVLink, updates `MissionStore`, handles stream-rate refresh and automatic retries until key telemetry types are seen, forwards frames to the UI; (2) optional net watchdog (internet reachability probe, failsafe RTL after sustained offline—see TUI code); (3) main UI thread runs the event loop and draws.
- **Command feedback:** Recent messages distinguish TUI→link send results (`[1]`) from FC replies such as `COMMAND_ACK` (`[2]`), with pending-command timeout hints if the FC stays silent.

**Default keybindings** (press **`h`** in-app for the full help overlay):

| Key | Action |
|-----|--------|
| `q` | Quit |
| `h` | Help popup |
| `a` / `d` | Arm / Disarm |
| `f` | Force arm |
| `g` / `u` | GUIDED / AUTO (DO_SET_MODE) |
| `m` | AUTO + mission start |
| `t` | Takeoff (fixed altitude in code) |
| `r` / `l` | RTL / Land |
| `s` | Manual stream / mission-list retry (same path as periodic auto-retry) |
| `i` | Interrupt: snapshot mission, GUIDED hover (needs AUTO mission + GPS + home) |
| `w` | Waypoint entry mode (lat lon alt or alt-only); finish override first if active |
| `c` | Cancel override / resume mission from snapshot (mission upload handshake) |

Waypoint entry uses **Enter** to submit, **Esc** to cancel, normal typing for coordinates.

### Binary: `raw`

Single-threaded loop: opens the same MAVLink URL as the TUI (via `mavlink_connect`), prints decoded telemetry lines to **stdout**. No panels, no mission override UI. Uses `drone_server` for connection helpers and `VehicleIds`; message decoding is local to the file.

---

## Build and run

Run all commands from **`drone-server/`** (the crate root). If you use the parent workspace, `cargo` can still be invoked from `Code/` with `-p drone-server`.

```bash
cargo build
cargo run              # same as: cargo run --bin tui  (see default-run in Cargo.toml)
cargo run --bin tui
cargo run --bin raw
```

Pass binary flags **after** `--` so Cargo does not swallow them:

```bash
cargo run --bin tui -- --help
cargo run --bin tui -- --serial
cargo run --bin tui -- --serial /dev/ttyACM0 --baud 921600
```

### Connection defaults

- **UDP:** listens as **`udpin:0.0.0.0:14550`** — typical when MAVProxy or a GCS forwards MAVLink to the Jetson.
- **Serial:** `cargo run -- --serial` uses **`/dev/ttyACM0`** at **115200** unless you override device/baud. On the Jetson over SSH, confirm nodes with `ls /dev/ttyACM* /dev/ttyUSB*` or `/dev/serial/by-id`. Add your user to **`dialout`** if you get permission denied.

---

## Integration

- **`drone-http`** (binary `drone-http`, default port **3001**) is the HTTP/WebSocket API used by the **Gateway** and **frontend** (via gateway proxies). It wraps the same MAVLink runtime as the TUI.
- **`tui`** and **`raw`** remain for field debugging and override/resume workflows not exposed in the web UI.
- For system-wide context (frontend, gateway, model server), see **Code/README.md**.

### `drone-http` endpoints (summary)

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/health` | Liveness |
| GET | `/v1/telemetry` | Latest HUD fields |
| GET | `/v1/position` | Lat/lon for map |
| GET | `/v1/mission` | FC mission waypoints |
| POST | `/v1/mission/upload` | Upload mission |
| POST | `/v1/mission/clear` | Clear FC mission |
| POST | `/v1/apply-tool` | Execute drone tool |
| GET | `/v1/logs` | Flight event log |
| GET | `/v1/logs/mavlink` | MAVLink log snapshot |
| WS | `/v1/ws/telemetry` | Live telemetry |
| WS | `/v1/ws/logs` | Live flight + MAVLink logs |

---

## Repository layout

```
drone-server/
├── Cargo.toml              # default-run = "tui"; bins: tui, drone-http, raw
├── README.md               # This file
├── AGENTS.md               # Maintainer/agent-oriented map of the crate
└── src/
    ├── lib.rs              # Crate root: modules + public re-exports
    ├── cmd.rs              # MAVLink command builders and send helpers
    ├── mission.rs          # Serde mission types
    ├── mission_store.rs    # FC mission mirror + override snapshot/upload state
    ├── mavlink_connect.rs  # URL resolution, CLI help, connection tuning
    ├── flight_log.rs       # Flight event log entries
    ├── mavlink_log.rs      # MAVLink row formatting for dashboard
    ├── logs_hub.rs         # Combined log buffers + WS broadcast
    ├── telemetry_hub.rs    # Telemetry snapshot for HTTP/WS
    ├── mavlink_http_runtime.rs
    ├── http_mission_tools.rs
    ├── mission_upload.rs
    ├── tool_dispatch.rs
    ├── mavlink_streams.rs
    ├── geo.rs
    └── bin/
        ├── drone_http.rs   # HTTP/WS API (production)
        ├── raw.rs          # Console telemetry binary
        └── tui/            # TUI binary
```

---

## Older docs

Planning notes (`ROADMAP.md`, `OVERRIDE_RESUME_BRAINSTORM.md`) are not in this tree; see git history or issue tracker if needed.
