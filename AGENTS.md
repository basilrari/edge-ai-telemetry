# Drone Server — For Agents

This file gives **entry points**, **conventions**, and **project context** so agents can work on the Drone Server without full repo context.

---

## Role in the project

- **Code** is the SAR (Search and Rescue) drone repo. See [Code/README.md](../README.md) for the full architecture.
- **Drone Server** is the component that **talks MAVLink to the flight controller**. It receives high-level commands (RTL, waypoints, arm, modes) and turns them into MAVLink. In the diagram: **Gateway** (and later LLM/frontend) → **Drone Server** → MAVLink → FC.
- This crate is the Drone Server: a **library** (`drone_server`) plus two **binaries** (TUI and raw). A future HTTP/WebSocket API can wrap the same library for the Gateway to call.

---

## Entry points and layout

| Path | Purpose |
|------|---------|
| **src/lib.rs** | Library root. Declares `pub mod mission`, `pub mod cmd`; re-exports from `cmd`: `arm`, `disarm`, `force_arm`, `land`, `rtl`, `set_mode_auto`, `set_mode_guided`, `takeoff`, `takeoff_alt`, `goto_global`, `VehicleIds`, `DEFAULT_TAKEOFF_ALTITUDE_M`, `CUSTOM_MODE_GUIDED`, `CUSTOM_MODE_AUTO`. |
| **src/cmd.rs** | MAVLink command builders (ArduPilotMega). Functions that return `MavMessage` or send via `MavConnection`: arm, disarm, force_arm, land, rtl, set_mode_guided, set_mode_auto, takeoff, takeoff_alt, goto_global; struct `VehicleIds`. |
| **src/mission.rs** | Protocol-agnostic mission types: `WaypointCommand`, `Waypoint`, `Mission` (serde). Used for mission representation; MAVLink mission protocol (upload/download) is not fully wired here yet. |
| **src/bin/tui.rs** | TUI binary. Connects to MAVLink (UDP by default), runs a receive thread and draws ratatui panels (vehicle, attitude, GPS, battery, HUD, mission waypoints, messages). Keybindings: arm (a), disarm (d), GUIDED (g), AUTO (u), RTL (r), land (l), takeoff (t), force arm (f), mission start (m), help (h). Uses `drone_server::{force_arm, land, rtl, VehicleIds}`; other commands built locally with mavlink types. |
| **src/bin/raw.rs** | Raw binary. Same UDP connection, no TUI; prints telemetry to stdout. No dependency on `drone_server` lib (standalone). |

- **Endpoints**: This crate does **not** expose HTTP endpoints. It is a library + TUI/raw binaries. The **Gateway** (in `gateway/`) exposes `/status` and `/infer`; when drone tools are applied, the Gateway will eventually call the Drone Server (e.g. via a local API or process); that integration is not yet implemented.

---

## Conventions

- **MAVLink**: Uses `mavlink` crate (ArduPilotMega). Connection is currently UDP (`udpin:0.0.0.0:14550`) in both binaries; serial/USB can be added by changing the connection string.
- **Run from this folder**: All `cargo run` / `cargo build` should be run from `drone-server/`. Workspace root is `Code/` with `members = ["drone-server"]`.
- **Control roadmap**: [ROADMAP.md](ROADMAP.md) lists steps 1–6 (remote control → waypoint injection → autonomous path → override → resume → LLM). Step 2 (inject waypoints remotely) is next.

---

## Adding features (for agents)

- **Waypoint injection**: Add TUI key(s) or API that calls MAVLink `MISSION_WRITE_PARTIAL_LIST` or guided `goto_global`; use current position from telemetry (GLOBAL_POSITION_INT) or from parameters. See ROADMAP step 2.
- **Predefined paths**: Generate waypoints (e.g. square, figure-8) from home/current position and upload via mission protocol; can be triggered from TUI or later from Gateway/frontend.
- **API for Gateway**: Add a binary or module that runs an HTTP/WebSocket server and translates JSON (e.g. `{"command": "rtl"}` or `{"waypoints": [...]}`) into `drone_server` and mavlink calls. Keep TUI and raw binaries unchanged.

---

## Dependencies

- **mavlink** (ardupilotmega, direct-serial, udp), **ratatui**, **crossterm** (TUI), **serde**, **serde_json** (mission types). No dependency on `gateway` or `frontend`; the Gateway will depend on or call Drone Server once the API exists.

For the overall system (frontend, gateway, model server, drone server), always refer to **Code/README.md**.
