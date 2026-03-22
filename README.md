# Drone Server

**Drone Server** is the MAVLink telemetry and control layer for the SAR (Search and Rescue) drone. It talks to the flight controller over USB/serial (MAVLink), provides a TUI for in-field use, and exposes a Rust library that the **Gateway** (or future API) will call for drone commands (waypoints, RTL, arm, modes, etc.).

## Role in the project

- In the [project architecture](../README.md), the **Drone Server** is the component that receives structured commands (from the Gateway/LLM or from the frontend) and translates them into MAVLink for the flight controller.
- This crate is that server: today it is the **TUI + library**; later an HTTP/WebSocket API can be added so the Gateway calls it (e.g. “RTL”, “go to [lat, lon]”, “upload waypoints”) instead of (or in addition to) the TUI.
- Control roadmap (remote control → waypoint injection → autonomous path → override → resume → LLM): see [ROADMAP.md](ROADMAP.md).

## What’s in this crate

- **Library (`drone_server`)**: MAVLink command helpers (`cmd`), mission types (`mission`), re-exports (arm, disarm, force_arm, land, rtl, set_mode_*, takeoff, goto_global, VehicleIds, etc.). Use this from any Rust binary or service that needs to send MAVLink to the FC.
- **TUI (`tui`)**: Interactive terminal UI: telemetry panels (vehicle, attitude, GPS, battery, HUD), mission waypoints, messages log, keybindings for arm, disarm, GUIDED, AUTO, RTL, land, takeoff, force arm, mission start. Press `h` for help.
- **Raw (`raw`)**: Console-only telemetry (no TUI, no threads); useful for debugging or headless logs.

## Build and run

From this folder:

```bash
cargo run          # TUI (default)
cargo run --bin tui
cargo run --bin raw
```

Use `--` before flags so they are passed to the binary (not to Cargo).

- **Default:** UDP **`udpin:0.0.0.0:14550`** (MAVProxy / GCS forwarding to the Jetson).
- **USB to Pixhawk on the Jetson:** `cargo run -- --serial` (optional device path and `--baud`; defaults **`/dev/ttyACM0`**, **115200**). Over SSH, run `ls /dev/ttyACM* /dev/ttyUSB*` or `ls /dev/serial/by-id` to see the actual node. Add your user to **`dialout`** if opening the port is denied.

## Project structure

```
drone-server/
├── src/
│   ├── lib.rs              # Library root: cmd, mission, mavlink_connect, re-exports
│   ├── mavlink_connect.rs  # CLI/env MAVLink URL (UDP vs serial) for binaries
│   ├── cmd.rs              # MAVLink command builders (arm, rtl, land, set_mode_*, etc.)
│   ├── mission.rs          # Mission types (Waypoint, Mission, WaypointCommand)
│   ├── main.rs             # (removed; default is tui)
│   └── bin/
│       ├── tui.rs          # TUI entrypoint
│       └── raw.rs          # Raw console entrypoint
├── Cargo.toml
├── ROADMAP.md       # Control roadmap (steps 1–6)
├── README.md
└── AGENTS.md        # For agents: entry points, conventions, project context
```

## For agents and integration

- **AGENTS.md** in this folder describes entry points, key files, and how this crate fits the rest of the Code project.
- External callers (e.g. Gateway, frontend backend) will eventually talk to the Drone Server via an API (e.g. REST or WebSocket) that uses this library to send MAVLink; that API is not implemented yet.
