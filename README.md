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

Use `--` before flags so they are passed to the binary (not to Cargo), e.g. `cargo run -- --help`.

## MAVLink connection (UDP vs USB serial)

The **`tui`** and **`raw`** binaries use the same rules to choose how they connect. Resolution order:

1. **`--mavlink-url <URL>`** — full address for `mavlink::connect` (e.g. UDP listen or serial).
2. **`--serial [DEVICE]`** — direct connection to the flight controller over USB serial on the Jetson. Optional **`--baud <RATE>`** (default **115200**). Default device **`/dev/ttyACM0`** if you omit the path.
3. **`--udp`** — force the default UDP listen below and **ignore** `MAVLINK_URL`.
4. **`MAVLINK_URL`** environment variable — used when none of the above apply.
5. **Default** — listen on UDP **`udpin:0.0.0.0:14550`** (typical when MAVProxy or a GCS forwards MAVLink to this port).

### Examples

**UDP / MAVProxy (default)** — no arguments:

```bash
cargo run
cargo run --bin raw
```

**Pixhawk plugged into the Jetson over USB** (often `/dev/ttyACM0`; sometimes `/dev/ttyUSB0`):

```bash
cargo run -- --serial
cargo run -- --serial /dev/ttyACM0 --baud 57600
cargo run --bin raw -- --serial
```

**Explicit URL** (same formats the `mavlink` crate accepts):

```bash
cargo run -- --mavlink-url serial:/dev/ttyACM0:115200
cargo run -- --mavlink-url udpin:0.0.0.0:14550
```

**Environment variable**:

```bash
export MAVLINK_URL=serial:/dev/ttyACM0:57600
cargo run
```

**Help**:

```bash
cargo run -- --help
```

### Finding the device and fixing permissions

- List serial devices: `ls -l /dev/ttyACM* /dev/ttyUSB*`
- After plugging in the FC, check the kernel log: `dmesg | tail`
- If you get permission errors, add your user to the **`dialout`** group (then log out and back in), or adjust udev rules.

Match **`--baud`** (or the baud in `MAVLINK_URL` / `--mavlink-url`) to the baud configured on that serial port on the autopilot (e.g. ArduPilot `SERIAL*_BAUD`).

For remote control over the network while the Jetson talks to the FC over USB, run this crate on the Jetson with **`--serial`**, or use a MAVLink router that exposes UDP and run with the default **`udpin:0.0.0.0:14550`**, plus SSH tunnel or VPN as needed.

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
