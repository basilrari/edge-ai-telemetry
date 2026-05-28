//! MAVLink connection URL for binaries: default UDP listen or `--serial` to the FC.
//! When no CLI link args are given, [`open_connection`] tries MAVProxy/UDP then USB serial.

use crate::tool_dispatch::wait_autopilot_heartbeat;
use mavlink::ardupilotmega::MavMessage;
use mavlink::{connect, Connection, MavConnection};
use std::io;
use std::path::Path;
use std::time::Duration;

/// Accept both MAVLink v1 and v2 (some USB/serial links still emit v1).
pub fn tune_connection<M: mavlink::Message + Send + Sync>(conn: &mut impl MavConnection<M>) {
    conn.set_allow_recv_any_version(true);
}

/// Default: MAVProxy / GCS forwards to this UDP port.
pub const DEFAULT_UDP_URL: &str = "udpin:0.0.0.0:14550";

/// Pixhawk-class FC on Jetson USB is usually `ttyACM0` (confirm with `ls /dev/ttyACM*` over SSH).
pub const DEFAULT_SERIAL_DEVICE: &str = "/dev/ttyACM0";

/// Match ArduPilot `SERIAL*_BAUD` for that port if the link fails.
pub const DEFAULT_SERIAL_BAUD: u32 = 115200;

#[derive(Debug, PartialEq, Eq)]
pub enum MavlinkArgsError {
    Help,
    Invalid(String),
}

/// How the vehicle link was opened (for HTTP / UI).
#[derive(Clone, Debug, serde::Serialize)]
pub struct LinkInfo {
    /// `udp_mavproxy` or `serial_usb`
    pub kind: String,
    pub display: String,
    pub url: String,
}

const AUTO_HEARTBEAT_TIMEOUT: Duration = Duration::from_secs(4);

pub fn default_udp_display() -> &'static str {
    "udp:0.0.0.0:14550 (udpin)"
}

pub fn serial_url(device: &str, baud: u32) -> String {
    format!("serial:{device}:{baud}")
}

fn serial_device_from_url(url: &str) -> Option<&str> {
    let rest = url.strip_prefix("serial:")?;
    let (device, _baud) = rest.rsplit_once(':')?;
    Some(device)
}

/// Human-readable startup diagnostics for connection-open failures.
pub fn open_error_message(mavlink_url: &str, err: &io::Error) -> String {
    let mut lines = vec![format!(
        "Failed to open MAVLink connection: {err}\nLink: {mavlink_url}"
    )];
    if let Some(code) = err.raw_os_error() {
        lines.push(format!("OS error code: {code}"));
    }

    if mavlink_url.starts_with("serial:") {
        let dev = serial_device_from_url(mavlink_url).unwrap_or("/dev/ttyACM0");
        lines.push(format!("Serial device: {dev}"));
        match err.kind() {
            io::ErrorKind::PermissionDenied => {
                lines.push("Cause: permission denied to serial device.".to_string());
                lines.push("Checks: whoami; id; ls -l /dev/ttyACM* /dev/ttyUSB*".to_string());
                lines.push("Expected: user in 'dialout' and device group rw for dialout.".to_string());
                lines.push("If GUI/app terminal is sandboxed, run from a normal terminal or grant raw-usb/serial-port.".to_string());
            }
            _ => {}
        }
        match err.raw_os_error() {
            Some(6) => {
                lines.push("Cause: no such device/address (ENXIO).".to_string());
                lines.push("Check cable/power, verify device exists: ls -l /dev/ttyACM* /dev/ttyUSB*".to_string());
                lines.push("If this only fails in one terminal app, it is likely sandbox/device access.".to_string());
            }
            Some(16) => {
                lines.push("Cause: serial device busy (EBUSY).".to_string());
                lines.push(format!("Check holder: sudo lsof {dev}"));
            }
            Some(2) => {
                lines.push("Cause: serial device not found (ENOENT).".to_string());
                lines.push("Use --serial /dev/ttyACM0 (or actual device) and verify with ls.".to_string());
            }
            Some(5) => {
                lines.push("Cause: I/O error (EIO) on serial link.".to_string());
                lines.push("Check USB cable quality, FC power, and baud rate (--baud).".to_string());
            }
            _ => {}
        }
    } else if mavlink_url.starts_with("udpin:") {
        lines.push("Tip: no telemetry on UDP usually means nothing is forwarding MAVLink to this port.".to_string());
        lines.push("Use --serial for direct FC USB, or forward to UDP 14550 via mavproxy/mavlink-router.".to_string());
    }

    lines.join("\n")
}

pub fn usage_string() -> &'static str {
    "\
  --serial [DEVICE]   USB serial to FC (default device /dev/ttyACM0)
  --baud <RATE>       With --serial only (default 115200)

  Default: UDP listen udpin:0.0.0.0:14550

If GPS/battery/HUD stay at zero, you are probably not on the FC link: use
--serial when the Pixhawk is on USB (e.g. Jetson /dev/ttyACM0), or forward
MAVLink to UDP 14550 from MAVProxy / mavlink-router.

Examples:
  cargo run
  cargo run -- --serial
  cargo run -- --serial /dev/ttyUSB0 --baud 57600
"
}

/// Parse `std::env::args().skip(1)`: `--serial` → serial URL, else default UDP.
pub fn resolve_from_args(
    args: impl IntoIterator<Item = String>,
) -> Result<(String, String), MavlinkArgsError> {
    let mut args = args.into_iter().peekable();
    let mut use_serial = false;
    let mut serial_device: Option<String> = None;
    let mut baud: Option<u32> = None;

    while let Some(a) = args.next() {
        match a.as_str() {
            "-h" | "--help" => return Err(MavlinkArgsError::Help),
            "--serial" => {
                use_serial = true;
                if let Some(next) = args.peek() {
                    if !next.starts_with('-') {
                        serial_device = Some(args.next().unwrap());
                    }
                }
            }
            "--baud" => {
                let s = args
                    .next()
                    .ok_or_else(|| MavlinkArgsError::Invalid("--baud requires a number".into()))?;
                let n: u32 = s
                    .parse()
                    .map_err(|_| MavlinkArgsError::Invalid(format!("invalid baud: {s}")))?;
                baud = Some(n);
            }
            other => {
                return Err(MavlinkArgsError::Invalid(format!(
                    "unknown argument: {other}\n\n{}",
                    usage_string()
                )));
            }
        }
    }

    if baud.is_some() && !use_serial {
        return Err(MavlinkArgsError::Invalid(
            "--baud is only valid with --serial\n\n".to_string() + usage_string(),
        ));
    }

    if use_serial {
        let dev = serial_device
            .as_deref()
            .unwrap_or(DEFAULT_SERIAL_DEVICE);
        let b = baud.unwrap_or(DEFAULT_SERIAL_BAUD);
        let u = serial_url(dev, b);
        let display = format!("{u} (USB serial)");
        return Ok((u, display));
    }

    Ok((
        DEFAULT_UDP_URL.to_string(),
        default_udp_display().to_string(),
    ))
}

fn serial_device_candidates() -> Vec<String> {
    let mut out = Vec::new();
    let mut push = |p: &str| {
        if Path::new(p).exists() && !out.iter().any(|x| x == p) {
            out.push(p.to_string());
        }
    };
    push(DEFAULT_SERIAL_DEVICE);
    if let Ok(entries) = std::fs::read_dir("/dev/serial/by-id") {
        for ent in entries.flatten() {
            if let Ok(target) = std::fs::read_link(ent.path()) {
                if let Some(s) = target.to_str() {
                    let dev = if s.starts_with("../") {
                        format!("/dev/{}", s.trim_start_matches("../"))
                    } else if s.starts_with('/') {
                        s.to_string()
                    } else {
                        continue;
                    };
                    push(&dev);
                }
            }
        }
    }
    if let Ok(entries) = std::fs::read_dir("/dev") {
        for ent in entries.flatten() {
            let name = ent.file_name();
            let name = name.to_string_lossy();
            if name.starts_with("ttyACM") || name.starts_with("ttyUSB") {
                push(&format!("/dev/{name}"));
            }
        }
    }
    out
}

fn try_open_url(
    url: &str,
    display: &str,
    kind: &str,
    timeout: Duration,
) -> Result<(Connection<MavMessage>, LinkInfo), ()> {
    let mut conn = connect::<MavMessage>(url).map_err(|_| ())?;
    tune_connection(&mut conn);
    wait_autopilot_heartbeat(&mut conn, timeout).map_err(|_| ())?;
    Ok((
        conn,
        LinkInfo {
            kind: kind.to_string(),
            display: display.to_string(),
            url: url.to_string(),
        },
    ))
}

/// Open MAVLink: explicit CLI args, or auto-detect UDP (MAVProxy) then serial USB.
pub fn open_connection(
    args: Vec<String>,
) -> Result<(Connection<MavMessage>, LinkInfo), String> {
    if !args.is_empty() {
        let (url, display) = resolve_from_args(args).map_err(|e| match e {
            MavlinkArgsError::Help => "help requested".to_string(),
            MavlinkArgsError::Invalid(m) => m,
        })?;
        let kind = if url.starts_with("serial:") {
            "serial_usb"
        } else {
            "udp_mavproxy"
        };
        let mut conn = connect::<MavMessage>(&url)
            .map_err(|e| open_error_message(&url, &e))?;
        tune_connection(&mut conn);
        wait_autopilot_heartbeat(&mut conn, Duration::from_secs(60))
            .map_err(|e| format!("{e}\nLink: {url}"))?;
        return Ok((
            conn,
            LinkInfo {
                kind: kind.to_string(),
                display,
                url,
            },
        ));
    }

    if let Ok(pair) = try_open_url(
        DEFAULT_UDP_URL,
        "MAVProxy / UDP (udpin:0.0.0.0:14550)",
        "udp_mavproxy",
        AUTO_HEARTBEAT_TIMEOUT,
    ) {
        return Ok(pair);
    }

    for dev in serial_device_candidates() {
        let url = serial_url(&dev, DEFAULT_SERIAL_BAUD);
        let display = format!("{url} (USB serial)");
        if let Ok(pair) = try_open_url(&url, &display, "serial_usb", AUTO_HEARTBEAT_TIMEOUT) {
            return Ok(pair);
        }
    }

    Err(
        "No MAVLink link: UDP 14550 had no autopilot heartbeat and no serial FC responded.\n\
         Start MAVProxy forwarding to 14550, or connect Pixhawk USB (/dev/ttyACM0)."
            .to_string(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_udp() {
        let (u, _) = resolve_from_args(std::iter::empty()).unwrap();
        assert_eq!(u, DEFAULT_UDP_URL);
    }

    #[test]
    fn serial_default_device() {
        let (u, _) = resolve_from_args(["--serial".to_string()].into_iter()).unwrap();
        assert_eq!(u, serial_url(DEFAULT_SERIAL_DEVICE, DEFAULT_SERIAL_BAUD));
    }

    #[test]
    fn baud_without_serial_errors() {
        assert!(matches!(
            resolve_from_args(["--baud".to_string(), "57600".to_string()].into_iter()),
            Err(MavlinkArgsError::Invalid(_))
        ));
    }
}
