//! Choose MAVLink connection URL for binaries (UDP / MAVProxy vs direct USB serial).

/// Default: listen for MAVLink UDP (e.g. from MAVProxy or a ground station forwarding to this port).
pub const DEFAULT_UDP_URL: &str = "udpin:0.0.0.0:14550";

/// Typical Pixhawk USB gadget on Linux (Jetson).
pub const DEFAULT_SERIAL_DEVICE: &str = "/dev/ttyACM0";

/// Common baud for ArduPilot serial; match `SERIAL*_BAUD` on the FC if this fails.
pub const DEFAULT_SERIAL_BAUD: u32 = 115200;

#[derive(Debug, PartialEq, Eq)]
pub enum MavlinkArgsError {
    /// Print help and exit successfully.
    Help,
    Invalid(String),
}

/// Human-readable label for the default UDP bind (matches previous log messages).
pub fn default_udp_display() -> &'static str {
    "udp:0.0.0.0:14550 (udpin)"
}

/// Build a `serial:` URL understood by `mavlink::connect` (`serial:<port>:<baud>`).
pub fn serial_url(device: &str, baud: u32) -> String {
    format!("serial:{device}:{baud}")
}

pub fn usage_string() -> &'static str {
    "\
MAVLink connection options (first applicable):

  --mavlink-url <URL>     Full address, e.g. udpin:0.0.0.0:14550 or serial:/dev/ttyACM0:115200
  --serial [DEVICE]       Direct FC USB serial (default device /dev/ttyACM0)
  --baud <RATE>           With --serial only (default 115200)
  --udp                   Default UDP listen (ignores MAVLINK_URL)
  MAVLINK_URL             Environment variable when no --serial / --udp / --mavlink-url

  Default: udpin:0.0.0.0:14550

Examples:
  drone-server-tui
  drone-server-tui --serial
  drone-server-tui --serial /dev/ttyUSB0 --baud 57600
  MAVLINK_URL=serial:/dev/ttyACM0:57600 drone-server-tui
  drone-server-tui --mavlink-url udpin:0.0.0.0:14550
"
}

/// Parse arguments after the program name (`std::env::args().skip(1)`).
///
/// Precedence: `--mavlink-url` &gt; `--serial` (+ `--baud`) &gt; `--udp` (default bind) &gt;
/// `MAVLINK_URL` env &gt; default UDP.
pub fn resolve_from_args(
    args: impl IntoIterator<Item = String>,
) -> Result<(String, String), MavlinkArgsError> {
    let mut args = args.into_iter().peekable();
    let mut url: Option<String> = None;
    let mut use_serial = false;
    let mut serial_device: Option<String> = None;
    let mut baud: Option<u32> = None;
    let mut force_udp = false;

    while let Some(a) = args.next() {
        match a.as_str() {
            "-h" | "--help" => return Err(MavlinkArgsError::Help),
            "--mavlink-url" => {
                let u = args.next().ok_or_else(|| {
                    MavlinkArgsError::Invalid(
                        "--mavlink-url requires a value (e.g. serial:/dev/ttyACM0:115200)".into(),
                    )
                })?;
                url = Some(u);
            }
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
            "--udp" => {
                force_udp = true;
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

    if url.is_some() && (use_serial || force_udp || baud.is_some()) {
        return Err(MavlinkArgsError::Invalid(
            "do not combine --mavlink-url with --serial, --baud, or --udp\n\n".to_string()
                + usage_string(),
        ));
    }

    if force_udp && use_serial {
        return Err(MavlinkArgsError::Invalid(
            "cannot use both --udp and --serial\n\n".to_string() + usage_string(),
        ));
    }

    if let Some(u) = url {
        let display = format!("{u} (--mavlink-url)");
        return Ok((u, display));
    }

    if use_serial {
        let dev = serial_device
            .as_deref()
            .unwrap_or(DEFAULT_SERIAL_DEVICE);
        let b = baud.unwrap_or(DEFAULT_SERIAL_BAUD);
        let u = serial_url(dev, b);
        let display = format!("{u} (direct serial)");
        return Ok((u, display));
    }

    if force_udp {
        return Ok((
            DEFAULT_UDP_URL.to_string(),
            default_udp_display().to_string(),
        ));
    }

    if let Ok(env_url) = std::env::var("MAVLINK_URL") {
        if !env_url.is_empty() {
            let display = format!("{env_url} (MAVLINK_URL)");
            return Ok((env_url, display));
        }
    }

    Ok((
        DEFAULT_UDP_URL.to_string(),
        default_udp_display().to_string(),
    ))
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

    #[test]
    fn mavlink_url_wins_over_env() {
        std::env::set_var("MAVLINK_URL", "serial:/dev/foo:1");
        let (u, _) = resolve_from_args(
            ["--mavlink-url".to_string(), "udpin:1.2.3.4:7".to_string()].into_iter(),
        )
        .unwrap();
        assert_eq!(u, "udpin:1.2.3.4:7");
        std::env::remove_var("MAVLINK_URL");
    }

    #[test]
    fn udp_ignores_env() {
        std::env::set_var("MAVLINK_URL", "serial:/dev/foo:1");
        let (u, _) = resolve_from_args(["--udp".to_string()].into_iter()).unwrap();
        assert_eq!(u, DEFAULT_UDP_URL);
        std::env::remove_var("MAVLINK_URL");
    }
}
