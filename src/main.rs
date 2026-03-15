//! MAVLink telemetry reader for Pixhawk over serial.
//! Prints one-line human-readable lines for HEARTBEAT, ATTITUDE, GLOBAL_POSITION_INT, SYS_STATUS, BATTERY_STATUS only.

use mavlink::ardupilotmega::{MavMessage, MavModeFlag, MavState};
use mavlink::{Connectable, MavConnection, SerialConfig};

const SERIAL_PORT: &str = "/dev/ttyACM0";
const BAUD_RATE: u32 = 115200;
const U16_MAX: u16 = 65535;

fn rad_to_deg(rad: f32) -> f32 {
    rad.to_degrees()
}

fn mav_state_short(s: MavState) -> &'static str {
    match s {
        MavState::MAV_STATE_UNINIT => "UNINIT",
        MavState::MAV_STATE_BOOT => "BOOT",
        MavState::MAV_STATE_CALIBRATING => "CALIBRATING",
        MavState::MAV_STATE_STANDBY => "STANDBY",
        MavState::MAV_STATE_ACTIVE => "ACTIVE",
        MavState::MAV_STATE_CRITICAL => "CRITICAL",
        MavState::MAV_STATE_EMERGENCY => "EMERGENCY",
        MavState::MAV_STATE_POWEROFF => "POWEROFF",
        MavState::MAV_STATE_FLIGHT_TERMINATION => "FLIGHT_TERMINATION",
    }
}

fn mav_mode_flags_short(m: MavModeFlag) -> String {
    let s = format!("{:?}", m);
    let s = s
        .trim_start_matches("MavModeFlag(")
        .trim_end_matches(')')
        .to_string();
    let s = s.replace("MAV_MODE_FLAG_", "");
    let s = s.replace("_ENABLED", "");
    let s = s.replace("_INPUT", "");
    let s = s.replace(" | ", "|");
    if s.is_empty() || s == "empty" {
        "NONE".to_string()
    } else {
        s
    }
}

fn main() {
    let config = SerialConfig::new(SERIAL_PORT.to_string(), BAUD_RATE);

    let connection = match config.connect::<MavMessage>() {
        Ok(conn) => {
            eprintln!("Connected to {} at {} baud (Ctrl+C to stop).\n", SERIAL_PORT, BAUD_RATE);
            conn
        }
        Err(e) => {
            eprintln!("Failed to open serial port '{}': {}", SERIAL_PORT, e);
            eprintln!("Ensure the Pixhawk is connected over USB and you have permission to access the port (e.g. add yourself to dialout group).");
            std::process::exit(1);
        }
    };

    loop {
        match connection.recv_frame() {
            Ok(frame) => {
                let msg = &frame.msg;
                match msg {
                    MavMessage::HEARTBEAT(d) => {
                        println!(
                            "HB status={} mode_flags={} custom={}",
                            mav_state_short(d.system_status),
                            mav_mode_flags_short(d.base_mode),
                            d.custom_mode
                        );
                    }
                    MavMessage::ATTITUDE(d) => {
                        println!(
                            "ATT roll={:.1}deg pitch={:.1}deg yaw={:.1}deg",
                            rad_to_deg(d.roll),
                            rad_to_deg(d.pitch),
                            rad_to_deg(d.yaw)
                        );
                    }
                    MavMessage::GLOBAL_POSITION_INT(d) => {
                        let lat_deg = d.lat as f64 / 1e7;
                        let lon_deg = d.lon as f64 / 1e7;
                        let alt_m = d.alt as f64 / 1000.0;
                        println!("POS lat={:.6} lon={:.6} alt={:.1}m", lat_deg, lon_deg, alt_m);
                    }
                    MavMessage::SYS_STATUS(d) => {
                        let vbat_v = d.voltage_battery as f32 / 100.0;
                        let batt_pct = if d.battery_remaining < 0 {
                            "?".to_string()
                        } else {
                            format!("{}%", d.battery_remaining)
                        };
                        println!("SYS vbat={:.2}V batt={}", vbat_v, batt_pct);
                    }
                    MavMessage::BATTERY_STATUS(d) => {
                        if d.voltages[0] != 0 && d.voltages[0] != U16_MAX {
                            let cell1_v = d.voltages[0] as f32 / 1000.0;
                            let batt_pct = if d.battery_remaining < 0 {
                                "?".to_string()
                            } else {
                                format!("{}%", d.battery_remaining)
                            };
                            println!("BAT cell1={:.2}V batt={}", cell1_v, batt_pct);
                        }
                    }
                    _ => {}
                }
            }
            Err(e) => {
                eprintln!("Parse/read error (skipping): {}", e);
            }
        }
    }
}
