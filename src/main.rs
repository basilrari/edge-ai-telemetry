//! MAVLink telemetry reader for Pixhawk over serial.
//! Prints one-line human-readable lines for HEARTBEAT, ATTITUDE, GLOBAL_POSITION_INT, SYS_STATUS, BATTERY_STATUS only.

#[allow(deprecated)]
use mavlink::ardupilotmega::{
    COMMAND_LONG_DATA, MavCmd, MavMessage, MavModeFlag, MavState, REQUEST_DATA_STREAM_DATA,
};
use mavlink::{Connectable, MavConnection, SerialConfig};

const SERIAL_PORT: &str = "/dev/ttyACM0";
const BAUD_RATE: u32 = 115200;
const U16_MAX: u16 = 65535;
const TARGET_SYSTEM: u8 = 1;
const TARGET_COMPONENT: u8 = 1;

// Message IDs
const MSG_ID_ATTITUDE: f32 = 30.0;
const MSG_ID_GLOBAL_POSITION_INT: f32 = 33.0;
const MSG_ID_SYS_STATUS: f32 = 1.0;
const MSG_ID_BATTERY_STATUS: f32 = 147.0;

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

fn request_stream_rates(connection: &impl MavConnection<MavMessage>) {
    let requests: [(f32, f32, &str); 4] = [
        (MSG_ID_ATTITUDE, 1_000_000.0 / 30.0, "ATTITUDE 30 Hz"),
        (MSG_ID_GLOBAL_POSITION_INT, 1_000_000.0 / 10.0, "GLOBAL_POSITION_INT 10 Hz"),
        (MSG_ID_SYS_STATUS, 1_000_000.0 / 5.0, "SYS_STATUS 5 Hz"),
        (MSG_ID_BATTERY_STATUS, 1_000_000.0 / 2.0, "BATTERY_STATUS 2 Hz"),
    ];
    for (msg_id, interval_us, name) in requests {
        let cmd = COMMAND_LONG_DATA {
            param1: msg_id,
            param2: interval_us,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            target_system: TARGET_SYSTEM,
            target_component: TARGET_COMPONENT,
            confirmation: 0,
        };
        match connection.send_default(&MavMessage::COMMAND_LONG(cmd)) {
            Ok(_) => eprintln!("Requested {}: ok", name),
            Err(e) => eprintln!("Request {}: {}", name, e),
        }
    }
    #[allow(deprecated)]
    let fallback = REQUEST_DATA_STREAM_DATA {
        req_message_rate: 10,
        target_system: TARGET_SYSTEM,
        target_component: TARGET_COMPONENT,
        req_stream_id: 0,
        start_stop: 1,
    };
    #[allow(deprecated)]
    match connection.send_default(&MavMessage::REQUEST_DATA_STREAM(fallback)) {
        Ok(_) => eprintln!("Requested REQUEST_DATA_STREAM (stream 0, 10 Hz): ok"),
        Err(e) => eprintln!("Request REQUEST_DATA_STREAM: {}", e),
    }
}

fn main() {
    let config = SerialConfig::new(SERIAL_PORT.to_string(), BAUD_RATE);

    let connection = match config.connect::<MavMessage>() {
        Ok(conn) => {
            eprintln!("Connected to {} at {} baud (Ctrl+C to stop).\n", SERIAL_PORT, BAUD_RATE);
            request_stream_rates(&conn);
            eprintln!("");
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
