//! Raw console MAVLink telemetry reader (no TUI, no threads).

#![allow(deprecated)]

use drone_server::mavlink_connect::{self, MavlinkArgsError};
use drone_server::VehicleIds;
use mavlink::connect;
use mavlink::ardupilotmega::{
    COMMAND_LONG_DATA, MavCmd, MavMessage, MavModeFlag, MavState, MavType, REQUEST_DATA_STREAM_DATA,
};
use mavlink::ardupilotmega::GpsFixType;
use mavlink::{MavConnection, MavFrame};

const U16_MAX: u16 = 65535;

const MSG_ID_ATTITUDE: f32 = 30.0;
const MSG_ID_GLOBAL_POSITION_INT: f32 = 33.0;
const MSG_ID_SYS_STATUS: f32 = 1.0;
const MSG_ID_BATTERY_STATUS: f32 = 147.0;
const MSG_ID_GPS_RAW_INT: f32 = 24.0;
const MSG_ID_HOME_POSITION: f32 = 242.0;
const MSG_ID_VFR_HUD: f32 = 74.0;
const MSG_ID_RAW_IMU: f32 = 27.0;
const MSG_ID_LOCAL_POSITION_NED: f32 = 32.0;
const MSG_ID_RC_CHANNELS: f32 = 65.0;
const MSG_ID_SERVO_OUTPUT_RAW: f32 = 36.0;
const MSG_ID_NAV_CONTROLLER_OUTPUT: f32 = 62.0;
const MSG_ID_AHRS2: f32 = 178.0;
const MSG_ID_EKF_STATUS_REPORT: f32 = 193.0;
const MSG_ID_VIBRATION: f32 = 241.0;
const MSG_ID_DISTANCE_SENSOR: f32 = 132.0;
const MSG_ID_STATUSTEXT: f32 = 253.0;
const MSG_ID_MISSION_CURRENT: f32 = 42.0;
const MSG_ID_PARAM_VALUE: f32 = 22.0;

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
        MavState::MAV_STATE_FLIGHT_TERMINATION => "TERM",
    }
}

fn gps_fix_short(f: GpsFixType) -> &'static str {
    match f {
        GpsFixType::GPS_FIX_TYPE_NO_GPS => "NO_GPS",
        GpsFixType::GPS_FIX_TYPE_NO_FIX => "NO_FIX",
        GpsFixType::GPS_FIX_TYPE_2D_FIX => "2D",
        GpsFixType::GPS_FIX_TYPE_3D_FIX => "3D",
        GpsFixType::GPS_FIX_TYPE_DGPS => "DGPS",
        GpsFixType::GPS_FIX_TYPE_RTK_FLOAT => "RTK_FLT",
        GpsFixType::GPS_FIX_TYPE_RTK_FIXED => "RTK_FIX",
        GpsFixType::GPS_FIX_TYPE_STATIC => "STATIC",
        GpsFixType::GPS_FIX_TYPE_PPP => "PPP",
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

fn arducopter_mode_name(custom_mode: u32) -> &'static str {
    match custom_mode {
        0 => "STABILIZE", 1 => "ACRO", 2 => "ALT_HOLD", 3 => "AUTO", 4 => "GUIDED",
        5 => "LOITER", 6 => "RTL", 7 => "CIRCLE", 8 => "POSITION", 9 => "LAND",
        10 => "DRIFT", 11 => "SPORT", 12 => "FLIP", 13 => "AUTOTUNE", 14 => "POSHOLD",
        15 => "BRAKE", 16 => "THROW", 17 => "AVOID_ADMIN", 18 => "GUIDED_NOGPS",
        19 => "SMART_RTL", 20 => "FLOWHOLD", 21 => "FOLLOW", 22 => "ZIGZAG",
        23 => "SYSTEMID", 24 => "AUTOROTATE", 25 => "AUTO_RTL",
        _ => "UNKNOWN",
    }
}

fn heartbeat_from_autopilot(frame: &MavFrame<MavMessage>, mavtype: MavType) -> bool {
    const MAV_COMP_ID_AUTOPILOT1: u8 = 1;
    if mavtype == MavType::MAV_TYPE_GCS {
        return false;
    }
    frame.header.component_id == MAV_COMP_ID_AUTOPILOT1
}

fn request_stream_rates(connection: &impl MavConnection<MavMessage>, ids: VehicleIds) {
    let requests: [(f32, f32, &str); 19] = [
        (MSG_ID_ATTITUDE, 1_000_000.0 / 30.0, "ATTITUDE 30 Hz"),
        (MSG_ID_GLOBAL_POSITION_INT, 1_000_000.0 / 10.0, "GLOBAL_POSITION_INT 10 Hz"),
        (MSG_ID_SYS_STATUS, 1_000_000.0 / 5.0, "SYS_STATUS 5 Hz"),
        (MSG_ID_BATTERY_STATUS, 1_000_000.0 / 2.0, "BATTERY_STATUS 2 Hz"),
        (MSG_ID_GPS_RAW_INT, 1_000_000.0 / 5.0, "GPS_RAW_INT 5 Hz"),
        (MSG_ID_HOME_POSITION, 1_000_000.0 / 1.0, "HOME_POSITION 1 Hz"),
        (MSG_ID_VFR_HUD, 1_000_000.0 / 5.0, "VFR_HUD 5 Hz"),
        (MSG_ID_RAW_IMU, 1_000_000.0 / 10.0, "RAW_IMU 10 Hz"),
        (MSG_ID_LOCAL_POSITION_NED, 1_000_000.0 / 10.0, "LOCAL_POSITION_NED 10 Hz"),
        (MSG_ID_RC_CHANNELS, 1_000_000.0 / 5.0, "RC_CHANNELS 5 Hz"),
        (MSG_ID_SERVO_OUTPUT_RAW, 1_000_000.0 / 5.0, "SERVO_OUTPUT_RAW 5 Hz"),
        (MSG_ID_NAV_CONTROLLER_OUTPUT, 1_000_000.0 / 5.0, "NAV_CONTROLLER_OUTPUT 5 Hz"),
        (MSG_ID_AHRS2, 1_000_000.0 / 2.0, "AHRS2 2 Hz"),
        (MSG_ID_EKF_STATUS_REPORT, 1_000_000.0 / 2.0, "EKF_STATUS_REPORT 2 Hz"),
        (MSG_ID_VIBRATION, 1_000_000.0 / 2.0, "VIBRATION 2 Hz"),
        (MSG_ID_DISTANCE_SENSOR, 1_000_000.0 / 5.0, "DISTANCE_SENSOR 5 Hz"),
        (MSG_ID_STATUSTEXT, 1_000_000.0 / 1.0, "STATUSTEXT 1 Hz"),
        (MSG_ID_MISSION_CURRENT, 1_000_000.0 / 1.0, "MISSION_CURRENT 1 Hz"),
        (MSG_ID_PARAM_VALUE, 0.0, "PARAM_VALUE default"),
    ];
    for (msg_id, interval_us, _name) in requests {
        let cmd = COMMAND_LONG_DATA {
            param1: msg_id,
            param2: if interval_us > 0.0 { interval_us } else { 0.0 },
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            target_system: ids.system_id,
            target_component: ids.component_id,
            confirmation: 0,
        };
        let _ = connection.send_default(&MavMessage::COMMAND_LONG(cmd));
    }
    for stream_id in 0u8..=6u8 {
        let req = REQUEST_DATA_STREAM_DATA {
            req_message_rate: 5,
            target_system: ids.system_id,
            target_component: ids.component_id,
            req_stream_id: stream_id,
            start_stop: 1,
        };
        let _ = connection.send_default(&MavMessage::REQUEST_DATA_STREAM(req));
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (mavlink_url, link_display) = match mavlink_connect::resolve_from_args(args) {
        Ok(v) => v,
        Err(MavlinkArgsError::Help) => {
            println!("Usage: raw [OPTIONS]\n\n{}", mavlink_connect::usage_string());
            return;
        }
        Err(MavlinkArgsError::Invalid(m)) => {
            eprintln!("{m}");
            std::process::exit(2);
        }
    };

    println!("MAVLink: {}", link_display);
    println!("Waiting for first heartbeat...");

    let mut connection = match connect::<MavMessage>(&mavlink_url) {
        Ok(conn) => conn,
        Err(e) => {
            eprintln!("{}", mavlink_connect::open_error_message(&mavlink_url, &e));
            std::process::exit(1);
        }
    };
    mavlink_connect::tune_connection(&mut connection);

    let mut first_autopilot_heartbeat = true;
    loop {
        match connection.recv_frame() {
            Ok(frame) => {
                let msg = &frame.msg;
                match msg {
                    MavMessage::HEARTBEAT(d) => {
                        if first_autopilot_heartbeat && heartbeat_from_autopilot(&frame, d.mavtype) {
                            first_autopilot_heartbeat = false;
                            println!(
                                "HEARTBEAT from SYS={}, mode={}.",
                                frame.header.system_id,
                                arducopter_mode_name(d.custom_mode)
                            );
                            let ids = VehicleIds::new(frame.header.system_id, frame.header.component_id);
                            request_stream_rates(&connection, ids);
                        }
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
                    MavMessage::GPS_RAW_INT(d) => {
                        let lat_deg = d.lat as f64 / 1e7;
                        let lon_deg = d.lon as f64 / 1e7;
                        let alt_m = d.alt as f64 / 1000.0;
                        let hdop_str = if d.eph == U16_MAX {
                            "?".to_string()
                        } else {
                            format!("{:.2}", d.eph as f32 / 100.0)
                        };
                        println!(
                            "GPS fix={} sats={} hdop={} lat={:.6} lon={:.6} alt={:.1}m",
                            gps_fix_short(d.fix_type),
                            d.satellites_visible,
                            hdop_str,
                            lat_deg,
                            lon_deg,
                            alt_m
                        );
                    }
                    MavMessage::HOME_POSITION(d) => {
                        let lat_deg = d.latitude as f64 / 1e7;
                        let lon_deg = d.longitude as f64 / 1e7;
                        let alt_m = d.altitude as f64 / 1000.0;
                        println!("HOME lat={:.6} lon={:.6} alt={:.1}m", lat_deg, lon_deg, alt_m);
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
                    MavMessage::VFR_HUD(d) => {
                        println!(
                            "HUD air={:.1}m/s ground={:.1}m/s heading={}deg throttle={} alt={:.1}m climb={:.1}m/s",
                            d.airspeed,
                            d.groundspeed,
                            d.heading,
                            d.throttle,
                            d.alt,
                            d.climb
                        );
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
