//! MAVLink telemetry TUI for Pixhawk over serial.
//! Real-time dashboard using ratatui + crossterm.

#![allow(deprecated)]

use std::io;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use crossterm::execute;
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Gauge, Paragraph, Wrap};
use ratatui::Frame;

use mavlink::ardupilotmega::{
    COMMAND_LONG_DATA, MavCmd, MavMessage, MavModeFlag, MavState, REQUEST_DATA_STREAM_DATA,
};
use mavlink::ardupilotmega::GpsFixType;
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

const STATUSTEXT_MAX: usize = 8;
const ROLL_PITCH_RANGE: f32 = 45.0; // ±45° for gauge

#[derive(Default)]
struct TelemetryState {
    // Heartbeat
    heartbeat_status: Option<MavState>,
    heartbeat_base_mode: Option<MavModeFlag>,
    heartbeat_custom_mode: Option<u32>,
    // Attitude (radians)
    attitude_roll: Option<f32>,
    attitude_pitch: Option<f32>,
    attitude_yaw: Option<f32>,
    // Global position
    global_lat: Option<f64>,
    global_lon: Option<f64>,
    global_alt: Option<f64>,
    // GPS
    gps_fix_type: Option<GpsFixType>,
    gps_sats: Option<u8>,
    gps_eph: Option<u16>,
    gps_lat: Option<f64>,
    gps_lon: Option<f64>,
    gps_alt: Option<f64>,
    // Home
    home_lat: Option<f64>,
    home_lon: Option<f64>,
    home_alt: Option<f64>,
    // SYS_STATUS
    sys_voltage_battery: Option<u16>,
    sys_battery_remaining: Option<i8>,
    // BATTERY_STATUS
    bat_voltage_cell1: Option<u16>,
    bat_remaining: Option<i8>,
    // VFR_HUD
    hud_airspeed: Option<f32>,
    hud_groundspeed: Option<f32>,
    hud_heading: Option<i16>,
    hud_throttle: Option<u16>,
    hud_alt: Option<f32>,
    hud_climb: Option<f32>,
    // RAW_IMU (for summary)
    imu_received: bool,
    // LOCAL_POSITION_NED
    local_ned_x: Option<f32>,
    local_ned_y: Option<f32>,
    local_ned_z: Option<f32>,
    // RC
    rc_chancount: Option<u8>,
    rc_rssi: Option<u8>,
    rc_chan1_4: Option<(u16, u16, u16, u16)>,
    // SERVO
    servo_received: bool,
    // NAV_CONTROLLER
    nav_wp_dist: Option<u16>,
    nav_bearing: Option<i16>,
    // AHRS2
    ahrs2_received: bool,
    // EKF
    ekf_flags: Option<u16>,
    ekf_velocity_var: Option<f32>,
    // VIBRATION
    vib_x: Option<f32>,
    vib_y: Option<f32>,
    vib_z: Option<f32>,
    // DISTANCE_SENSOR
    distance_cm: Option<u16>,
    // STATUSTEXT
    statustext_lines: Vec<String>,
    // MISSION
    mission_seq: Option<u16>,
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

fn is_armed(flags: MavModeFlag) -> bool {
    flags.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED)
}

fn request_stream_rates(connection: &impl MavConnection<MavMessage>) {
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
            target_system: TARGET_SYSTEM,
            target_component: TARGET_COMPONENT,
            confirmation: 0,
        };
        let _ = connection.send_default(&MavMessage::COMMAND_LONG(cmd));
    }
    let fallback = REQUEST_DATA_STREAM_DATA {
        req_message_rate: 10,
        target_system: TARGET_SYSTEM,
        target_component: TARGET_COMPONENT,
        req_stream_id: 0,
        start_stop: 1,
    };
    let _ = connection.send_default(&MavMessage::REQUEST_DATA_STREAM(fallback));
}

fn apply_message(state: &mut TelemetryState, msg: MavMessage) {
    match msg {
        MavMessage::HEARTBEAT(d) => {
            state.heartbeat_status = Some(d.system_status);
            state.heartbeat_base_mode = Some(d.base_mode);
            state.heartbeat_custom_mode = Some(d.custom_mode);
        }
        MavMessage::ATTITUDE(d) => {
            state.attitude_roll = Some(d.roll);
            state.attitude_pitch = Some(d.pitch);
            state.attitude_yaw = Some(d.yaw);
        }
        MavMessage::GLOBAL_POSITION_INT(d) => {
            state.global_lat = Some(d.lat as f64 / 1e7);
            state.global_lon = Some(d.lon as f64 / 1e7);
            state.global_alt = Some(d.alt as f64 / 1000.0);
        }
        MavMessage::GPS_RAW_INT(d) => {
            state.gps_fix_type = Some(d.fix_type);
            state.gps_sats = Some(d.satellites_visible);
            state.gps_eph = if d.eph != U16_MAX { Some(d.eph) } else { None };
            state.gps_lat = Some(d.lat as f64 / 1e7);
            state.gps_lon = Some(d.lon as f64 / 1e7);
            state.gps_alt = Some(d.alt as f64 / 1000.0);
        }
        MavMessage::HOME_POSITION(d) => {
            state.home_lat = Some(d.latitude as f64 / 1e7);
            state.home_lon = Some(d.longitude as f64 / 1e7);
            state.home_alt = Some(d.altitude as f64 / 1000.0);
        }
        MavMessage::SYS_STATUS(d) => {
            state.sys_voltage_battery = Some(d.voltage_battery);
            state.sys_battery_remaining = Some(d.battery_remaining);
        }
        MavMessage::BATTERY_STATUS(d) => {
            state.bat_voltage_cell1 = if d.voltages[0] != 0 && d.voltages[0] != U16_MAX {
                Some(d.voltages[0])
            } else {
                None
            };
            state.bat_remaining = Some(d.battery_remaining);
        }
        MavMessage::VFR_HUD(d) => {
            state.hud_airspeed = Some(d.airspeed);
            state.hud_groundspeed = Some(d.groundspeed);
            state.hud_heading = Some(d.heading);
            state.hud_throttle = Some(d.throttle);
            state.hud_alt = Some(d.alt);
            state.hud_climb = Some(d.climb);
        }
        MavMessage::RAW_IMU(_) => state.imu_received = true,
        MavMessage::LOCAL_POSITION_NED(d) => {
            state.local_ned_x = Some(d.x);
            state.local_ned_y = Some(d.y);
            state.local_ned_z = Some(d.z);
        }
        MavMessage::RC_CHANNELS(d) => {
            state.rc_chancount = Some(d.chancount);
            state.rc_rssi = Some(d.rssi);
            state.rc_chan1_4 = Some((d.chan1_raw, d.chan2_raw, d.chan3_raw, d.chan4_raw));
        }
        MavMessage::SERVO_OUTPUT_RAW(_) => state.servo_received = true,
        MavMessage::NAV_CONTROLLER_OUTPUT(d) => {
            state.nav_wp_dist = Some(d.wp_dist);
            state.nav_bearing = Some(d.nav_bearing);
        }
        MavMessage::AHRS2(_) => state.ahrs2_received = true,
        MavMessage::EKF_STATUS_REPORT(d) => {
            state.ekf_flags = Some(d.flags.bits());
            state.ekf_velocity_var = Some(d.velocity_variance);
        }
        MavMessage::VIBRATION(d) => {
            state.vib_x = Some(d.vibration_x);
            state.vib_y = Some(d.vibration_y);
            state.vib_z = Some(d.vibration_z);
        }
        MavMessage::DISTANCE_SENSOR(d) => {
            state.distance_cm = Some(d.current_distance);
        }
        MavMessage::STATUSTEXT(d) => {
            let s = d.text.to_str().unwrap_or("").trim().to_string();
            if !s.is_empty() {
                state.statustext_lines.push(s);
                if state.statustext_lines.len() > STATUSTEXT_MAX {
                    state.statustext_lines.remove(0);
                }
            }
        }
        MavMessage::MISSION_CURRENT(d) => state.mission_seq = Some(d.seq),
        _ => {}
    }
}

fn draw_ui(frame: &mut Frame, state: &TelemetryState) {
    let area = frame.area();

    // Header
    let header_status = state
        .heartbeat_status
        .map(mav_state_short)
        .unwrap_or("---");
    let header_mode = state
        .heartbeat_base_mode
        .as_ref()
        .map(|m| mav_mode_flags_short(*m))
        .unwrap_or_else(|| "---".to_string());
    let header_custom = state
        .heartbeat_custom_mode
        .map(|c| c.to_string())
        .unwrap_or_else(|| "---".to_string());
    let armed = state
        .heartbeat_base_mode
        .map(is_armed)
        .unwrap_or(false);
    let armed_str = if armed { "ARMED" } else { "DISARMED" };
    let armed_style = if armed { Color::Red } else { Color::Green };
    let header = Line::from(vec![
        Span::raw(" HB "),
        Span::styled(header_status, Style::default().fg(Color::Cyan)),
        Span::raw(" | "),
        Span::raw(header_mode),
        Span::raw(" | custom="),
        Span::raw(header_custom),
        Span::raw(" | "),
        Span::styled(armed_str, Style::default().fg(armed_style).add_modifier(Modifier::BOLD)),
    ]);
    let header_block = Block::default()
        .borders(Borders::BOTTOM)
        .border_style(Style::default().fg(Color::DarkGray));
    frame.render_widget(
        Paragraph::new(header).block(header_block),
        Rect { x: area.x, y: area.y, width: area.width, height: 1 },
    );

    let main_area = Rect {
        x: area.x,
        y: area.y + 1,
        width: area.width,
        height: area.height.saturating_sub(3),
    };

    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage(33),
            Constraint::Percentage(34),
            Constraint::Percentage(33),
        ])
        .split(main_area);

    // Left: Attitude
    let roll_deg = state.attitude_roll.map(|r| r.to_degrees());
    let pitch_deg = state.attitude_pitch.map(|r| r.to_degrees());
    let yaw_deg = state.attitude_yaw.map(|r| r.to_degrees());
    let roll_str = roll_deg.map(|r| format!("{:+.1}°", r)).unwrap_or_else(|| "---".to_string());
    let pitch_str = pitch_deg.map(|p| format!("{:+.1}°", p)).unwrap_or_else(|| "---".to_string());
    let yaw_str = yaw_deg.map(|y| format!("{:.1}°", y)).unwrap_or_else(|| "---".to_string());
    let roll_gauge = roll_deg.map(|r| ((r + ROLL_PITCH_RANGE) / (2.0 * ROLL_PITCH_RANGE)) as u16).unwrap_or(50).min(100);
    let pitch_gauge = pitch_deg.map(|p| ((p + ROLL_PITCH_RANGE) / (2.0 * ROLL_PITCH_RANGE)) as u16).unwrap_or(50).min(100);
    let left_inner = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),
            Constraint::Length(3),
            Constraint::Length(2),
            Constraint::Min(0),
        ])
        .split(chunks[0]);
    let att_block = Block::default().title(" Attitude ").borders(Borders::ALL).border_style(Style::default().fg(Color::DarkGray));
    frame.render_widget(
        Paragraph::new(vec![
            Line::from(format!("Roll:  {}  Pitch: {}", roll_str, pitch_str)),
            Line::from(format!("Yaw:   {}", yaw_str)),
        ])
        .block(att_block),
        left_inner[0],
    );
    frame.render_widget(
        Gauge::default()
            .gauge_style(Style::default().fg(Color::Yellow))
            .ratio(roll_gauge as f64 / 100.0)
            .label("R"),
        left_inner[1],
    );
    frame.render_widget(
        Gauge::default()
            .gauge_style(Style::default().fg(Color::Magenta))
            .ratio(pitch_gauge as f64 / 100.0)
            .label("P"),
        left_inner[2],
    );

    // Center: GPS + Position
    let gps_fix_str = state.gps_fix_type.map(gps_fix_short).unwrap_or("---");
    let gps_color = match state.gps_fix_type {
        Some(GpsFixType::GPS_FIX_TYPE_3D_FIX) | Some(GpsFixType::GPS_FIX_TYPE_RTK_FIXED) | Some(GpsFixType::GPS_FIX_TYPE_RTK_FLOAT) => Color::Green,
        Some(GpsFixType::GPS_FIX_TYPE_2D_FIX) | Some(GpsFixType::GPS_FIX_TYPE_DGPS) => Color::Yellow,
        _ => Color::Red,
    };
    let sats_str = state.gps_sats.map(|s| s.to_string()).unwrap_or_else(|| "?".to_string());
    let hdop_str = state
        .gps_eph
        .map(|e| format!("{:.2}", e as f32 / 100.0))
        .unwrap_or_else(|| "?".to_string());
    let lat_str = state
        .global_lat
        .or(state.gps_lat)
        .map(|v| format!("{:.6}", v))
        .unwrap_or_else(|| "---".to_string());
    let lon_str = state
        .global_lon
        .or(state.gps_lon)
        .map(|v| format!("{:.6}", v))
        .unwrap_or_else(|| "---".to_string());
    let alt_str = state
        .global_alt
        .or(state.gps_alt)
        .map(|v| format!("{:.1}m", v))
        .unwrap_or_else(|| "---".to_string());
    let home_str = match (state.home_lat, state.home_lon, state.home_alt) {
        (Some(la), Some(lo), Some(a)) => format!("{:.6}, {:.6}, {:.1}m", la, lo, a),
        _ => "---".to_string(),
    };
    let center_inner = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(4),
            Constraint::Length(5),
            Constraint::Min(0),
        ])
        .split(chunks[1]);
    frame.render_widget(
        Block::default().title(" GPS / Position ").borders(Borders::ALL).border_style(Style::default().fg(Color::DarkGray)),
        chunks[1],
    );
    frame.render_widget(
        Paragraph::new(vec![
            Line::from(vec![
                Span::styled(gps_fix_str, Style::default().fg(gps_color)),
                Span::raw("  sats="),
                Span::raw(&sats_str),
                Span::raw("  hdop="),
                Span::raw(&hdop_str),
            ]),
            Line::from(format!("lat {}  lon {}", lat_str, lon_str)),
            Line::from(format!("alt {}", alt_str)),
        ]),
        Rect { x: center_inner[0].x + 1, y: center_inner[0].y + 1, width: center_inner[0].width.saturating_sub(2), height: center_inner[0].height.saturating_sub(1) },
    );
    frame.render_widget(
        Paragraph::new(Line::from(format!("Home: {}", home_str))),
        Rect { x: center_inner[1].x + 1, y: center_inner[1].y + 1, width: center_inner[1].width.saturating_sub(2), height: 1 },
    );

    // Right: Battery + HUD
    let vbat_str = state
        .sys_voltage_battery
        .map(|v| format!("{:.2}V", v as f32 / 100.0))
        .unwrap_or_else(|| "---".to_string());
    let batt_pct_str = state
        .sys_battery_remaining
        .or(state.bat_remaining)
        .map(|r| if r < 0 { "?".to_string() } else { format!("{}%", r) })
        .unwrap_or_else(|| "---".to_string());
    let cell1_str = state
        .bat_voltage_cell1
        .map(|v| format!("{:.2}V", v as f32 / 1000.0))
        .unwrap_or_else(|| "---".to_string());
    let batt_ratio = state
        .sys_battery_remaining
        .or(state.bat_remaining)
        .map(|r| if r < 0 { 0.5 } else { r as f64 / 100.0 })
        .unwrap_or(0.0);
    let batt_color = if batt_ratio >= 0.5 { Color::Green } else if batt_ratio >= 0.2 { Color::Yellow } else { Color::Red };
    let air_str = state.hud_airspeed.map(|v| format!("{:.1}", v)).unwrap_or_else(|| "---".to_string());
    let ground_str = state.hud_groundspeed.map(|v| format!("{:.1}", v)).unwrap_or_else(|| "---".to_string());
    let heading_str = state.hud_heading.map(|h| h.to_string()).unwrap_or_else(|| "---".to_string());
    let throttle_str = state.hud_throttle.map(|t| t.to_string()).unwrap_or_else(|| "---".to_string());
    let hud_alt_str = state.hud_alt.map(|a| format!("{:.1}m", a)).unwrap_or_else(|| "---".to_string());
    let climb_str = state.hud_climb.map(|c| format!("{:.1}m/s", c)).unwrap_or_else(|| "---".to_string());
    let right_inner = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(5),
            Constraint::Length(6),
            Constraint::Min(0),
        ])
        .split(chunks[2]);
    frame.render_widget(
        Block::default().title(" Battery ").borders(Borders::ALL).border_style(Style::default().fg(Color::DarkGray)),
        right_inner[0],
    );
    frame.render_widget(
        Paragraph::new(vec![
            Line::from(format!("VBat {}  {}", vbat_str, batt_pct_str)),
            Line::from(format!("Cell1 {}", cell1_str)),
        ]),
        Rect { x: right_inner[0].x + 1, y: right_inner[0].y + 1, width: right_inner[0].width.saturating_sub(2), height: 2 },
    );
    frame.render_widget(
        Gauge::default()
            .gauge_style(Style::default().fg(batt_color))
            .ratio(batt_ratio.clamp(0.0, 1.0))
            .label("Batt %"),
        Rect { x: right_inner[0].x + 1, y: right_inner[0].y + 3, width: right_inner[0].width.saturating_sub(2), height: 1 },
    );
    frame.render_widget(
        Block::default().title(" HUD ").borders(Borders::ALL).border_style(Style::default().fg(Color::DarkGray)),
        right_inner[1],
    );
    frame.render_widget(
        Paragraph::new(vec![
            Line::from(format!("Air {} m/s   Ground {} m/s", air_str, ground_str)),
            Line::from(format!("Hdg {}°   Throttle {}%", heading_str, throttle_str)),
            Line::from(format!("Alt {}   Climb {}", hud_alt_str, climb_str)),
        ]),
        Rect { x: right_inner[1].x + 1, y: right_inner[1].y + 1, width: right_inner[1].width.saturating_sub(2), height: right_inner[1].height.saturating_sub(2) },
    );

    // Bottom: Status lines, RC summary, IMU/VIB/EKF health
    let bottom_y = main_area.y + main_area.height.saturating_sub(1);
    let bottom_height = (area.height).saturating_sub(main_area.height + 1).min(10);
    if bottom_height > 0 {
        let bottom_area = Rect {
            x: area.x,
            y: bottom_y,
            width: area.width,
            height: bottom_height,
        };
        let rc_summary = match (state.rc_chancount, state.rc_rssi, state.rc_chan1_4) {
            (Some(n), Some(r), Some((c1, c2, c3, c4))) => format!("RC: {}ch rssi={}  [1-4]: {} {} {} {}", n, r, c1, c2, c3, c4),
            (Some(n), _, _) => format!("RC: {}ch", n),
            _ => "RC: ---".to_string(),
        };
        let vib_str = match (state.vib_x, state.vib_y, state.vib_z) {
            (Some(x), Some(y), Some(z)) => format!("{:.2}/{:.2}/{:.2}", x, y, z),
            _ => "---".to_string(),
        };
        let ekf_str = state
            .ekf_velocity_var
            .map(|v| format!("var={:.4}", v))
            .unwrap_or_else(|| "---".to_string());
        let health_line = format!(
            "IMU: {}  SERVO: {}  AHRS2: {}  VIB: {}  EKF: {}  Dist: {}cm",
            if state.imu_received { "ok" } else { "---" },
            if state.servo_received { "ok" } else { "---" },
            if state.ahrs2_received { "ok" } else { "---" },
            vib_str,
            ekf_str,
            state.distance_cm.map(|d| d.to_string()).unwrap_or_else(|| "---".to_string())
        );
        let mut status_text: Vec<Line> = if state.statustext_lines.is_empty() {
            vec![Line::from("(no status messages)")]
        } else {
            state
                .statustext_lines
                .iter()
                .rev()
                .take(STATUSTEXT_MAX)
                .map(|s| Line::from(s.as_str()))
                .collect()
        };
        status_text.push(Line::from(rc_summary));
        status_text.push(Line::from(health_line));
        frame.render_widget(
            Paragraph::new(status_text)
                .block(Block::default().title(" Status | RC | Health ").borders(Borders::ALL).border_style(Style::default().fg(Color::DarkGray)))
                .wrap(Wrap { trim: true }),
            bottom_area,
        );
    }

    // Footer
    let footer_y = area.height.saturating_sub(1);
    let footer = Paragraph::new(Line::from(Span::styled(
        " q = quit | Ctrl+C = exit ",
        Style::default().fg(Color::DarkGray),
    )));
    frame.render_widget(footer, Rect { x: area.x, y: footer_y, width: area.width, height: 1 });
}

fn run_ui(rx: mpsc::Receiver<MavMessage>) -> io::Result<()> {
    let backend = CrosstermBackend::new(io::stdout());
    let mut state = TelemetryState::default();

    crossterm::terminal::enable_raw_mode()?;
    execute!(io::stdout(), crossterm::terminal::EnterAlternateScreen)?;
    let mut terminal = ratatui::Terminal::new(backend)?;

    loop {
        while let Ok(msg) = rx.try_recv() {
            apply_message(&mut state, msg);
        }
        terminal.draw(|f| draw_ui(f, &state))?;
        if event::poll(Duration::from_millis(100))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press {
                    if key.code == KeyCode::Char('q') || (key.code == KeyCode::Char('c') && key.modifiers.contains(KeyModifiers::CONTROL)) {
                        break;
                    }
                }
            }
        }
    }

    execute!(io::stdout(), crossterm::terminal::LeaveAlternateScreen)?;
    crossterm::terminal::disable_raw_mode()?;
    Ok(())
}

fn main() {
    let config = SerialConfig::new(SERIAL_PORT.to_string(), BAUD_RATE);

    let connection = match config.connect::<MavMessage>() {
        Ok(conn) => {
            request_stream_rates(&conn);
            conn
        }
        Err(e) => {
            eprintln!("Failed to open serial port '{}': {}", SERIAL_PORT, e);
            eprintln!("Ensure the Pixhawk is connected over USB and you have permission to access the port.");
            std::process::exit(1);
        }
    };

    let (tx, rx) = mpsc::channel();
    let _recv_handle = thread::spawn(move || {
        loop {
            match connection.recv_frame() {
                Ok(frame) => {
                    let _ = tx.send(frame.msg);
                }
                Err(_) => {}
            }
        }
    });

    if let Err(e) = run_ui(rx) {
        eprintln!("UI error: {}", e);
        std::process::exit(1);
    }
}
