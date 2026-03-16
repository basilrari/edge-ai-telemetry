//! MAVLink telemetry TUI with Vehicle info and Messages log.

#![allow(deprecated)]

use std::collections::VecDeque;
use std::io;
use std::sync::mpsc;
use std::thread;

use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use mavlink::ardupilotmega::{
    COMMAND_LONG_DATA, MavCmd, MavMessage, MavModeFlag, MavState, MavType, REQUEST_DATA_STREAM_DATA,
};
use mavlink::ardupilotmega::GpsFixType;
use mavlink::{connect, MavConnection, MavFrame};
use ratatui::layout::{Constraint, Direction, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph, Wrap};
use ratatui::{Frame, Terminal};

const MAVLINK_UDP_BIND: &str = "udpin:0.0.0.0:14550";
const MAVLINK_UDP_DISPLAY: &str = "udp:0.0.0.0:14550";
const U16_MAX: u16 = 65535;
const TARGET_SYSTEM: u8 = 1;
const TARGET_COMPONENT: u8 = 1;

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
const MSG_ID_COMMAND_ACK: f32 = 77.0;
const MSG_ID_SYSTEM_TIME: f32 = 2.0;

const RECENT_MESSAGES_MAX: usize = 24;

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

fn mav_type_short(t: MavType) -> String {
    let s = format!("{:?}", t);
    s.replace("MAV_TYPE_", "")
}

fn is_armed(base_mode: MavModeFlag) -> bool {
    base_mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED)
}

/// Format time_boot_ms as human-readable uptime (e.g. "14m32s" or "2h05m").
fn format_uptime_ms(ms: u32) -> String {
    let secs = ms / 1000;
    if secs >= 3600 {
        let h = secs / 3600;
        let m = (secs % 3600) / 60;
        format!("{}h{:02}m", h, m)
    } else if secs >= 60 {
        let m = secs / 60;
        let s = secs % 60;
        format!("{}m{:02}s", m, s)
    } else {
        format!("{}s", secs)
    }
}

fn mav_cmd_short(cmd: u16) -> &'static str {
    match cmd {
        16 => "WAYPOINT",
        17 => "LOITER_UNLIM",
        18 => "LOITER_TURNS",
        19 => "LOITER_TIME",
        20 => "RTL",
        21 => "LAND",
        22 => "TAKEOFF",
        23 => "LAND_LOCAL",
        24 => "TAKEOFF_LOCAL",
        25 => "FOLLOW",
        30 => "CONTINUE_AND_CHANGE_ALT",
        31 => "LOITER_TO_ALT",
        34 => "ORBIT",
        80 => "ROI",
        81 => "PATHPLANNING",
        82 => "SPLINE_WAYPOINT",
        84 => "VTOL_TAKEOFF",
        _ => "CMD_?",
    }
}

/// MAVLink frame: 0=GLOBAL (AMSL), 3=RELATIVE_ALT (rel home), 10=TERRAIN_ALT
fn alt_frame_short(frame: u8) -> &'static str {
    match frame {
        0 => "AMSL",
        3 => "rel",
        10 => "terrain",
        _ => "?",
    }
}

fn waypoint_line(wp: &Waypoint, current_seq: Option<u16>) -> String {
    let prefix = if current_seq == Some(wp.seq) { "*" } else { " " };
    let cmd = mav_cmd_short(wp.command);
    let alt_suffix = alt_frame_short(wp.frame);
    match wp.command {
        22 => format!("{}{}: {} alt={:.0}m {} (target)", prefix, wp.seq, cmd, wp.alt, alt_suffix), // TAKEOFF
        16 | 21 | 31 | 82 => format!(
            "{}{}: {} lat={:.5} lon={:.5} alt={:.0}m {} (target)",
            prefix, wp.seq, cmd, wp.lat, wp.lon, wp.alt, alt_suffix
        ),
        _ => format!("{}{}: {} lat={:.5} lon={:.5} alt={:.0}m {} (target)", prefix, wp.seq, cmd, wp.lat, wp.lon, wp.alt, alt_suffix),
    }
}

#[derive(Clone)]
struct Waypoint {
    seq: u16,
    command: u16,
    lat: f64,
    lon: f64,
    alt: f32,
    #[allow(dead_code)]
    frame: u8,
}

#[derive(Default)]
struct TelemetryState {
    heartbeat_status: Option<String>,
    heartbeat_mode: Option<String>,
    heartbeat_custom: Option<u32>,
    roll: Option<f32>,
    pitch: Option<f32>,
    yaw: Option<f32>,
    gps_fix: Option<String>,
    gps_sats: Option<u8>,
    gps_hdop: Option<String>,
    lat: Option<f64>,
    lon: Option<f64>,
    alt: Option<f64>,
    home_lat: Option<f64>,
    home_lon: Option<f64>,
    home_alt: Option<f64>,
    vbat: Option<f32>,
    batt_pct: Option<String>,
    cell1_v: Option<f32>,
    airspeed: Option<f32>,
    groundspeed: Option<f32>,
    heading: Option<i16>,
    throttle: Option<u16>,
    climb: Option<f32>,
    vehicle_info: Vec<String>,
    recent_messages: VecDeque<String>,
    first_heartbeat_logged: bool,
    vehicle_sysid: Option<u8>,
    vehicle_compid: Option<u8>,
    vehicle_type_name: Option<String>,
    vehicle_mode_name: Option<String>,
    armed: Option<bool>,
    sys_voltage: Option<f32>,
    sys_current: Option<f32>,
    sys_load: Option<u16>,
    time_boot_ms: Option<u32>,
    mission_waypoints: Vec<Waypoint>,
    mission_current_seq: Option<u16>,
}

impl TelemetryState {
    fn push_recent(&mut self, line: String) {
        self.recent_messages.push_back(line);
        while self.recent_messages.len() > RECENT_MESSAGES_MAX {
            self.recent_messages.pop_front();
        }
    }
}

fn request_stream_rates(connection: &impl MavConnection<MavMessage>) {
    let requests: [(f32, f32, &str); 21] = [
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
        (MSG_ID_STATUSTEXT, 1_000_000.0 / 2.0, "STATUSTEXT 2 Hz"),
        (MSG_ID_MISSION_CURRENT, 1_000_000.0 / 1.0, "MISSION_CURRENT 1 Hz"),
        (MSG_ID_PARAM_VALUE, 0.0, "PARAM_VALUE default"),
        (MSG_ID_COMMAND_ACK, 1_000_000.0 / 5.0, "COMMAND_ACK 5 Hz"),
        (MSG_ID_SYSTEM_TIME, 1_000_000.0 / 2.0, "SYSTEM_TIME 2 Hz"),
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

fn statustext_to_str(d: &mavlink::ardupilotmega::STATUSTEXT_DATA) -> String {
    d.text
        .to_str()
        .unwrap_or("")
        .trim()
        .to_string()
}

fn apply_message(state: &mut TelemetryState, frame: &MavFrame<MavMessage>) {
    let msg = &frame.msg;
    match msg {
        MavMessage::HEARTBEAT(d) => {
            state.vehicle_sysid = Some(frame.header.system_id);
            state.vehicle_compid = Some(frame.header.component_id);
            state.vehicle_type_name = Some(mav_type_short(d.mavtype));
            state.vehicle_mode_name = Some(arducopter_mode_name(d.custom_mode).to_string());
            state.armed = Some(is_armed(d.base_mode));
            if !state.first_heartbeat_logged {
                state.first_heartbeat_logged = true;
                state.push_recent(format!(
                    "HEARTBEAT from SYS={}, mode={}.",
                    frame.header.system_id,
                    arducopter_mode_name(d.custom_mode)
                ));
            }
            state.heartbeat_status = Some(mav_state_short(d.system_status).to_string());
            state.heartbeat_mode = Some(mav_mode_flags_short(d.base_mode));
            state.heartbeat_custom = Some(d.custom_mode);
        }
        MavMessage::ATTITUDE(d) => {
            state.roll = Some(rad_to_deg(d.roll));
            state.pitch = Some(rad_to_deg(d.pitch));
            state.yaw = Some(rad_to_deg(d.yaw));
        }
        MavMessage::GLOBAL_POSITION_INT(d) => {
            state.lat = Some(d.lat as f64 / 1e7);
            state.lon = Some(d.lon as f64 / 1e7);
            state.alt = Some(d.alt as f64 / 1000.0);
        }
        MavMessage::GPS_RAW_INT(d) => {
            state.gps_fix = Some(gps_fix_short(d.fix_type).to_string());
            state.gps_sats = Some(d.satellites_visible);
            state.gps_hdop = Some(if d.eph == U16_MAX {
                "?".to_string()
            } else {
                format!("{:.2}", d.eph as f32 / 100.0)
            });
        }
        MavMessage::HOME_POSITION(d) => {
            state.home_lat = Some(d.latitude as f64 / 1e7);
            state.home_lon = Some(d.longitude as f64 / 1e7);
            state.home_alt = Some(d.altitude as f64 / 1000.0);
        }
        MavMessage::SYS_STATUS(d) => {
            state.vbat = Some(d.voltage_battery as f32 / 100.0);
            state.sys_voltage = Some(d.voltage_battery as f32 / 100.0);
            if d.current_battery >= 0 {
                state.sys_current = Some(d.current_battery as f32 / 100.0);
            }
            state.sys_load = Some(d.load);
            state.batt_pct = Some(if d.battery_remaining < 0 {
                "?".to_string()
            } else {
                format!("{}%", d.battery_remaining)
            });
        }
        MavMessage::BATTERY_STATUS(d) => {
            if d.voltages[0] != 0 && d.voltages[0] != U16_MAX {
                state.cell1_v = Some(d.voltages[0] as f32 / 1000.0);
            }
            if d.battery_remaining >= 0 {
                state.batt_pct = Some(format!("{}%", d.battery_remaining));
            }
        }
        MavMessage::VFR_HUD(d) => {
            state.airspeed = Some(d.airspeed);
            state.groundspeed = Some(d.groundspeed);
            state.heading = Some(d.heading);
            state.throttle = Some(d.throttle);
            state.climb = Some(d.climb);
        }
        MavMessage::STATUSTEXT(d) => {
            let text = statustext_to_str(d);
            if !text.is_empty() {
                state.push_recent(text.clone());
                if text.starts_with("AP:") {
                    let rest = text.trim_start_matches("AP:").trim();
                    if rest.contains("ArduCopter") || rest.contains("ArduPlane") || rest.contains("ArduRover") {
                        let label = "Firmware";
                        update_vehicle_line(state, label, rest);
                    } else if rest.starts_with("Frame:") {
                        let value = rest.trim_start_matches("Frame:").trim();
                        update_vehicle_line(state, "Frame", value);
                    } else if !rest.is_empty() && !rest.starts_with("ChibiOS") && rest.len() < 40 {
                        update_vehicle_line(state, "Board", rest);
                    }
                }
            }
        }
        MavMessage::COMMAND_ACK(d) => {
            let cmd_str = format!("{:?}", d.command);
            let result_str = match d.result {
                mavlink::ardupilotmega::MavResult::MAV_RESULT_ACCEPTED => "accepted",
                _ => "failed",
            };
            state.push_recent(format!("ACK: {} {}", cmd_str, result_str));
        }
        MavMessage::SYSTEM_TIME(d) => {
            state.time_boot_ms = Some(d.time_boot_ms);
        }
        MavMessage::MISSION_ITEM_INT(d) => {
            let lat = d.x as f64 / 1e7;
            let lon = d.y as f64 / 1e7;
            let alt = d.z;
            let wp = Waypoint {
                seq: d.seq,
                command: d.command as u16,
                lat,
                lon,
                alt,
                frame: d.frame as u8,
            };
            if let Some(pos) = state.mission_waypoints.iter().position(|w| w.seq == d.seq) {
                state.mission_waypoints[pos] = wp;
            } else {
                state.mission_waypoints.push(wp);
                state.mission_waypoints.sort_by_key(|w| w.seq);
            }
        }
        MavMessage::MISSION_CURRENT(d) => {
            state.mission_current_seq = Some(d.seq);
        }
        _ => {}
    }
}

fn update_vehicle_line(state: &mut TelemetryState, label: &str, value: &str) {
    let line = format!("{}: {}", label, value);
    if let Some(pos) = state.vehicle_info.iter().position(|s| s.starts_with(label)) {
        state.vehicle_info[pos] = line;
    } else {
        state.vehicle_info.push(line);
    }
}

/// Panel border/style colors for a distinct look per section.
fn vehicle_style() -> Style {
    Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)
}
fn attitude_style() -> Style {
    Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)
}
fn gps_style() -> Style {
    Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)
}
fn battery_style() -> Style {
    Style::default().fg(Color::Magenta).add_modifier(Modifier::BOLD)
}
fn hud_style() -> Style {
    Style::default().fg(Color::Blue).add_modifier(Modifier::BOLD)
}
fn mission_style() -> Style {
    Style::default().fg(Color::LightCyan).add_modifier(Modifier::BOLD)
}
fn messages_style() -> Style {
    Style::default().fg(Color::DarkGray).add_modifier(Modifier::BOLD)
}

fn draw_ui(f: &mut Frame, state: &TelemetryState) {
    // Side-by-side: left column (telemetry panels), right column (mission + messages)
    let main_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(38), Constraint::Percentage(62)])
        .split(f.area());

    let left = main_chunks[0];
    let right = main_chunks[1];

    // Left column: vertical stack of panels
    let left_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(7),
            Constraint::Length(4),
            Constraint::Length(5),
            Constraint::Length(4),
            Constraint::Length(4),
        ])
        .split(left);

    let vehicle_lines: Vec<Line> = {
        let mut lines = Vec::new();
        lines.push(Line::from(Span::raw(format!(
            "SYS: {}  COMP: {}  TYPE: {}",
            state.vehicle_sysid.map(|u| u.to_string()).as_deref().unwrap_or("—"),
            state.vehicle_compid.map(|u| u.to_string()).as_deref().unwrap_or("—"),
            state.vehicle_type_name.as_deref().unwrap_or("—")
        ))));
        let armed_display = state.armed
            .map(|b| if b { "ARMED".to_string() } else { "false".to_string() })
            .unwrap_or_else(|| "—".to_string());
        let armed_style = if state.armed == Some(true) {
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::DarkGray)
        };
        lines.push(Line::from(vec![
            Span::raw("MODE: "),
            Span::raw(state.vehicle_mode_name.as_deref().unwrap_or("—")),
            Span::raw("  ARMED: "),
            Span::styled(armed_display, armed_style),
        ]));
        lines.push(Line::from(Span::raw(format!(
            "Vbat: {:.2}V  Current: {}  Load: {}%  Uptime: {}",
            state.sys_voltage.unwrap_or(0.0),
            state.sys_current.map(|c| format!("{:.2}A", c)).as_deref().unwrap_or("—"),
            state.sys_load.map(|l| (l / 10).to_string()).as_deref().unwrap_or("—"),
            state.time_boot_ms.map(format_uptime_ms).as_deref().unwrap_or("—")
        ))));
        for s in &state.vehicle_info {
            lines.push(Line::from(Span::raw(s.as_str())));
        }
        lines
    };
    let vehicle_block = Block::default()
        .title(" Vehicle ")
        .borders(Borders::ALL)
        .border_style(vehicle_style());
    f.render_widget(
        Paragraph::new(vehicle_lines).block(vehicle_block).wrap(Wrap { trim: true }),
        left_chunks[0],
    );

    let att_line = format!(
        "Roll {:.1}°  Pitch {:.1}°  Yaw {:.1}°",
        state.roll.unwrap_or(0.0),
        state.pitch.unwrap_or(0.0),
        state.yaw.unwrap_or(0.0)
    );
    let att_block = Block::default()
        .title(" Attitude ")
        .borders(Borders::ALL)
        .border_style(attitude_style());
    f.render_widget(
        Paragraph::new(att_line).block(att_block).wrap(Wrap { trim: true }),
        left_chunks[1],
    );

    let home_str = match (state.home_lat, state.home_lon, state.home_alt) {
        (Some(lat), Some(lon), Some(alt)) => format!("Home: {:.6}, {:.6}, {:.1}m AMSL", lat, lon, alt),
        _ => "Home: —".to_string(),
    };
    let gps_pos_line = format!(
        "Fix {}  Sats {}  HDOP {}  |  Lat {:.6}  Lon {:.6}  Alt {:.1}m\n{}",
        state.gps_fix.as_deref().unwrap_or("—"),
        state.gps_sats.map(|u| u.to_string()).as_deref().unwrap_or("—"),
        state.gps_hdop.as_deref().unwrap_or("—"),
        state.lat.unwrap_or(0.0),
        state.lon.unwrap_or(0.0),
        state.alt.unwrap_or(0.0),
        home_str
    );
    let gps_block = Block::default()
        .title(" GPS / Position ")
        .borders(Borders::ALL)
        .border_style(gps_style());
    f.render_widget(
        Paragraph::new(gps_pos_line).block(gps_block).wrap(Wrap { trim: true }),
        left_chunks[2],
    );

    let bat_line = format!(
        "VBat {:.2}V  Batt {}  Cell1 {:.2}V",
        state.vbat.unwrap_or(0.0),
        state.batt_pct.as_deref().unwrap_or("—"),
        state.cell1_v.unwrap_or(0.0)
    );
    let bat_block = Block::default()
        .title(" Battery ")
        .borders(Borders::ALL)
        .border_style(battery_style());
    f.render_widget(
        Paragraph::new(bat_line).block(bat_block).wrap(Wrap { trim: true }),
        left_chunks[3],
    );

    let hud_line = format!(
        "Air {:.1}  Grd {:.1}  Hdg {}°  Thr {}  Climb {:.1}",
        state.airspeed.unwrap_or(0.0),
        state.groundspeed.unwrap_or(0.0),
        state.heading.unwrap_or(0),
        state.throttle.unwrap_or(0),
        state.climb.unwrap_or(0.0)
    );
    let hud_block = Block::default()
        .title(" HUD ")
        .borders(Borders::ALL)
        .border_style(hud_style());
    f.render_widget(
        Paragraph::new(hud_line).block(hud_block).wrap(Wrap { trim: true }),
        left_chunks[4],
    );

    // Right column: mission (fills space) + messages (fixed height)
    let right_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Ratio(1, 3), Constraint::Ratio(2, 3)])
        .split(right);

    let mission_lines: Vec<Line> = if state.mission_waypoints.is_empty() {
        vec![Line::from(Span::raw("(no waypoints received)"))]
    } else {
        let header = Line::from(Span::styled(
            "(* = current WP)  alt: AMSL = above sea level, rel = relative to home",
            Style::default().fg(Color::DarkGray),
        ));
        let mut lines = vec![header];
        lines.extend(
            state
                .mission_waypoints
                .iter()
                .map(|w| {
                    let raw = waypoint_line(w, state.mission_current_seq);
                    let is_current = state.mission_current_seq == Some(w.seq);
                    Line::from(if is_current {
                        Span::styled(raw, Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD))
                    } else {
                        Span::raw(raw)
                    })
                }),
        );
        lines
    };
    let mission_block = Block::default()
        .title(" Mission (waypoints) ")
        .borders(Borders::ALL)
        .border_style(mission_style());
    let mission_area = right_chunks[0];
    let total_lines = mission_lines.len() as u16;
    let visible_lines = mission_area.height.saturating_sub(2); // inner height minus borders
    let scroll_offset = if total_lines <= visible_lines {
        0
    } else if let Some(seq) = state.mission_current_seq {
        let cur = 1 + seq as u16; // line index: 0 = header, 1 = wp0, ...
        let vis = visible_lines;
        let centered = cur.saturating_sub(vis / 2);
        centered.min(total_lines.saturating_sub(vis))
    } else {
        0
    };
    f.render_widget(
        Paragraph::new(mission_lines)
            .block(mission_block)
            .wrap(Wrap { trim: true })
            .scroll((scroll_offset, 0)),
        mission_area,
    );

    let msg_lines: Vec<Line> = if state.recent_messages.is_empty() {
        vec![Line::from(Span::raw("—"))]
    } else {
        state
            .recent_messages
            .iter()
            .map(|s| Line::from(Span::raw(s.as_str())))
            .collect()
    };
    let msg_block = Block::default()
        .title(" Messages (q = quit) ")
        .borders(Borders::ALL)
        .border_style(messages_style());
    f.render_widget(
        Paragraph::new(msg_lines).block(msg_block).wrap(Wrap { trim: true }),
        right_chunks[1],
    );
}

fn run_ui(rx: mpsc::Receiver<MavFrame<MavMessage>>) -> io::Result<()> {
    crossterm::terminal::enable_raw_mode()?;
    let backend = ratatui::backend::CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;
    crossterm::execute!(
        io::stdout(),
        crossterm::terminal::EnterAlternateScreen
    )?;

    let mut state = TelemetryState::default();

    loop {
        while let Ok(frame) = rx.try_recv() {
            apply_message(&mut state, &frame);
        }
        terminal.draw(|f| draw_ui(f, &state))?;
        if event::poll(std::time::Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press && key.code == KeyCode::Char('q') {
                    break;
                }
            }
        }
    }

    crossterm::execute!(
        io::stdout(),
        crossterm::terminal::LeaveAlternateScreen
    )?;
    crossterm::terminal::disable_raw_mode()?;
    Ok(())
}

fn main() {
    eprintln!("Listening for MAVLink on {}", MAVLINK_UDP_DISPLAY);
    eprintln!("Waiting for first heartbeat...");
    eprintln!("Press q to quit the TUI.");

    let connection = match connect::<MavMessage>(MAVLINK_UDP_BIND) {
        Ok(conn) => conn,
        Err(e) => {
            eprintln!("Failed to bind: {}", e);
            std::process::exit(1);
        }
    };
    request_stream_rates(&connection);

    let (tx, rx) = mpsc::channel();
    let _recv_handle = thread::spawn(move || {
        let mut first_heartbeat_done = false;
        let mut mission_count: Option<u16> = None;
        loop {
            match connection.recv_frame() {
                Ok(frame) => {
                    if !first_heartbeat_done {
                        if let MavMessage::HEARTBEAT(_) = &frame.msg {
                            first_heartbeat_done = true;
                            let target_sys = frame.header.system_id;
                            let target_comp = frame.header.component_id;
                            let req = mavlink::ardupilotmega::MISSION_REQUEST_LIST_DATA {
                                target_system: target_sys,
                                target_component: target_comp,
                            };
                            let _ = connection.send_default(&MavMessage::MISSION_REQUEST_LIST(req));
                        }
                    }
                    if let MavMessage::MISSION_COUNT(d) = &frame.msg {
                        if mission_count.is_none() {
                            mission_count = Some(d.count);
                            if d.count > 0 {
                                let sys = frame.header.system_id;
                                let comp = frame.header.component_id;
                                let req = mavlink::ardupilotmega::MISSION_REQUEST_INT_DATA {
                                    target_system: sys,
                                    target_component: comp,
                                    seq: 0,
                                };
                                let _ = connection.send_default(&MavMessage::MISSION_REQUEST_INT(req));
                            }
                        }
                    }
                    if let MavMessage::MISSION_ITEM_INT(d) = &frame.msg {
                        if let Some(count) = mission_count {
                            let next_seq = d.seq + 1;
                            if next_seq < count {
                                let sys = frame.header.system_id;
                                let comp = frame.header.component_id;
                                let req = mavlink::ardupilotmega::MISSION_REQUEST_INT_DATA {
                                    target_system: sys,
                                    target_component: comp,
                                    seq: next_seq,
                                };
                                let _ = connection.send_default(&MavMessage::MISSION_REQUEST_INT(req));
                            } else {
                                mission_count = None;
                            }
                        }
                    }
                    let _ = tx.send(frame);
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
