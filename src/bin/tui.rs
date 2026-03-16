//! MAVLink telemetry TUI with Vehicle info and Messages log.

#![allow(deprecated)]

use std::collections::VecDeque;
use std::io;
use std::sync::mpsc;
use std::thread;

use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use mavlink::ardupilotmega::{
    COMMAND_LONG_DATA, MavCmd, MavMessage, MavModeFlag, MavState, REQUEST_DATA_STREAM_DATA,
};
use mavlink::ardupilotmega::GpsFixType;
use mavlink::{connect, MavConnection, MavFrame};
use ratatui::layout::{Constraint, Direction, Layout};
use ratatui::style::{Color, Style};
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

const RECENT_MESSAGES_MAX: usize = 8;

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
    let requests: [(f32, f32, &str); 20] = [
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

fn draw_ui(f: &mut Frame, state: &TelemetryState) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(5),
            Constraint::Min(4),
            Constraint::Length(3),
        ])
        .split(f.area());

    let top = chunks[0];
    let mid = chunks[1];
    let bottom = chunks[2];

    let vehicle_lines: Vec<Line> = if state.vehicle_info.is_empty() {
        vec![Line::from(Span::raw("—"))]
    } else {
        state
            .vehicle_info
            .iter()
            .map(|s| Line::from(Span::raw(s.as_str())))
            .collect()
    };
    let vehicle_block = Block::default()
        .title(" Vehicle ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Cyan));
    f.render_widget(
        Paragraph::new(vehicle_lines).block(vehicle_block).wrap(Wrap { trim: true }),
        top,
    );

    let mid_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(4),
            Constraint::Length(5),
            Constraint::Length(4),
            Constraint::Length(4),
        ])
        .split(mid);

    let att_line = format!(
        "Roll {:.1}°  Pitch {:.1}°  Yaw {:.1}°",
        state.roll.unwrap_or(0.0),
        state.pitch.unwrap_or(0.0),
        state.yaw.unwrap_or(0.0)
    );
    let att_block = Block::default()
        .title(" Attitude ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Green));
    f.render_widget(
        Paragraph::new(att_line).block(att_block).wrap(Wrap { trim: true }),
        mid_chunks[0],
    );

    let gps_pos_line = format!(
        "Fix {}  Sats {}  HDOP {}  |  Lat {:.6}  Lon {:.6}  Alt {:.1}m",
        state.gps_fix.as_deref().unwrap_or("—"),
        state.gps_sats.map(|u| u.to_string()).as_deref().unwrap_or("—"),
        state.gps_hdop.as_deref().unwrap_or("—"),
        state.lat.unwrap_or(0.0),
        state.lon.unwrap_or(0.0),
        state.alt.unwrap_or(0.0)
    );
    let gps_block = Block::default()
        .title(" GPS / Position ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Yellow));
    f.render_widget(
        Paragraph::new(gps_pos_line).block(gps_block).wrap(Wrap { trim: true }),
        mid_chunks[1],
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
        .border_style(Style::default().fg(Color::Magenta));
    f.render_widget(
        Paragraph::new(bat_line).block(bat_block).wrap(Wrap { trim: true }),
        mid_chunks[2],
    );

    let hud_line = format!(
        "Air {:.1} m/s  Ground {:.1} m/s  Hdg {}°  Throttle {}  Climb {:.1} m/s",
        state.airspeed.unwrap_or(0.0),
        state.groundspeed.unwrap_or(0.0),
        state.heading.unwrap_or(0),
        state.throttle.unwrap_or(0),
        state.climb.unwrap_or(0.0)
    );
    let hud_block = Block::default()
        .title(" HUD ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Blue));
    f.render_widget(
        Paragraph::new(hud_line).block(hud_block).wrap(Wrap { trim: true }),
        mid_chunks[3],
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
        .title(" Messages ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::DarkGray));
    f.render_widget(
        Paragraph::new(msg_lines).block(msg_block).wrap(Wrap { trim: true }),
        bottom,
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
        loop {
            match connection.recv_frame() {
                Ok(frame) => {
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
