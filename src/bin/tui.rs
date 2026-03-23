//! MAVLink telemetry TUI with Vehicle info and Messages log.

#![allow(deprecated)]

use std::collections::VecDeque;
use std::io;
use std::net::{SocketAddr, TcpStream};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use drone_server::{
    force_arm, goto_global_command_int, land, mission_set_current, mission_start, rtl,
    set_mode_auto, set_mode_guided, MissionStore, VehicleIds,
};
use drone_server::mavlink_connect::{self, MavlinkArgsError};
use mavlink::ardupilotmega::{
    COMMAND_LONG_DATA, MavCmd, MavMessage, MavModeFlag, MavResult, MavState, MavType,
    REQUEST_DATA_STREAM_DATA,
};
use mavlink::ardupilotmega::GpsFixType;
use mavlink::{connect, MavConnection, MavFrame};
use ratatui::layout::{Constraint, Direction, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Clear, Paragraph, Wrap};
use ratatui::{Frame, Terminal};

const U16_MAX: u16 = 65535;
const TARGET_SYSTEM: u8 = 1;
const TARGET_COMPONENT: u8 = 1;
/// Horizontal distance (m) to consider a waypoint "reached" during override.
const REACHED_THRESHOLD_M: f64 = 10.0;
const INTERNET_OFFLINE_RTL_AFTER_SECS: u64 = 30;
const INTERNET_CHECK_PERIOD_SECS: u64 = 2;

/// State for override/resume: normal mission, paused (interrupt, wait for 'c'), running override waypoints, or resuming.
#[derive(Clone, Debug)]
pub enum OverrideState {
    MissionRunning,
    /// Interrupt: drone is hovering, press 'c' to resume mission. Can press 'w' to inject a waypoint.
    Paused,
    OverrideActive {
        waypoints: Vec<(f64, f64, f64)>,
        index: usize,
        /// When true, resume mission after last waypoint; when false, go to Paused.
        resume_after: bool,
    },
    Resuming { resume_seq: u16 },
}

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

const RECENT_MESSAGES_MAX: usize = 32;
/// After this, if there was no matching FC reply, show a hint once.
const PENDING_CMD_TIMEOUT: Duration = Duration::from_secs(3);

/// Re-send stream setup until these MAVLink messages have been seen at least once.
const STREAM_AUTO_RETRY_FIRST_DELAY: Duration = Duration::from_millis(500);
const STREAM_AUTO_RETRY_INTERVAL: Duration = Duration::from_secs(2);
const STREAM_AUTO_RETRY_MAX_ATTEMPTS: u32 = 35;

/// Tracks which high-rate telemetry types we have received (recv thread).
#[derive(Default)]
struct TelemetryCoverage {
    heartbeat: bool,
    attitude: bool,
    global_position_int: bool,
    gps_raw_int: bool,
    sys_status: bool,
    vfr_hud: bool,
    home_position: bool,
}

impl TelemetryCoverage {
    fn update(&mut self, msg: &MavMessage) {
        match msg {
            MavMessage::HEARTBEAT(_) => self.heartbeat = true,
            MavMessage::ATTITUDE(_) => self.attitude = true,
            MavMessage::GLOBAL_POSITION_INT(_) => self.global_position_int = true,
            MavMessage::GPS_RAW_INT(_) => self.gps_raw_int = true,
            MavMessage::SYS_STATUS(_) => self.sys_status = true,
            MavMessage::VFR_HUD(_) => self.vfr_hud = true,
            MavMessage::HOME_POSITION(_) => self.home_position = true,
            _ => {}
        }
    }

    fn is_complete(&self) -> bool {
        self.heartbeat
            && self.attitude
            && self.global_position_int
            && self.gps_raw_int
            && self.sys_status
            && self.vfr_hud
            && self.home_position
    }
}

fn vehicle_ids_from_state(state: &TelemetryState) -> VehicleIds {
    VehicleIds::new(
        state.vehicle_sysid.unwrap_or(TARGET_SYSTEM),
        state.vehicle_compid.unwrap_or(TARGET_COMPONENT),
    )
}

fn mav_result_desc(r: MavResult) -> String {
    format!("{:?}", r)
}

/// Last command we sent from the TUI: used to correlate COMMAND_ACK and mode telemetry.
#[derive(Clone)]
struct PendingFeedback {
    label: String,
    /// COMMAND_ACK.command we expect for this action (if FC sends ACK).
    expect_cmd: Option<MavCmd>,
    /// ArduCopter `custom_mode` we expect on HEARTBEAT after SET_MODE / DO_SET_MODE (fallback).
    expect_copter_mode: Option<u32>,
    sent_at: Instant,
    timeout_warned: bool,
}

impl PendingFeedback {
    fn new(
        label: impl Into<String>,
        expect_cmd: Option<MavCmd>,
        expect_copter_mode: Option<u32>,
    ) -> Self {
        Self {
            label: label.into(),
            expect_cmd,
            expect_copter_mode,
            sent_at: Instant::now(),
            timeout_warned: false,
        }
    }
}

/// Log whether the MAVLink stack accepted the message for transmit; record pending FC feedback.
fn log_outgoing<T>(
    state: &mut TelemetryState,
    pending: PendingFeedback,
    send_result: Result<T, mavlink::error::MessageWriteError>,
) {
    match send_result.map(|_| ()) {
        Ok(()) => {
            state.push_recent(format!(
                "[1] TUI → link: OK ({} queued for send)",
                pending.label
            ));
            state.pending_feedback = Some(pending);
        }
        Err(e) => {
            state.push_recent(format!(
                "[1] TUI → link: FAILED ({}): {}",
                pending.label, e
            ));
            state.pending_feedback = None;
        }
    }
}

/// Two-step send (e.g. mode then mission start): first leg must succeed before second.
fn log_outgoing_two<T2>(
    state: &mut TelemetryState,
    label1: &str,
    r1: Result<(), mavlink::error::MessageWriteError>,
    pending2: PendingFeedback,
    r2: Result<T2, mavlink::error::MessageWriteError>,
) {
    match r1 {
        Ok(()) => state.push_recent(format!("[1] TUI → link: OK ({})", label1)),
        Err(e) => {
            state.push_recent(format!("[1] TUI → link: FAILED ({}): {}", label1, e));
            state.pending_feedback = None;
            return;
        }
    }
    log_outgoing(state, pending2, r2);
}

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

/// Short mode string for status bar (ArduCopter custom_mode).
fn format_mode_short(custom_mode: u32) -> &'static str {
    match custom_mode {
        0 => "STAB",
        3 => "AUTO",
        4 => "GUIDED",
        5 => "LOITER",
        6 => "RTL",
        9 => "LAND",
        _ => "MODE?",
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

#[derive(Default, Clone, Copy)]
struct NetWatchdogStatus {
    online: Option<bool>,
    last_check: Option<Instant>,
    last_ok: Option<Instant>,
    offline_since: Option<Instant>,
    rtl_sent_for_current_outage: bool,
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
    /// base_mode from HEARTBEAT (bits for MAV_MODE_FLAG).
    heartbeat_base_mode_bits: Option<u8>,
    armed: Option<bool>,
    sys_voltage: Option<f32>,
    sys_current: Option<f32>,
    sys_load: Option<u16>,
    time_boot_ms: Option<u32>,
    mission_waypoints: Vec<Waypoint>,
    mission_current_seq: Option<u16>,
    net_online: Option<bool>,
    net_secs_since_last_check: Option<u64>,
    net_secs_since_last_ok: Option<u64>,
    net_offline_secs: Option<u64>,
    net_rtl_sent_for_current_outage: bool,
    /// When true, draw the help popup (h to toggle).
    pub show_help_popup: bool,
    /// Last TUI command we are waiting to correlate with FC (COMMAND_ACK / mode).
    pending_feedback: Option<PendingFeedback>,
}

impl TelemetryState {
    fn push_recent(&mut self, line: String) {
        self.recent_messages.push_back(line);
        while self.recent_messages.len() > RECENT_MESSAGES_MAX {
            self.recent_messages.pop_front();
        }
    }
}

fn check_pending_feedback_timeout(state: &mut TelemetryState) {
    let should_warn = match &state.pending_feedback {
        Some(p) if !p.timeout_warned && p.sent_at.elapsed() >= PENDING_CMD_TIMEOUT => {
            Some(p.label.clone())
        }
        _ => None,
    };
    if let Some(label) = should_warn {
        if let Some(ref mut p) = state.pending_feedback {
            p.timeout_warned = true;
        }
        state.push_recent(format!(
            "[3] No FC reply yet for \"{}\" (no matching COMMAND_ACK / mode within {:?}). Check SYS/COMP in Vehicle, link, prearm, and other GCS.",
            label, PENDING_CMD_TIMEOUT
        ));
    }
}

/// Set target_system and target_component on a COMMAND_LONG message.
fn with_vehicle(mut msg: MavMessage, ids: VehicleIds) -> MavMessage {
    if let MavMessage::COMMAND_LONG(ref mut d) = msg {
        d.target_system = ids.system_id;
        d.target_component = ids.component_id;
    }
    msg
}

// Command builders (mirror mav-core::cmd; we use local mavlink 0.17 types).
const MODE_FLAG_CUSTOM_MODE_ENABLED: f32 = 1.0;
const ARDUCOPTER_MODE_GUIDED: f32 = 4.0;
const ARDUCOPTER_MODE_AUTO: f32 = 3.0;

fn cmd_arm() -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: 1.0,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        ..COMMAND_LONG_DATA::default()
    })
}
fn cmd_disarm() -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: 0.0,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        ..COMMAND_LONG_DATA::default()
    })
}
/// Send COMMAND_LONG mode change to GUIDED (ArduCopter custom_mode 4).
fn cmd_set_mode_guided_long<C>(
    conn: &mut C,
    ids: VehicleIds,
) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
        command: MavCmd::MAV_CMD_DO_SET_MODE,
        confirmation: 0,
        param1: MODE_FLAG_CUSTOM_MODE_ENABLED,
        param2: ARDUCOPTER_MODE_GUIDED,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });
    conn.send_default(&msg).map(|_| ())
}

/// Send COMMAND_LONG mode change to AUTO (ArduCopter custom_mode 3).
fn cmd_set_mode_auto_long<C>(
    conn: &mut C,
    ids: VehicleIds,
) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
        command: MavCmd::MAV_CMD_DO_SET_MODE,
        confirmation: 0,
        param1: MODE_FLAG_CUSTOM_MODE_ENABLED,
        param2: ARDUCOPTER_MODE_AUTO,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });
    conn.send_default(&msg).map(|_| ())
}

fn cmd_takeoff_alt(altitude_m: f32) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param7: altitude_m,
        command: MavCmd::MAV_CMD_NAV_TAKEOFF,
        ..COMMAND_LONG_DATA::default()
    })
}
/// COMMAND_LONG to start the loaded waypoint mission (follow preloaded waypoints).
fn cmd_mission_start(ids: VehicleIds) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
        command: MavCmd::MAV_CMD_MISSION_START,
        confirmation: 0,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        ..COMMAND_LONG_DATA::default()
    })
}

/// `MAV_COMP_ID_AUTOPILOT1` = 1. Companion / router heartbeats often use another component; if we
/// send `SET_MESSAGE_INTERVAL` there, the flight controller never streams GPS / SYS_STATUS / HUD.
fn heartbeat_from_autopilot(hdr: &MavFrame<MavMessage>, mavtype: MavType) -> bool {
    const MAV_COMP_ID_AUTOPILOT1: u8 = 1;
    if mavtype == MavType::MAV_TYPE_GCS {
        return false;
    }
    hdr.header.component_id == MAV_COMP_ID_AUTOPILOT1
}

fn request_stream_rates(connection: &impl MavConnection<MavMessage>, ids: VehicleIds) {
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
            target_system: ids.system_id,
            target_component: ids.component_id,
            confirmation: 0,
        };
        let _ = connection.send_default(&MavMessage::COMMAND_LONG(cmd));
    }
    // Legacy stream IDs (ArduPilot still honors these when message intervals are ignored).
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

/// Re-request mission list and message intervals (same as first handshake).
fn refresh_mavlink_streams(connection: &impl MavConnection<MavMessage>, ids: VehicleIds) {
    let req = mavlink::ardupilotmega::MISSION_REQUEST_LIST_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
    };
    let _ = connection.send_default(&MavMessage::MISSION_REQUEST_LIST(req));
    request_stream_rates(connection, ids);
}

fn internet_is_reachable() -> bool {
    // Use raw IP endpoints so this check does not depend on DNS availability.
    const TARGETS: [&str; 3] = ["1.1.1.1:53", "8.8.8.8:53", "1.1.1.1:443"];
    let timeout = Duration::from_millis(1200);
    TARGETS.iter().any(|target| {
        target
            .parse::<SocketAddr>()
            .ok()
            .map(|addr| TcpStream::connect_timeout(&addr, timeout).is_ok())
            .unwrap_or(false)
    })
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
            state.heartbeat_base_mode_bits = Some(d.base_mode.bits());
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
            // Fallback when COMMAND_ACK for DO_SET_MODE is missing: mode change still shows on stream.
            if let Some(p) = state.pending_feedback.take() {
                if p.expect_copter_mode == Some(d.custom_mode) {
                    state.push_recent(format!(
                        "[2] FC → telemetry: mode now {} ({})",
                        arducopter_mode_name(d.custom_mode),
                        p.label
                    ));
                } else {
                    state.pending_feedback = Some(p);
                }
            }
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
            let correlates = state
                .pending_feedback
                .as_ref()
                .and_then(|p| p.expect_cmd)
                .map(|c| c == d.command)
                .unwrap_or(false);
            let tag = if correlates {
                "[2] FC → COMMAND_ACK (matches last TUI command)"
            } else {
                "[2] FC → COMMAND_ACK (other / not last TUI key)"
            };
            state.push_recent(format!(
                "{} {:?} → {}",
                tag,
                d.command,
                mav_result_desc(d.result)
            ));
            if correlates {
                state.pending_feedback = None;
            }
            if d.command == MavCmd::MAV_CMD_COMPONENT_ARM_DISARM && d.result != MavResult::MAV_RESULT_ACCEPTED {
                state.push_recent("Tip: press g for GUIDED then a to arm, or f for force arm".to_string());
            }
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

fn draw_ui(
    f: &mut Frame,
    state: &TelemetryState,
    override_state: &Arc<Mutex<OverrideState>>,
    waypoint_input: Option<&str>,
) {
    // Side-by-side: left column (telemetry panels), right column (mission + messages)
    let main_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(38), Constraint::Percentage(62)])
        .split(f.area());

    let left = main_chunks[0];
    let right = main_chunks[1];

    // Left column: status bar + vertical stack of panels
    let left_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(1),
            Constraint::Length(7),
            Constraint::Length(4),
            Constraint::Length(5),
            Constraint::Length(4),
            Constraint::Length(4),
        ])
        .split(left);

    let status_line = {
        let sys = state
            .vehicle_sysid
            .map(|u| u.to_string())
            .unwrap_or_else(|| "?".to_string());
        let mode = state
            .heartbeat_custom
            .map(format_mode_short)
            .unwrap_or("?");
        let armed_str = state
            .armed
            .map(|b| if b { "yes" } else { "no" })
            .unwrap_or("?");
        let override_str = if let Ok(os) = override_state.lock() {
            match &*os {
                OverrideState::MissionRunning => String::new(),
                OverrideState::Paused => "  PAUSED (c=resume)".to_string(),
                OverrideState::OverrideActive { waypoints, index, .. } => {
                    format!("  OVERRIDE {}/{}", index + 1, waypoints.len())
                }
                OverrideState::Resuming { .. } => "  RESUMING".to_string(),
            }
        } else {
            String::new()
        };
        let net_str = match state.net_online {
            Some(true) => format!(
                "  NET=UP ok:{}s chk:{}s",
                state.net_secs_since_last_ok.unwrap_or(0),
                state.net_secs_since_last_check.unwrap_or(0)
            ),
            Some(false) => format!(
                "  NET=DOWN {}s{} chk:{}s",
                state.net_offline_secs.unwrap_or(0),
                if state.net_rtl_sent_for_current_outage {
                    " RTL_SENT"
                } else {
                    ""
                },
                state.net_secs_since_last_check.unwrap_or(0)
            ),
            None => "  NET=CHECKING".to_string(),
        };
        format!(
            "SYS={}  MODE={}  ARMED={}{}{}",
            sys, mode, armed_str, net_str, override_str
        )
    };
    f.render_widget(
        Paragraph::new(Line::from(Span::raw(status_line))).wrap(Wrap { trim: true }),
        left_chunks[0],
    );

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
        left_chunks[1],
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
        left_chunks[2],
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
        left_chunks[3],
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
        left_chunks[4],
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
        left_chunks[5],
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
        .title(" Messages [1]=TUI [2]=FC [3]=timeout | s=retry streams (h=help) ")
        .borders(Borders::ALL)
        .border_style(messages_style());
    f.render_widget(
        Paragraph::new(msg_lines).block(msg_block).wrap(Wrap { trim: true }),
        right_chunks[1],
    );

    if let Some(buf) = waypoint_input {
        let popup_w = 62_u16.min(f.area().width);
        let popup_h = 6_u16.min(f.area().height);
        let area = ratatui::layout::Rect {
            x: f.area().width.saturating_sub(popup_w) / 2,
            y: f.area().height.saturating_sub(popup_h) / 2,
            width: popup_w,
            height: popup_h,
        };
        f.render_widget(Clear, area);
        let block = Block::default()
            .title(" Override waypoint ")
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Yellow))
            .style(Style::default().bg(Color::Black));
        let inner = block.inner(area);
        f.render_widget(block, area);
        let popup_text = vec![
            Line::from(Span::styled(
                format!("  {}_", buf),
                Style::default().fg(Color::Yellow),
            )),
            Line::from(""),
            Line::from(Span::styled(
                "  lat lon alt (space-sep)  or  alt only.  Enter=go  Esc=cancel",
                Style::default().fg(Color::DarkGray),
            )),
        ];
        f.render_widget(
            Paragraph::new(popup_text).wrap(Wrap { trim: true }),
            inner,
        );
    }

    if state.show_help_popup {
        let help_bg = Color::White;
        let help_fg = Color::Black;
        let help_text = vec![
            Line::from(""),
            Line::from(Span::styled(" Keys (press h or Esc to close) ", Style::default().fg(Color::DarkGray).add_modifier(Modifier::BOLD))),
            Line::from(""),
            Line::from(Span::styled("  q     Quit the TUI", Style::default().fg(help_fg))),
            Line::from(Span::styled("  a     Arm motors (need GUIDED or armable mode)", Style::default().fg(help_fg))),
            Line::from(Span::styled("  d     Disarm motors", Style::default().fg(help_fg))),
            Line::from(Span::styled("  f     Force arm (bypasses some pre-arm checks)", Style::default().fg(help_fg))),
            Line::from(Span::styled("  g     Set mode GUIDED", Style::default().fg(help_fg))),
            Line::from(Span::styled("  u     Set mode AUTO", Style::default().fg(help_fg))),
            Line::from(Span::styled("  m     Set AUTO and start mission (follow", Style::default().fg(help_fg))),
            Line::from(Span::styled("         waypoints)", Style::default().fg(help_fg))),
            Line::from(Span::styled("  i     Interrupt: pause mission, hover here (c=resume)", Style::default().fg(help_fg))),
            Line::from(Span::styled("  w     Inject waypoint (during mission or when paused)", Style::default().fg(help_fg))),
            Line::from(Span::styled("         lat lon alt, or just alt. Then resume or stay paused", Style::default().fg(help_fg))),
            Line::from(Span::styled("  c     Resume mission (when paused or after override)", Style::default().fg(help_fg))),
            Line::from(Span::styled("  r     RTL (return to launch)", Style::default().fg(help_fg))),
            Line::from(Span::styled("  l     Land", Style::default().fg(help_fg))),
            Line::from(Span::styled("  t     Takeoff 10 m", Style::default().fg(help_fg))),
            Line::from(Span::styled("  s     Retry mission list + telemetry streams", Style::default().fg(help_fg))),
            Line::from(""),
            Line::from(Span::styled(" If arm fails: try g then a, or use f for force arm. ", Style::default().fg(Color::DarkGray))),
        ];
        let area = ratatui::layout::Rect {
            x: f.area().width.saturating_sub(52) / 2,
            y: f.area().height.saturating_sub(20) / 2,
            width: 52.min(f.area().width),
            height: 20.min(f.area().height),
        };
        f.render_widget(Clear, area);
        let block = Block::default()
            .title(" Help ")
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::DarkGray))
            .style(Style::default().bg(help_bg));
        let inner = block.inner(area);
        f.render_widget(block, area);
        f.render_widget(
            Paragraph::new(help_text)
                .wrap(Wrap { trim: true })
                .style(Style::default().bg(help_bg).fg(help_fg)),
            inner,
        );
    }
}

/// Parse "lat lon alt" (three numbers) or "alt" (one number; uses current lat/lon).
fn parse_waypoint_input(
    s: &str,
    current_lat: Option<f64>,
    current_lon: Option<f64>,
    _current_alt: Option<f64>,
) -> Result<(f64, f64, f64), String> {
    let parts: Vec<&str> = s.split_whitespace().collect();
    if parts.is_empty() {
        return Err("empty".to_string());
    }
    if parts.len() == 1 {
        let alt: f64 = parts[0].parse().map_err(|_| "alt must be a number")?;
        let lat = current_lat.ok_or("current position (GPS) needed for 'alt only'")?;
        let lon = current_lon.ok_or("current position (GPS) needed for 'alt only'")?;
        return Ok((lat, lon, alt));
    }
    if parts.len() != 3 {
        return Err("use: lat lon alt (space-sep), or just alt".to_string());
    }
    let lat: f64 = parts[0].parse().map_err(|_| "lat must be a number")?;
    let lon: f64 = parts[1].parse().map_err(|_| "lon must be a number")?;
    let alt: f64 = parts[2].parse().map_err(|_| "alt must be a number")?;
    if !(-90.0..=90.0).contains(&lat) {
        return Err("lat must be -90..90".to_string());
    }
    if !(-180.0..=180.0).contains(&lon) {
        return Err("lon must be -180..180".to_string());
    }
    Ok((lat, lon, alt))
}

fn horizontal_distance_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let lat1_rad = lat1.to_radians();
    let lat2_rad = lat2.to_radians();
    let a = dlat.sin().mul_add(
        dlat.sin(),
        dlon.sin() * dlon.sin() * lat1_rad.cos() * lat2_rad.cos(),
    );
    let a = a.min(1.0).max(0.0);
    let c = 2.0 * (1.0 - a).sqrt().atan2(a.sqrt());
    6371000.0 * c // Earth radius in meters
}

fn run_ui<C: MavConnection<MavMessage> + Send>(
    rx: mpsc::Receiver<MavFrame<MavMessage>>,
    log_rx: mpsc::Receiver<String>,
    stream_retry_tx: mpsc::Sender<()>,
    conn: Arc<Mutex<C>>,
    mission_store: Arc<Mutex<MissionStore>>,
    override_state: Arc<Mutex<OverrideState>>,
    net_watchdog_status: Arc<Mutex<NetWatchdogStatus>>,
) -> io::Result<()> {
    crossterm::terminal::enable_raw_mode()?;
    let backend = ratatui::backend::CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;
    crossterm::execute!(
        io::stdout(),
        crossterm::terminal::EnterAlternateScreen
    )?;

    let mut state = TelemetryState::default();
    let mut waypoint_input: Option<String> = None;

    loop {
        while let Ok(frame) = rx.try_recv() {
            apply_message(&mut state, &frame);
        }
        while let Ok(line) = log_rx.try_recv() {
            state.push_recent(line);
        }
        check_pending_feedback_timeout(&mut state);
        if let Ok(ns) = net_watchdog_status.lock() {
            let now = Instant::now();
            state.net_online = ns.online;
            state.net_secs_since_last_check =
                ns.last_check.map(|t| now.duration_since(t).as_secs());
            state.net_secs_since_last_ok = ns.last_ok.map(|t| now.duration_since(t).as_secs());
            state.net_offline_secs = ns.offline_since.map(|t| now.duration_since(t).as_secs());
            state.net_rtl_sent_for_current_outage = ns.rtl_sent_for_current_outage;
        }
        terminal.draw(|f| draw_ui(f, &state, &override_state, waypoint_input.as_deref()))?;
        if event::poll(std::time::Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if key.kind != KeyEventKind::Press {
                    continue;
                }
                if waypoint_input.is_some() {
                    match key.code {
                        KeyCode::Enter => {
                            let s = waypoint_input.take().unwrap_or_default();
                            let s = s.trim().to_string();
                            let (lat, lon, alt) = match parse_waypoint_input(&s, state.lat, state.lon, state.alt) {
                                Ok(t) => t,
                                Err(e) => {
                                    waypoint_input = Some(s);
                                    state.push_recent(format!("Waypoint parse: {}", e));
                                    continue;
                                }
                            };
                            let (ok, resume_after) = {
                                let mut os = match override_state.lock() {
                                    Ok(g) => g,
                                    Err(_) => continue,
                                };
                                if matches!(&*os, OverrideState::OverrideActive { .. }) {
                                    state.push_recent("Override: finish current override first.".to_string());
                                    continue;
                                }
                                let from_paused = matches!(&*os, OverrideState::Paused);
                                let resume_after = !from_paused; // from mission => resume after; from paused => stay paused after
                                if !from_paused {
                                    let mut store = match mission_store.lock() {
                                        Ok(g) => g,
                                        Err(_) => continue,
                                    };
                                    if !store.ensure_snapshot_for_pause() {
                                        state.push_recent("Override: no mission or current WP (wait for mission download).".to_string());
                                        continue;
                                    }
                                }
                                *os = OverrideState::OverrideActive {
                                    waypoints: vec![(lat, lon, alt)],
                                    index: 0,
                                    resume_after,
                                };
                                (true, resume_after)
                            };
                            if ok {
                                let ids = VehicleIds::new(
                                    state.vehicle_sysid.unwrap_or(TARGET_SYSTEM),
                                    state.vehicle_compid.unwrap_or(TARGET_COMPONENT),
                                );
                                if let Ok(mut c) = conn.lock() {
                                    let r1 = set_mode_guided(&mut *c, ids);
                                    let r2 = c.send_default(&goto_global_command_int(ids, lat, lon, alt));
                                    log_outgoing_two(
                                        &mut state,
                                        "GUIDED (SET_MODE)",
                                        r1,
                                        PendingFeedback::new(
                                            "Override waypoint (DO_REPOSITION)",
                                            Some(MavCmd::MAV_CMD_DO_REPOSITION),
                                            None,
                                        ),
                                        r2,
                                    );
                                    if resume_after {
                                        state.push_recent(format!(
                                            "Override: go to {:.5} {:.5} {:.0}m, then resume mission.",
                                            lat, lon, alt
                                        ));
                                    } else {
                                        state.push_recent(format!(
                                            "Override: go to {:.5} {:.5} {:.0}m, then hover (c=resume).",
                                            lat, lon, alt
                                        ));
                                    }
                                }
                            }
                        }
                        KeyCode::Esc => {
                            waypoint_input = None;
                            state.push_recent("Waypoint input cancelled.".to_string());
                        }
                        KeyCode::Backspace => {
                            if let Some(ref mut buf) = waypoint_input {
                                buf.pop();
                            }
                        }
                        KeyCode::Char(c) if !c.is_control() => {
                            if let Some(ref mut buf) = waypoint_input {
                                buf.push(c);
                            }
                        }
                        _ => {}
                    }
                    continue;
                }
                if state.show_help_popup {
                    if matches!(key.code, KeyCode::Char('h') | KeyCode::Char('q') | KeyCode::Esc) {
                        state.show_help_popup = false;
                    }
                } else {
                match key.code {
                    KeyCode::Char('q') => break,
                    KeyCode::Char('h') => state.show_help_popup = true,
                    KeyCode::Char('a') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(c) = conn.lock() {
                            let msg = with_vehicle(cmd_arm(), ids);
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "ARM",
                                    Some(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM),
                                    None,
                                ),
                                c.send_default(&msg),
                            );
                        }
                    }
                    KeyCode::Char('d') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(c) = conn.lock() {
                            let msg = with_vehicle(cmd_disarm(), ids);
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "DISARM",
                                    Some(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM),
                                    None,
                                ),
                                c.send_default(&msg),
                            );
                        }
                    }
                    KeyCode::Char('g') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(mut c) = conn.lock() {
                            let r = cmd_set_mode_guided_long(&mut *c, ids);
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "GUIDED (DO_SET_MODE)",
                                    Some(MavCmd::MAV_CMD_DO_SET_MODE),
                                    Some(4),
                                ),
                                r,
                            );
                        }
                    }
                    KeyCode::Char('u') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(mut c) = conn.lock() {
                            let r = cmd_set_mode_auto_long(&mut *c, ids);
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "AUTO (DO_SET_MODE)",
                                    Some(MavCmd::MAV_CMD_DO_SET_MODE),
                                    Some(3),
                                ),
                                r,
                            );
                        }
                    }
                    KeyCode::Char('m') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(mut c) = conn.lock() {
                            let r1 = cmd_set_mode_auto_long(&mut *c, ids);
                            let msg = cmd_mission_start(ids);
                            let r2 = c.send_default(&msg);
                            log_outgoing_two(
                                &mut state,
                                "AUTO (DO_SET_MODE)",
                                r1,
                                PendingFeedback::new(
                                    "MISSION_START",
                                    Some(MavCmd::MAV_CMD_MISSION_START),
                                    None,
                                ),
                                r2,
                            );
                        }
                    }
                    KeyCode::Char('t') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(c) = conn.lock() {
                            let msg = with_vehicle(cmd_takeoff_alt(10.0), ids);
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "TAKEOFF 10m",
                                    Some(MavCmd::MAV_CMD_NAV_TAKEOFF),
                                    None,
                                ),
                                c.send_default(&msg),
                            );
                        }
                    }
                    KeyCode::Char('s') => {
                        let _ = stream_retry_tx.send(());
                    }
                    KeyCode::Char('f') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(mut c) = conn.lock() {
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "FORCE_ARM",
                                    Some(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM),
                                    None,
                                ),
                                force_arm(&mut *c, ids),
                            );
                        }
                    }
                    KeyCode::Char('r') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(mut c) = conn.lock() {
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "RTL",
                                    Some(MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH),
                                    None,
                                ),
                                rtl(&mut *c, ids),
                            );
                        }
                    }
                    KeyCode::Char('l') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(mut c) = conn.lock() {
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "LAND",
                                    Some(MavCmd::MAV_CMD_NAV_LAND),
                                    None,
                                ),
                                land(&mut *c, ids),
                            );
                        }
                    }
                    KeyCode::Char('w') => {
                        if waypoint_input.is_none() {
                            // Allow when in AUTO (during mission) or Paused (interrupt), and not already running override waypoints
                            let in_override = override_state.lock().map(|g| matches!(&*g, OverrideState::OverrideActive { .. })).unwrap_or(false);
                            let can_waypoint = override_state.lock().map(|g| matches!(&*g, OverrideState::MissionRunning | OverrideState::Paused)).unwrap_or(false);
                            if !in_override && can_waypoint {
                                waypoint_input = Some(String::new());
                                state.push_recent("Enter waypoint: lat lon alt (space-sep), or just alt (m). Enter=go Esc=cancel".to_string());
                            } else if in_override {
                                state.push_recent("Waypoint: finish current override first.".to_string());
                            } else {
                                state.push_recent("Waypoint: start mission (u then m) or interrupt (i) first.".to_string());
                            }
                        }
                    }
                    KeyCode::Char('i') => {
                        // Interrupt: pause mission, hover here. Press 'c' to resume. Can press 'w' to inject a waypoint while paused.
                        // DO_REPOSITION uses MAV_FRAME_GLOBAL_RELATIVE_ALT so altitude must be relative to home, not AMSL.
                        if state.heartbeat_custom != Some(3) {
                            state.push_recent("Interrupt (i): switch to AUTO and start mission first.".to_string());
                            continue;
                        }
                        let (lat, lon, alt_rel) = match (state.lat, state.lon, state.alt, state.home_alt) {
                            (Some(la), Some(lo), Some(al), Some(home_al)) => (la, lo, al - home_al),
                            (Some(_), Some(_), None, _) | (None, _, _, _) | (_, None, _, _) => {
                                state.push_recent("Interrupt: no position (need GPS).".to_string());
                                continue;
                            }
                            (_, _, Some(_), None) => {
                                state.push_recent("Interrupt: need home position (wait for HOME_POSITION).".to_string());
                                continue;
                            }
                        };
                        let ok = {
                            let mut os = match override_state.lock() {
                                Ok(g) => g,
                                Err(_) => continue,
                            };
                            if matches!(&*os, OverrideState::OverrideActive { .. }) {
                                state.push_recent("Interrupt: finish current override first.".to_string());
                                continue;
                            }
                            let mut store = match mission_store.lock() {
                                Ok(g) => g,
                                Err(_) => continue,
                            };
                            if !store.ensure_snapshot_for_pause() {
                                state.push_recent("Interrupt: no mission or current WP (wait for mission download).".to_string());
                                continue;
                            }
                            *os = OverrideState::Paused;
                            true
                        };
                        if ok {
                            let ids = VehicleIds::new(
                                state.vehicle_sysid.unwrap_or(TARGET_SYSTEM),
                                state.vehicle_compid.unwrap_or(TARGET_COMPONENT),
                            );
                            if let Ok(mut c) = conn.lock() {
                                let r1 = set_mode_guided(&mut *c, ids);
                                let r2 = c.send_default(&goto_global_command_int(ids, lat, lon, alt_rel));
                                log_outgoing_two(
                                    &mut state,
                                    "GUIDED (SET_MODE)",
                                    r1,
                                    PendingFeedback::new(
                                        "Interrupt hover (DO_REPOSITION)",
                                        Some(MavCmd::MAV_CMD_DO_REPOSITION),
                                        None,
                                    ),
                                    r2,
                                );
                                state.push_recent(
                                    "Interrupt: hovering. Press c to resume mission, or w to inject waypoint."
                                        .to_string(),
                                );
                            }
                        }
                    }
                    KeyCode::Char('c') => {
                        // Cancel override: force resume mission now (get unstuck if stuck in OverrideActive/Resuming)
                        let (snapshot_items, resume_seq) = {
                            let store = match mission_store.lock() {
                                Ok(g) => g,
                                Err(_) => continue,
                            };
                            match store.get_snapshot() {
                                Some((items, seq)) => (items.to_vec(), seq),
                                None => {
                                    if override_state.lock().map(|g| !matches!(&*g, OverrideState::MissionRunning)).unwrap_or(false) {
                                        override_state.lock().ok().map(|mut g| *g = OverrideState::MissionRunning);
                                        state.push_recent("Override cancelled (no snapshot). State reset.".to_string());
                                    }
                                    continue;
                                }
                            }
                        };
                        let ids = VehicleIds::new(
                            state.vehicle_sysid.unwrap_or(TARGET_SYSTEM),
                            state.vehicle_compid.unwrap_or(TARGET_COMPONENT),
                        );
                        {
                            let mut store = mission_store.lock().unwrap();
                            store.set_upload_pending(snapshot_items.clone());
                        }
                        let count = snapshot_items.len() as u16;
                        if let Ok(c) = conn.lock() {
                            let _ = c.send_default(&MavMessage::MISSION_COUNT(
                                mavlink::ardupilotmega::MISSION_COUNT_DATA {
                                    count,
                                    target_system: ids.system_id,
                                    target_component: ids.component_id,
                                },
                            ));
                        }
                        if let Ok(mut st) = override_state.lock() {
                            *st = OverrideState::Resuming { resume_seq };
                        }
                        state.push_recent("Cancel override: resuming mission (upload + set current + AUTO).".to_string());
                    }
                    _ => {}
                }
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
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (mavlink_url, link_display) = match mavlink_connect::resolve_from_args(args) {
        Ok(v) => v,
        Err(MavlinkArgsError::Help) => {
            eprintln!("Usage: tui [OPTIONS]\n\n{}", mavlink_connect::usage_string());
            return;
        }
        Err(MavlinkArgsError::Invalid(m)) => {
            eprintln!("{m}");
            std::process::exit(2);
        }
    };

    eprintln!("MAVLink: {}", link_display);
    eprintln!("Waiting for first heartbeat...");
    eprintln!("Press h for help. q=quit.");

    let mut connection = match connect::<MavMessage>(&mavlink_url) {
        Ok(conn) => conn,
        Err(e) => {
            eprintln!("{}", mavlink_connect::open_error_message(&mavlink_url, &e));
            std::process::exit(1);
        }
    };
    mavlink_connect::tune_connection(&mut connection);

    let conn = Arc::new(Mutex::new(connection));
    let (tx, rx) = mpsc::channel();
    let (log_tx, log_rx) = mpsc::channel::<String>();
    let (stream_retry_tx, stream_retry_rx) = mpsc::channel::<()>();
    let mission_store = Arc::new(Mutex::new(MissionStore::new()));
    let override_state = Arc::new(Mutex::new(OverrideState::MissionRunning));
    let watchdog_vehicle_ids = Arc::new(Mutex::new(None::<VehicleIds>));
    let net_watchdog_status = Arc::new(Mutex::new(NetWatchdogStatus::default()));

    let recv_conn = Arc::clone(&conn);
    let recv_store = Arc::clone(&mission_store);
    let recv_override = Arc::clone(&override_state);
    let recv_watchdog_vehicle_ids = Arc::clone(&watchdog_vehicle_ids);
    let _recv_handle = thread::spawn(move || {
        let mut autopilot_handshake_done = false;
        let mut stream_auto_spawned = false;
        let coverage = Arc::new(Mutex::new(TelemetryCoverage::default()));
        let vehicle_ids_for_retry = Arc::new(Mutex::new(VehicleIds::default()));
        let mut mission_count: Option<u16> = None;
        let mut vehicle_ids = VehicleIds::default();
        loop {
            let mut manual_retry = false;
            while stream_retry_rx.try_recv().is_ok() {
                manual_retry = true;
            }
            if manual_retry {
                let ids = *vehicle_ids_for_retry.lock().unwrap();
                if let Ok(c) = recv_conn.lock() {
                    refresh_mavlink_streams(&*c, ids);
                    let _ = log_tx.send(
                        "Streams: manual retry (mission list + stream rates).".to_string(),
                    );
                }
            }

            let frame = match recv_conn.lock().unwrap().recv_frame() {
                Ok(f) => f,
                Err(_) => continue,
            };

            if let Ok(mut cov) = coverage.lock() {
                cov.update(&frame.msg);
            }

            if !autopilot_handshake_done {
                if let MavMessage::HEARTBEAT(d) = &frame.msg {
                    if heartbeat_from_autopilot(&frame, d.mavtype) {
                        autopilot_handshake_done = true;
                        vehicle_ids =
                            VehicleIds::new(frame.header.system_id, frame.header.component_id);
                        *vehicle_ids_for_retry.lock().unwrap() = vehicle_ids;
                        let c = recv_conn.lock().unwrap();
                        refresh_mavlink_streams(&*c, vehicle_ids);
                        if !stream_auto_spawned {
                            stream_auto_spawned = true;
                            let conn = Arc::clone(&recv_conn);
                            let cov = Arc::clone(&coverage);
                            let vid = Arc::clone(&vehicle_ids_for_retry);
                            let log = log_tx.clone();
                            thread::spawn(move || {
                                thread::sleep(STREAM_AUTO_RETRY_FIRST_DELAY);
                                for attempt in 1..=STREAM_AUTO_RETRY_MAX_ATTEMPTS {
                                    if cov.lock().unwrap().is_complete() {
                                        let _ = log.send(
                                            "Streams: telemetry complete (all key messages seen)."
                                                .to_string(),
                                        );
                                        return;
                                    }
                                    let ids = *vid.lock().unwrap();
                                    if let Ok(c) = conn.lock() {
                                        refresh_mavlink_streams(&*c, ids);
                                    }
                                    if attempt == 1
                                        || attempt % 5 == 0
                                        || attempt == STREAM_AUTO_RETRY_MAX_ATTEMPTS
                                    {
                                        let _ = log.send(format!(
                                            "Streams: auto-retry {}/{}…",
                                            attempt, STREAM_AUTO_RETRY_MAX_ATTEMPTS
                                        ));
                                    }
                                    thread::sleep(STREAM_AUTO_RETRY_INTERVAL);
                                }
                                let _ = log.send(
                                    "Streams: auto-retry stopped (incomplete). Press s to retry."
                                        .to_string(),
                                );
                            });
                        }
                    }
                }
            }
            if let MavMessage::HEARTBEAT(d) = &frame.msg {
                if heartbeat_from_autopilot(&frame, d.mavtype) {
                    let ids = VehicleIds::new(frame.header.system_id, frame.header.component_id);
                    vehicle_ids = ids;
                    *vehicle_ids_for_retry.lock().unwrap() = vehicle_ids;
                    if let Ok(mut g) = recv_watchdog_vehicle_ids.lock() {
                        *g = Some(ids);
                    }
                }
            }

            // Update mission store from FC
            if let MavMessage::MISSION_ITEM_INT(d) = &frame.msg {
                if let Ok(mut store) = recv_store.lock() {
                    store.update_from_item(d);
                }
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
                        let _ = recv_conn.lock().unwrap().send_default(&MavMessage::MISSION_REQUEST_INT(req));
                    } else {
                        mission_count = None;
                    }
                }
            }
            if let MavMessage::MISSION_CURRENT(d) = &frame.msg {
                if let Ok(mut store) = recv_store.lock() {
                    store.update_current_seq(d.seq);
                }
            }

            // Upload handshake: FC requested an item during our upload
            if let MavMessage::MISSION_REQUEST_INT(d) = &frame.msg {
                if let (Ok(mut store), Ok(conn_lock)) = (recv_store.lock(), recv_conn.lock()) {
                    if let Some(mut item) = store.take_upload_item(d.seq) {
                        item.target_system = frame.header.system_id;
                        item.target_component = frame.header.component_id;
                        let _ = conn_lock.send_default(&MavMessage::MISSION_ITEM_INT(item));
                    }
                }
            }
            if let MavMessage::MISSION_ACK(_) = &frame.msg {
                let resume_seq = if let Ok(state) = recv_override.lock() {
                    if let OverrideState::Resuming { resume_seq } = *state {
                        Some(resume_seq)
                    } else {
                        None
                    }
                } else {
                    None
                };
                if let Some(seq) = resume_seq {
                    if let Ok(mut store) = recv_store.lock() {
                        store.set_upload_done();
                    }
                    if let Ok(mut conn_lock) = recv_conn.lock() {
                        let _ = mission_set_current(&mut *conn_lock, vehicle_ids, seq);
                        let _ = set_mode_auto(&mut *conn_lock, vehicle_ids);
                        let _ = mission_start(&mut *conn_lock, vehicle_ids);
                    }
                    // Keep snapshot so multiple interrupts/waypoint injections work in the same session
                    if let Ok(mut state) = recv_override.lock() {
                        *state = OverrideState::MissionRunning;
                    }
                }
            }

            // Override: check if we reached current override waypoint
            if let MavMessage::GLOBAL_POSITION_INT(d) = &frame.msg {
                let lat = d.lat as f64 / 1e7;
                let lon = d.lon as f64 / 1e7;
                let mut state_guard = match recv_override.lock() {
                    Ok(g) => g,
                    Err(_) => continue,
                };
                if let OverrideState::OverrideActive { waypoints, index, resume_after } = &mut *state_guard {
                    if *index < waypoints.len() {
                        let (wp_lat, wp_lon, _wp_alt) = waypoints[*index];
                        let dist = horizontal_distance_m(lat, lon, wp_lat, wp_lon);
                        if dist < REACHED_THRESHOLD_M {
                            *index += 1;
                            if *index >= waypoints.len() {
                                if *resume_after {
                                    // Override done -> start resume mission
                                    let (snapshot_items, resume_seq) = {
                                        let store = match recv_store.lock() {
                                            Ok(s) => s,
                                            Err(_) => continue,
                                        };
                                        match store.get_snapshot() {
                                            Some((items, seq)) => (items.to_vec(), seq),
                                            None => continue,
                                        }
                                    };
                                    drop(state_guard);
                                    {
                                        let mut store = recv_store.lock().unwrap();
                                        store.set_upload_pending(snapshot_items.clone());
                                    }
                                    let count = snapshot_items.len() as u16;
                                    let _ = recv_conn.lock().unwrap().send_default(&MavMessage::MISSION_COUNT(
                                        mavlink::ardupilotmega::MISSION_COUNT_DATA {
                                            count,
                                            target_system: vehicle_ids.system_id,
                                            target_component: vehicle_ids.component_id,
                                        },
                                    ));
                                    if let Ok(mut st) = recv_override.lock() {
                                        *st = OverrideState::Resuming { resume_seq };
                                    }
                                } else {
                                    // Override done -> stay paused (hover at this position)
                                    drop(state_guard);
                                    if let Ok(mut st) = recv_override.lock() {
                                        *st = OverrideState::Paused;
                                    }
                                }
                            } else {
                                let (wl, wlon, walt) = waypoints[*index];
                                drop(state_guard);
                                let msg = goto_global_command_int(vehicle_ids, wl, wlon, walt);
                                let _ = recv_conn.lock().unwrap().send_default(&msg);
                            }
                        }
                    }
                }
            }

            if let MavMessage::MISSION_COUNT(d) = &frame.msg {
                if mission_count.is_none() && recv_store.lock().map(|s| s.upload_pending.is_none()).unwrap_or(true) {
                    mission_count = Some(d.count);
                    if d.count > 0 {
                        let sys = frame.header.system_id;
                        let comp = frame.header.component_id;
                        let req = mavlink::ardupilotmega::MISSION_REQUEST_INT_DATA {
                            target_system: sys,
                            target_component: comp,
                            seq: 0,
                        };
                        let _ = recv_conn.lock().unwrap().send_default(&MavMessage::MISSION_REQUEST_INT(req));
                    }
                }
            }

            let _ = tx.send(frame);
        }
    });

    let watchdog_conn = Arc::clone(&conn);
    let watchdog_vehicle_ids_thread = Arc::clone(&watchdog_vehicle_ids);
    let net_watchdog_status_thread = Arc::clone(&net_watchdog_status);
    let _net_watchdog_handle = thread::spawn(move || {
        loop {
            thread::sleep(Duration::from_secs(INTERNET_CHECK_PERIOD_SECS));
            let now = Instant::now();
            let online = internet_is_reachable();
            if let Ok(mut s) = net_watchdog_status_thread.lock() {
                s.last_check = Some(now);
                s.online = Some(online);
                if online {
                    s.last_ok = Some(now);
                    s.offline_since = None;
                    s.rtl_sent_for_current_outage = false;
                } else if s.offline_since.is_none() {
                    s.offline_since = Some(now);
                }
            }
            if online {
                continue;
            }
            let (offline_elapsed, already_sent) = match net_watchdog_status_thread.lock() {
                Ok(s) => (
                    s.offline_since
                        .map(|t| now.duration_since(t))
                        .unwrap_or(Duration::from_secs(0)),
                    s.rtl_sent_for_current_outage,
                ),
                Err(_) => continue,
            };
            if already_sent {
                continue;
            }
            if offline_elapsed < Duration::from_secs(INTERNET_OFFLINE_RTL_AFTER_SECS) {
                continue;
            }
            let ids = match watchdog_vehicle_ids_thread.lock() {
                Ok(g) => *g,
                Err(_) => None,
            };
            if let Some(ids) = ids {
                if let Ok(mut c) = watchdog_conn.lock() {
                    let _ = rtl(&mut *c, ids);
                    if let Ok(mut s) = net_watchdog_status_thread.lock() {
                        s.rtl_sent_for_current_outage = true;
                    }
                    eprintln!("Failsafe: internet offline >=30s, sent RTL.");
                }
            }
        }
    });

    if let Err(e) = run_ui(
        rx,
        log_rx,
        stream_retry_tx,
        conn,
        mission_store,
        override_state,
        net_watchdog_status,
    ) {
        eprintln!("UI error: {}", e);
        std::process::exit(1);
    }
}
