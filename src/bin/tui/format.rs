//! Pure string/formatting helpers for MAVLink and mission display.

use mavlink::ardupilotmega::{MavModeFlag, MavResult, MavState, MavType};
use mavlink::ardupilotmega::GpsFixType;

use crate::state::Waypoint;

pub(crate) fn mav_result_desc(r: MavResult) -> String {
    format!("{:?}", r)
}

pub(crate) fn rad_to_deg(rad: f32) -> f32 {
    rad.to_degrees()
}

pub(crate) fn mav_state_short(s: MavState) -> &'static str {
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

pub(crate) fn gps_fix_short(f: GpsFixType) -> &'static str {
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

pub(crate) fn mav_mode_flags_short(m: MavModeFlag) -> String {
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

pub(crate) fn arducopter_mode_name(custom_mode: u32) -> &'static str {
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
pub(crate) fn format_mode_short(custom_mode: u32) -> &'static str {
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

pub(crate) fn mav_type_short(t: MavType) -> String {
    let s = format!("{:?}", t);
    s.replace("MAV_TYPE_", "")
}

pub(crate) fn is_armed(base_mode: MavModeFlag) -> bool {
    base_mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED)
}

/// Format time_boot_ms as human-readable uptime (e.g. "14m32s" or "2h05m").
pub(crate) fn format_uptime_ms(ms: u32) -> String {
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

pub(crate) fn mav_cmd_short(cmd: u16) -> &'static str {
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
pub(crate) fn alt_frame_short(frame: u8) -> &'static str {
    match frame {
        0 => "AMSL",
        3 => "rel",
        10 => "terrain",
        _ => "?",
    }
}

pub(crate) fn waypoint_line(wp: &Waypoint, current_seq: Option<u16>) -> String {
    let prefix = if current_seq == Some(wp.seq) { "*" } else { " " };
    let cmd = mav_cmd_short(wp.command);
    let alt_suffix = alt_frame_short(wp.frame);
    match wp.command {
        22 => format!("{}{}: {} alt={:.0}m {} (target)", prefix, wp.seq, cmd, wp.alt, alt_suffix), // TAKEOFF
        16 | 21 | 31 | 82 => format!(
            "{}{}: {} lat={:.5} lon={:.5} alt={:.0}m {} (target)",
            prefix, wp.seq, cmd, wp.lat, wp.lon, wp.alt, alt_suffix
        ),
        _ => format!(
            "{}{}: {} lat={:.5} lon={:.5} alt={:.0}m {} (target)",
            prefix, wp.seq, cmd, wp.lat, wp.lon, wp.alt, alt_suffix
        ),
    }
}
