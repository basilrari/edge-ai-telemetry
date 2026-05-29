//! Format MAVLink frames for the dashboard Pixhawk log panel (TUI/raw-style summaries).

#![allow(deprecated)]

use mavlink::ardupilotmega::{GpsFixType, MavMessage, MavModeFlag};
use mavlink::MavFrame;

use crate::mavlink_streams::heartbeat_from_autopilot;

#[derive(Clone, Debug, serde::Serialize)]
pub struct MavlinkLogEntry {
    pub ts_ms: u64,
    pub msg_id: u32,
    pub msg_name: String,
    pub value: String,
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

fn arducopter_mode_name(custom_mode: u32) -> &'static str {
    match custom_mode {
        0 => "STABILIZE",
        1 => "ACRO",
        2 => "ALT_HOLD",
        3 => "AUTO",
        4 => "GUIDED",
        5 => "LOITER",
        6 => "RTL",
        7 => "CIRCLE",
        9 => "LAND",
        19 => "SMART_RTL",
        _ => "MODE",
    }
}

/// Build a log row for high-signal MAVLink messages (returns `None` for unhandled types).
pub fn format_mavlink_frame(frame: &MavFrame<MavMessage>) -> Option<MavlinkLogEntry> {
    let ts_ms = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);

    let (msg_id, msg_name, value) = match &frame.msg {
        MavMessage::HEARTBEAT(d) if heartbeat_from_autopilot(frame, d.mavtype) => {
            let armed = d
                .base_mode
                .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
            (
                0,
                "HEARTBEAT".to_string(),
                format!(
                    "mode={} armed={}",
                    arducopter_mode_name(d.custom_mode),
                    armed
                ),
            )
        }
        MavMessage::GLOBAL_POSITION_INT(d) => (
            33,
            "GLOBAL_POSITION_INT".into(),
            format!(
                "lat={:.5} lon={:.5} alt={:.1}m rel={:.1}m",
                d.lat as f64 / 1e7,
                d.lon as f64 / 1e7,
                d.alt as f64 / 1000.0,
                d.relative_alt as f64 / 1000.0
            ),
        ),
        MavMessage::ATTITUDE(d) => (
            30,
            "ATTITUDE".into(),
            format!(
                "roll={:.1}° pitch={:.1}° yaw={:.1}°",
                d.roll.to_degrees(),
                d.pitch.to_degrees(),
                d.yaw.to_degrees()
            ),
        ),
        MavMessage::VFR_HUD(d) => (
            74,
            "VFR_HUD".into(),
            format!(
                "gs={:.1} as={:.1} hdg={}° alt={:.1}m climb={:.1}",
                d.groundspeed, d.airspeed, d.heading, d.alt, d.climb
            ),
        ),
        MavMessage::SYS_STATUS(d) => {
            let v = if d.voltage_battery != u16::MAX {
                format!("{:.1}V", d.voltage_battery as f32 / 100.0)
            } else {
                "—".into()
            };
            let a = if d.current_battery >= 0 {
                format!("{:.1}A", d.current_battery as f32 / 100.0)
            } else {
                "—".into()
            };
            let pct = if d.battery_remaining >= 0 {
                format!("{}%", d.battery_remaining)
            } else {
                "—".into()
            };
            (
                1,
                "SYS_STATUS".into(),
                format!("vbat={v} curr={a} rem={pct}"),
            )
        }
        MavMessage::GPS_RAW_INT(d) => (
            24,
            "GPS_RAW_INT".into(),
            format!(
                "fix={} sats={} lat={:.5} lon={:.5}",
                gps_fix_short(d.fix_type),
                d.satellites_visible,
                d.lat as f64 / 1e7,
                d.lon as f64 / 1e7
            ),
        ),
        MavMessage::MISSION_CURRENT(d) => (
            42,
            "MISSION_CURRENT".into(),
            format!("seq={}", d.seq),
        ),
        MavMessage::STATUSTEXT(d) => {
            let text = d
                .text
                .to_str()
                .unwrap_or("")
                .trim()
                .trim_end_matches('\0');
            if text.is_empty() {
                return None;
            }
            (253, "STATUSTEXT".into(), text.to_string())
        }
        MavMessage::COMMAND_ACK(d) => (
            77,
            "COMMAND_ACK".into(),
            format!("cmd={:?} result={:?}", d.command, d.result),
        ),
        _ => return None,
    };

    Some(MavlinkLogEntry {
        ts_ms,
        msg_id,
        msg_name,
        value,
    })
}
