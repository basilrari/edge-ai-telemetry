//! Apply incoming MAVLink to `TelemetryState` and TUI command-send feedback.

use mavlink::ardupilotmega::{MavCmd, MavMessage, MavResult};
use mavlink::MavFrame;

use crate::consts::{PENDING_CMD_TIMEOUT, U16_MAX};
use crate::format::{
    arducopter_mode_name, gps_fix_short, is_armed, mav_mode_flags_short, mav_result_desc,
    mav_state_short, mav_type_short, rad_to_deg,
};
use crate::state::{PendingFeedback, TelemetryState, Waypoint};

fn statustext_to_str(d: &mavlink::ardupilotmega::STATUSTEXT_DATA) -> String {
    d.text
        .to_str()
        .unwrap_or("")
        .trim()
        .to_string()
}

/// Log whether the MAVLink stack accepted the message for transmit; record pending FC feedback.
pub(crate) fn log_outgoing<T>(
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
pub(crate) fn log_outgoing_two<T2>(
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

pub(crate) fn check_pending_feedback_timeout(state: &mut TelemetryState) {
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

pub(crate) fn apply_message(state: &mut TelemetryState, frame: &MavFrame<MavMessage>) {
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
                    if rest.contains("ArduCopter")
                        || rest.contains("ArduPlane")
                        || rest.contains("ArduRover")
                    {
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
            if d.command == MavCmd::MAV_CMD_COMPONENT_ARM_DISARM
                && d.result != MavResult::MAV_RESULT_ACCEPTED
            {
                state.push_recent(
                    "Tip: press g for GUIDED then a to arm, or f for force arm".to_string(),
                );
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
