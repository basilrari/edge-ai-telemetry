//! Map LLM / gateway **drone** tool names to MAVLink sends (ArduCopter-oriented).
//!
//! Optional JSON **`params`** on `POST /v1/apply-tool` (same object the gateway forwards):
//! - **`takeoff`**: `{"altitude_m": 10}` (default 10 m, matches TUI `t`).
//! - **`mission_set_current`**: `{"seq": 0}` (required).
//! - **`goto_location`**: `{"lat_deg":..,"lon_deg":..,"alt_m":..}` — `alt_m` is **relative to home**
//!   (same convention as TUI interrupt / `COMMAND_INT` DO_REPOSITION).
//! - **`mission_interrupt`**: pause AUTO mission and hold (TUI `i`); needs GPS + home + recv thread.
//! - **`mission_resume`**: upload snapshot and resume (TUI `c`); recv completes on `MISSION_ACK`.
//! - **`waypoint_inject`**: guided goto; `{"lat_deg","lon_deg","alt_m"}` or `{"waypoint_text":"…"}` (TUI `w`).
//! - **`move_forward`**: optional `{"speed_m_s": 3}` forward body-frame velocity.

#![allow(deprecated)]

use crate::{
    arm, disarm, force_arm, goto_global_command_int, land, mission_set_current, mission_start, rtl,
    set_mode_auto, set_mode_guided, takeoff_alt, VehicleIds, DEFAULT_TAKEOFF_ALTITUDE_M,
};
use mavlink::ardupilotmega::{
    COMMAND_LONG_DATA, MavCmd, MavFrame, MavMessage, MavType, PositionTargetTypemask,
    SET_POSITION_TARGET_LOCAL_NED_DATA,
};
use mavlink::MavConnection;
use serde_json::Value;

/// Tool names accepted by [`apply_llm_drone_tool`] (also listed on `GET /health`).
pub const LLM_DRONE_TOOL_NAMES: &[&str] = &[
    "arm",
    "disarm",
    "force_arm",
    "set_mode_auto",
    "set_mode_guided",
    "takeoff",
    "start_mission",
    "mission_set_current",
    "goto_location",
    "move_forward",
    "hover",
    "return_to_home",
    "land_immediately",
    "circle_search",
    "retry_streams",
    "mission_interrupt",
    "mission_resume",
    "waypoint_inject",
];

const MODE_FLAG_CUSTOM_MODE_ENABLED: f32 = 1.0;
const ARDUCOPTER_MODE_CIRCLE: f32 = 7.0;

fn set_command_long_targets(mut msg: MavMessage, ids: VehicleIds) -> MavMessage {
    if let MavMessage::COMMAND_LONG(ref mut d) = msg {
        d.target_system = ids.system_id;
        d.target_component = ids.component_id;
    }
    msg
}

fn set_arducopter_mode_long<C>(
    conn: &mut C,
    ids: VehicleIds,
    custom_mode: f32,
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
        param2: custom_mode,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });
    conn.send_default(&msg).map(|_| ())
}

fn send_body_forward_velocity<C>(
    conn: &mut C,
    ids: VehicleIds,
    vx_m_s: f32,
) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    let type_mask = PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Y_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Z_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VY_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AX_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AY_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    let msg = MavMessage::SET_POSITION_TARGET_LOCAL_NED(SET_POSITION_TARGET_LOCAL_NED_DATA {
        time_boot_ms: 0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        vx: vx_m_s,
        vy: 0.0,
        vz: 0.0,
        afx: 0.0,
        afy: 0.0,
        afz: 0.0,
        yaw: 0.0,
        yaw_rate: 0.0,
        type_mask,
        target_system: ids.system_id,
        target_component: ids.component_id,
        coordinate_frame: MavFrame::MAV_FRAME_BODY_NED,
    });
    conn.send_default(&msg).map(|_| ())
}

fn f32_param(params: &Value, key: &str, default: f32) -> f32 {
    params
        .get(key)
        .and_then(|v| v.as_f64())
        .map(|x| x as f32)
        .unwrap_or(default)
}

/// Apply one tool; `params` is usually `{}` from the LLM path, or carries `seq` / positions for operator/API calls.
pub fn apply_llm_drone_tool<C>(
    conn: &mut C,
    ids: VehicleIds,
    tool: &str,
    params: &Value,
) -> Result<(), String>
where
    C: MavConnection<MavMessage>,
{
    let params = match params {
        Value::Object(m) => Value::Object(m.clone()),
        Value::Null => Value::Object(Default::default()),
        _ => Value::Object(Default::default()),
    };

    match tool {
        "arm" => {
            let msg = set_command_long_targets(arm(), ids);
            conn.send_default(&msg).map(|_| ()).map_err(|e| e.to_string())
        }
        "disarm" => {
            let msg = set_command_long_targets(disarm(), ids);
            conn.send_default(&msg).map(|_| ()).map_err(|e| e.to_string())
        }
        "force_arm" => force_arm(conn, ids).map_err(|e| e.to_string()),
        "set_mode_auto" => set_mode_auto(conn, ids).map_err(|e| e.to_string()),
        "set_mode_guided" | "hover" => set_mode_guided(conn, ids).map_err(|e| e.to_string()),
        "takeoff" => {
            let alt = f32_param(&params, "altitude_m", DEFAULT_TAKEOFF_ALTITUDE_M);
            let msg = set_command_long_targets(takeoff_alt(alt), ids);
            conn.send_default(&msg).map(|_| ()).map_err(|e| e.to_string())
        }
        "start_mission" => {
            // TUI `m`: AUTO then MISSION_START (no local mission validation; FC may reject).
            set_mode_auto(conn, ids)
                .map_err(|e| e.to_string())
                .and_then(|_| mission_start(conn, ids).map_err(|e| e.to_string()))
        }
        "mission_set_current" => {
            let seq = params
                .get("seq")
                .and_then(|v| v.as_u64())
                .ok_or_else(|| {
                    "mission_set_current requires params.seq (u64), e.g. {\"seq\":0}".to_string()
                })?;
            if seq > u16::MAX as u64 {
                return Err("mission_set_current: seq out of range".into());
            }
            mission_set_current(conn, ids, seq as u16).map_err(|e| e.to_string())
        }
        "goto_location" => {
            let lat = params
                .get("lat_deg")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| "goto_location requires params.lat_deg".to_string())?;
            let lon = params
                .get("lon_deg")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| "goto_location requires params.lon_deg".to_string())?;
            let alt = params
                .get("alt_m")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| {
                    "goto_location requires params.alt_m (relative to home, meters)".to_string()
                })?;
            let msg = goto_global_command_int(ids, lat, lon, alt);
            conn.send_default(&msg).map(|_| ()).map_err(|e| e.to_string())
        }
        "return_to_home" => rtl(conn, ids).map_err(|e| e.to_string()),
        "land_immediately" => land(conn, ids).map_err(|e| e.to_string()),
        "move_forward" => {
            let vx = f32_param(&params, "speed_m_s", 3.0);
            set_mode_guided(conn, ids)
                .map_err(|e| e.to_string())
                .and_then(|_| send_body_forward_velocity(conn, ids, vx).map_err(|e| e.to_string()))
        }
        "circle_search" => set_arducopter_mode_long(conn, ids, ARDUCOPTER_MODE_CIRCLE)
            .map_err(|e| e.to_string()),
        "retry_streams" => request_mission_and_streams(conn, ids).map_err(|e| e.to_string()),
        other => Err(format!("unknown_drone_tool:{other}")),
    }
}

/// Best-effort nudge like TUI `s`: re-request mission list + a few data streams.
fn request_mission_and_streams<C>(conn: &mut C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    use mavlink::ardupilotmega::{MISSION_REQUEST_LIST_DATA, REQUEST_DATA_STREAM_DATA};

    let _ = conn.send_default(&MavMessage::MISSION_REQUEST_LIST(MISSION_REQUEST_LIST_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
    }));

    for stream_id in 0u8..=6u8 {
        let req = REQUEST_DATA_STREAM_DATA {
            req_message_rate: 4,
            target_system: ids.system_id,
            target_component: ids.component_id,
            req_stream_id: stream_id,
            start_stop: 1,
        };
        let _ = conn.send_default(&MavMessage::REQUEST_DATA_STREAM(req));
    }
    Ok(())
}

/// Wait until we see a non-GCS HEARTBEAT from the autopilot component (same heuristic as `raw` / TUI).
pub fn wait_autopilot_heartbeat<C>(
    conn: &mut C,
    timeout: std::time::Duration,
) -> Result<VehicleIds, String>
where
    C: MavConnection<MavMessage>,
{
    let deadline = std::time::Instant::now() + timeout;
    loop {
        if std::time::Instant::now() > deadline {
            return Err(format!(
                "no autopilot HEARTBEAT within {:?}; check MAVLink link (UDP 14550 or --serial)",
                timeout
            ));
        }
        match conn.recv_frame() {
            Ok(frame) => {
                if let MavMessage::HEARTBEAT(d) = &frame.msg {
                    if d.mavtype != MavType::MAV_TYPE_GCS && frame.header.component_id == 1 {
                        return Ok(VehicleIds::new(
                            frame.header.system_id,
                            frame.header.component_id,
                        ));
                    }
                }
            }
            Err(e) => {
                return Err(format!("MAVLink recv while waiting for heartbeat: {e}"));
            }
        }
    }
}
