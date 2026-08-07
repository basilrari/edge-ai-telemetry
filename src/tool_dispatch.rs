//! Map LLM / gateway **drone** tool names to MAVLink sends (ArduCopter-oriented).
//!
//! Optional JSON **`params`** on `POST /v1/apply-tool` (same object the gateway forwards):
//! - **`arm`**: sends **arm** only (current flight mode unchanged). Use **`set_mode_guided`** / **`takeoff`** separately when needed.
//! - **`takeoff`**: optional `{"altitude_m": <m>}` (meters above home). `0` or omitted with no usable telemetry → **15 m**; omit with telemetry to use **current**
//!   altitude above home from telemetry (`GLOBAL_POSITION_INT`). **`MAV_CMD_NAV_TAKEOFF` only** — use after **`arm`**.
//! - **`mission_set_current`**: `{"seq": 0}` (required) — sets **current mission item index** on the FC (`DO_SET_MISSION_CURRENT`); does not upload a mission or replace **`start_mission`** for “fly the mission”.
//! - **`goto_location`**: `{"lat_deg":..,"lon_deg":..,"alt_m":..}` — `alt_m` is **relative to home**
//!   (same convention as TUI interrupt / `COMMAND_INT` DO_REPOSITION). Omitted or `<= 0` → **15 m**. **No** automatic takeoff; the LLM should emit **`arm`**, **`takeoff`**, then **`goto_location`** when starting from the ground.
//! - **`mission_interrupt`**: pause AUTO mission and hold (TUI `i`); needs GPS + home + recv thread.
//! - **`mission_resume`**: continue the **existing FC mission** after interrupt (AUTO + MISSION_START); **no upload**.
//! - **`waypoint_inject`**: guided goto; `{"lat_deg","lon_deg","alt_m"}` or `{"waypoint_text":"…"}` (TUI `w`). **No** automatic takeoff.
//! - **`move_forward`**: optional `{"speed_m_s": 3}` forward body-frame velocity.

#![allow(deprecated)]

use crate::mavlink_http_runtime::{
    altitude_above_home_from_params, resolve_takeoff_altitude_m, TelemetryCache,
};
use crate::{
    arm, disarm, force_arm, goto_global_command_int, land, mission_set_current, mission_start, rtl,
    set_mode_auto, set_mode_guided, set_mode_guided_long, takeoff_alt, with_vehicle, VehicleIds,
    CUSTOM_MODE_GUIDED,
};
use std::time::Duration;
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

fn set_arducopter_mode_long<C>(
    conn: &C,
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
    conn: &C,
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
///
/// For **`takeoff`**, pass `telem` so omitted `altitude_m` can use the vehicle's current height above home.
pub fn apply_llm_drone_tool<C>(
    conn: &C,
    ids: VehicleIds,
    tool: &str,
    params: &Value,
    telem: Option<&TelemetryCache>,
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
            let msg = with_vehicle(arm(), ids);
            conn.send_default(&msg).map(|_| ()).map_err(|e| e.to_string())
        }
        "disarm" => {
            let msg = with_vehicle(disarm(), ids);
            conn.send_default(&msg).map(|_| ()).map_err(|e| e.to_string())
        }
        "force_arm" => force_arm(conn, ids).map_err(|e| e.to_string()),
        "set_mode_auto" => set_mode_auto(conn, ids).map_err(|e| e.to_string()),
        "set_mode_guided" | "hover" => set_mode_guided_long(conn, ids).map_err(|e| e.to_string()),
        "takeoff" => {
            let alt = if params.get("altitude_m").is_some() {
                altitude_above_home_from_params(&params, "altitude_m")
            } else {
                let telem = telem.ok_or_else(|| {
                    "takeoff: altitude_m omitted; telemetry required for current altitude".to_string()
                })?;
                resolve_takeoff_altitude_m(&params, telem)?
            };
            let msg = with_vehicle(takeoff_alt(alt), ids);
            conn.send_default(&msg).map(|_| ()).map_err(|e| e.to_string())
        }
        "start_mission" => {
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
            let alt = altitude_above_home_from_params(&params, "alt_m") as f64;
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

fn request_mission_and_streams<C>(conn: &C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
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

pub fn wait_autopilot_heartbeat<C>(
    conn: &C,
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
        match conn.try_recv() {
            Ok((header, msg)) => {
                if let MavMessage::HEARTBEAT(d) = &msg {
                    if d.mavtype != MavType::MAV_TYPE_GCS && header.component_id == 1 {
                        return Ok(VehicleIds::new(header.system_id, header.component_id));
                    }
                }
            }
            Err(_) => {
                std::thread::sleep(Duration::from_millis(50));
            }
        }
    }
}

/// MAVLink command we expect the FC to ACK for a single-tool apply (HTTP `wait_for=ack`).
pub fn expected_ack_command(tool: &str) -> Option<MavCmd> {
    match tool {
        "arm" | "disarm" | "force_arm" => Some(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM),
        "takeoff" => Some(MavCmd::MAV_CMD_NAV_TAKEOFF),
        "land_immediately" => Some(MavCmd::MAV_CMD_NAV_LAND),
        "return_to_home" => Some(MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH),
        "goto_location" | "waypoint_inject" => Some(MavCmd::MAV_CMD_DO_REPOSITION),
        "set_mode_auto" | "set_mode_guided" | "hover" | "circle_search" => {
            Some(MavCmd::MAV_CMD_DO_SET_MODE)
        }
        "mission_set_current" => Some(MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT),
        "start_mission" => Some(MavCmd::MAV_CMD_MISSION_START),
        "move_forward" | "retry_streams" => None,
        _ => None,
    }
}
