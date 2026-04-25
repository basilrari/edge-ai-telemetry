//! Map LLM / gateway **drone** tool names to MAVLink sends (ArduCopter-oriented).

#![allow(deprecated)]

use crate::VehicleIds;
use crate::{land, rtl, set_mode_guided};
use mavlink::ardupilotmega::{
    COMMAND_LONG_DATA, MavCmd, MavFrame, MavMessage, MavType, PositionTargetTypemask,
    SET_POSITION_TARGET_LOCAL_NED_DATA,
};
use mavlink::MavConnection;

/// Tool names returned by the gateway LLM prompt (`gateway/src/llm.rs`).
pub const LLM_DRONE_TOOL_NAMES: &[&str] = &[
    "move_forward",
    "hover",
    "return_to_home",
    "land_immediately",
    "circle_search",
];

const MODE_FLAG_CUSTOM_MODE_ENABLED: f32 = 1.0;
/// ArduCopter `CIRCLE` mode (orbit about point ahead of vehicle).
const ARDUCOPTER_MODE_CIRCLE: f32 = 7.0;

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

/// Body-frame forward velocity (NED body); requires **Guided** (or position mode).
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

/// Apply a single gateway/LLM drone tool name over an existing MAVLink connection.
pub fn apply_llm_drone_tool<C>(
    conn: &mut C,
    ids: VehicleIds,
    tool: &str,
) -> Result<(), String>
where
    C: MavConnection<MavMessage>,
{
    match tool {
        "return_to_home" => rtl(conn, ids).map_err(|e| e.to_string()),
        "land_immediately" => land(conn, ids).map_err(|e| e.to_string()),
        "hover" => set_mode_guided(conn, ids).map_err(|e| e.to_string()),
        "move_forward" => {
            set_mode_guided(conn, ids)
                .map_err(|e| e.to_string())
                .and_then(|_| {
                    send_body_forward_velocity(conn, ids, 3.0).map_err(|e| e.to_string())
                })
        }
        "circle_search" => set_arducopter_mode_long(conn, ids, ARDUCOPTER_MODE_CIRCLE)
            .map_err(|e| e.to_string()),
        other => Err(format!("unknown_drone_tool:{other}")),
    }
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
