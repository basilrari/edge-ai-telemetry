//! Helpers that build ArduPilotMega MAVLink command messages.

use mavlink::ardupilotmega::{COMMAND_INT_DATA, COMMAND_LONG_DATA, MavCmd, MavFrame, MavMessage};
use mavlink::{MavConnection, MavHeader};

/// Standard GCS identity (matches ArduPilot `SYSID_MYGCS` default and Mission Planner component).
pub const GCS_SYSTEM_ID: u8 = 255;
pub const GCS_COMPONENT_ID: u8 = 190;

pub fn gcs_header() -> MavHeader {
    MavHeader {
        system_id: GCS_SYSTEM_ID,
        component_id: GCS_COMPONENT_ID,
        sequence: 0,
    }
}

/// Send with explicit GCS header (avoids ambiguous component id 0 on mission protocol).
pub fn send_gcs<C>(conn: &C, msg: &MavMessage) -> Result<usize, mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    conn.send(&gcs_header(), msg)
}

/// `MAV_CMD_DO_SET_MODE` param1: select custom mode (ArduPilot convention).
const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: f32 = 1.0;

fn ardupilot_set_custom_mode<C>(
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
        param1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        param2: custom_mode,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });
    conn.send_default(&msg).map(|_| ())
}

/// ArduPilot Copter custom mode: Guided.
pub const CUSTOM_MODE_GUIDED: u32 = 4;
/// ArduPilot Copter custom mode: Auto.
pub const CUSTOM_MODE_AUTO: u32 = 3;

/// Build a COMMAND_LONG that arms the vehicle.
/// Caller should set `target_system` (and optionally `target_component`) before sending.
pub fn arm() -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: 1.0, // 1 = arm
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        ..COMMAND_LONG_DATA::default()
    })
}

/// Build a COMMAND_LONG that disarms the vehicle.
pub fn disarm() -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: 0.0, // 0 = disarm
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        ..COMMAND_LONG_DATA::default()
    })
}

/// Stamp `target_system` / `target_component` on a COMMAND_LONG (TUI `with_vehicle`).
pub fn with_vehicle(mut msg: MavMessage, ids: VehicleIds) -> MavMessage {
    if let MavMessage::COMMAND_LONG(ref mut d) = msg {
        d.target_system = ids.system_id;
        d.target_component = ids.component_id;
    }
    msg
}

/// TUI `g`: `MAV_CMD_DO_SET_MODE` → ArduCopter GUIDED (custom mode 4).
pub fn set_mode_guided_long<C>(conn: &C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    ardupilot_set_custom_mode(conn, ids, CUSTOM_MODE_GUIDED as f32)
}

/// TUI `u`: `MAV_CMD_DO_SET_MODE` → ArduCopter AUTO (custom mode 3).
pub fn set_mode_auto_long<C>(conn: &C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    ardupilot_set_custom_mode(conn, ids, CUSTOM_MODE_AUTO as f32)
}

/// Same as [`set_mode_guided_long`] (HTTP tools and interrupt path use this name).
pub fn set_mode_guided<C>(conn: &C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    set_mode_guided_long(conn, ids)
}

/// Same as [`set_mode_auto_long`].
pub fn set_mode_auto<C>(conn: &C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    set_mode_auto_long(conn, ids)
}

/// Default takeoff altitude in meters when not specified.
pub const DEFAULT_TAKEOFF_ALTITUDE_M: f32 = 10.0;

/// Build a COMMAND_LONG that commands takeoff.
/// Uses a default altitude of 10 m if not specified by the MAV_CMD_NAV_TAKEOFF semantics (param7).
pub fn takeoff() -> MavMessage {
    takeoff_alt(DEFAULT_TAKEOFF_ALTITUDE_M)
}

/// Build a COMMAND_LONG that commands takeoff to the given altitude (meters).
pub fn takeoff_alt(altitude_m: f32) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param7: altitude_m,
        command: MavCmd::MAV_CMD_NAV_TAKEOFF,
        ..COMMAND_LONG_DATA::default()
    })
}

/// Build a COMMAND_LONG that repositions the vehicle to a global position (guided).
/// Latitude and longitude in degrees; altitude in meters (e.g. AMSL or relative per frame).
/// Uses MAV_CMD_DO_REPOSITION (param5=lat, param6=lon, param7=alt).
/// Note: ArduCopter often rejects COMMAND_LONG for DO_REPOSITION (MAV_RESULT_COMMAND_INT_ONLY);
/// use [goto_global_command_int] for reliable guided reposition.
pub fn goto_global(lat_deg: f64, lon_deg: f64, altitude_m: f64) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param5: lat_deg as f32,
        param6: lon_deg as f32,
        param7: altitude_m as f32,
        command: MavCmd::MAV_CMD_DO_REPOSITION,
        ..COMMAND_LONG_DATA::default()
    })
}

/// Build COMMAND_INT for MAV_CMD_DO_REPOSITION (guided reposition). ArduCopter accepts this
/// when COMMAND_LONG may be rejected. Uses MAV_FRAME_GLOBAL_RELATIVE_ALT; param2=1 requests
/// transition to GUIDED. Caller must send the returned message.
pub fn goto_global_command_int(
    ids: VehicleIds,
    lat_deg: f64,
    lon_deg: f64,
    altitude_m: f64,
) -> MavMessage {
    MavMessage::COMMAND_INT(COMMAND_INT_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
        frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command: MavCmd::MAV_CMD_DO_REPOSITION,
        current: 0,
        autocontinue: 0,
        param1: -1.0,  // speed: -1 = default
        param2: 1.0,   // MAV_DO_REPOSITION_FLAGS_CHANGE_MODE: transition to GUIDED
        param3: 0.0,
        param4: f32::NAN, // yaw: use current
        x: (lat_deg * 1e7).round() as i32,
        y: (lon_deg * 1e7).round() as i32,
        z: altitude_m as f32,
        ..COMMAND_INT_DATA::default()
    })
}

/// Target system and component for command routing.
#[derive(Debug, Clone, Copy)]
pub struct VehicleIds {
    pub system_id: u8,
    pub component_id: u8,
}

impl Default for VehicleIds {
    fn default() -> Self {
        Self { system_id: 1, component_id: 1 }
    }
}

impl VehicleIds {
    pub const fn new(system_id: u8, component_id: u8) -> Self {
        Self { system_id, component_id }
    }
}

/// Send a force-arm command (param2 = 211 magic value) to the vehicle.
pub fn force_arm<C>(conn: &C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation: 0,
        param1: 1.0,   // arm
        param2: 211.0, // force arm magic value
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        ..COMMAND_LONG_DATA::default()
    });
    conn.send_default(&msg).map(|_| ())
}

/// Send RTL (return to launch) command to the vehicle.
pub fn rtl<C>(conn: &C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
        command: MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
        confirmation: 0,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        ..COMMAND_LONG_DATA::default()
    });
    conn.send_default(&msg).map(|_| ())
}

/// Send land command to the vehicle.
pub fn land<C>(conn: &C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
        command: MavCmd::MAV_CMD_NAV_LAND,
        confirmation: 0,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        ..COMMAND_LONG_DATA::default()
    });
    conn.send_default(&msg).map(|_| ())
}

/// Set the current mission item index (MAV_CMD_DO_SET_MISSION_CURRENT). Use when resuming after override.
pub fn mission_set_current<C>(
    conn: &C,
    ids: VehicleIds,
    seq: u16,
) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
        command: MavCmd::MAV_CMD_DO_SET_MISSION_CURRENT,
        confirmation: 0,
        param1: seq as f32,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        ..COMMAND_LONG_DATA::default()
    });
    conn.send_default(&msg).map(|_| ())
}

/// MAV_CMD_MISSION_START as a message (TUI `m` after AUTO).
pub fn mission_start_message(ids: VehicleIds) -> MavMessage {
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

/// Start mission execution (MAV_CMD_MISSION_START). Call after set_mode_auto and mission_set_current when resuming.
pub fn mission_start<C>(conn: &C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    conn.send_default(&mission_start_message(ids)).map(|_| ())
}

/// Resume AUTO mission on the FC without uploading mission items (prompt/override path only).
pub fn resume_mission_execution<C>(conn: &C, ids: VehicleIds, resume_seq: u16) -> Result<(), String>
where
    C: MavConnection<MavMessage>,
{
    set_mode_auto(conn, ids).map_err(|e| e.to_string())?;
    mission_set_current(conn, ids, resume_seq).map_err(|e| e.to_string())?;
    mission_start(conn, ids).map_err(|e| e.to_string())
}
