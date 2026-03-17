//! Helpers that build ArduPilotMega MAVLink command messages.

#[allow(deprecated)]
use mavlink::ardupilotmega::{
    COMMAND_LONG_DATA, MavCmd, MavMessage, MavMode, MavModeFlag, SET_MODE_DATA,
};
use mavlink::MavConnection;

fn base_mode_guided() -> u8 {
    (MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED.bits()
        | MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED.bits()
        | MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED.bits()
        | MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED.bits()) as u8
}

fn base_mode_auto() -> u8 {
    (MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED.bits()
        | MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED.bits()
        | MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED.bits()
        | MavModeFlag::MAV_MODE_FLAG_AUTO_ENABLED.bits()) as u8
}

fn to_set_mode_base_mode(base_mode: u8) -> MavMode {
    match base_mode {
        // 0b0101_1000 = custom + guided + stabilize + manual input.
        89 => MavMode::MAV_MODE_GUIDED_DISARMED,
        // 0b0101_1100 = custom + auto + stabilize + manual input.
        93 => MavMode::MAV_MODE_AUTO_DISARMED,
        _ => MavMode::MAV_MODE_PREFLIGHT,
    }
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

/// Set vehicle mode to Guided (ArduCopter custom_mode 4) via SET_MODE message.
#[allow(deprecated)]
pub fn set_mode_guided<C>(conn: &mut C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    let base_mode = to_set_mode_base_mode(base_mode_guided());
    let msg = MavMessage::SET_MODE(SET_MODE_DATA {
        target_system: ids.system_id,
        base_mode,
        custom_mode: CUSTOM_MODE_GUIDED,
    });
    conn.send_default(&msg).map(|_| ())
}

/// Set vehicle mode to Auto (ArduCopter custom_mode 3) via SET_MODE message.
#[allow(deprecated)]
pub fn set_mode_auto<C>(conn: &mut C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    let base_mode = to_set_mode_base_mode(base_mode_auto());
    let msg = MavMessage::SET_MODE(SET_MODE_DATA {
        target_system: ids.system_id,
        base_mode,
        custom_mode: CUSTOM_MODE_AUTO,
    });
    conn.send_default(&msg).map(|_| ())
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
pub fn goto_global(lat_deg: f64, lon_deg: f64, altitude_m: f64) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param5: lat_deg as f32,
        param6: lon_deg as f32,
        param7: altitude_m as f32,
        command: MavCmd::MAV_CMD_DO_REPOSITION,
        ..COMMAND_LONG_DATA::default()
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
pub fn force_arm<C>(conn: &mut C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
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
pub fn rtl<C>(conn: &mut C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
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
pub fn land<C>(conn: &mut C, ids: VehicleIds) -> Result<(), mavlink::error::MessageWriteError>
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
