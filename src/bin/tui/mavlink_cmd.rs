//! Local MAVLink command builders and stream rate requests (TUI binary).

use drone_server::VehicleIds;
use mavlink::ardupilotmega::{COMMAND_LONG_DATA, MavCmd, MavMessage};
use mavlink::MavConnection;

// Command builders (mirror mav-core::cmd; we use local mavlink 0.17 types).
const MODE_FLAG_CUSTOM_MODE_ENABLED: f32 = 1.0;
const ARDUCOPTER_MODE_GUIDED: f32 = 4.0;
const ARDUCOPTER_MODE_AUTO: f32 = 3.0;

/// Set target_system and target_component on a COMMAND_LONG message.
pub(crate) fn with_vehicle(mut msg: MavMessage, ids: VehicleIds) -> MavMessage {
    if let MavMessage::COMMAND_LONG(ref mut d) = msg {
        d.target_system = ids.system_id;
        d.target_component = ids.component_id;
    }
    msg
}

pub(crate) fn cmd_arm() -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: 1.0,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        ..COMMAND_LONG_DATA::default()
    })
}

pub(crate) fn cmd_disarm() -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: 0.0,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        ..COMMAND_LONG_DATA::default()
    })
}

/// Send COMMAND_LONG mode change to GUIDED (ArduCopter custom_mode 4).
pub(crate) fn cmd_set_mode_guided_long<C>(
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
pub(crate) fn cmd_set_mode_auto_long<C>(
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

pub(crate) fn cmd_takeoff_alt(altitude_m: f32) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param7: altitude_m,
        command: MavCmd::MAV_CMD_NAV_TAKEOFF,
        ..COMMAND_LONG_DATA::default()
    })
}

/// COMMAND_LONG to start the loaded waypoint mission (follow preloaded waypoints).
pub(crate) fn cmd_mission_start(ids: VehicleIds) -> MavMessage {
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

