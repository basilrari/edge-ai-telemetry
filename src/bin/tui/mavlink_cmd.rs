//! Local MAVLink command builders and stream rate requests (TUI binary).

use drone_server::VehicleIds;
use mavlink::ardupilotmega::{
    COMMAND_LONG_DATA, MavCmd, MavMessage, MavType, REQUEST_DATA_STREAM_DATA,
};
use mavlink::{MavConnection, MavFrame};

use crate::consts::{
    MSG_ID_AHRS2, MSG_ID_ATTITUDE, MSG_ID_BATTERY_STATUS, MSG_ID_COMMAND_ACK,
    MSG_ID_DISTANCE_SENSOR, MSG_ID_EKF_STATUS_REPORT, MSG_ID_GLOBAL_POSITION_INT,
    MSG_ID_GPS_RAW_INT, MSG_ID_HOME_POSITION, MSG_ID_LOCAL_POSITION_NED,
    MSG_ID_MISSION_CURRENT, MSG_ID_NAV_CONTROLLER_OUTPUT, MSG_ID_PARAM_VALUE,
    MSG_ID_RAW_IMU, MSG_ID_RC_CHANNELS, MSG_ID_SERVO_OUTPUT_RAW, MSG_ID_STATUSTEXT,
    MSG_ID_SYS_STATUS, MSG_ID_SYSTEM_TIME, MSG_ID_VFR_HUD, MSG_ID_VIBRATION,
};

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

/// `MAV_COMP_ID_AUTOPILOT1` = 1. Companion / router heartbeats often use another component; if we
/// send `SET_MESSAGE_INTERVAL` there, the flight controller never streams GPS / SYS_STATUS / HUD.
pub(crate) fn heartbeat_from_autopilot(hdr: &MavFrame<MavMessage>, mavtype: MavType) -> bool {
    const MAV_COMP_ID_AUTOPILOT1: u8 = 1;
    if mavtype == MavType::MAV_TYPE_GCS {
        return false;
    }
    hdr.header.component_id == MAV_COMP_ID_AUTOPILOT1
}

pub(crate) fn request_stream_rates(connection: &impl MavConnection<MavMessage>, ids: VehicleIds) {
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
pub(crate) fn refresh_mavlink_streams(connection: &impl MavConnection<MavMessage>, ids: VehicleIds) {
    let req = mavlink::ardupilotmega::MISSION_REQUEST_LIST_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
    };
    let _ = connection.send_default(&MavMessage::MISSION_REQUEST_LIST(req));
    request_stream_rates(connection, ids);
}
