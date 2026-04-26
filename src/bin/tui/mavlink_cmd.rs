//! MAVLink helpers for the TUI — thin wrappers around [`drone_server::cmd`] so HTTP and TUI share one implementation.

use drone_server::VehicleIds;
use mavlink::ardupilotmega::MavMessage;
use mavlink::MavConnection;

pub(crate) fn with_vehicle(msg: MavMessage, ids: VehicleIds) -> MavMessage {
    drone_server::with_vehicle(msg, ids)
}

pub(crate) fn cmd_arm() -> MavMessage {
    drone_server::arm()
}

pub(crate) fn cmd_disarm() -> MavMessage {
    drone_server::disarm()
}

pub(crate) fn cmd_set_mode_guided_long<C>(
    conn: &mut C,
    ids: VehicleIds,
) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    drone_server::set_mode_guided_long(conn, ids)
}

pub(crate) fn cmd_set_mode_auto_long<C>(
    conn: &mut C,
    ids: VehicleIds,
) -> Result<(), mavlink::error::MessageWriteError>
where
    C: MavConnection<MavMessage>,
{
    drone_server::set_mode_auto_long(conn, ids)
}

pub(crate) fn cmd_takeoff_alt(altitude_m: f32) -> MavMessage {
    drone_server::takeoff_alt(altitude_m)
}

pub(crate) fn cmd_mission_start(ids: VehicleIds) -> MavMessage {
    drone_server::mission_start_message(ids)
}
