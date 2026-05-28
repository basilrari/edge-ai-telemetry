//! Background MAVLink receive loop for `drone-http`: mission mirror, resume handshake, override progression, telemetry cache.

use std::sync::{Arc, Mutex};
use std::thread;

use crate::flight_log::FlightLog;
use crate::mavlink_connect::LinkInfo;
use crate::telemetry_hub::TelemetryHub;
use crate::geo::horizontal_distance_m;
use crate::mavlink_streams::{heartbeat_from_autopilot, refresh_mavlink_streams};
use crate::{
    goto_global_command_int, mission_set_current, mission_start, set_mode_auto, MissionStore,
    VehicleIds,
};
use mavlink::ardupilotmega::{MavMessage, MavModeFlag};
use mavlink::{MavConnection, MavFrame};

/// Horizontal distance (m) to consider a waypoint reached (same as TUI).
const REACHED_THRESHOLD_M: f64 = 10.0;

/// Same states as the TUI override/mission-resume state machine (HTTP path has no UI channel).
#[derive(Clone, Debug)]
pub enum HttpOverrideState {
    MissionRunning,
    Paused,
    OverrideActive {
        waypoints: Vec<(f64, f64, f64)>,
        index: usize,
        resume_after: bool,
    },
    Resuming { resume_seq: u16 },
    /// Planner upload in progress (wait for MISSION_ACK).
    Uploading,
}

impl Default for HttpOverrideState {
    fn default() -> Self {
        Self::MissionRunning
    }
}

/// Default target height above home when altitude is omitted, zero, or non-positive.
pub const DEFAULT_ALTITUDE_ABOVE_HOME_M: f32 = 15.0;

/// Meters above home: finite values `> 0` are used as-is; otherwise `DEFAULT_ALTITUDE_ABOVE_HOME_M`.
pub fn altitude_above_home_m(value: Option<f64>) -> f32 {
    match value {
        Some(v) if v.is_finite() && v > 0.0 => v as f32,
        _ => DEFAULT_ALTITUDE_ABOVE_HOME_M,
    }
}

pub fn altitude_above_home_from_params(
    params: &serde_json::Value,
    key: &str,
) -> f32 {
    altitude_above_home_m(params.get(key).and_then(|v| v.as_f64()))
}

/// Resolve `NAV_TAKEOFF` target altitude (meters above home, same convention as `goto_location` `alt_m`).
pub fn resolve_takeoff_altitude_m(
    params: &serde_json::Value,
    telem: &TelemetryCache,
) -> Result<f32, String> {
    if params.get("altitude_m").is_some() {
        return Ok(altitude_above_home_from_params(params, "altitude_m"));
    }
    if let Some(rel) = telem.relative_alt_m {
        return Ok(altitude_above_home_m(Some(rel)));
    }
    if let (Some(alt), Some(home)) = (telem.alt_amsl_m, telem.home_alt_m) {
        return Ok(altitude_above_home_m(Some((alt - home).max(0.0))));
    }
    Err(
        "takeoff: no params.altitude_m and no position telemetry yet (GLOBAL_POSITION_INT); \
         wait for GPS or specify altitude_m"
            .into(),
    )
}

#[cfg(test)]
mod altitude_tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn zero_or_missing_uses_default() {
        assert_eq!(altitude_above_home_m(None), DEFAULT_ALTITUDE_ABOVE_HOME_M);
        assert_eq!(altitude_above_home_m(Some(0.0)), DEFAULT_ALTITUDE_ABOVE_HOME_M);
        assert_eq!(altitude_above_home_m(Some(-1.0)), DEFAULT_ALTITUDE_ABOVE_HOME_M);
    }

    #[test]
    fn positive_unchanged() {
        assert_eq!(altitude_above_home_m(Some(30.0)), 30.0);
    }

    #[test]
    fn takeoff_explicit_zero_defaults() {
        let telem = TelemetryCache::default();
        let params = json!({ "altitude_m": 0 });
        assert_eq!(
            resolve_takeoff_altitude_m(&params, &telem).unwrap(),
            DEFAULT_ALTITUDE_ABOVE_HOME_M
        );
    }
}

pub fn arducopter_mode_name(custom_mode: u32) -> &'static str {
    match custom_mode {
        0 => "STABILIZE",
        1 => "ACRO",
        2 => "ALT_HOLD",
        3 => "AUTO",
        4 => "GUIDED",
        5 => "LOITER",
        6 => "RTL",
        7 => "CIRCLE",
        8 => "POSITION",
        9 => "LAND",
        10 => "DRIFT",
        11 => "SPORT",
        12 => "FLIP",
        13 => "AUTOTUNE",
        14 => "POSHOLD",
        15 => "BRAKE",
        16 => "THROW",
        17 => "AVOID_ADSB",
        18 => "GUIDED_NOGPS",
        19 => "SMART_RTL",
        20 => "FLOWHOLD",
        21 => "FOLLOW",
        22 => "ZIGZAG",
        23 => "SYSTEMID",
        24 => "AUTOROTATE",
        25 => "AUTO_RTL",
        _ => "UNKNOWN",
    }
}

/// Latest position / HUD for HTTP tools and dashboard.
#[derive(Default)]
pub struct TelemetryCache {
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    pub alt_amsl_m: Option<f64>,
    pub home_alt_m: Option<f64>,
    pub heartbeat_custom_mode: Option<u32>,
    pub mode_name: Option<String>,
    /// From HEARTBEAT `base_mode` (autopilot only).
    pub armed: Option<bool>,
    /// From GLOBAL_POSITION_INT `relative_alt` (meters above home).
    pub relative_alt_m: Option<f64>,
    pub roll_deg: Option<f32>,
    pub pitch_deg: Option<f32>,
    pub yaw_deg: Option<f32>,
    pub groundspeed_m_s: Option<f32>,
    pub airspeed_m_s: Option<f32>,
    pub heading_deg: Option<i16>,
    pub climb_m_s: Option<f32>,
}

fn telem_update_from_frame(cache: &mut TelemetryCache, frame: &MavFrame<MavMessage>) {
    match &frame.msg {
        MavMessage::HEARTBEAT(d) if heartbeat_from_autopilot(frame, d.mavtype) => {
            cache.heartbeat_custom_mode = Some(d.custom_mode);
            cache.mode_name = Some(arducopter_mode_name(d.custom_mode).to_string());
            cache.armed = Some(
                d.base_mode
                    .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED),
            );
        }
        MavMessage::ATTITUDE(d) => {
            cache.roll_deg = Some(d.roll.to_degrees());
            cache.pitch_deg = Some(d.pitch.to_degrees());
            cache.yaw_deg = Some(d.yaw.to_degrees());
        }
        MavMessage::GLOBAL_POSITION_INT(d) => {
            cache.lat = Some(d.lat as f64 / 1e7);
            cache.lon = Some(d.lon as f64 / 1e7);
            cache.alt_amsl_m = Some(d.alt as f64 / 1000.0);
            cache.relative_alt_m = Some(d.relative_alt as f64 / 1000.0);
        }
        MavMessage::VFR_HUD(d) => {
            cache.airspeed_m_s = Some(d.airspeed);
            cache.groundspeed_m_s = Some(d.groundspeed);
            cache.heading_deg = Some(d.heading);
            cache.climb_m_s = Some(d.climb);
        }
        MavMessage::HOME_POSITION(d) => {
            cache.home_alt_m = Some(d.altitude as f64 / 1000.0);
        }
        _ => {}
    }
}

pub fn spawn_http_mavlink_recv_thread<C>(
    recv_conn: Arc<C>,
    recv_store: Arc<Mutex<MissionStore>>,
    recv_override: Arc<Mutex<HttpOverrideState>>,
    recv_telem: Arc<Mutex<TelemetryCache>>,
    recv_vehicle_ids: Arc<Mutex<VehicleIds>>,
    flight_log: FlightLog,
    telemetry_hub: TelemetryHub,
    link_info: LinkInfo,
) -> thread::JoinHandle<()>
where
    C: MavConnection<MavMessage> + Send + Sync + 'static,
{
    thread::spawn(move || {
        let mut autopilot_handshake_done = false;
        let mut mission_count: Option<u16> = None;
        let mut vehicle_ids = VehicleIds::default();

        loop {
            // Do not wrap the connection in an outer Mutex: serial MAVLink uses separate
            // read/write locks internally; holding one mutex across `recv_frame()` blocks HTTP apply-tool.
            let frame = match recv_conn.recv_frame() {
                Ok(f) => f,
                Err(_) => continue,
            };

            if let Ok(mut t) = recv_telem.lock() {
                telem_update_from_frame(&mut t, &frame);
                telemetry_hub.maybe_publish(&link_info, &t);
            }

            if let MavMessage::STATUSTEXT(d) = &frame.msg {
                let text = d
                    .text
                    .to_str()
                    .unwrap_or("")
                    .trim()
                    .trim_end_matches('\0');
                if !text.is_empty() {
                    flight_log.push("info", format!("FC: {text}"));
                }
            }

            if !autopilot_handshake_done {
                if let MavMessage::HEARTBEAT(d) = &frame.msg {
                    if heartbeat_from_autopilot(&frame, d.mavtype) {
                        autopilot_handshake_done = true;
                        vehicle_ids =
                            VehicleIds::new(frame.header.system_id, frame.header.component_id);
                        if let Ok(mut g) = recv_vehicle_ids.lock() {
                            *g = vehicle_ids;
                        }
                        refresh_mavlink_streams(recv_conn.as_ref(), vehicle_ids);
                    }
                }
            }

            if let MavMessage::HEARTBEAT(d) = &frame.msg {
                if heartbeat_from_autopilot(&frame, d.mavtype) {
                    vehicle_ids = VehicleIds::new(frame.header.system_id, frame.header.component_id);
                    if let Ok(mut g) = recv_vehicle_ids.lock() {
                        *g = vehicle_ids;
                    }
                }
            }

            if let MavMessage::MISSION_ITEM_INT(d) = &frame.msg {
                if let Ok(mut store) = recv_store.lock() {
                    store.update_from_item(d);
                }
                if let Some(count) = mission_count {
                    let next_seq = d.seq + 1;
                    if next_seq < count {
                        let sys = frame.header.system_id;
                        let comp = frame.header.component_id;
                        let req = mavlink::ardupilotmega::MISSION_REQUEST_INT_DATA {
                            target_system: sys,
                            target_component: comp,
                            seq: next_seq,
                        };
                        let _ = recv_conn.send_default(&MavMessage::MISSION_REQUEST_INT(req));
                    } else {
                        mission_count = None;
                    }
                }
            }
            if let MavMessage::MISSION_CURRENT(d) = &frame.msg {
                if let Ok(mut store) = recv_store.lock() {
                    store.update_current_seq(d.seq);
                }
            }

            if let MavMessage::MISSION_REQUEST_INT(d) = &frame.msg {
                if let Ok(mut store) = recv_store.lock() {
                    if let Some(mut item) = store.take_upload_item(d.seq) {
                        item.target_system = frame.header.system_id;
                        item.target_component = frame.header.component_id;
                        let _ = recv_conn.send_default(&MavMessage::MISSION_ITEM_INT(item));
                    }
                }
            }

            if let MavMessage::MISSION_ACK(_) = &frame.msg {
                let resume_seq = if let Ok(guard) = recv_override.lock() {
                    match &*guard {
                        HttpOverrideState::Resuming { resume_seq } => Some(*resume_seq),
                        _ => None,
                    }
                } else {
                    None
                };
                if let Some(seq) = resume_seq {
                    if let Ok(mut store) = recv_store.lock() {
                        store.set_upload_done();
                    }
                    let _ = mission_set_current(recv_conn.as_ref(), vehicle_ids, seq);
                    let _ = set_mode_auto(recv_conn.as_ref(), vehicle_ids);
                    let _ = mission_start(recv_conn.as_ref(), vehicle_ids);
                    if let Ok(mut state) = recv_override.lock() {
                        *state = HttpOverrideState::MissionRunning;
                    }
                } else {
                    let is_upload = recv_override
                        .lock()
                        .map(|g| matches!(*g, HttpOverrideState::Uploading))
                        .unwrap_or(false);
                    if is_upload {
                        if let Ok(mut store) = recv_store.lock() {
                            if let Some(items) = store.upload_pending.clone() {
                                store.items = items;
                                store.current_seq = Some(0);
                            }
                            store.set_upload_done();
                        }
                        if let Ok(mut state) = recv_override.lock() {
                            *state = HttpOverrideState::MissionRunning;
                        }
                    }
                }
            }

            if let MavMessage::GLOBAL_POSITION_INT(d) = &frame.msg {
                let lat = d.lat as f64 / 1e7;
                let lon = d.lon as f64 / 1e7;
                let mut state_guard = match recv_override.lock() {
                    Ok(g) => g,
                    Err(_) => continue,
                };
                if let HttpOverrideState::OverrideActive {
                    waypoints,
                    index,
                    resume_after,
                } = &mut *state_guard
                {
                    if *index < waypoints.len() {
                        let (wp_lat, wp_lon, _wp_alt) = waypoints[*index];
                        let dist = horizontal_distance_m(lat, lon, wp_lat, wp_lon);
                        if dist < REACHED_THRESHOLD_M {
                            *index += 1;
                            if *index >= waypoints.len() {
                                if *resume_after {
                                    let (snapshot_items, resume_seq) = {
                                        let store = match recv_store.lock() {
                                            Ok(s) => s,
                                            Err(_) => continue,
                                        };
                                        match store.get_snapshot() {
                                            Some((items, seq)) => (items.to_vec(), seq),
                                            None => continue,
                                        }
                                    };
                                    drop(state_guard);
                                    {
                                        let mut store = recv_store.lock().unwrap();
                                        store.set_upload_pending(snapshot_items.clone());
                                    }
                                    let count = snapshot_items.len() as u16;
                                    let _ = recv_conn.send_default(&MavMessage::MISSION_COUNT(
                                        mavlink::ardupilotmega::MISSION_COUNT_DATA {
                                            count,
                                            target_system: vehicle_ids.system_id,
                                            target_component: vehicle_ids.component_id,
                                        },
                                    ));
                                    if let Ok(mut st) = recv_override.lock() {
                                        *st = HttpOverrideState::Resuming { resume_seq };
                                    }
                                } else {
                                    drop(state_guard);
                                    if let Ok(mut st) = recv_override.lock() {
                                        *st = HttpOverrideState::Paused;
                                    }
                                }
                            } else {
                                let (wl, wlon, walt) = waypoints[*index];
                                drop(state_guard);
                                let msg = goto_global_command_int(vehicle_ids, wl, wlon, walt);
                                let _ = recv_conn.send_default(&msg);
                            }
                        }
                    }
                }
            }

            if let MavMessage::MISSION_COUNT(d) = &frame.msg {
                if mission_count.is_none()
                    && recv_store
                        .lock()
                        .map(|s| s.upload_pending.is_none())
                        .unwrap_or(true)
                {
                    mission_count = Some(d.count);
                    if d.count > 0 {
                        let sys = frame.header.system_id;
                        let comp = frame.header.component_id;
                        let req = mavlink::ardupilotmega::MISSION_REQUEST_INT_DATA {
                            target_system: sys,
                            target_component: comp,
                            seq: 0,
                        };
                        let _ = recv_conn.send_default(&MavMessage::MISSION_REQUEST_INT(req));
                    }
                }
            }
        }
    })
}
