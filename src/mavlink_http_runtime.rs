//! Background MAVLink receive loop for `drone-http`: mission mirror, resume handshake, override progression, telemetry cache.

use std::sync::{Arc, Mutex};
use std::thread;

use crate::geo::horizontal_distance_m;
use crate::mavlink_streams::{heartbeat_from_autopilot, refresh_mavlink_streams};
use crate::{
    goto_global_command_int, mission_set_current, mission_start, set_mode_auto, MissionStore,
    VehicleIds,
};
use mavlink::ardupilotmega::MavMessage;
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
}

impl Default for HttpOverrideState {
    fn default() -> Self {
        Self::MissionRunning
    }
}

/// Latest position / mode for HTTP tools (`mission_interrupt`, `waypoint_inject` text parsing).
#[derive(Default)]
pub struct TelemetryCache {
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    pub alt_amsl_m: Option<f64>,
    pub home_alt_m: Option<f64>,
    pub heartbeat_custom_mode: Option<u32>,
}

fn telem_update_from_frame(cache: &mut TelemetryCache, frame: &MavFrame<MavMessage>) {
    match &frame.msg {
        MavMessage::HEARTBEAT(d) if heartbeat_from_autopilot(frame, d.mavtype) => {
            cache.heartbeat_custom_mode = Some(d.custom_mode);
        }
        MavMessage::GLOBAL_POSITION_INT(d) => {
            cache.lat = Some(d.lat as f64 / 1e7);
            cache.lon = Some(d.lon as f64 / 1e7);
            cache.alt_amsl_m = Some(d.alt as f64 / 1000.0);
        }
        MavMessage::HOME_POSITION(d) => {
            cache.home_alt_m = Some(d.altitude as f64 / 1000.0);
        }
        _ => {}
    }
}

pub fn spawn_http_mavlink_recv_thread<C>(
    recv_conn: Arc<Mutex<C>>,
    recv_store: Arc<Mutex<MissionStore>>,
    recv_override: Arc<Mutex<HttpOverrideState>>,
    recv_telem: Arc<Mutex<TelemetryCache>>,
    recv_vehicle_ids: Arc<Mutex<VehicleIds>>,
) -> thread::JoinHandle<()>
where
    C: MavConnection<MavMessage> + Send + 'static,
{
    thread::spawn(move || {
        let mut autopilot_handshake_done = false;
        let mut mission_count: Option<u16> = None;
        let mut vehicle_ids = VehicleIds::default();

        loop {
            let frame = match recv_conn.lock().unwrap().recv_frame() {
                Ok(f) => f,
                Err(_) => continue,
            };

            if let Ok(mut t) = recv_telem.lock() {
                telem_update_from_frame(&mut t, &frame);
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
                        if let Ok(c) = recv_conn.lock() {
                            refresh_mavlink_streams(&*c, vehicle_ids);
                        }
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
                        let _ = recv_conn
                            .lock()
                            .unwrap()
                            .send_default(&MavMessage::MISSION_REQUEST_INT(req));
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
                if let (Ok(mut store), Ok(conn_lock)) = (recv_store.lock(), recv_conn.lock()) {
                    if let Some(mut item) = store.take_upload_item(d.seq) {
                        item.target_system = frame.header.system_id;
                        item.target_component = frame.header.component_id;
                        let _ = conn_lock.send_default(&MavMessage::MISSION_ITEM_INT(item));
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
                    if let Ok(mut conn_lock) = recv_conn.lock() {
                        let _ = mission_set_current(&mut *conn_lock, vehicle_ids, seq);
                        let _ = set_mode_auto(&mut *conn_lock, vehicle_ids);
                        let _ = mission_start(&mut *conn_lock, vehicle_ids);
                    }
                    if let Ok(mut state) = recv_override.lock() {
                        *state = HttpOverrideState::MissionRunning;
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
                                    let _ = recv_conn.lock().unwrap().send_default(&MavMessage::MISSION_COUNT(
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
                                let _ = recv_conn.lock().unwrap().send_default(&msg);
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
                        let _ = recv_conn
                            .lock()
                            .unwrap()
                            .send_default(&MavMessage::MISSION_REQUEST_INT(req));
                    }
                }
            }
        }
    })
}
