//! MAVLink receive thread: handshake, streams, mission sync, override progression.

use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread;

use drone_server::{
    geo::horizontal_distance_m,
    goto_global_command_int,
    mavlink_streams::{heartbeat_from_autopilot, refresh_mavlink_streams},
    mission_set_current, mission_start, set_mode_auto, MissionStore, VehicleIds,
};
use mavlink::ardupilotmega::MavMessage;
use mavlink::{MavConnection, MavFrame};

use crate::consts::{
    REACHED_THRESHOLD_M, STREAM_AUTO_RETRY_FIRST_DELAY, STREAM_AUTO_RETRY_INTERVAL,
    STREAM_AUTO_RETRY_MAX_ATTEMPTS,
};
use crate::state::{OverrideState, TelemetryCoverage};

pub(crate) fn spawn_recv_thread<C>(
    recv_conn: Arc<Mutex<C>>,
    recv_store: Arc<Mutex<MissionStore>>,
    recv_override: Arc<Mutex<OverrideState>>,
    recv_watchdog_vehicle_ids: Arc<Mutex<Option<VehicleIds>>>,
    tx: mpsc::Sender<MavFrame<MavMessage>>,
    log_tx: mpsc::Sender<String>,
    stream_retry_rx: mpsc::Receiver<()>,
) -> thread::JoinHandle<()>
where
    C: MavConnection<MavMessage> + Send + 'static,
{
    thread::spawn(move || {
        let mut autopilot_handshake_done = false;
        let mut stream_auto_spawned = false;
        let coverage = Arc::new(Mutex::new(TelemetryCoverage::default()));
        let vehicle_ids_for_retry = Arc::new(Mutex::new(VehicleIds::default()));
        let mut mission_count: Option<u16> = None;
        let mut vehicle_ids = VehicleIds::default();
        loop {
            let mut manual_retry = false;
            while stream_retry_rx.try_recv().is_ok() {
                manual_retry = true;
            }
            if manual_retry {
                let ids = *vehicle_ids_for_retry.lock().unwrap();
                if let Ok(c) = recv_conn.lock() {
                    refresh_mavlink_streams(&*c, ids);
                    let _ = log_tx.send(
                        "Streams: manual retry (mission list + stream rates).".to_string(),
                    );
                }
            }

            let frame = match recv_conn.lock().unwrap().recv_frame() {
                Ok(f) => f,
                Err(_) => continue,
            };

            if let Ok(mut cov) = coverage.lock() {
                cov.update(&frame.msg);
            }

            if !autopilot_handshake_done {
                if let MavMessage::HEARTBEAT(d) = &frame.msg {
                    if heartbeat_from_autopilot(&frame, d.mavtype) {
                        autopilot_handshake_done = true;
                        vehicle_ids =
                            VehicleIds::new(frame.header.system_id, frame.header.component_id);
                        *vehicle_ids_for_retry.lock().unwrap() = vehicle_ids;
                        let c = recv_conn.lock().unwrap();
                        refresh_mavlink_streams(&*c, vehicle_ids);
                        if !stream_auto_spawned {
                            stream_auto_spawned = true;
                            let conn = Arc::clone(&recv_conn);
                            let cov = Arc::clone(&coverage);
                            let vid = Arc::clone(&vehicle_ids_for_retry);
                            let log = log_tx.clone();
                            thread::spawn(move || {
                                thread::sleep(STREAM_AUTO_RETRY_FIRST_DELAY);
                                for attempt in 1..=STREAM_AUTO_RETRY_MAX_ATTEMPTS {
                                    if cov.lock().unwrap().is_complete() {
                                        let _ = log.send(
                                            "Streams: telemetry complete (all key messages seen)."
                                                .to_string(),
                                        );
                                        return;
                                    }
                                    let ids = *vid.lock().unwrap();
                                    if let Ok(c) = conn.lock() {
                                        refresh_mavlink_streams(&*c, ids);
                                    }
                                    if attempt == 1
                                        || attempt % 5 == 0
                                        || attempt == STREAM_AUTO_RETRY_MAX_ATTEMPTS
                                    {
                                        let _ = log.send(format!(
                                            "Streams: auto-retry {}/{}…",
                                            attempt, STREAM_AUTO_RETRY_MAX_ATTEMPTS
                                        ));
                                    }
                                    thread::sleep(STREAM_AUTO_RETRY_INTERVAL);
                                }
                                let _ = log.send(
                                    "Streams: auto-retry stopped (incomplete). Press s to retry."
                                        .to_string(),
                                );
                            });
                        }
                    }
                }
            }
            if let MavMessage::HEARTBEAT(d) = &frame.msg {
                if heartbeat_from_autopilot(&frame, d.mavtype) {
                    let ids = VehicleIds::new(frame.header.system_id, frame.header.component_id);
                    vehicle_ids = ids;
                    *vehicle_ids_for_retry.lock().unwrap() = vehicle_ids;
                    if let Ok(mut g) = recv_watchdog_vehicle_ids.lock() {
                        *g = Some(ids);
                    }
                }
            }

            // Update mission store from FC
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

            let upload_seq = match &frame.msg {
                MavMessage::MISSION_REQUEST_INT(d) => Some(d.seq),
                #[allow(deprecated)]
                MavMessage::MISSION_REQUEST(d) => Some(d.seq),
                _ => None,
            };
            if let Some(seq) = upload_seq {
                if let (Ok(mut store), Ok(conn_lock)) = (recv_store.lock(), recv_conn.lock()) {
                    if let Some(mut item) = store.take_upload_item(seq) {
                        item.target_system = frame.header.system_id;
                        item.target_component = frame.header.component_id;
                        let _ = conn_lock.send_default(&MavMessage::MISSION_ITEM_INT(item));
                        store.note_upload_item_sent();
                    }
                }
            }
            if let MavMessage::MISSION_ACK(_) = &frame.msg {
                let resume_seq = if let Ok(state) = recv_override.lock() {
                    if let OverrideState::Resuming { resume_seq } = *state {
                        Some(resume_seq)
                    } else {
                        None
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
                    // Keep snapshot so multiple interrupts/waypoint injections work in the same session
                    if let Ok(mut state) = recv_override.lock() {
                        *state = OverrideState::MissionRunning;
                    }
                } else if recv_store
                    .lock()
                    .map(|s| s.upload_pending.is_some() && s.upload_ready_for_ack())
                    .unwrap_or(false)
                {
                    if let Ok(mut store) = recv_store.lock() {
                        if let Some(items) = store.upload_pending.clone() {
                            store.items = items;
                            store.current_seq = Some(0);
                        }
                        store.set_upload_done();
                    }
                }
            }

            // Override: check if we reached current override waypoint
            if let MavMessage::GLOBAL_POSITION_INT(d) = &frame.msg {
                let lat = d.lat as f64 / 1e7;
                let lon = d.lon as f64 / 1e7;
                let mut state_guard = match recv_override.lock() {
                    Ok(g) => g,
                    Err(_) => continue,
                };
                if let OverrideState::OverrideActive {
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
                                    // Override done -> start resume mission
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
                                        *st = OverrideState::Resuming { resume_seq };
                                    }
                                } else {
                                    // Override done -> stay paused (hover at this position)
                                    drop(state_guard);
                                    if let Ok(mut st) = recv_override.lock() {
                                        *st = OverrideState::Paused;
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

            let _ = tx.send(frame);
        }
    })
}
