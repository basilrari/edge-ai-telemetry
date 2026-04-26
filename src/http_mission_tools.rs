//! TUI-equivalent **interrupt**, **resume**, and **waypoint inject** for `drone-http`.
//! Requires [`crate::mavlink_http_runtime::TelemetryCache`] and mission/override state updated by the recv thread.

use std::sync::{Arc, Mutex};

use crate::geo::parse_waypoint_input;
use crate::goto_global_command_int;
use crate::set_mode_guided;
use crate::MissionStore;
use crate::VehicleIds;
use crate::mavlink_http_runtime::{HttpOverrideState, TelemetryCache};
use mavlink::ardupilotmega::MavMessage;
use mavlink::MavConnection;
use serde_json::Value;

const ARDUCOPTER_CUSTOM_AUTO: u32 = 3;

/// Mission interrupt (TUI `i`): snapshot mission, GUIDED + DO_REPOSITION at current position (alt relative to home).
pub fn mission_interrupt<C: MavConnection<MavMessage>>(
    conn: &mut C,
    ids: VehicleIds,
    mission: &Arc<Mutex<MissionStore>>,
    override_state: &Arc<Mutex<HttpOverrideState>>,
    telem: &Arc<Mutex<TelemetryCache>>,
) -> Result<(), String> {
    let custom = telem
        .lock()
        .map_err(|e| format!("telem_lock:{e}"))?
        .heartbeat_custom_mode;
    if custom != Some(ARDUCOPTER_CUSTOM_AUTO) {
        return Err(
            "mission_interrupt: need AUTO mode (custom_mode=3) and mission running; use start_mission first"
                .into(),
        );
    }

    let (lat, lon, alt_rel) = {
        let t = telem.lock().map_err(|e| format!("telem_lock:{e}"))?;
        match (t.lat, t.lon, t.alt_amsl_m, t.home_alt_m) {
            (Some(la), Some(lo), Some(al), Some(home_al)) => (la, lo, al - home_al),
            (Some(_), Some(_), None, _) | (None, _, _, _) | (_, None, _, _) => {
                return Err("mission_interrupt: need GPS position (GLOBAL_POSITION_INT)".into());
            }
            (_, _, Some(_), None) => {
                return Err("mission_interrupt: need HOME_POSITION for relative altitude".into());
            }
        }
    };

    {
        let mut os = override_state.lock().map_err(|e| format!("override_lock:{e}"))?;
        if matches!(&*os, HttpOverrideState::OverrideActive { .. }) {
            return Err("mission_interrupt: finish waypoint_inject override first".into());
        }
        let mut store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        if !store.ensure_snapshot_for_pause() {
            return Err(
                "mission_interrupt: no mission snapshot (wait for mission download on link)".into(),
            );
        }
        *os = HttpOverrideState::Paused;
    }

    set_mode_guided(conn, ids).map_err(|e| e.to_string())?;
    let msg = goto_global_command_int(ids, lat, lon, alt_rel);
    conn.send_default(&msg).map(|_| ()).map_err(|e| e.to_string())?;
    Ok(())
}

/// Resume mission after interrupt (TUI `c`): `MISSION_COUNT` upload handshake; recv thread completes on `MISSION_ACK`.
pub fn mission_resume<C: MavConnection<MavMessage>>(
    conn: &mut C,
    ids: VehicleIds,
    mission: &Arc<Mutex<MissionStore>>,
    override_state: &Arc<Mutex<HttpOverrideState>>,
) -> Result<(), String> {
    let (snapshot_items, resume_seq) = {
        let store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        match store.get_snapshot() {
            Some((items, seq)) => (items.to_vec(), seq),
            None => {
                let mut os = override_state.lock().map_err(|e| format!("override_lock:{e}"))?;
                if !matches!(*os, HttpOverrideState::MissionRunning) {
                    *os = HttpOverrideState::MissionRunning;
                    return Err(
                        "mission_resume: no snapshot; override state reset (same as TUI c)"
                            .into(),
                    );
                }
                return Err("mission_resume: no snapshot (nothing to resume)".into());
            }
        }
    };

    {
        let mut store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        store.set_upload_pending(snapshot_items.clone());
    }
    let count = snapshot_items.len() as u16;
    conn.send_default(&MavMessage::MISSION_COUNT(
        mavlink::ardupilotmega::MISSION_COUNT_DATA {
            count,
            target_system: ids.system_id,
            target_component: ids.component_id,
        },
    ))
    .map(|_| ())
    .map_err(|e| e.to_string())?;

    {
        let mut os = override_state.lock().map_err(|e| format!("override_lock:{e}"))?;
        *os = HttpOverrideState::Resuming { resume_seq };
    }
    Ok(())
}

/// Guided waypoint inject (TUI `w` after Enter). Params: `{"lat_deg","lon_deg","alt_m"}` **or** `{"waypoint_text":"..."}` (same syntax as TUI).
pub fn waypoint_inject<C: MavConnection<MavMessage>>(
    conn: &mut C,
    ids: VehicleIds,
    mission: &Arc<Mutex<MissionStore>>,
    override_state: &Arc<Mutex<HttpOverrideState>>,
    telem: &Arc<Mutex<TelemetryCache>>,
    params: &Value,
) -> Result<(), String> {
    let (lat, lon, alt) = if let (Some(lat), Some(lon), Some(alt)) = (
        params.get("lat_deg").and_then(|v| v.as_f64()),
        params.get("lon_deg").and_then(|v| v.as_f64()),
        params.get("alt_m").and_then(|v| v.as_f64()),
    ) {
        (lat, lon, alt)
    } else if let Some(text) = params.get("waypoint_text").and_then(|v| v.as_str()) {
        let t = telem.lock().map_err(|e| format!("telem_lock:{e}"))?;
        parse_waypoint_input(text, t.lat, t.lon, t.alt_amsl_m)
            .map_err(|e| format!("waypoint_inject parse: {e}"))?
    } else {
        return Err(
            "waypoint_inject: provide params {\"lat_deg\",\"lon_deg\",\"alt_m\"} or {\"waypoint_text\":\"lat lon alt\"}"
                .into(),
        );
    };

    {
        let mut os = override_state.lock().map_err(|e| format!("override_lock:{e}"))?;
        if matches!(&*os, HttpOverrideState::OverrideActive { .. }) {
            return Err("waypoint_inject: finish current override first".into());
        }
        let from_paused = matches!(&*os, HttpOverrideState::Paused);
        let resume_after = !from_paused;
        if !from_paused {
            let mut store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
            if !store.ensure_snapshot_for_pause() {
                return Err(
                    "waypoint_inject: need mission or interrupt first (no snapshot)".into(),
                );
            }
        }
        *os = HttpOverrideState::OverrideActive {
            waypoints: vec![(lat, lon, alt)],
            index: 0,
            resume_after,
        };
    }

    set_mode_guided(conn, ids).map_err(|e| e.to_string())?;
    let msg = goto_global_command_int(ids, lat, lon, alt);
    conn.send_default(&msg).map(|_| ()).map_err(|e| e.to_string())?;
    Ok(())
}
