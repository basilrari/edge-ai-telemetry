//! Build ArduPilot mission items from planner JSON and upload via MAVLink handshake.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::cmd::DEFAULT_TAKEOFF_ALTITUDE_M;
use crate::{mission_set_current, mission_start, set_mode_auto, MissionStore, VehicleIds, send_gcs, GCS_COMPONENT_ID, GCS_SYSTEM_ID};
use crate::mavlink_http_runtime::HttpOverrideState;
use crate::mavlink_http_runtime::TelemetryCache;
use mavlink::ardupilotmega::{
    MavCmd, MavFrame, MavMessage, MISSION_CLEAR_ALL_DATA, MISSION_COUNT_DATA, MISSION_ITEM_INT_DATA,
};
use mavlink::MavConnection;
use serde::Deserialize;

const MIN_TAKEOFF_ALT_M: f32 = 2.0;
const MAX_TAKEOFF_ALT_M: f32 = 120.0;
const MIN_WP_ALT_M: f32 = 2.0;
const MAX_WP_ALT_M: f32 = 120.0;
const MAX_WAYPOINTS: usize = 120;
const UPLOAD_POLL_ITERATIONS: u32 = 300;
const UPLOAD_POLL_INTERVAL_MS: u64 = 100;
const UPLOAD_CLEAR_ACK_POLLS: u32 = 50;

pub fn mission_request_for_us(target_system: u8, target_component: u8) -> bool {
    (target_system == 0 || target_system == GCS_SYSTEM_ID)
        && (target_component == 0 || target_component == GCS_COMPONENT_ID)
}

#[derive(Debug, Deserialize)]
pub struct PlannerWaypoint {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
}

#[derive(Debug, Deserialize)]
pub struct MissionUploadRequest {
    #[serde(default = "default_true")]
    pub include_takeoff: bool,
    #[serde(default = "default_takeoff_alt")]
    pub takeoff_alt_m: f64,
    #[serde(default = "default_true")]
    pub include_rtl: bool,
    pub waypoints: Vec<PlannerWaypoint>,
}

fn default_true() -> bool {
    true
}

fn default_takeoff_alt() -> f64 {
    15.0
}

fn mission_item_int(
    ids: VehicleIds,
    seq: u16,
    command: MavCmd,
    lat_deg: f64,
    lon_deg: f64,
    alt_m: f32,
) -> MISSION_ITEM_INT_DATA {
    MISSION_ITEM_INT_DATA {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x: (lat_deg * 1e7).round() as i32,
        y: (lon_deg * 1e7).round() as i32,
        z: alt_m,
        seq,
        command,
        target_system: ids.system_id,
        target_component: ids.component_id,
        frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        current: 0,
        autocontinue: 1,
    }
}

/// Pick a takeoff altitude from existing NAV_WAYPOINT items, else default.
pub fn infer_takeoff_alt_m(items: &[MISSION_ITEM_INT_DATA]) -> f32 {
    let mut alts: Vec<f32> = items
        .iter()
        .filter(|it| it.command == MavCmd::MAV_CMD_NAV_WAYPOINT)
        .map(|it| it.z)
        .filter(|z| z.is_finite() && *z >= MIN_TAKEOFF_ALT_M && *z <= MAX_TAKEOFF_ALT_M)
        .collect();
    alts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    alts.first()
        .copied()
        .unwrap_or(DEFAULT_TAKEOFF_ALTITUDE_M)
        .clamp(MIN_TAKEOFF_ALT_M, MAX_TAKEOFF_ALT_M)
}

/// Insert NAV_TAKEOFF at seq 0 and renumber following items (no-op if already present).
pub fn prepend_nav_takeoff(
    items: Vec<MISSION_ITEM_INT_DATA>,
    ids: VehicleIds,
    takeoff_alt_m: f32,
) -> Vec<MISSION_ITEM_INT_DATA> {
    if MissionStore::items_have_nav_takeoff(&items) {
        return items;
    }
    let alt = takeoff_alt_m.clamp(MIN_TAKEOFF_ALT_M, MAX_TAKEOFF_ALT_M);
    let mut out = vec![mission_item_int(
        ids,
        0,
        MavCmd::MAV_CMD_NAV_TAKEOFF,
        0.0,
        0.0,
        alt,
    )];
    for mut item in items {
        item.seq = out.len() as u16;
        out.push(item);
    }
    out
}

pub fn build_mission_items(
    ids: VehicleIds,
    req: &MissionUploadRequest,
) -> Result<Vec<MISSION_ITEM_INT_DATA>, String> {
    if req.waypoints.is_empty() {
        return Err("mission upload: at least one waypoint is required".into());
    }
    if req.waypoints.len() > MAX_WAYPOINTS {
        return Err(format!(
            "mission upload: at most {MAX_WAYPOINTS} waypoints allowed"
        ));
    }

    let takeoff_alt = if req.include_takeoff {
        let a = req.takeoff_alt_m as f32;
        if !a.is_finite() || a < MIN_TAKEOFF_ALT_M || a > MAX_TAKEOFF_ALT_M {
            return Err(format!(
                "mission upload: takeoff_alt_m must be between {MIN_TAKEOFF_ALT_M} and {MAX_TAKEOFF_ALT_M} m"
            ));
        }
        a
    } else {
        0.0
    };

    let mut items: Vec<MISSION_ITEM_INT_DATA> = Vec::new();
    let mut seq: u16 = 0;

    if req.include_takeoff {
        items.push(mission_item_int(
            ids,
            seq,
            MavCmd::MAV_CMD_NAV_TAKEOFF,
            0.0,
            0.0,
            takeoff_alt,
        ));
        seq += 1;
    }

    for wp in &req.waypoints {
        if !wp.lat_deg.is_finite() || !wp.lon_deg.is_finite() {
            return Err("mission upload: invalid waypoint lat/lon".into());
        }
        if wp.lat_deg.abs() > 90.0 || wp.lon_deg.abs() > 180.0 {
            return Err("mission upload: waypoint lat/lon out of range".into());
        }
        if wp.lat_deg.abs() < 1e-6 && wp.lon_deg.abs() < 1e-6 {
            return Err("mission upload: waypoint cannot be 0,0".into());
        }
        let alt = wp.alt_m as f32;
        if !alt.is_finite() || alt < MIN_WP_ALT_M || alt > MAX_WP_ALT_M {
            return Err(format!(
                "mission upload: waypoint alt_m must be between {MIN_WP_ALT_M} and {MAX_WP_ALT_M} m"
            ));
        }
        items.push(mission_item_int(
            ids,
            seq,
            MavCmd::MAV_CMD_NAV_WAYPOINT,
            wp.lat_deg,
            wp.lon_deg,
            alt,
        ));
        seq += 1;
    }

    if req.include_rtl {
        items.push(mission_item_int(
            ids,
            seq,
            MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0.0,
            0.0,
            0.0,
        ));
    }

    if !req.include_takeoff {
        return Err(
            "mission upload: include_takeoff is required for ArduCopter AUTO missions".into(),
        );
    }

    Ok(items)
}

/// Upload pre-built mission items to the FC (shared by planner upload and start_mission fixup).
pub fn upload_mission_items<C: MavConnection<MavMessage>>(
    conn: &C,
    ids: VehicleIds,
    mission: &Arc<Mutex<MissionStore>>,
    http_override: Option<&Arc<Mutex<HttpOverrideState>>>,
    items: Vec<MISSION_ITEM_INT_DATA>,
) -> Result<usize, String> {
    if items.is_empty() {
        return Err("mission upload: empty mission".into());
    }

    {
        let store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        if store.upload_pending.is_some() {
            return Err("mission upload: another upload is in progress".into());
        }
    }

    if let Some(os_arc) = http_override {
        let mut os = os_arc.lock().map_err(|e| format!("override_lock:{e}"))?;
        if matches!(&*os, HttpOverrideState::Resuming { .. }) {
            return Err("mission upload: wait for mission resume to finish".into());
        }
        *os = HttpOverrideState::Uploading;
    }

    // Clear any stale FC mission state before a fresh upload (avoids ArduPilot upload timeouts).
    {
        let mut store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        store.awaiting_clear_ack = true;
        store.upload_items_sent = 0;
    }
    send_gcs(conn, &MavMessage::MISSION_CLEAR_ALL(MISSION_CLEAR_ALL_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
    }))
    .map_err(|e| e.to_string())?;
    for _ in 0..UPLOAD_CLEAR_ACK_POLLS {
        std::thread::sleep(Duration::from_millis(UPLOAD_POLL_INTERVAL_MS));
        let cleared = mission
            .lock()
            .map_err(|e| format!("mission_lock:{e}"))?
            .awaiting_clear_ack;
        if !cleared {
            break;
        }
    }
    {
        let mut store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        store.awaiting_clear_ack = false;
    }

    {
        let mut store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        store.upload_done = false;
        store.set_upload_pending(items.clone());
    }

    let count = items.len() as u16;
    send_gcs(conn, &MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
        count,
        target_system: ids.system_id,
        target_component: ids.component_id,
    }))
    .map_err(|e| e.to_string())?;

    if let Some(mut first) = items.first().cloned() {
        first.target_system = ids.system_id;
        first.target_component = ids.component_id;
        if send_gcs(conn, &MavMessage::MISSION_ITEM_INT(first)).is_ok() {
            if let Ok(mut store) = mission.lock() {
                store.note_upload_item_sent(0);
            }
        }
    }

    for _ in 0..UPLOAD_POLL_ITERATIONS {
        std::thread::sleep(Duration::from_millis(UPLOAD_POLL_INTERVAL_MS));
        let (done, failed, fail_reason) = {
            let store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
            (
                store.upload_done,
                store.upload_failed,
                store.upload_fail_reason.clone(),
            )
        };
        if failed {
            return Err(fail_reason.unwrap_or_else(|| {
                "mission upload: rejected by flight controller".into()
            }));
        }
        if done {
            return Ok(items.len());
        }
    }

    {
        let mut store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        store.upload_pending = None;
        store.upload_done = false;
        store.awaiting_clear_ack = false;
        store.upload_items_sent = 0;
        store.upload_sent_seqs.clear();
        store.upload_failed = false;
        store.upload_fail_reason = None;
    }
    if let Some(os_arc) = http_override {
        if let Ok(mut os) = os_arc.lock() {
            *os = HttpOverrideState::MissionRunning;
        }
    }

    Err("mission upload: timed out waiting for MISSION_ACK from flight controller".into())
}

/// If the FC mission lacks NAV_TAKEOFF, prepend one and re-upload (Mission Planner / TUI `m` only — not LLM prompts).
pub fn ensure_nav_takeoff_on_fc<C: MavConnection<MavMessage>>(
    conn: &C,
    ids: VehicleIds,
    mission: &Arc<Mutex<MissionStore>>,
    http_override: Option<&Arc<Mutex<HttpOverrideState>>>,
) -> Result<bool, String> {
    let needs_fixup = {
        let store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        !store.items.is_empty() && !store.has_nav_takeoff()
    };
    if !needs_fixup {
        return Ok(false);
    }

    let items = {
        let store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        let alt = infer_takeoff_alt_m(&store.items);
        prepend_nav_takeoff(store.items.clone(), ids, alt)
    };

    upload_mission_items(conn, ids, mission, http_override, items)?;
    Ok(true)
}

pub fn mission_upload<C: MavConnection<MavMessage>>(
    conn: &C,
    ids: VehicleIds,
    mission: &Arc<Mutex<MissionStore>>,
    override_state: &Arc<Mutex<HttpOverrideState>>,
    req: &MissionUploadRequest,
) -> Result<usize, String> {
    let items = build_mission_items(ids, req)?;
    upload_mission_items(
        conn,
        ids,
        mission,
        Some(override_state),
        items,
    )
}

/// Clear all mission items on the flight controller and local mission cache.
pub fn mission_clear<C: MavConnection<MavMessage>>(
    conn: &C,
    ids: VehicleIds,
    mission: &Arc<Mutex<MissionStore>>,
    http_override: Option<&Arc<Mutex<HttpOverrideState>>>,
) -> Result<(), String> {
    {
        let store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        if store.upload_pending.is_some() {
            return Err("mission clear: wait for mission upload to finish".into());
        }
    }

    if let Some(os_arc) = http_override {
        let mut os = os_arc.lock().map_err(|e| format!("override_lock:{e}"))?;
        if matches!(&*os, HttpOverrideState::Resuming { .. } | HttpOverrideState::Uploading) {
            return Err("mission clear: wait for mission transfer to finish".into());
        }
        *os = HttpOverrideState::MissionRunning;
    }

    send_gcs(conn, &MavMessage::MISSION_CLEAR_ALL(MISSION_CLEAR_ALL_DATA {
        target_system: ids.system_id,
        target_component: ids.component_id,
    }))
    .map_err(|e| e.to_string())?;

    {
        let mut store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        store.clear_local();
    }

    Ok(())
}

const AIRBORNE_MIN_M: f64 = 2.5;

/// AUTO + MISSION_START; when already airborne, skip NAV_TAKEOFF by setting current WP first.
pub fn start_auto_mission<C: MavConnection<MavMessage>>(
    conn: &C,
    ids: VehicleIds,
    mission: &Arc<Mutex<MissionStore>>,
    telem: &TelemetryCache,
) -> Result<(), String> {
    let airborne = telem
        .relative_alt_m
        .map(|a| a > AIRBORNE_MIN_M)
        .unwrap_or(false);

    let items = {
        let store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        if store.items.is_empty() {
            return Err(
                "start_mission: no mission on the link — upload a mission with takeoff from the Mission page first."
                    .into(),
            );
        }
        let mut items = store.items.clone();
        if !MissionStore::items_have_nav_takeoff(&items) && !airborne {
            let alt = infer_takeoff_alt_m(&items);
            items = prepend_nav_takeoff(items, ids, alt);
        }
        items
    };

    // On the ground, push the mission to the FC before AUTO so ArduPilot does not
    // PreArm-fail with "Mode requires mission" when the link cache and FC are out of sync.
    if !airborne {
        upload_mission_items(conn, ids, mission, None, items.clone())?;
    }

    if airborne {
        if let Some(seq) = items
            .iter()
            .find(|it| it.command == MavCmd::MAV_CMD_NAV_WAYPOINT)
            .map(|it| it.seq)
        {
            mission_set_current(conn, ids, seq).map_err(|e| e.to_string())?;
        }
    }

    set_mode_auto(conn, ids).map_err(|e| e.to_string())?;
    mission_start(conn, ids).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::VehicleIds;

    #[test]
    fn mission_request_for_us_accepts_gcs_targets() {
        assert!(mission_request_for_us(255, 190));
        assert!(mission_request_for_us(255, 0));
        assert!(mission_request_for_us(0, 190));
        assert!(!mission_request_for_us(1, 1));
    }

    #[test]
    fn prepend_takeoff_renumbers_items() {
        let ids = VehicleIds::default();
        let wp = mission_item_int(ids, 0, MavCmd::MAV_CMD_NAV_WAYPOINT, 23.0, 120.0, 15.0);
        let rtl = mission_item_int(ids, 1, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0, 0.0);
        let fixed = prepend_nav_takeoff(vec![wp, rtl], ids, 12.0);
        assert_eq!(fixed.len(), 3);
        assert_eq!(fixed[0].command, MavCmd::MAV_CMD_NAV_TAKEOFF);
        assert_eq!(fixed[0].seq, 0);
        assert_eq!(fixed[0].z, 12.0);
        assert_eq!(fixed[1].command, MavCmd::MAV_CMD_NAV_WAYPOINT);
        assert_eq!(fixed[1].seq, 1);
        assert_eq!(fixed[2].command, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH);
        assert_eq!(fixed[2].seq, 2);
    }
}
