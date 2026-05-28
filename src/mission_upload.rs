//! Build ArduPilot mission items from planner JSON and upload via MAVLink handshake.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::MissionStore;
use crate::VehicleIds;
use crate::mavlink_http_runtime::HttpOverrideState;
use mavlink::ardupilotmega::{MavCmd, MavFrame, MavMessage, MISSION_COUNT_DATA, MISSION_ITEM_INT_DATA};
use mavlink::MavConnection;
use serde::Deserialize;

const MIN_TAKEOFF_ALT_M: f32 = 2.0;
const MAX_TAKEOFF_ALT_M: f32 = 120.0;
const MIN_WP_ALT_M: f32 = 2.0;
const MAX_WP_ALT_M: f32 = 120.0;
const MAX_WAYPOINTS: usize = 120;

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

pub fn mission_upload<C: MavConnection<MavMessage>>(
    conn: &C,
    ids: VehicleIds,
    mission: &Arc<Mutex<MissionStore>>,
    override_state: &Arc<Mutex<HttpOverrideState>>,
    req: &MissionUploadRequest,
) -> Result<usize, String> {
    let items = build_mission_items(ids, req)?;

    {
        let store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        if store.upload_pending.is_some() {
            return Err("mission upload: another upload is in progress".into());
        }
    }

    {
        let mut os = override_state.lock().map_err(|e| format!("override_lock:{e}"))?;
        if matches!(&*os, HttpOverrideState::Resuming { .. }) {
            return Err("mission upload: wait for mission resume to finish".into());
        }
        *os = HttpOverrideState::Uploading;
    }

    {
        let mut store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        store.upload_done = false;
        store.set_upload_pending(items.clone());
    }

    let count = items.len() as u16;
    conn.send_default(&MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
        count,
        target_system: ids.system_id,
        target_component: ids.component_id,
    }))
    .map_err(|e| e.to_string())?;

    for _ in 0..150 {
        std::thread::sleep(Duration::from_millis(100));
        let done = mission
            .lock()
            .map_err(|e| format!("mission_lock:{e}"))?
            .upload_done;
        if done {
            return Ok(items.len());
        }
    }

    {
        let mut store = mission.lock().map_err(|e| format!("mission_lock:{e}"))?;
        store.upload_pending = None;
        store.upload_done = false;
    }
    if let Ok(mut os) = override_state.lock() {
        *os = HttpOverrideState::MissionRunning;
    }

    Err("mission upload: timed out waiting for MISSION_ACK from flight controller".into())
}
