//! Broadcast channel for live telemetry snapshots (WebSocket subscribers).

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde::Serialize;
use tokio::sync::broadcast;

use crate::mavlink_connect::LinkInfo;
use crate::mavlink_http_runtime::{arducopter_mode_name, TelemetryCache};

const MIN_PUBLISH_INTERVAL: Duration = Duration::from_millis(100);

#[derive(Clone, Debug, Serialize)]
pub struct TelemetrySnapshot {
    pub ok: bool,
    pub link: LinkInfo,
    pub lat_deg: Option<f64>,
    pub lon_deg: Option<f64>,
    pub alt_amsl_m: Option<f64>,
    pub alt_rel_m: Option<f64>,
    pub groundspeed_m_s: Option<f32>,
    pub airspeed_m_s: Option<f32>,
    pub climb_m_s: Option<f32>,
    pub heading_deg: Option<i16>,
    pub roll_deg: Option<f32>,
    pub pitch_deg: Option<f32>,
    pub yaw_deg: Option<f32>,
    pub armed: Option<bool>,
    pub mode: Option<String>,
    pub ts_ms: u64,
}

impl TelemetrySnapshot {
    pub fn from_cache(link: &LinkInfo, t: &TelemetryCache) -> Self {
        let mode = t
            .mode_name
            .clone()
            .or_else(|| t.heartbeat_custom_mode.map(|m| arducopter_mode_name(m).to_string()));
        Self {
            ok: t.lat.is_some() && t.lon.is_some(),
            link: link.clone(),
            lat_deg: t.lat,
            lon_deg: t.lon,
            alt_amsl_m: t.alt_amsl_m,
            alt_rel_m: t.relative_alt_m,
            groundspeed_m_s: t.groundspeed_m_s,
            airspeed_m_s: t.airspeed_m_s,
            climb_m_s: t.climb_m_s,
            heading_deg: t.heading_deg,
            roll_deg: t.roll_deg,
            pitch_deg: t.pitch_deg,
            yaw_deg: t.yaw_deg,
            armed: t.armed,
            mode,
            ts_ms: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_millis() as u64)
                .unwrap_or(0),
        }
    }
}

#[derive(Clone)]
pub struct TelemetryHub {
    tx: broadcast::Sender<TelemetrySnapshot>,
    throttle: Arc<Mutex<Instant>>,
}

impl TelemetryHub {
    pub fn new() -> Self {
        let (tx, _) = broadcast::channel(128);
        Self {
            tx,
            throttle: Arc::new(Mutex::new(Instant::now() - MIN_PUBLISH_INTERVAL)),
        }
    }

    pub fn subscribe(&self) -> broadcast::Receiver<TelemetrySnapshot> {
        self.tx.subscribe()
    }

    /// Publish at most ~10 Hz (called from MAVLink recv thread).
    pub fn maybe_publish(&self, link: &LinkInfo, telem: &TelemetryCache) {
        let mut gate = self.throttle.lock().unwrap();
        if gate.elapsed() < MIN_PUBLISH_INTERVAL {
            return;
        }
        *gate = Instant::now();
        drop(gate);
        let snap = TelemetrySnapshot::from_cache(link, telem);
        let _ = self.tx.send(snap);
    }

    pub fn snapshot_now(&self, link: &LinkInfo, telem: &TelemetryCache) -> TelemetrySnapshot {
        TelemetrySnapshot::from_cache(link, telem)
    }
}
