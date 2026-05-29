//! Flight events + MAVLink (Pixhawk) log buffers with WebSocket broadcast.

use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde::Serialize;
use tokio::sync::broadcast;

use crate::flight_log::FlightLogEntry;
use crate::mavlink_log::{format_mavlink_frame, MavlinkLogEntry};
use mavlink::ardupilotmega::MavMessage;
use mavlink::MavFrame;

const MAX_FLIGHT: usize = 500;
const MAX_MAVLINK: usize = 800;

const HIGH_RATE_MIN_INTERVAL: Duration = Duration::from_millis(900);

#[derive(Clone, Debug, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum LogWsMessage {
    Snapshot {
        flight: Vec<FlightLogEntry>,
        mavlink: Vec<MavlinkLogEntry>,
    },
    Flight {
        entry: FlightLogEntry,
    },
    Mavlink {
        entry: MavlinkLogEntry,
    },
}

#[derive(Clone)]
pub struct LogsHub {
    inner: Arc<Mutex<LogsHubInner>>,
    tx: broadcast::Sender<LogWsMessage>,
}

struct LogsHubInner {
    flight: VecDeque<FlightLogEntry>,
    mavlink: VecDeque<MavlinkLogEntry>,
    last_mavlink_emit: HashMap<String, Instant>,
}

impl LogsHub {
    pub fn new() -> Self {
        let (tx, _) = broadcast::channel(256);
        Self {
            inner: Arc::new(Mutex::new(LogsHubInner {
                flight: VecDeque::new(),
                mavlink: VecDeque::new(),
                last_mavlink_emit: HashMap::new(),
            })),
            tx,
        }
    }

    pub fn subscribe(&self) -> broadcast::Receiver<LogWsMessage> {
        self.tx.subscribe()
    }

    pub fn snapshot_ws(&self) -> LogWsMessage {
        let inner = self.inner.lock().unwrap();
        LogWsMessage::Snapshot {
            flight: inner.flight.iter().cloned().collect(),
            mavlink: inner.mavlink.iter().cloned().collect(),
        }
    }

    pub fn flight_snapshot(&self) -> Vec<FlightLogEntry> {
        self.inner
            .lock()
            .map(|q| q.flight.iter().cloned().collect())
            .unwrap_or_default()
    }

    pub fn mavlink_snapshot(&self) -> Vec<MavlinkLogEntry> {
        self.inner
            .lock()
            .map(|q| q.mavlink.iter().cloned().collect())
            .unwrap_or_default()
    }

    pub fn push_flight(&self, level: &str, message: impl Into<String>) {
        let ts_ms = now_ms();
        let entry = FlightLogEntry {
            ts_ms,
            level: level.to_string(),
            message: message.into(),
        };
        if let Ok(mut inner) = self.inner.lock() {
            inner.flight.push_back(entry.clone());
            while inner.flight.len() > MAX_FLIGHT {
                inner.flight.pop_front();
            }
        }
        let _ = self.tx.send(LogWsMessage::Flight { entry });
    }

    pub fn log_mavlink_frame(&self, frame: &MavFrame<MavMessage>) {
        let Some(entry) = format_mavlink_frame(frame) else {
            return;
        };
        if !Self::should_emit_mavlink(&self.inner, &entry.msg_name) {
            return;
        }
        if let Ok(mut inner) = self.inner.lock() {
            inner.mavlink.push_back(entry.clone());
            while inner.mavlink.len() > MAX_MAVLINK {
                inner.mavlink.pop_front();
            }
        }
        let _ = self.tx.send(LogWsMessage::Mavlink { entry });
    }

    fn should_emit_mavlink(inner: &Arc<Mutex<LogsHubInner>>, msg_name: &str) -> bool {
        let always = matches!(
            msg_name,
            "STATUSTEXT" | "MISSION_CURRENT" | "COMMAND_ACK"
        );
        if always {
            return true;
        }
        let Ok(mut guard) = inner.lock() else {
            return false;
        };
        let now = Instant::now();
        let last = guard.last_mavlink_emit.get(msg_name).copied();
        if let Some(prev) = last {
            if now.duration_since(prev) < HIGH_RATE_MIN_INTERVAL {
                return false;
            }
        }
        guard.last_mavlink_emit.insert(msg_name.to_string(), now);
        true
    }
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}
