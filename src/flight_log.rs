//! Ring buffer of recent flight / MAVLink events for the HTTP UI.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

const MAX_ENTRIES: usize = 500;

#[derive(Clone, Debug, serde::Serialize)]
pub struct FlightLogEntry {
    pub ts_ms: u64,
    pub level: String,
    pub message: String,
}

#[derive(Clone, Default)]
pub struct FlightLog(Arc<Mutex<VecDeque<FlightLogEntry>>>);

impl FlightLog {
    pub fn new() -> Self {
        Self(Arc::new(Mutex::new(VecDeque::new())))
    }

    pub fn push(&self, level: &str, message: impl Into<String>) {
        let ts_ms = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0);
        let entry = FlightLogEntry {
            ts_ms,
            level: level.to_string(),
            message: message.into(),
        };
        if let Ok(mut q) = self.0.lock() {
            q.push_back(entry);
            while q.len() > MAX_ENTRIES {
                q.pop_front();
            }
        }
    }

    pub fn snapshot(&self) -> Vec<FlightLogEntry> {
        self.0
            .lock()
            .map(|q| q.iter().cloned().collect())
            .unwrap_or_default()
    }
}
