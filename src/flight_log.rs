//! Flight log entry type for the HTTP UI.

#[derive(Clone, Debug, serde::Serialize)]
pub struct FlightLogEntry {
    pub ts_ms: u64,
    pub level: String,
    pub message: String,
}
