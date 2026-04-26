//! Shared constants for the TUI binary.

use std::time::Duration;

pub(crate) const U16_MAX: u16 = 65535;
pub(crate) const TARGET_SYSTEM: u8 = 1;
pub(crate) const TARGET_COMPONENT: u8 = 1;
/// Horizontal distance (m) to consider a waypoint "reached" during override.
pub(crate) const REACHED_THRESHOLD_M: f64 = 10.0;
pub(crate) const INTERNET_OFFLINE_RTL_AFTER_SECS: u64 = 30;
pub(crate) const INTERNET_CHECK_PERIOD_SECS: u64 = 2;

pub(crate) const RECENT_MESSAGES_MAX: usize = 32;
/// After this, if there was no matching FC reply, show a hint once.
pub(crate) const PENDING_CMD_TIMEOUT: Duration = Duration::from_secs(3);

/// Re-send stream setup until these MAVLink messages have been seen at least once.
pub(crate) const STREAM_AUTO_RETRY_FIRST_DELAY: Duration = Duration::from_millis(500);
pub(crate) const STREAM_AUTO_RETRY_INTERVAL: Duration = Duration::from_secs(2);
pub(crate) const STREAM_AUTO_RETRY_MAX_ATTEMPTS: u32 = 35;
