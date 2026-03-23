//! Shared constants for the TUI binary.

use std::time::Duration;

pub(crate) const U16_MAX: u16 = 65535;
pub(crate) const TARGET_SYSTEM: u8 = 1;
pub(crate) const TARGET_COMPONENT: u8 = 1;
/// Horizontal distance (m) to consider a waypoint "reached" during override.
pub(crate) const REACHED_THRESHOLD_M: f64 = 10.0;
pub(crate) const INTERNET_OFFLINE_RTL_AFTER_SECS: u64 = 30;
pub(crate) const INTERNET_CHECK_PERIOD_SECS: u64 = 2;

pub(crate) const MSG_ID_ATTITUDE: f32 = 30.0;
pub(crate) const MSG_ID_GLOBAL_POSITION_INT: f32 = 33.0;
pub(crate) const MSG_ID_SYS_STATUS: f32 = 1.0;
pub(crate) const MSG_ID_BATTERY_STATUS: f32 = 147.0;
pub(crate) const MSG_ID_GPS_RAW_INT: f32 = 24.0;
pub(crate) const MSG_ID_HOME_POSITION: f32 = 242.0;
pub(crate) const MSG_ID_VFR_HUD: f32 = 74.0;
pub(crate) const MSG_ID_RAW_IMU: f32 = 27.0;
pub(crate) const MSG_ID_LOCAL_POSITION_NED: f32 = 32.0;
pub(crate) const MSG_ID_RC_CHANNELS: f32 = 65.0;
pub(crate) const MSG_ID_SERVO_OUTPUT_RAW: f32 = 36.0;
pub(crate) const MSG_ID_NAV_CONTROLLER_OUTPUT: f32 = 62.0;
pub(crate) const MSG_ID_AHRS2: f32 = 178.0;
pub(crate) const MSG_ID_EKF_STATUS_REPORT: f32 = 193.0;
pub(crate) const MSG_ID_VIBRATION: f32 = 241.0;
pub(crate) const MSG_ID_DISTANCE_SENSOR: f32 = 132.0;
pub(crate) const MSG_ID_STATUSTEXT: f32 = 253.0;
pub(crate) const MSG_ID_MISSION_CURRENT: f32 = 42.0;
pub(crate) const MSG_ID_PARAM_VALUE: f32 = 22.0;
pub(crate) const MSG_ID_COMMAND_ACK: f32 = 77.0;
pub(crate) const MSG_ID_SYSTEM_TIME: f32 = 2.0;

pub(crate) const RECENT_MESSAGES_MAX: usize = 32;
/// After this, if there was no matching FC reply, show a hint once.
pub(crate) const PENDING_CMD_TIMEOUT: Duration = Duration::from_secs(3);

/// Re-send stream setup until these MAVLink messages have been seen at least once.
pub(crate) const STREAM_AUTO_RETRY_FIRST_DELAY: Duration = Duration::from_millis(500);
pub(crate) const STREAM_AUTO_RETRY_INTERVAL: Duration = Duration::from_secs(2);
pub(crate) const STREAM_AUTO_RETRY_MAX_ATTEMPTS: u32 = 35;
