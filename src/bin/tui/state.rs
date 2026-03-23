//! TUI runtime state: telemetry, override flow, net mirror, MAVLink coverage.

use std::collections::VecDeque;
use std::time::Instant;

use drone_server::VehicleIds;
use mavlink::ardupilotmega::{MavCmd, MavMessage};

use crate::consts::RECENT_MESSAGES_MAX;

/// State for override/resume: normal mission, paused (interrupt, wait for 'c'), running override waypoints, or resuming.
#[derive(Clone, Debug)]
pub(crate) enum OverrideState {
    MissionRunning,
    /// Interrupt: drone is hovering, press 'c' to resume mission. Can press 'w' to inject a waypoint.
    Paused,
    OverrideActive {
        waypoints: Vec<(f64, f64, f64)>,
        index: usize,
        /// When true, resume mission after last waypoint; when false, go to Paused.
        resume_after: bool,
    },
    Resuming { resume_seq: u16 },
}

/// Tracks which high-rate telemetry types we have received (recv thread).
#[derive(Default)]
pub(crate) struct TelemetryCoverage {
    pub(crate) heartbeat: bool,
    pub(crate) attitude: bool,
    pub(crate) global_position_int: bool,
    pub(crate) gps_raw_int: bool,
    pub(crate) sys_status: bool,
    pub(crate) vfr_hud: bool,
    pub(crate) home_position: bool,
}

impl TelemetryCoverage {
    pub(crate) fn update(&mut self, msg: &MavMessage) {
        match msg {
            MavMessage::HEARTBEAT(_) => self.heartbeat = true,
            MavMessage::ATTITUDE(_) => self.attitude = true,
            MavMessage::GLOBAL_POSITION_INT(_) => self.global_position_int = true,
            MavMessage::GPS_RAW_INT(_) => self.gps_raw_int = true,
            MavMessage::SYS_STATUS(_) => self.sys_status = true,
            MavMessage::VFR_HUD(_) => self.vfr_hud = true,
            MavMessage::HOME_POSITION(_) => self.home_position = true,
            _ => {}
        }
    }

    pub(crate) fn is_complete(&self) -> bool {
        self.heartbeat
            && self.attitude
            && self.global_position_int
            && self.gps_raw_int
            && self.sys_status
            && self.vfr_hud
            && self.home_position
    }
}

/// Last command we sent from the TUI: used to correlate COMMAND_ACK and mode telemetry.
#[derive(Clone)]
pub(crate) struct PendingFeedback {
    pub(crate) label: String,
    /// COMMAND_ACK.command we expect for this action (if FC sends ACK).
    pub(crate) expect_cmd: Option<MavCmd>,
    /// ArduCopter `custom_mode` we expect on HEARTBEAT after SET_MODE / DO_SET_MODE (fallback).
    pub(crate) expect_copter_mode: Option<u32>,
    pub(crate) sent_at: Instant,
    pub(crate) timeout_warned: bool,
}

impl PendingFeedback {
    pub(crate) fn new(
        label: impl Into<String>,
        expect_cmd: Option<MavCmd>,
        expect_copter_mode: Option<u32>,
    ) -> Self {
        Self {
            label: label.into(),
            expect_cmd,
            expect_copter_mode,
            sent_at: Instant::now(),
            timeout_warned: false,
        }
    }
}

#[derive(Clone)]
pub(crate) struct Waypoint {
    pub(crate) seq: u16,
    pub(crate) command: u16,
    pub(crate) lat: f64,
    pub(crate) lon: f64,
    pub(crate) alt: f32,
    #[allow(dead_code)]
    pub(crate) frame: u8,
}

#[derive(Default, Clone, Copy)]
pub(crate) struct NetWatchdogStatus {
    pub(crate) online: Option<bool>,
    pub(crate) last_check: Option<Instant>,
    pub(crate) last_ok: Option<Instant>,
    pub(crate) offline_since: Option<Instant>,
    pub(crate) rtl_sent_for_current_outage: bool,
}

#[derive(Default)]
pub(crate) struct TelemetryState {
    pub(crate) heartbeat_status: Option<String>,
    pub(crate) heartbeat_mode: Option<String>,
    pub(crate) heartbeat_custom: Option<u32>,
    pub(crate) roll: Option<f32>,
    pub(crate) pitch: Option<f32>,
    pub(crate) yaw: Option<f32>,
    pub(crate) gps_fix: Option<String>,
    pub(crate) gps_sats: Option<u8>,
    pub(crate) gps_hdop: Option<String>,
    pub(crate) lat: Option<f64>,
    pub(crate) lon: Option<f64>,
    pub(crate) alt: Option<f64>,
    pub(crate) home_lat: Option<f64>,
    pub(crate) home_lon: Option<f64>,
    pub(crate) home_alt: Option<f64>,
    pub(crate) vbat: Option<f32>,
    pub(crate) batt_pct: Option<String>,
    pub(crate) cell1_v: Option<f32>,
    pub(crate) airspeed: Option<f32>,
    pub(crate) groundspeed: Option<f32>,
    pub(crate) heading: Option<i16>,
    pub(crate) throttle: Option<u16>,
    pub(crate) climb: Option<f32>,
    pub(crate) vehicle_info: Vec<String>,
    pub(crate) recent_messages: VecDeque<String>,
    pub(crate) first_heartbeat_logged: bool,
    pub(crate) vehicle_sysid: Option<u8>,
    pub(crate) vehicle_compid: Option<u8>,
    pub(crate) vehicle_type_name: Option<String>,
    pub(crate) vehicle_mode_name: Option<String>,
    /// base_mode from HEARTBEAT (bits for MAV_MODE_FLAG).
    pub(crate) heartbeat_base_mode_bits: Option<u8>,
    pub(crate) armed: Option<bool>,
    pub(crate) sys_voltage: Option<f32>,
    pub(crate) sys_current: Option<f32>,
    pub(crate) sys_load: Option<u16>,
    pub(crate) time_boot_ms: Option<u32>,
    pub(crate) mission_waypoints: Vec<Waypoint>,
    pub(crate) mission_current_seq: Option<u16>,
    pub(crate) net_online: Option<bool>,
    pub(crate) net_secs_since_last_check: Option<u64>,
    pub(crate) net_secs_since_last_ok: Option<u64>,
    pub(crate) net_offline_secs: Option<u64>,
    pub(crate) net_rtl_sent_for_current_outage: bool,
    /// When true, draw the help popup (h to toggle).
    pub(crate) show_help_popup: bool,
    /// Last TUI command we are waiting to correlate with FC (COMMAND_ACK / mode).
    pub(crate) pending_feedback: Option<PendingFeedback>,
}

impl TelemetryState {
    pub(crate) fn push_recent(&mut self, line: String) {
        self.recent_messages.push_back(line);
        while self.recent_messages.len() > RECENT_MESSAGES_MAX {
            self.recent_messages.pop_front();
        }
    }
}

pub(crate) fn vehicle_ids_from_state(state: &TelemetryState) -> VehicleIds {
    VehicleIds::new(
        state.vehicle_sysid.unwrap_or(crate::consts::TARGET_SYSTEM),
        state.vehicle_compid.unwrap_or(crate::consts::TARGET_COMPONENT),
    )
}
