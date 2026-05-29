//! Mission store: holds the current mission from the FC and a snapshot for override/resume.

use std::collections::HashSet;

#[allow(deprecated)]
use mavlink::ardupilotmega::{MavCmd, MISSION_ITEM_INT_DATA};

/// Stored mission item for re-upload (clone of MAVLink MISSION_ITEM_INT).
pub type StoredMissionItem = MISSION_ITEM_INT_DATA;

/// Mission store: live mission from FC, snapshot at pause, and upload state.
pub struct MissionStore {
    /// Full mission items as received from the FC (MISSION_ITEM_INT).
    pub items: Vec<StoredMissionItem>,
    /// Current waypoint index from MISSION_CURRENT.
    pub current_seq: Option<u16>,
    /// Snapshot taken when we start override: (mission items, current_seq). Cleared on resume.
    pub snapshot: Option<(Vec<StoredMissionItem>, u16)>,
    /// When set, recv thread will respond to MISSION_REQUEST_INT with these items (upload to FC).
    pub upload_pending: Option<Vec<StoredMissionItem>>,
    /// Set to true when MISSION_ACK received during upload (so caller knows upload finished).
    pub upload_done: bool,
    /// Ignore the next MISSION_ACK (e.g. after MISSION_CLEAR_ALL before a planner upload).
    pub awaiting_clear_ack: bool,
    /// How many mission items we have sent to the FC during the current upload handshake.
    pub upload_items_sent: u16,
    /// Unique mission seq numbers successfully sent during the current upload.
    pub upload_sent_seqs: HashSet<u16>,
    /// Set when the FC rejects or cancels an in-progress upload.
    pub upload_failed: bool,
    pub upload_fail_reason: Option<String>,
}

impl Default for MissionStore {
    fn default() -> Self {
        Self {
            items: Vec::new(),
            current_seq: None,
            snapshot: None,
            upload_pending: None,
            upload_done: false,
            awaiting_clear_ack: false,
            upload_items_sent: 0,
            upload_sent_seqs: HashSet::new(),
            upload_failed: false,
            upload_fail_reason: None,
        }
    }
}

impl MissionStore {
    pub fn new() -> Self {
        Self::default()
    }

    /// Update store from a received MISSION_ITEM_INT (from FC).
    pub fn update_from_item(&mut self, d: &MISSION_ITEM_INT_DATA) {
        let mut item = d.clone();
        Self::normalize_item_command(&mut item);
        if let Some(existing) = self.items.iter().find(|w| w.seq == item.seq) {
            if existing.command == MavCmd::MAV_CMD_NAV_TAKEOFF
                && item.command == MavCmd::MAV_CMD_NAV_WAYPOINT
            {
                item.command = MavCmd::MAV_CMD_NAV_TAKEOFF;
            }
        }
        let seq = item.seq;
        if let Some(pos) = self.items.iter().position(|w| w.seq == seq) {
            self.items[pos] = item;
        } else {
            self.items.push(item);
            self.items.sort_by_key(|w| w.seq);
        }
    }

    /// ArduPilot may report the takeoff slot as NAV_WAYPOINT at 0,0 — keep it as TAKEOFF for UI/tools.
    fn normalize_item_command(item: &mut MISSION_ITEM_INT_DATA) {
        if item.command == MavCmd::MAV_CMD_NAV_TAKEOFF {
            return;
        }
        if item.seq == 0
            && item.command == MavCmd::MAV_CMD_NAV_WAYPOINT
            && item.z > 0.0
            && item.x == 0
            && item.y == 0
        {
            item.command = MavCmd::MAV_CMD_NAV_TAKEOFF;
        }
    }

    /// Update current sequence from MISSION_CURRENT.
    /// If we already have a snapshot, update its seq so it stays in sync (enables multiple interrupts).
    pub fn update_current_seq(&mut self, seq: u16) {
        self.current_seq = Some(seq);
        if let Some((_, s)) = self.snapshot.as_mut() {
            *s = seq;
        }
    }

    /// Take a snapshot of current mission for override. Call when switching to GUIDED for override.
    /// Returns true if we had a non-empty mission to snapshot.
    pub fn snapshot_for_override(&mut self) -> bool {
        let seq = match self.current_seq {
            Some(s) => s,
            None => return false,
        };
        if self.items.is_empty() {
            return false;
        }
        self.snapshot = Some((self.items.clone(), seq));
        true
    }

    /// Ensure we have a snapshot for pause/interrupt. Uses existing snapshot (seq updated by MISSION_CURRENT)
    /// or takes a new one. Returns true if we have a valid snapshot to use.
    pub fn ensure_snapshot_for_pause(&mut self) -> bool {
        if self.snapshot.is_some() {
            return true;
        }
        self.snapshot_for_override()
    }

    /// Clear snapshot after resume. Not used when we want multiple interrupts (snapshot is kept and seq updated).
    pub fn clear_snapshot(&mut self) {
        self.snapshot = None;
    }

    /// Start upload: set pending items. Caller must then send MISSION_COUNT(count).
    pub fn set_upload_pending(&mut self, items: Vec<StoredMissionItem>) {
        self.upload_done = false;
        self.upload_failed = false;
        self.upload_fail_reason = None;
        self.upload_items_sent = 0;
        self.upload_sent_seqs.clear();
        self.upload_pending = Some(items);
    }

    pub fn note_upload_item_sent(&mut self, seq: u16) {
        self.upload_items_sent = self.upload_items_sent.saturating_add(1);
        self.upload_sent_seqs.insert(seq);
    }

    /// True once every pending item seq has been sent at least once (ArduPilot pull protocol).
    pub fn all_upload_items_sent(&self) -> bool {
        let Some(items) = &self.upload_pending else {
            return false;
        };
        !items.is_empty()
            && items
                .iter()
                .all(|it| self.upload_sent_seqs.contains(&it.seq))
    }

    pub fn upload_ready_for_ack(&self) -> bool {
        self.all_upload_items_sent()
    }

    pub fn mark_upload_failed(&mut self, reason: impl Into<String>) {
        self.upload_failed = true;
        self.upload_fail_reason = Some(reason.into());
        self.upload_pending = None;
        self.upload_sent_seqs.clear();
        self.upload_items_sent = 0;
    }

    /// Get the item to send for seq (for MISSION_REQUEST(_INT) response).
    pub fn take_upload_item(&mut self, seq: u16) -> Option<StoredMissionItem> {
        let items = self.upload_pending.as_ref()?;
        items
            .iter()
            .find(|it| it.seq == seq)
            .cloned()
            .or_else(|| items.get(seq as usize).cloned())
    }

    /// Mark upload as done (e.g. on MISSION_ACK). Clears upload_pending.
    pub fn set_upload_done(&mut self) {
        self.upload_pending = None;
        self.upload_done = true;
        self.upload_failed = false;
        self.upload_fail_reason = None;
        self.awaiting_clear_ack = false;
        self.upload_items_sent = 0;
        self.upload_sent_seqs.clear();
    }

    /// Drop all cached mission state (after FC clear or local reset).
    pub fn clear_local(&mut self) {
        self.items.clear();
        self.current_seq = None;
        self.snapshot = None;
        self.upload_pending = None;
        self.upload_done = false;
        self.awaiting_clear_ack = false;
        self.upload_items_sent = 0;
        self.upload_sent_seqs.clear();
        self.upload_failed = false;
        self.upload_fail_reason = None;
    }

    /// Whether we have a snapshot (override was started and not yet resumed).
    pub fn has_snapshot(&self) -> bool {
        self.snapshot.is_some()
    }

    /// Get snapshot for resume. Returns (items, current_seq) if present.
    pub fn get_snapshot(&self) -> Option<(&[StoredMissionItem], u16)> {
        self.snapshot.as_ref().map(|(a, b)| (a.as_slice(), *b))
    }

    /// Whether the loaded mission includes a NAV_TAKEOFF item (ArduCopter AUTO requirement).
    pub fn has_nav_takeoff(&self) -> bool {
        Self::items_have_nav_takeoff(&self.items)
    }

    pub fn items_have_nav_takeoff(items: &[StoredMissionItem]) -> bool {
        items
            .iter()
            .any(|it| it.command == MavCmd::MAV_CMD_NAV_TAKEOFF)
    }

    /// Same checks as TUI **`m`** before AUTO + MISSION_START: mission downloaded and includes NAV_TAKEOFF.
    pub fn validate_ready_for_start_mission(&self) -> Result<(), String> {
        if self.items.is_empty() {
            return Err(
                "start_mission: no mission on the link — upload a mission with takeoff from the Mission page first, then try again.".to_string(),
            );
        }
        if !self.has_nav_takeoff() {
            return Err(format!(
                "start_mission: loaded mission has {} item(s) but no NAV_TAKEOFF (ArduCopter AUTO needs a TAKEOFF mission item first). \
                 Upload a mission with takeoff from the Mission Planner — prompts cannot modify waypoints on the FC.",
                self.items.len()
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::ardupilotmega::MavFrame;

    fn wp_item(seq: u16, command: MavCmd, x: i32, y: i32, z: f32) -> MISSION_ITEM_INT_DATA {
        MISSION_ITEM_INT_DATA {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x,
            y,
            z,
            seq,
            command,
            target_system: 1,
            target_component: 1,
            frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current: 0,
            autocontinue: 1,
        }
    }

    #[test]
    fn normalize_zero_zero_waypoint_as_takeoff() {
        let mut item = wp_item(0, MavCmd::MAV_CMD_NAV_WAYPOINT, 0, 0, 15.0);
        MissionStore::normalize_item_command(&mut item);
        assert_eq!(item.command, MavCmd::MAV_CMD_NAV_TAKEOFF);
    }

    #[test]
    fn preserve_takeoff_when_fc_rewrites_command() {
        let mut store = MissionStore::new();
        store.update_from_item(&wp_item(
            0,
            MavCmd::MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            15.0,
        ));
        store.update_from_item(&wp_item(
            0,
            MavCmd::MAV_CMD_NAV_WAYPOINT,
            23_558_000,
            120_473_000,
            15.0,
        ));
        assert_eq!(store.items[0].command, MavCmd::MAV_CMD_NAV_TAKEOFF);
    }

    #[test]
    fn upload_ready_requires_unique_seqs_not_retry_count() {
        let mut store = MissionStore::new();
        store.set_upload_pending(vec![
            wp_item(0, MavCmd::MAV_CMD_NAV_TAKEOFF, 0, 0, 15.0),
            wp_item(1, MavCmd::MAV_CMD_NAV_WAYPOINT, 1, 1, 15.0),
        ]);
        for _ in 0..6 {
            store.note_upload_item_sent(0);
        }
        assert!(!store.upload_ready_for_ack());
        store.note_upload_item_sent(1);
        assert!(store.upload_ready_for_ack());
    }
}
