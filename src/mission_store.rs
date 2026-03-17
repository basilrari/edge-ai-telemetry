//! Mission store: holds the current mission from the FC and a snapshot for override/resume.

#[allow(deprecated)]
use mavlink::ardupilotmega::MISSION_ITEM_INT_DATA;

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
}

impl Default for MissionStore {
    fn default() -> Self {
        Self {
            items: Vec::new(),
            current_seq: None,
            snapshot: None,
            upload_pending: None,
            upload_done: false,
        }
    }
}

impl MissionStore {
    pub fn new() -> Self {
        Self::default()
    }

    /// Update store from a received MISSION_ITEM_INT (from FC).
    pub fn update_from_item(&mut self, d: &MISSION_ITEM_INT_DATA) {
        let seq = d.seq;
        if let Some(pos) = self.items.iter().position(|w| w.seq == seq) {
            self.items[pos] = d.clone();
        } else {
            self.items.push(d.clone());
            self.items.sort_by_key(|w| w.seq);
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
        self.upload_pending = Some(items);
    }

    /// Get the next item to send for seq (for MISSION_REQUEST_INT response). Returns None if no upload or seq out of range.
    /// Does not clear upload_pending; that is done in set_upload_done() when MISSION_ACK is received.
    pub fn take_upload_item(&mut self, seq: u16) -> Option<StoredMissionItem> {
        let items = self.upload_pending.as_ref()?;
        items.get(seq as usize).cloned()
    }

    /// Mark upload as done (e.g. on MISSION_ACK). Clears upload_pending.
    pub fn set_upload_done(&mut self) {
        self.upload_pending = None;
        self.upload_done = true;
    }

    /// Whether we have a snapshot (override was started and not yet resumed).
    pub fn has_snapshot(&self) -> bool {
        self.snapshot.is_some()
    }

    /// Get snapshot for resume. Returns (items, current_seq) if present.
    pub fn get_snapshot(&self) -> Option<(&[StoredMissionItem], u16)> {
        self.snapshot.as_ref().map(|(a, b)| (a.as_slice(), *b))
    }
}
