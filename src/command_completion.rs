//! Correlate outbound MAVLink commands with inbound `COMMAND_ACK` for HTTP apply-tool.

use mavlink::ardupilotmega::{MavCmd, MavResult};
use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompletionStatus {
    Acked,
    Rejected,
    Timeout,
    NotApplicable,
    DispatchFailed,
}

impl CompletionStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Acked => "acked",
            Self::Rejected => "rejected",
            Self::Timeout => "timeout",
            Self::NotApplicable => "not_applicable",
            Self::DispatchFailed => "dispatch_failed",
        }
    }
}

#[derive(Debug, Clone)]
pub struct AckWaitOutcome {
    pub status: CompletionStatus,
    pub ack_command: Option<String>,
    pub ack_result: Option<String>,
    pub ack_wait_ms: u64,
}

struct PendingEntry {
    step_id: String,
    expect_cmd: MavCmd,
    resolved: Option<MavResult>,
    registered_at: Instant,
}

#[derive(Default)]
struct HubInner {
    /// FIFO per expected command (one ACK satisfies the oldest pending waiter).
    queues: std::collections::HashMap<u32, VecDeque<PendingEntry>>,
    by_id: std::collections::HashMap<String, MavResult>,
}

/// Shared between the MAVLink recv thread and HTTP apply-tool workers.
#[derive(Clone, Default)]
pub struct CommandCompletionHub {
    inner: Arc<(Mutex<HubInner>, Condvar)>,
}

impl CommandCompletionHub {
    pub fn register(&self, step_id: String, expect_cmd: MavCmd) {
        let (lock, cv) = &*self.inner;
        let mut g = lock.lock().expect("completion hub lock");
        let key = cmd_key(expect_cmd);
        g.by_id.remove(&step_id);
        g.queues.entry(key).or_default().push_back(PendingEntry {
            step_id: step_id.clone(),
            expect_cmd,
            resolved: None,
            registered_at: Instant::now(),
        });
        drop(g);
        cv.notify_all();
    }

    pub fn unregister(&self, step_id: &str) {
        let (lock, cv) = &*self.inner;
        let mut g = lock.lock().expect("completion hub lock");
        g.by_id.remove(step_id);
        for q in g.queues.values_mut() {
            q.retain(|e| e.step_id != step_id);
        }
        drop(g);
        cv.notify_all();
    }

    pub fn on_command_ack(&self, cmd: MavCmd, result: MavResult) {
        let (lock, cv) = &*self.inner;
        let mut g = lock.lock().expect("completion hub lock");
        let key = cmd_key(cmd);
        if let Some(q) = g.queues.get_mut(&key) {
            if let Some(mut entry) = q.pop_front() {
                entry.resolved = Some(result);
                g.by_id.insert(entry.step_id.clone(), result);
            }
        }
        drop(g);
        cv.notify_all();
    }

    pub fn wait(&self, step_id: &str, timeout: Duration) -> AckWaitOutcome {
        let (lock, cv) = &*self.inner;
        let deadline = Instant::now() + timeout;
        let start = Instant::now();
        loop {
            let mut g = lock.lock().expect("completion hub lock");
            if let Some(result) = g.by_id.get(step_id).copied() {
                g.by_id.remove(step_id);
                let status = if result == MavResult::MAV_RESULT_ACCEPTED {
                    CompletionStatus::Acked
                } else {
                    CompletionStatus::Rejected
                };
                return AckWaitOutcome {
                    status,
                    ack_command: None,
                    ack_result: Some(format!("{result:?}")),
                    ack_wait_ms: start.elapsed().as_millis() as u64,
                };
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                for q in g.queues.values_mut() {
                    q.retain(|e| e.step_id != step_id);
                }
                g.by_id.remove(step_id);
                return AckWaitOutcome {
                    status: CompletionStatus::Timeout,
                    ack_command: None,
                    ack_result: None,
                    ack_wait_ms: start.elapsed().as_millis() as u64,
                };
            }
            g = cv
                .wait_timeout(g, remaining)
                .expect("completion hub wait")
                .0;
        }
    }
}

fn cmd_key(cmd: MavCmd) -> u32 {
    cmd as u32
}

pub fn mav_result_accepted(result: MavResult) -> bool {
    result == MavResult::MAV_RESULT_ACCEPTED
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ack_resolves_registered_waiter() {
        let hub = CommandCompletionHub::default();
        hub.register("step-1".into(), MavCmd::MAV_CMD_NAV_TAKEOFF);
        std::thread::spawn({
            let hub = hub.clone();
            move || {
                std::thread::sleep(Duration::from_millis(20));
                hub.on_command_ack(
                    MavCmd::MAV_CMD_NAV_TAKEOFF,
                    MavResult::MAV_RESULT_ACCEPTED,
                );
            }
        });
        let out = hub.wait("step-1", Duration::from_secs(1));
        assert_eq!(out.status, CompletionStatus::Acked);
    }

    #[test]
    fn rejected_ack_maps_to_rejected_status() {
        let hub = CommandCompletionHub::default();
        hub.register("s".into(), MavCmd::MAV_CMD_COMPONENT_ARM_DISARM);
        hub.on_command_ack(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            MavResult::MAV_RESULT_DENIED,
        );
        let out = hub.wait("s", Duration::from_millis(100));
        assert_eq!(out.status, CompletionStatus::Rejected);
    }

    #[test]
    fn timeout_when_no_ack() {
        let hub = CommandCompletionHub::default();
        hub.register("t".into(), MavCmd::MAV_CMD_NAV_LAND);
        let out = hub.wait("t", Duration::from_millis(30));
        assert_eq!(out.status, CompletionStatus::Timeout);
    }
}
