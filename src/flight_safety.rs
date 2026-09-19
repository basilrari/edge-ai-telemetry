//! Apply-tool safety gates that need live telemetry (HTTP path only; TUI is operator-in-the-loop).

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::mavlink_http_runtime::TelemetryCache;

/// Relative altitude (m) treated as airborne. Same number `start_mission` uses to skip the mission takeoff.
pub const AIRBORNE_ALT_M: f64 = 2.5;

/// Disarm is refused at or above this relative altitude (includes descent that has not touched down).
pub const DISARM_BLOCK_ALT_M: f64 = 0.8;

/// How long `drone-http` will poll after a takeoff ACK before failing the step.
pub const TAKEOFF_COMPLETE_TIMEOUT: Duration = Duration::from_secs(40);

const TELEM_POLL: Duration = Duration::from_millis(200);

fn mode_is_landing(mode: &str) -> bool {
    matches!(
        mode,
        "LAND" | "RTL" | "QRTL" | "QLAND" | "AUTO_RTL"
    )
}

/// Height the climb wait treats as "takeoff complete".
///
/// Explicit `altitude_m`: 85% of that target, not below `AIRBORNE_ALT_M` unless the target itself is lower.
/// Omitted height: leave the ground (`AIRBORNE_ALT_M`).
pub fn takeoff_complete_need_m(target_m: Option<f32>) -> f64 {
    match target_m {
        Some(t) if t.is_finite() && t > 0.0 => {
            let t = t as f64;
            (t * 0.85).clamp(AIRBORNE_ALT_M.min(t), t)
        }
        _ => AIRBORNE_ALT_M,
    }
}

/// `Some` means do not send `COMPONENT_ARM_DISARM` (param=0).
pub fn disarm_block_reason(telem: &TelemetryCache) -> Option<String> {
    let landing = telem.mode_name.as_deref().is_some_and(mode_is_landing);
    if landing {
        let still_in_air = match telem.relative_alt_m {
            Some(alt) => alt >= DISARM_BLOCK_ALT_M,
            None => true,
        };
        if still_in_air {
            return Some(
                "disarm_blocked: aircraft is in LAND/RTL. Motors must keep running until it is on the ground."
                    .into(),
            );
        }
    }
    match telem.relative_alt_m {
        Some(alt) if alt >= DISARM_BLOCK_ALT_M => Some(format!(
            "disarm_blocked: aircraft is airborne (alt_rel={alt:.1} m). Motors would stop in flight. Land or RTL, then disarm on the ground."
        )),
        Some(_) => None,
        None => Some(
            "disarm_blocked: no altitude telemetry. Refusing to cut motors."
                .into(),
        ),
    }
}

/// Poll until relative altitude reaches the takeoff-complete height, or `timeout`.
pub fn wait_for_takeoff_altitude(
    telem: &Arc<Mutex<TelemetryCache>>,
    target_m: Option<f32>,
    timeout: Duration,
) -> Result<f64, String> {
    let need = takeoff_complete_need_m(target_m);
    let deadline = Instant::now() + timeout;
    loop {
        let alt = telem
            .lock()
            .map_err(|e| format!("telem_lock:{e}"))?
            .relative_alt_m;
        if let Some(a) = alt {
            if a >= need {
                return Ok(a);
            }
        }
        if Instant::now() >= deadline {
            let shown = alt
                .map(|a| format!("{a:.1}"))
                .unwrap_or_else(|| "unknown".into());
            return Err(format!(
                "takeoff_incomplete: alt_rel={shown} m, need {need:.1} m within {}s",
                timeout.as_secs()
            ));
        }
        std::thread::sleep(TELEM_POLL);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn telem_alt_mode(alt: Option<f64>, mode: Option<&str>) -> TelemetryCache {
        let mut telem = TelemetryCache::default();
        telem.relative_alt_m = alt;
        telem.mode_name = mode.map(str::to_string);
        telem
    }

    #[test]
    fn takeoff_need_uses_85_percent_of_explicit_height() {
        let need = takeoff_complete_need_m(Some(20.0));
        assert!((need - 17.0).abs() < 1e-9);
    }

    #[test]
    fn takeoff_need_without_height_is_airborne_floor() {
        assert_eq!(takeoff_complete_need_m(None), AIRBORNE_ALT_M);
        assert_eq!(takeoff_complete_need_m(Some(0.0)), AIRBORNE_ALT_M);
    }

    #[test]
    fn low_explicit_target_is_not_raised_above_itself() {
        assert_eq!(takeoff_complete_need_m(Some(2.0)), 2.0);
    }

    #[test]
    fn disarm_allowed_on_ground_with_alt() {
        assert_eq!(disarm_block_reason(&telem_alt_mode(Some(0.4), Some("STABILIZE"))), None);
    }

    #[test]
    fn disarm_blocked_when_airborne() {
        let r = disarm_block_reason(&telem_alt_mode(Some(12.4), Some("GUIDED"))).unwrap();
        assert!(r.starts_with("disarm_blocked:"));
        assert!(r.contains("12.4"));
    }

    #[test]
    fn disarm_blocked_just_off_the_ground() {
        let r = disarm_block_reason(&telem_alt_mode(Some(1.0), Some("GUIDED"))).unwrap();
        assert!(r.contains("1.0"));
    }

    #[test]
    fn disarm_blocked_in_land_even_near_ground() {
        let r = disarm_block_reason(&telem_alt_mode(Some(1.2), Some("LAND"))).unwrap();
        assert!(r.contains("LAND/RTL"));
    }

    #[test]
    fn disarm_allowed_in_land_after_touchdown() {
        assert_eq!(
            disarm_block_reason(&telem_alt_mode(Some(0.3), Some("LAND"))),
            None
        );
    }

    #[test]
    fn disarm_blocked_in_rtl() {
        assert!(disarm_block_reason(&telem_alt_mode(Some(8.0), Some("RTL")))
            .unwrap()
            .contains("LAND/RTL"));
    }

    #[test]
    fn disarm_blocked_without_altitude() {
        let r = disarm_block_reason(&telem_alt_mode(None, Some("STABILIZE"))).unwrap();
        assert!(r.contains("no altitude telemetry"));
    }

    #[test]
    fn wait_returns_immediately_if_already_at_height() {
        let telem = Arc::new(Mutex::new(telem_alt_mode(Some(20.0), None)));
        let a = wait_for_takeoff_altitude(&telem, Some(15.0), Duration::from_millis(50)).unwrap();
        assert!(a >= 15.0 * 0.85);
    }

    #[test]
    fn wait_times_out_on_the_ground() {
        let telem = Arc::new(Mutex::new(telem_alt_mode(Some(0.2), None)));
        let e = wait_for_takeoff_altitude(&telem, Some(15.0), Duration::from_millis(50)).unwrap_err();
        assert!(e.starts_with("takeoff_incomplete:"));
        assert!(e.contains("need"));
    }
}
