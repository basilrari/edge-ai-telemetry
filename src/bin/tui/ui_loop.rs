//! Main TUI event loop: keyboard handling, telemetry drain, draw.

use std::io;
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use drone_server::{
    force_arm, goto_global_command_int, land, rtl, set_mode_guided, MissionStore, VehicleIds,
};
use mavlink::ardupilotmega::{MavCmd, MavMessage};
use mavlink::{MavConnection, MavFrame};
use ratatui::Terminal;

use crate::consts::{TARGET_COMPONENT, TARGET_SYSTEM};
use drone_server::geo::parse_waypoint_input;
use crate::mavlink_cmd::{
    cmd_arm, cmd_disarm, cmd_mission_start, cmd_set_mode_auto_long, cmd_set_mode_guided_long,
    cmd_takeoff_alt, with_vehicle,
};
use crate::render::draw_ui;
use crate::state::{
    NetWatchdogStatus, OverrideState, PendingFeedback, TelemetryState, vehicle_ids_from_state,
};
use crate::telemetry::{
    apply_message, check_pending_feedback_timeout, log_outgoing, log_outgoing_two,
};

pub(crate) fn run_ui<C: MavConnection<MavMessage> + Send>(
    rx: mpsc::Receiver<MavFrame<MavMessage>>,
    log_rx: mpsc::Receiver<String>,
    stream_retry_tx: mpsc::Sender<()>,
    conn: Arc<Mutex<C>>,
    mission_store: Arc<Mutex<MissionStore>>,
    override_state: Arc<Mutex<OverrideState>>,
    net_watchdog_status: Arc<Mutex<NetWatchdogStatus>>,
) -> io::Result<()> {
    crossterm::terminal::enable_raw_mode()?;
    let backend = ratatui::backend::CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;
    crossterm::execute!(
        io::stdout(),
        crossterm::terminal::EnterAlternateScreen
    )?;

    let mut state = TelemetryState::default();
    let mut waypoint_input: Option<String> = None;

    'ui: loop {
        // Handle keys before MAVLink so "[1] TUI → link" is queued before any COMMAND_ACK in the
        // same frame, and the following draw shows it immediately.
        'keys: while event::poll(Duration::ZERO)? {
            match event::read()? {
                Event::Key(key) => {
                if key.kind != KeyEventKind::Press {
                    continue 'keys;
                }
                if waypoint_input.is_some() {
                    match key.code {
                        KeyCode::Enter => {
                            let s = waypoint_input.take().unwrap_or_default();
                            let s = s.trim().to_string();
                            let (lat, lon, alt) = match parse_waypoint_input(&s, state.lat, state.lon, state.alt) {
                                Ok(t) => t,
                                Err(e) => {
                                    waypoint_input = Some(s);
                                    state.push_recent(format!("Waypoint parse: {}", e));
                                    continue 'ui;
                                }
                            };
                            let (ok, resume_after) = {
                                let mut os = match override_state.lock() {
                                    Ok(g) => g,
                                    Err(_) => continue 'ui,
                                };
                                if matches!(&*os, OverrideState::OverrideActive { .. }) {
                                    state.push_recent("Override: finish current override first.".to_string());
                                    continue 'ui;
                                }
                                let from_paused = matches!(&*os, OverrideState::Paused);
                                let resume_after = !from_paused; // from mission => resume after; from paused => stay paused after
                                if !from_paused {
                                    let mut store = match mission_store.lock() {
                                        Ok(g) => g,
                                        Err(_) => continue 'ui,
                                    };
                                    if !store.ensure_snapshot_for_pause() {
                                        state.push_recent("Override: no mission or current WP (wait for mission download).".to_string());
                                        continue 'ui;
                                    }
                                }
                                *os = OverrideState::OverrideActive {
                                    waypoints: vec![(lat, lon, alt)],
                                    index: 0,
                                    resume_after,
                                };
                                (true, resume_after)
                            };
                            if ok {
                                let ids = VehicleIds::new(
                                    state.vehicle_sysid.unwrap_or(TARGET_SYSTEM),
                                    state.vehicle_compid.unwrap_or(TARGET_COMPONENT),
                                );
                                if let Ok(mut c) = conn.lock() {
                                    let r1 = set_mode_guided(&mut *c, ids);
                                    let r2 = c.send_default(&goto_global_command_int(ids, lat, lon, alt));
                                    log_outgoing_two(
                                        &mut state,
                                        "GUIDED (SET_MODE)",
                                        r1,
                                        PendingFeedback::new(
                                            "Override waypoint (DO_REPOSITION)",
                                            Some(MavCmd::MAV_CMD_DO_REPOSITION),
                                            None,
                                        ),
                                        r2,
                                    );
                                    if resume_after {
                                        state.push_recent(format!(
                                            "Override: go to {:.5} {:.5} {:.0}m, then resume mission.",
                                            lat, lon, alt
                                        ));
                                    } else {
                                        state.push_recent(format!(
                                            "Override: go to {:.5} {:.5} {:.0}m, then hover (c=resume).",
                                            lat, lon, alt
                                        ));
                                    }
                                }
                            }
                        }
                        KeyCode::Esc => {
                            waypoint_input = None;
                            state.push_recent("Waypoint input cancelled.".to_string());
                        }
                        KeyCode::Backspace => {
                            if let Some(ref mut buf) = waypoint_input {
                                buf.pop();
                            }
                        }
                        KeyCode::Char(c) if !c.is_control() => {
                            if let Some(ref mut buf) = waypoint_input {
                                buf.push(c);
                            }
                        }
                        _ => {}
                    }
                    continue 'ui;
                }
                if state.show_help_popup {
                    if matches!(key.code, KeyCode::Char('h') | KeyCode::Char('q') | KeyCode::Esc) {
                        state.show_help_popup = false;
                    }
                } else {
                match key.code {
                    KeyCode::Char('q') => break 'ui,
                    KeyCode::Char('h') => state.show_help_popup = true,
                    KeyCode::Char('a') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(c) = conn.lock() {
                            let msg = with_vehicle(cmd_arm(), ids);
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "ARM",
                                    Some(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM),
                                    None,
                                ),
                                c.send_default(&msg),
                            );
                        }
                    }
                    KeyCode::Char('d') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(c) = conn.lock() {
                            let msg = with_vehicle(cmd_disarm(), ids);
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "DISARM",
                                    Some(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM),
                                    None,
                                ),
                                c.send_default(&msg),
                            );
                        }
                    }
                    KeyCode::Char('g') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(mut c) = conn.lock() {
                            let r = cmd_set_mode_guided_long(&mut *c, ids);
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "GUIDED (DO_SET_MODE)",
                                    Some(MavCmd::MAV_CMD_DO_SET_MODE),
                                    Some(4),
                                ),
                                r,
                            );
                        }
                    }
                    KeyCode::Char('u') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(mut c) = conn.lock() {
                            let r = cmd_set_mode_auto_long(&mut *c, ids);
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "AUTO (DO_SET_MODE)",
                                    Some(MavCmd::MAV_CMD_DO_SET_MODE),
                                    Some(3),
                                ),
                                r,
                            );
                        }
                    }
                    KeyCode::Char('m') => {
                        let ids = vehicle_ids_from_state(&state);
                        {
                            let store = match mission_store.lock() {
                                Ok(g) => g,
                                Err(_) => continue 'ui,
                            };
                            if store.items.is_empty() {
                                state.push_recent(
                                    "Mission start blocked: no mission downloaded yet (wait for MISSION_ITEM_INT)."
                                        .to_string(),
                                );
                                continue 'ui;
                            }
                        }
                        if let Ok(c) = conn.lock() {
                            match drone_server::mission_upload::ensure_nav_takeoff_on_fc(
                                &*c,
                                ids,
                                &mission_store,
                                None,
                            ) {
                                Ok(true) => state.push_recent(
                                    "Inserted NAV_TAKEOFF at mission start and re-uploaded to FC."
                                        .to_string(),
                                ),
                                Ok(false) => {}
                                Err(e) => {
                                    state.push_recent(format!("Mission fixup failed: {e}"));
                                    continue 'ui;
                                }
                            }
                        }
                        if let Ok(store) = mission_store.lock() {
                            if let Err(e) = store.validate_ready_for_start_mission() {
                                state.push_recent(e);
                                continue 'ui;
                            }
                        }
                        if let Ok(mut c) = conn.lock() {
                            let r1 = cmd_set_mode_auto_long(&mut *c, ids);
                            let msg = cmd_mission_start(ids);
                            let r2 = c.send_default(&msg);
                            log_outgoing_two(
                                &mut state,
                                "AUTO (DO_SET_MODE)",
                                r1,
                                PendingFeedback::new(
                                    "MISSION_START",
                                    Some(MavCmd::MAV_CMD_MISSION_START),
                                    None,
                                ),
                                r2,
                            );
                        }
                    }
                    KeyCode::Char('t') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(c) = conn.lock() {
                            let msg = with_vehicle(cmd_takeoff_alt(10.0), ids);
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "TAKEOFF 10m",
                                    Some(MavCmd::MAV_CMD_NAV_TAKEOFF),
                                    None,
                                ),
                                c.send_default(&msg),
                            );
                        }
                    }
                    KeyCode::Char('s') => {
                        let _ = stream_retry_tx.send(());
                    }
                    KeyCode::Char('k') => {
                        state.recent_messages.clear();
                    }
                    KeyCode::Char('f') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(mut c) = conn.lock() {
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "FORCE_ARM",
                                    Some(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM),
                                    None,
                                ),
                                force_arm(&mut *c, ids),
                            );
                        }
                    }
                    KeyCode::Char('r') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(mut c) = conn.lock() {
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "RTL",
                                    Some(MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH),
                                    None,
                                ),
                                rtl(&mut *c, ids),
                            );
                        }
                    }
                    KeyCode::Char('l') => {
                        let ids = vehicle_ids_from_state(&state);
                        if let Ok(mut c) = conn.lock() {
                            log_outgoing(
                                &mut state,
                                PendingFeedback::new(
                                    "LAND",
                                    Some(MavCmd::MAV_CMD_NAV_LAND),
                                    None,
                                ),
                                land(&mut *c, ids),
                            );
                        }
                    }
                    KeyCode::Char('w') => {
                        if waypoint_input.is_none() {
                            // Allow when in AUTO (during mission) or Paused (interrupt), and not already running override waypoints
                            let in_override = override_state.lock().map(|g| matches!(&*g, OverrideState::OverrideActive { .. })).unwrap_or(false);
                            let can_waypoint = override_state.lock().map(|g| matches!(&*g, OverrideState::MissionRunning | OverrideState::Paused)).unwrap_or(false);
                            if !in_override && can_waypoint {
                                waypoint_input = Some(String::new());
                                state.push_recent("Enter waypoint: lat lon alt (space-sep), or just alt (m). Enter=go Esc=cancel".to_string());
                            } else if in_override {
                                state.push_recent("Waypoint: finish current override first.".to_string());
                            } else {
                                state.push_recent("Waypoint: start mission (u then m) or interrupt (i) first.".to_string());
                            }
                        }
                    }
                    KeyCode::Char('i') => {
                        // Interrupt: pause mission, hover here. Press 'c' to resume. Can press 'w' to inject a waypoint while paused.
                        // DO_REPOSITION uses MAV_FRAME_GLOBAL_RELATIVE_ALT so altitude must be relative to home, not AMSL.
                        if state.heartbeat_custom != Some(3) {
                            state.push_recent("Interrupt (i): switch to AUTO and start mission first.".to_string());
                            continue 'ui;
                        }
                        let (lat, lon, alt_rel) = match (state.lat, state.lon, state.alt, state.home_alt) {
                            (Some(la), Some(lo), Some(al), Some(home_al)) => (la, lo, al - home_al),
                            (Some(_), Some(_), None, _) | (None, _, _, _) | (_, None, _, _) => {
                                state.push_recent("Interrupt: no position (need GPS).".to_string());
                                continue 'ui;
                            }
                            (_, _, Some(_), None) => {
                                state.push_recent("Interrupt: need home position (wait for HOME_POSITION).".to_string());
                                continue 'ui;
                            }
                        };
                        let ok = {
                            let mut os = match override_state.lock() {
                                Ok(g) => g,
                                Err(_) => continue 'ui,
                            };
                            if matches!(&*os, OverrideState::OverrideActive { .. }) {
                                state.push_recent("Interrupt: finish current override first.".to_string());
                                continue 'ui;
                            }
                            let mut store = match mission_store.lock() {
                                Ok(g) => g,
                                Err(_) => continue 'ui,
                            };
                            if !store.ensure_snapshot_for_pause() {
                                state.push_recent("Interrupt: no mission or current WP (wait for mission download).".to_string());
                                continue 'ui;
                            }
                            *os = OverrideState::Paused;
                            true
                        };
                        if ok {
                            let ids = VehicleIds::new(
                                state.vehicle_sysid.unwrap_or(TARGET_SYSTEM),
                                state.vehicle_compid.unwrap_or(TARGET_COMPONENT),
                            );
                            if let Ok(mut c) = conn.lock() {
                                let r1 = set_mode_guided(&mut *c, ids);
                                let r2 = c.send_default(&goto_global_command_int(ids, lat, lon, alt_rel));
                                log_outgoing_two(
                                    &mut state,
                                    "GUIDED (SET_MODE)",
                                    r1,
                                    PendingFeedback::new(
                                        "Interrupt hover (DO_REPOSITION)",
                                        Some(MavCmd::MAV_CMD_DO_REPOSITION),
                                        None,
                                    ),
                                    r2,
                                );
                                state.push_recent(
                                    "Interrupt: hovering. Press c to resume mission, or w to inject waypoint."
                                        .to_string(),
                                );
                            }
                        }
                    }
                    KeyCode::Char('c') => {
                        // Cancel override: force resume mission now (get unstuck if stuck in OverrideActive/Resuming)
                        let (snapshot_items, resume_seq) = {
                            let store = match mission_store.lock() {
                                Ok(g) => g,
                                Err(_) => continue 'ui,
                            };
                            match store.get_snapshot() {
                                Some((items, seq)) => (items.to_vec(), seq),
                                None => {
                                    if override_state.lock().map(|g| !matches!(&*g, OverrideState::MissionRunning)).unwrap_or(false) {
                                        override_state.lock().ok().map(|mut g| *g = OverrideState::MissionRunning);
                                        state.push_recent("Override cancelled (no snapshot). State reset.".to_string());
                                    }
                                    continue 'ui;
                                }
                            }
                        };
                        let ids = VehicleIds::new(
                            state.vehicle_sysid.unwrap_or(TARGET_SYSTEM),
                            state.vehicle_compid.unwrap_or(TARGET_COMPONENT),
                        );
                        {
                            let mut store = mission_store.lock().unwrap();
                            store.set_upload_pending(snapshot_items.clone());
                        }
                        let count = snapshot_items.len() as u16;
                        if let Ok(c) = conn.lock() {
                            let _ = c.send_default(&MavMessage::MISSION_COUNT(
                                mavlink::ardupilotmega::MISSION_COUNT_DATA {
                                    count,
                                    target_system: ids.system_id,
                                    target_component: ids.component_id,
                                },
                            ));
                        }
                        if let Ok(mut st) = override_state.lock() {
                            *st = OverrideState::Resuming { resume_seq };
                        }
                        state.push_recent("Cancel override: resuming mission (upload + set current + AUTO).".to_string());
                    }
                    _ => {}
                }
                }
                }
                Event::Resize(_, _) => {}
                _ => {}
            }
        }

        while let Ok(frame) = rx.try_recv() {
            apply_message(&mut state, &frame);
        }
        while let Ok(line) = log_rx.try_recv() {
            state.push_recent(line);
        }
        check_pending_feedback_timeout(&mut state);
        if let Ok(ns) = net_watchdog_status.lock() {
            let now = Instant::now();
            state.net_online = ns.online;
            state.net_secs_since_last_check =
                ns.last_check.map(|t| now.duration_since(t).as_secs());
            state.net_secs_since_last_ok = ns.last_ok.map(|t| now.duration_since(t).as_secs());
            state.net_offline_secs = ns.offline_since.map(|t| now.duration_since(t).as_secs());
            state.net_rtl_sent_for_current_outage = ns.rtl_sent_for_current_outage;
        }
        terminal.draw(|f| draw_ui(f, &state, &override_state, waypoint_input.as_deref()))?;
        let _ = event::poll(Duration::from_millis(50))?;
    }

    crossterm::execute!(
        io::stdout(),
        crossterm::terminal::LeaveAlternateScreen
    )?;
    crossterm::terminal::disable_raw_mode()?;
    Ok(())
}
