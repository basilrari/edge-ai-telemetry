//! MAVLink telemetry TUI with Vehicle info and Messages log.

#![allow(deprecated)]

mod consts;
mod format;
mod mavlink_cmd;
mod recv;
mod render;
mod state;
mod telemetry;
mod ui_loop;
mod watchdog;

use std::sync::mpsc;
use std::sync::{Arc, Mutex};

use drone_server::mavlink_connect::{self, MavlinkArgsError};
use drone_server::{MissionStore, VehicleIds};
use mavlink::ardupilotmega::MavMessage;
use mavlink::connect;

use recv::spawn_recv_thread;
use state::{NetWatchdogStatus, OverrideState};
use ui_loop::run_ui;
use watchdog::spawn_net_watchdog;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (mavlink_url, link_display) = match mavlink_connect::resolve_from_args(args) {
        Ok(v) => v,
        Err(MavlinkArgsError::Help) => {
            eprintln!("Usage: tui [OPTIONS]\n\n{}", mavlink_connect::usage_string());
            return;
        }
        Err(MavlinkArgsError::Invalid(m)) => {
            eprintln!("{m}");
            std::process::exit(2);
        }
    };

    eprintln!("MAVLink: {}", link_display);
    eprintln!("Waiting for first heartbeat...");
    eprintln!("Press h for help. q=quit.");

    let mut connection = match connect::<MavMessage>(&mavlink_url) {
        Ok(conn) => conn,
        Err(e) => {
            eprintln!("{}", mavlink_connect::open_error_message(&mavlink_url, &e));
            std::process::exit(1);
        }
    };
    mavlink_connect::tune_connection(&mut connection);

    let conn = Arc::new(Mutex::new(connection));
    let (tx, rx) = mpsc::channel();
    let (log_tx, log_rx) = mpsc::channel::<String>();
    let (stream_retry_tx, stream_retry_rx) = mpsc::channel::<()>();
    let mission_store = Arc::new(Mutex::new(MissionStore::new()));
    let override_state = Arc::new(Mutex::new(OverrideState::MissionRunning));
    let watchdog_vehicle_ids = Arc::new(Mutex::new(None::<VehicleIds>));
    let net_watchdog_status = Arc::new(Mutex::new(NetWatchdogStatus::default()));

    let recv_conn = Arc::clone(&conn);
    let recv_store = Arc::clone(&mission_store);
    let recv_override = Arc::clone(&override_state);
    let recv_watchdog_vehicle_ids = Arc::clone(&watchdog_vehicle_ids);
    let _recv_handle = spawn_recv_thread(
        recv_conn,
        recv_store,
        recv_override,
        recv_watchdog_vehicle_ids,
        tx,
        log_tx,
        stream_retry_rx,
    );

    let watchdog_conn = Arc::clone(&conn);
    let watchdog_vehicle_ids_thread = Arc::clone(&watchdog_vehicle_ids);
    let net_watchdog_status_thread = Arc::clone(&net_watchdog_status);
    let _net_watchdog_handle = spawn_net_watchdog(
        watchdog_conn,
        watchdog_vehicle_ids_thread,
        net_watchdog_status_thread,
    );

    if let Err(e) = run_ui(
        rx,
        log_rx,
        stream_retry_tx,
        conn,
        mission_store,
        override_state,
        net_watchdog_status,
    ) {
        eprintln!("UI error: {}", e);
        std::process::exit(1);
    }
}
