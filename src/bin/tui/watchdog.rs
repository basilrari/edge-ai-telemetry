//! Internet reachability probe and failsafe RTL thread.

use std::net::{SocketAddr, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use drone_server::{rtl, VehicleIds};
use mavlink::ardupilotmega::MavMessage;
use mavlink::MavConnection;

use crate::consts::{
    INTERNET_CHECK_PERIOD_SECS, INTERNET_OFFLINE_RTL_AFTER_SECS,
};

pub(crate) fn internet_is_reachable() -> bool {
    // Use raw IP endpoints so this check does not depend on DNS availability.
    const TARGETS: [&str; 3] = ["1.1.1.1:53", "8.8.8.8:53", "1.1.1.1:443"];
    let timeout = Duration::from_millis(1200);
    TARGETS.iter().any(|target| {
        target
            .parse::<SocketAddr>()
            .ok()
            .map(|addr| TcpStream::connect_timeout(&addr, timeout).is_ok())
            .unwrap_or(false)
    })
}

pub(crate) fn spawn_net_watchdog<C>(
    watchdog_conn: Arc<Mutex<C>>,
    watchdog_vehicle_ids_thread: Arc<Mutex<Option<VehicleIds>>>,
    net_watchdog_status_thread: Arc<Mutex<crate::state::NetWatchdogStatus>>,
) -> thread::JoinHandle<()>
where
    C: MavConnection<MavMessage> + Send + 'static,
{
    thread::spawn(move || {
        loop {
            thread::sleep(Duration::from_secs(INTERNET_CHECK_PERIOD_SECS));
            let now = Instant::now();
            let online = internet_is_reachable();
            if let Ok(mut s) = net_watchdog_status_thread.lock() {
                s.last_check = Some(now);
                s.online = Some(online);
                if online {
                    s.last_ok = Some(now);
                    s.offline_since = None;
                    s.rtl_sent_for_current_outage = false;
                } else if s.offline_since.is_none() {
                    s.offline_since = Some(now);
                }
            }
            if online {
                continue;
            }
            let (offline_elapsed, already_sent) = match net_watchdog_status_thread.lock() {
                Ok(s) => (
                    s.offline_since
                        .map(|t| now.duration_since(t))
                        .unwrap_or(Duration::from_secs(0)),
                    s.rtl_sent_for_current_outage,
                ),
                Err(_) => continue,
            };
            if already_sent {
                continue;
            }
            if offline_elapsed < Duration::from_secs(INTERNET_OFFLINE_RTL_AFTER_SECS) {
                continue;
            }
            let ids = match watchdog_vehicle_ids_thread.lock() {
                Ok(g) => *g,
                Err(_) => None,
            };
            if let Some(ids) = ids {
                if let Ok(mut c) = watchdog_conn.lock() {
                    let _ = rtl(&mut *c, ids);
                    if let Ok(mut s) = net_watchdog_status_thread.lock() {
                        s.rtl_sent_for_current_outage = true;
                    }
                    eprintln!("Failsafe: internet offline >=30s, sent RTL.");
                }
            }
        }
    })
}
