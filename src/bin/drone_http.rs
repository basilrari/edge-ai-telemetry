//! HTTP control plane for the `drone_server` library: accepts gateway tool names and sends MAVLink.
//!
//! Run on the Jetson beside the gateway (default listen `0.0.0.0:3001`). MAVLink args match `tui` / `raw`
//! (`--serial`, `--baud`, or default UDP `udpin:0.0.0.0:14550`).
//!
//! ```text
//! RUST_LOG=info,drone_http=debug,tower_http=debug cargo run --bin drone-http -- --serial
//! ```

#![allow(deprecated)]

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use axum::extract::State;
use axum::http::{HeaderMap, StatusCode};
use axum::routing::{get, post};
use axum::{Json, Router};
use clap::Parser;
use drone_server::mavlink_connect::{self, MavlinkArgsError};
use drone_server::tool_dispatch::{apply_llm_drone_tool, wait_autopilot_heartbeat, LLM_DRONE_TOOL_NAMES};
use drone_server::VehicleIds;
use mavlink::ardupilotmega::MavMessage;
use mavlink::{connect, Connection};
use serde::{Deserialize, Serialize};
use tower_http::cors::{Any, CorsLayer};
use tower_http::trace::TraceLayer;
use tracing::{error, info, info_span, warn};

#[derive(Parser, Debug)]
#[command(name = "drone-http")]
struct Args {
    /// HTTP bind address (gateway should use `http://127.0.0.1:<port>` when co-located).
    #[arg(long, default_value = "0.0.0.0:3001")]
    listen: SocketAddr,
    /// MAVLink connection args (same as `tui`): e.g. `--serial` or `--serial /dev/ttyACM0 --baud 921600`.
    #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
    mavlink: Vec<String>,
}

struct AppState {
    conn: Mutex<Connection<MavMessage>>,
    vehicle_ids: VehicleIds,
}

#[derive(Deserialize)]
struct ApplyToolBody {
    tool: String,
}

#[derive(Serialize)]
struct ApplyToolResponse {
    ok: bool,
    tool: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
    target_system: u8,
    target_component: u8,
}

#[derive(Serialize)]
struct HealthResponse {
    status: &'static str,
    mavlink_target_system: u8,
    mavlink_target_component: u8,
    known_tools: &'static [&'static str],
}

fn request_id_from_headers(h: &HeaderMap) -> String {
    h.get("x-request-id")
        .and_then(|v| v.to_str().ok())
        .map(|s| s.to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| format!("drone-http-{}", std::process::id()))
}

async fn get_health(State(state): State<Arc<AppState>>) -> Json<HealthResponse> {
    let ids = state.vehicle_ids;
    Json(HealthResponse {
        status: "ok",
        mavlink_target_system: ids.system_id,
        mavlink_target_component: ids.component_id,
        known_tools: LLM_DRONE_TOOL_NAMES,
    })
}

async fn post_apply_tool(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
    Json(body): Json<ApplyToolBody>,
) -> (StatusCode, Json<ApplyToolResponse>) {
    let rid = request_id_from_headers(&headers);
    let span = info_span!(
        "drone_http_apply",
        request_id = %rid,
        tool = %body.tool
    );
    let _enter = span.enter();

    info!(
        request_id = %rid,
        tool = %body.tool,
        "apply_tool: received HTTP request"
    );

    let tool_for_blocking = body.tool.clone();
    let ids = state.vehicle_ids;
    let st = Arc::clone(&state);
    let tool_name_for_log = body.tool.clone();

    let result = tokio::task::spawn_blocking(move || {
        let mut conn = st
            .conn
            .lock()
            .map_err(|e| format!("internal_lock_failed:{e}"))?;
        apply_llm_drone_tool(&mut *conn, ids, &tool_for_blocking)
    })
    .await;

    let (ok, err) = match result {
        Ok(Ok(())) => {
            info!(request_id = %rid, tool = %tool_name_for_log, "apply_tool: MAVLink send OK");
            (true, None)
        }
        Ok(Err(e)) => {
            warn!(request_id = %rid, tool = %tool_name_for_log, error = %e, "apply_tool: MAVLink or tool error");
            (false, Some(e))
        }
        Err(join_err) => {
            error!(request_id = %rid, error = %join_err, "apply_tool: spawn_blocking failed");
            (false, Some(format!("task_join:{join_err}")))
        }
    };

    let status = if ok {
        StatusCode::OK
    } else {
        StatusCode::BAD_GATEWAY
    };

    (
        status,
        Json(ApplyToolResponse {
            ok,
            tool: body.tool,
            error: err,
            target_system: ids.system_id,
            target_component: ids.component_id,
        }),
    )
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_target(true)
        .init();

    let args = Args::parse();
    let (mavlink_url, link_display) = match mavlink_connect::resolve_from_args(args.mavlink.clone()) {
        Ok(v) => v,
        Err(MavlinkArgsError::Help) => {
            eprintln!("MAVLink options (same as `tui`):\n{}", mavlink_connect::usage_string());
            std::process::exit(0);
        }
        Err(MavlinkArgsError::Invalid(m)) => {
            eprintln!("{m}");
            std::process::exit(2);
        }
    };

    info!(%mavlink_url, display = %link_display, "opening MAVLink connection");
    let mut connection = match connect::<MavMessage>(&mavlink_url) {
        Ok(conn) => conn,
        Err(e) => {
            error!("{}", mavlink_connect::open_error_message(&mavlink_url, &e));
            std::process::exit(1);
        }
    };
    mavlink_connect::tune_connection(&mut connection);

    info!("waiting for autopilot HEARTBEAT (up to 60s)…");
    let vehicle_ids = match wait_autopilot_heartbeat(&mut connection, Duration::from_secs(60)) {
        Ok(ids) => {
            info!(
                system = ids.system_id,
                component = ids.component_id,
                "autopilot heartbeat OK"
            );
            ids
        }
        Err(e) => {
            error!(error = %e, "failed to acquire vehicle IDs from MAVLink");
            std::process::exit(1);
        }
    };

    let state = Arc::new(AppState {
        conn: Mutex::new(connection),
        vehicle_ids,
    });

    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_headers(Any)
        .allow_methods([
            axum::http::Method::GET,
            axum::http::Method::POST,
            axum::http::Method::OPTIONS,
        ]);

    let app = Router::new()
        .route("/health", get(get_health))
        .route("/v1/apply-tool", post(post_apply_tool))
        .layer(TraceLayer::new_for_http())
        .layer(cors)
        .with_state(state);

    info!(
        listen = %args.listen,
        "drone-http listening; POST /v1/apply-tool JSON {{\"tool\":\"return_to_home\"}}; GET /health"
    );
    eprintln!(
        "drone-http {} | MAVLink: {} | vehicle sys={} comp={}",
        args.listen,
        link_display,
        vehicle_ids.system_id,
        vehicle_ids.component_id
    );

    let listener = tokio::net::TcpListener::bind(args.listen)
        .await
        .expect("bind drone-http listen addr");
    axum::serve(listener, app)
        .with_graceful_shutdown(async {
            tokio::signal::ctrl_c()
                .await
                .expect("failed to listen for ctrl-c");
            info!("drone-http shutting down");
        })
        .await
        .expect("serve");
}
