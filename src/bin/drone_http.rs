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
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::State;
use axum::http::{HeaderMap, StatusCode};
use axum::routing::{get, post};
use axum::{Json, Router};
use clap::Parser;
use drone_server::flight_log::FlightLog;
use drone_server::http_mission_tools;
use drone_server::mission_upload::{self, MissionUploadRequest};
use drone_server::mavlink_connect::{self, LinkInfo};
use drone_server::mavlink_http_runtime::{
    arducopter_mode_name, spawn_http_mavlink_recv_thread, HttpOverrideState, TelemetryCache,
};
use drone_server::telemetry_hub::TelemetryHub;
use drone_server::tool_dispatch::{apply_llm_drone_tool, LLM_DRONE_TOOL_NAMES};
use drone_server::{MissionStore, VehicleIds};
use mavlink::ardupilotmega::MavMessage;
use mavlink::Connection;
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
    /// Shared MAVLink link; serial uses separate read/write locks inside the driver (no outer Mutex).
    conn: Arc<Connection<MavMessage>>,
    link: LinkInfo,
    vehicle_ids: Arc<Mutex<VehicleIds>>,
    mission: Arc<Mutex<MissionStore>>,
    override_state: Arc<Mutex<HttpOverrideState>>,
    telem: Arc<Mutex<TelemetryCache>>,
    flight_log: FlightLog,
    telemetry_hub: TelemetryHub,
}

fn default_empty_object() -> serde_json::Value {
    serde_json::json!({})
}

#[derive(Deserialize)]
struct ApplyToolBody {
    tool: String,
    #[serde(default = "default_empty_object")]
    params: serde_json::Value,
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
    link: LinkInfo,
}

#[derive(Serialize)]
struct TelemetryResponse {
    ok: bool,
    link: LinkInfo,
    #[serde(skip_serializing_if = "Option::is_none")]
    lat_deg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    lon_deg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    alt_amsl_m: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    alt_rel_m: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    groundspeed_m_s: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    airspeed_m_s: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    climb_m_s: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    heading_deg: Option<i16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    roll_deg: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pitch_deg: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    yaw_deg: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    armed: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    mode: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    home_lat_deg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    home_lon_deg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    home_alt_m: Option<f64>,
}

#[derive(Serialize)]
struct MissionWaypointJson {
    seq: u16,
    lat_deg: f64,
    lon_deg: f64,
    alt_m: f32,
    command: u16,
}

#[derive(Serialize)]
struct MissionResponse {
    ok: bool,
    current_seq: Option<u16>,
    waypoints: Vec<MissionWaypointJson>,
}

#[derive(Serialize)]
struct LogsResponse {
    entries: Vec<drone_server::flight_log::FlightLogEntry>,
}

fn request_id_from_headers(h: &HeaderMap) -> String {
    h.get("x-request-id")
        .and_then(|v| v.to_str().ok())
        .map(|s| s.to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| format!("drone-http-{}", std::process::id()))
}

#[derive(Serialize)]
struct PositionResponse {
    ok: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    lat_deg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    lon_deg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    alt_amsl_m: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
}

async fn get_position(State(state): State<Arc<AppState>>) -> Json<PositionResponse> {
    let t = match state.telem.lock() {
        Ok(g) => g,
        Err(_) => {
            return Json(PositionResponse {
                ok: false,
                lat_deg: None,
                lon_deg: None,
                alt_amsl_m: None,
                error: Some("telem_lock_poisoned".into()),
            });
        }
    };
    if let (Some(lat), Some(lon)) = (t.lat, t.lon) {
        Json(PositionResponse {
            ok: true,
            lat_deg: Some(lat),
            lon_deg: Some(lon),
            alt_amsl_m: t.alt_amsl_m,
            error: None,
        })
    } else {
        Json(PositionResponse {
            ok: false,
            lat_deg: None,
            lon_deg: None,
            alt_amsl_m: None,
            error: Some("no_global_position_yet".into()),
        })
    }
}

async fn get_health(State(state): State<Arc<AppState>>) -> Json<HealthResponse> {
    let ids = state
        .vehicle_ids
        .lock()
        .map(|g| *g)
        .unwrap_or_default();
    Json(HealthResponse {
        status: "ok",
        mavlink_target_system: ids.system_id,
        mavlink_target_component: ids.component_id,
        known_tools: LLM_DRONE_TOOL_NAMES,
        link: state.link.clone(),
    })
}

async fn get_telemetry(State(state): State<Arc<AppState>>) -> Json<TelemetryResponse> {
    let t = match state.telem.lock() {
        Ok(g) => g,
        Err(_) => {
            return Json(TelemetryResponse {
                ok: false,
                link: state.link.clone(),
                lat_deg: None,
                lon_deg: None,
                alt_amsl_m: None,
                alt_rel_m: None,
                groundspeed_m_s: None,
                airspeed_m_s: None,
                climb_m_s: None,
                heading_deg: None,
                roll_deg: None,
                pitch_deg: None,
                yaw_deg: None,
                armed: None,
                mode: None,
                home_lat_deg: None,
                home_lon_deg: None,
                home_alt_m: None,
            });
        }
    };
    Json(TelemetryResponse {
        ok: t.lat.is_some() && t.lon.is_some(),
        link: state.link.clone(),
        lat_deg: t.lat,
        lon_deg: t.lon,
        alt_amsl_m: t.alt_amsl_m,
        alt_rel_m: t.relative_alt_m,
        groundspeed_m_s: t.groundspeed_m_s,
        airspeed_m_s: t.airspeed_m_s,
        climb_m_s: t.climb_m_s,
        heading_deg: t.heading_deg,
        roll_deg: t.roll_deg,
        pitch_deg: t.pitch_deg,
        yaw_deg: t.yaw_deg,
        armed: t.armed,
        mode: t
            .mode_name
            .clone()
            .or_else(|| t.heartbeat_custom_mode.map(arducopter_mode_name).map(str::to_string)),
        home_lat_deg: t.home_lat_deg,
        home_lon_deg: t.home_lon_deg,
        home_alt_m: t.home_alt_m,
    })
}

async fn get_mission(State(state): State<Arc<AppState>>) -> Json<MissionResponse> {
    let store = match state.mission.lock() {
        Ok(g) => g,
        Err(_) => {
            return Json(MissionResponse {
                ok: false,
                current_seq: None,
                waypoints: vec![],
            });
        }
    };
    let waypoints: Vec<MissionWaypointJson> = store
        .items
        .iter()
        .map(|w| MissionWaypointJson {
            seq: w.seq,
            lat_deg: w.x as f64 / 1e7,
            lon_deg: w.y as f64 / 1e7,
            alt_m: w.z as f32,
            command: w.command as u16,
        })
        .collect();
    Json(MissionResponse {
        ok: true,
        current_seq: store.current_seq,
        waypoints,
    })
}

#[derive(Serialize)]
struct MissionUploadResponse {
    ok: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    item_count: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
}

async fn post_mission_upload(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
    Json(body): Json<MissionUploadRequest>,
) -> (StatusCode, Json<MissionUploadResponse>) {
    let rid = request_id_from_headers(&headers);
    let span = info_span!("drone_http_mission_upload", request_id = %rid);
    let _enter = span.enter();

    state.flight_log.push("info", "mission_upload: received planner mission");

    let st = Arc::clone(&state);
    let result = tokio::task::spawn_blocking(move || {
        let ids = st
            .vehicle_ids
            .lock()
            .map(|g| *g)
            .map_err(|e| format!("vehicle_ids_lock:{e}"))?;
        mission_upload::mission_upload(
            st.conn.as_ref(),
            ids,
            &st.mission,
            &st.override_state,
            &body,
        )
    })
    .await
    .unwrap_or_else(|e| Err(format!("spawn_blocking:{e}")));

    match result {
        Ok(count) => {
            state.flight_log.push(
                "info",
                format!("mission_upload: ok ({count} items on FC)"),
            );
            (
                StatusCode::OK,
                Json(MissionUploadResponse {
                    ok: true,
                    item_count: Some(count),
                    error: None,
                }),
            )
        }
        Err(e) => {
            warn!(request_id = %rid, error = %e, "mission_upload failed");
            state.flight_log.push("error", format!("mission_upload: {e}"));
            (
                StatusCode::BAD_REQUEST,
                Json(MissionUploadResponse {
                    ok: false,
                    item_count: None,
                    error: Some(e),
                }),
            )
        }
    }
}

#[derive(Serialize)]
struct MissionClearResponse {
    ok: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
}

async fn post_mission_clear(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
) -> (StatusCode, Json<MissionClearResponse>) {
    let rid = request_id_from_headers(&headers);
    let span = info_span!("drone_http_mission_clear", request_id = %rid);
    let _enter = span.enter();

    state.flight_log.push("info", "mission_clear: clearing FC mission");

    let st = Arc::clone(&state);
    let result = tokio::task::spawn_blocking(move || {
        let ids = st
            .vehicle_ids
            .lock()
            .map(|g| *g)
            .map_err(|e| format!("vehicle_ids_lock:{e}"))?;
        mission_upload::mission_clear(
            st.conn.as_ref(),
            ids,
            &st.mission,
            Some(&st.override_state),
        )
    })
    .await
    .unwrap_or_else(|e| Err(format!("spawn_blocking:{e}")));

    match result {
        Ok(()) => {
            state.flight_log.push("info", "mission_clear: ok");
            (
                StatusCode::OK,
                Json(MissionClearResponse {
                    ok: true,
                    error: None,
                }),
            )
        }
        Err(e) => {
            warn!(request_id = %rid, error = %e, "mission_clear failed");
            state.flight_log.push("error", format!("mission_clear: {e}"));
            (
                StatusCode::BAD_REQUEST,
                Json(MissionClearResponse {
                    ok: false,
                    error: Some(e),
                }),
            )
        }
    }
}

async fn get_logs(State(state): State<Arc<AppState>>) -> Json<LogsResponse> {
    Json(LogsResponse {
        entries: state.flight_log.snapshot(),
    })
}

async fn ws_telemetry(
    ws: WebSocketUpgrade,
    State(state): State<Arc<AppState>>,
) -> impl axum::response::IntoResponse {
    ws.on_upgrade(move |socket| handle_telemetry_ws(socket, state))
}

async fn handle_telemetry_ws(mut socket: WebSocket, state: Arc<AppState>) {
    let mut rx = state.telemetry_hub.subscribe();

    // Immediate snapshot so clients do not wait for next MAVLink frame.
    let initial = {
        let t = state.telem.lock().ok();
        t.map(|g| state.telemetry_hub.snapshot_now(&state.link, &g))
    };
    if let Some(snap) = initial {
        if let Ok(text) = serde_json::to_string(&snap) {
            if socket.send(Message::Text(text.into())).await.is_err() {
                return;
            }
        }
    }

    loop {
        tokio::select! {
            incoming = socket.recv() => {
                match incoming {
                    Some(Ok(Message::Close(_))) | None => break,
                    Some(Ok(Message::Ping(p))) => {
                        if socket.send(Message::Pong(p)).await.is_err() {
                            break;
                        }
                    }
                    Some(Err(_)) => break,
                    _ => {}
                }
            }
            msg = rx.recv() => {
                match msg {
                    Ok(snap) => {
                        let Ok(text) = serde_json::to_string(&snap) else { continue };
                        if socket.send(Message::Text(text.into())).await.is_err() {
                            break;
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(_) => break,
                }
            }
        }
    }
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
    state.flight_log.push(
        "info",
        format!("apply_tool: {} params={}", body.tool, body.params),
    );

    let tool_for_blocking = body.tool.clone();
    let params_for_blocking = body.params.clone();
    let st = Arc::clone(&state);
    let tool_name_for_log = body.tool.clone();

    let result = tokio::task::spawn_blocking(move || {
        let ids = st
            .vehicle_ids
            .lock()
            .map(|g| *g)
            .map_err(|e| format!("vehicle_ids_lock:{e}"))?;
        let conn = Arc::clone(&st.conn);
        match tool_for_blocking.as_str() {
            "mission_interrupt" => http_mission_tools::mission_interrupt(
                conn.as_ref(),
                ids,
                &st.mission,
                &st.override_state,
                &st.telem,
            ),
            "mission_resume" => http_mission_tools::mission_resume(
                conn.as_ref(),
                ids,
                &st.mission,
                &st.override_state,
            ),
            "waypoint_inject" => http_mission_tools::waypoint_inject(
                conn.as_ref(),
                ids,
                &st.mission,
                &st.override_state,
                &st.telem,
                &params_for_blocking,
            ),
            "goto_location" => apply_llm_drone_tool(
                conn.as_ref(),
                ids,
                "goto_location",
                &params_for_blocking,
                None,
            ),
            "start_mission" => {
                mission_upload::ensure_nav_takeoff_on_fc(
                    conn.as_ref(),
                    ids,
                    &st.mission,
                    Some(&st.override_state),
                )?;
                let telem_guard = st.telem.lock().map_err(|e| format!("telem_lock:{e}"))?;
                mission_upload::start_auto_mission(
                    conn.as_ref(),
                    ids,
                    &st.mission,
                    &telem_guard,
                )
            }
            _ => {
                let telem_guard = st.telem.lock().map_err(|e| format!("telem_lock:{e}"))?;
                apply_llm_drone_tool(
                    conn.as_ref(),
                    ids,
                    &tool_for_blocking,
                    &params_for_blocking,
                    Some(&*telem_guard),
                )
            }
        }
    })
    .await;

    let ids_resp = state
        .vehicle_ids
        .lock()
        .map(|g| *g)
        .unwrap_or_default();

    let (ok, err) = match result {
        Ok(Ok(())) => {
            info!(request_id = %rid, tool = %tool_name_for_log, "apply_tool: MAVLink send OK");
            state.flight_log.push("info", format!("OK: {tool_name_for_log}"));
            (true, None)
        }
        Ok(Err(e)) => {
            warn!(request_id = %rid, tool = %tool_name_for_log, error = %e, "apply_tool: MAVLink or tool error");
            state.flight_log.push("warn", format!("FAIL {tool_name_for_log}: {e}"));
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
            target_system: ids_resp.system_id,
            target_component: ids_resp.component_id,
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
    if args.mavlink.iter().any(|a| a == "-h" || a == "--help") {
        eprintln!("MAVLink options (same as `tui`):\n{}", mavlink_connect::usage_string());
        eprintln!("\nWith no MAVLink args, drone-http auto-detects UDP (MAVProxy) then USB serial.");
        std::process::exit(0);
    }

    let flight_log = FlightLog::new();
    let (connection, link) = match mavlink_connect::open_connection(args.mavlink.clone()) {
        Ok(v) => v,
        Err(e) => {
            error!(error = %e, "failed to open MAVLink");
            std::process::exit(1);
        }
    };
    info!(url = %link.url, kind = %link.kind, display = %link.display, "MAVLink connected");
    flight_log.push("info", format!("Link: {} ({})", link.display, link.kind));

    let conn = Arc::new(connection);
    let vehicle_ids = Arc::new(Mutex::new(VehicleIds::new(1, 1)));
    let mission = Arc::new(Mutex::new(MissionStore::default()));
    let override_state = Arc::new(Mutex::new(HttpOverrideState::default()));
    let telem = Arc::new(Mutex::new(TelemetryCache::default()));
    let telemetry_hub = TelemetryHub::new();

    let _recv_join = spawn_http_mavlink_recv_thread(
        Arc::clone(&conn),
        Arc::clone(&mission),
        Arc::clone(&override_state),
        Arc::clone(&telem),
        Arc::clone(&vehicle_ids),
        flight_log.clone(),
        telemetry_hub.clone(),
        link.clone(),
    );

    let state = Arc::new(AppState {
        conn,
        link: link.clone(),
        vehicle_ids: Arc::clone(&vehicle_ids),
        mission,
        override_state,
        telem,
        flight_log,
        telemetry_hub,
    });

    let ids_display = vehicle_ids.lock().map(|g| *g).unwrap_or_default();

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
        .route("/v1/position", get(get_position))
        .route("/v1/telemetry", get(get_telemetry))
        .route("/v1/mission", get(get_mission))
        .route("/v1/mission/upload", post(post_mission_upload))
        .route("/v1/mission/clear", post(post_mission_clear))
        .route("/v1/logs", get(get_logs))
        .route("/v1/apply-tool", post(post_apply_tool))
        .route("/v1/ws/telemetry", get(ws_telemetry))
        .layer(TraceLayer::new_for_http())
        .layer(cors)
        .with_state(state);

    info!(
        listen = %args.listen,
        "drone-http listening; GET /v1/position; POST /v1/apply-tool JSON {{\"tool\":\"return_to_home\"}}; GET /health"
    );
    eprintln!(
        "drone-http {} | MAVLink: {} | vehicle sys={} comp={}",
        args.listen,
        link.display,
        ids_display.system_id,
        ids_display.component_id
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
