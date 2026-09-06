//! REST handlers.

use std::sync::Arc;

use axum::Json;
use axum::body::Bytes;
use axum::extract::{Path, State};
use axum::http::{StatusCode, header};
use axum::response::{Html, IntoResponse, Response};
use olivaw_proto::Control;
use olivaw_proto::control::CONTROL_MAX_LEN;
use serde::{Deserialize, Serialize};

use crate::state::{CarSnapshot, Hub};

/// `GET /api/health`.
pub async fn health(State(hub): State<Arc<Hub>>) -> Json<serde_json::Value> {
    Json(serde_json::json!({
        "ok": true,
        "version": env!("CARGO_PKG_VERSION"),
        "cars": hub.cars.read().map_or(0, |c| c.len()),
        "ws_clients": hub.events.receiver_count(),
    }))
}

/// `GET /api/cars`.
pub async fn cars(State(hub): State<Arc<Hub>>) -> Json<Vec<CarSnapshot>> {
    Json(hub.snapshots())
}

/// `GET /api/cars/{id}`.
pub async fn car(State(hub): State<Arc<Hub>>, Path(id): Path<String>) -> Response {
    let cars = hub
        .cars
        .read()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    match cars.get(&id) {
        Some(c) => Json(c.clone()).into_response(),
        None => (StatusCode::NOT_FOUND, format!("no car {id}")).into_response(),
    }
}

/// `GET /api/cars/{id}/map.png`.
pub async fn map_png(State(hub): State<Arc<Hub>>, Path(id): Path<String>) -> Response {
    let maps = hub
        .maps
        .read()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    match maps.get(&id) {
        Some(m) => (
            [
                (header::CONTENT_TYPE, "image/png"),
                (header::CACHE_CONTROL, "no-cache"),
            ],
            m.png.clone(),
        )
            .into_response(),
        None => (StatusCode::NOT_FOUND, format!("no map yet for {id}")).into_response(),
    }
}

/// `GET /api/cars/{id}/map.pgm`.
pub async fn map_pgm(State(hub): State<Arc<Hub>>, Path(id): Path<String>) -> Response {
    let maps = hub
        .maps
        .read()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    match maps.get(&id) {
        Some(m) => (
            [(header::CONTENT_TYPE, "image/x-portable-graymap")],
            m.to_pgm(),
        )
            .into_response(),
        None => (StatusCode::NOT_FOUND, format!("no map yet for {id}")).into_response(),
    }
}

/// Body of `POST /api/cars/{id}/cmd`.
#[derive(Debug, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum Command {
    /// Brake and latch.
    EStop,
    /// Release the latch.
    Clear,
    /// Per-mille per side.
    Drive {
        /// `-1000..=1000`.
        left: i16,
        /// `-1000..=1000`.
        right: i16,
    },
    /// Lidar on/off.
    Lidar {
        /// Desired state.
        on: bool,
    },
    /// Duty cap.
    MaxDuty {
        /// `0..=1000`.
        permille: u16,
    },
}

/// Reply of the cmd endpoint.
#[derive(Debug, Serialize)]
pub struct CmdReply {
    /// Bytes published to `olivaw/<car>/cmd`.
    pub bytes: usize,
}

/// `POST /api/cars/{id}/cmd` — forward to the car over MQTT.
pub async fn cmd(
    State(hub): State<Arc<Hub>>,
    Path(id): Path<String>,
    Json(command): Json<Command>,
) -> Response {
    let payload = encode(&command);
    match hub.commander.send(&id, payload.clone()).await {
        Ok(()) => Json(CmdReply {
            bytes: payload.len(),
        })
        .into_response(),
        Err(e) => (StatusCode::BAD_GATEWAY, format!("publish failed: {e}")).into_response(),
    }
}

fn encode(command: &Command) -> Vec<u8> {
    let control = |c: Control| {
        let mut buf = [0u8; CONTROL_MAX_LEN];
        let n = c.encode(&mut buf);
        buf[..n].to_vec()
    };
    match command {
        Command::EStop => control(Control::EStop),
        Command::Clear => control(Control::Clear),
        Command::Drive { left, right } => format!("{left},{right}\n").into_bytes(),
        Command::Lidar { on: true } => control(Control::LidarOn),
        Command::Lidar { on: false } => control(Control::LidarOff),
        Command::MaxDuty { permille } => control(Control::SetMaxDuty(*permille)),
    }
}

/// Fallback when no dashboard build is available.
pub async fn no_dashboard() -> Html<&'static str> {
    Html(
        "<h1>olivaw-hub</h1><p>No dashboard build found. Run <code>pnpm build</code> in \
         <code>olivaw-dashboard</code> or pass <code>--dashboard-dir</code>. \
         API: <a href=\"/api/cars\">/api/cars</a>, WebSocket: <code>/ws</code>.</p>",
    )
}

/// Reject non-JSON bodies with a readable message (axum default is terse).
#[allow(dead_code)]
pub fn bad_body(body: &Bytes) -> Response {
    (
        StatusCode::BAD_REQUEST,
        format!("expected JSON command, got {} bytes", body.len()),
    )
        .into_response()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commands_encode_to_the_car_protocol() {
        assert_eq!(encode(&Command::EStop), vec![0x01]);
        assert_eq!(
            encode(&Command::Drive {
                left: 500,
                right: -250
            }),
            b"500,-250\n".to_vec()
        );
        assert_eq!(
            encode(&Command::MaxDuty { permille: 550 }),
            vec![0x30, 0x26, 0x02]
        );
        assert_eq!(encode(&Command::Lidar { on: true }), vec![0x10]);
    }
}
