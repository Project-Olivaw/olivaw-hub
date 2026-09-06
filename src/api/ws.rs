//! `/ws`: a `hello` snapshot, then every live event as JSON.

use std::sync::Arc;

use axum::extract::State;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::response::Response;
use tokio::sync::broadcast::error::RecvError;

use crate::state::{Hub, WsEvent};

/// Upgrade handler.
pub async fn upgrade(ws: WebSocketUpgrade, State(hub): State<Arc<Hub>>) -> Response {
    ws.on_upgrade(move |socket| session(socket, hub))
}

async fn session(mut socket: WebSocket, hub: Arc<Hub>) {
    let mut rx = hub.events.subscribe();
    let hello = WsEvent::Hello {
        cars: hub.snapshots(),
    };
    if send(&mut socket, &hello).await.is_err() {
        return;
    }
    loop {
        tokio::select! {
            event = rx.recv() => match event {
                Ok(event) => {
                    if send(&mut socket, &event).await.is_err() {
                        return;
                    }
                }
                Err(RecvError::Lagged(n)) => tracing::debug!("ws client lagged {n} events"),
                Err(RecvError::Closed) => return,
            },
            incoming = socket.recv() => match incoming {
                Some(Ok(Message::Close(_)) | Err(_)) | None => return,
                Some(Ok(_)) => {}
            }
        }
    }
}

async fn send(socket: &mut WebSocket, event: &WsEvent) -> Result<(), ()> {
    let json = serde_json::to_string(event).map_err(|_| ())?;
    socket
        .send(Message::Text(json.into()))
        .await
        .map_err(|_| ())
}
