//! HTTP + WebSocket API (axum). Serves the built dashboard at `/` when present.

pub mod http;
pub mod ws;

use std::sync::Arc;

use axum::Router;
use axum::routing::{get, post};
use tower_http::cors::CorsLayer;
use tower_http::services::{ServeDir, ServeFile};

use crate::state::Hub;

/// Build the router.
pub fn router(hub: Arc<Hub>) -> Router {
    let mut app = Router::new()
        .route("/api/health", get(http::health))
        .route("/api/cars", get(http::cars))
        .route("/api/cars/{id}", get(http::car))
        .route("/api/cars/{id}/map.png", get(http::map_png))
        .route("/api/cars/{id}/map.pgm", get(http::map_pgm))
        .route("/api/cars/{id}/cmd", post(http::cmd))
        .route("/ws", get(ws::upgrade))
        .layer(CorsLayer::permissive());

    if let Some(dir) = &hub.cfg.http.dashboard_dir
        && dir.join("index.html").is_file()
    {
        tracing::info!("serving dashboard from {}", dir.display());
        app = app
            .fallback_service(ServeDir::new(dir).fallback(ServeFile::new(dir.join("index.html"))));
    } else {
        app = app.fallback(http::no_dashboard);
    }
    app.with_state(hub)
}

/// Bind and serve until Ctrl-C.
pub async fn serve(hub: Arc<Hub>) -> anyhow::Result<()> {
    let bind = hub.cfg.http.bind;
    let listener = tokio::net::TcpListener::bind(bind).await?;
    tracing::info!("http: listening on http://{bind}  (ws://{bind}/ws)");
    axum::serve(listener, router(hub))
        .with_graceful_shutdown(async {
            let _ = tokio::signal::ctrl_c().await;
            tracing::info!("shutting down");
        })
        .await?;
    Ok(())
}
