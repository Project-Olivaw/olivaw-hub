//! olivaw-hub — the telemetry hub for Olivaw vehicles.
//!
//! ```text
//! car ──MQTT──▶ [broker] ──▶ ingest ──▶ pipeline ──▶ SLAM worker ──▶ state ──▶ HTTP + WebSocket ──▶ dashboard
//!                                          │              └──▶ rerun (--rerun)
//!                                          └──▶ recorder (--record) ; replay / simulator feed the same pipeline
//! ```

#![forbid(unsafe_code)]

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::Context as _;
use clap::Parser;
use tokio::sync::mpsc;
use tracing_subscriber::EnvFilter;

use olivaw_hub::config::HubConfig;
use olivaw_hub::ingest::CarEvent;
use olivaw_hub::state::Hub;
use olivaw_hub::{api, broker, ingest, pipeline, session, sim, viz};

/// Command line.
#[derive(Debug, Parser)]
#[allow(clippy::struct_excessive_bools)] // flags are flags
#[command(name = "olivaw-hub", version, about)]
struct Args {
    /// Path to `hub.toml` (defaults apply when absent).
    #[arg(long, env = "OLIVAW_HUB_CONFIG")]
    config: Option<PathBuf>,
    /// Do not start the embedded broker; connect to `[mqtt]` host/port instead.
    #[arg(long)]
    no_broker: bool,
    /// Feed a synthetic car driving around a room (no hardware needed).
    #[arg(long)]
    simulate: bool,
    /// Read an RPLIDAR plugged into this machine (auto-detects the port when no value is given)
    /// and publish it as car `--lidar-id`.
    #[arg(long, value_name = "PORT", num_args = 0..=1, default_missing_value = "")]
    lidar: Option<String>,
    /// Car id used for `--lidar`.
    #[arg(long, default_value = "mac-lidar")]
    lidar_id: String,
    /// Replay a `.olivawrec` session instead of listening to MQTT.
    #[arg(long, value_name = "FILE")]
    replay: Option<PathBuf>,
    /// Replay speed multiplier (0 = as fast as possible).
    #[arg(long, default_value_t = 1.0)]
    replay_speed: f64,
    /// Record every car event to this `.olivawrec` file.
    #[arg(long, value_name = "FILE")]
    record: Option<PathBuf>,
    /// Stream scans, trajectory and map to the rerun viewer (needs the `rerun` feature).
    #[arg(long)]
    rerun: bool,
    /// Only log decoded events; no SLAM, no HTTP.
    #[arg(long)]
    log_only: bool,
    /// Directory with the built dashboard to serve at `/`.
    #[arg(long, value_name = "DIR")]
    dashboard_dir: Option<PathBuf>,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| EnvFilter::new("info,rumqttd=warn")),
        )
        .init();
    let args = Args::parse();
    let mut cfg = HubConfig::load(args.config.as_deref()).context("loading config")?;
    if let Some(dir) = &args.dashboard_dir {
        cfg.http.dashboard_dir = Some(dir.clone());
    }
    tracing::info!("olivaw-hub {} — {cfg:?}", env!("CARGO_PKG_VERSION"));

    let (events_tx, events_rx) = mpsc::channel::<CarEvent>(256);

    // Event sources: exactly one of MQTT, replay, simulator (simulator may add to MQTT).
    if let Some(file) = &args.replay {
        session::replay::spawn(file, args.replay_speed, events_tx.clone())?;
    } else {
        if !args.no_broker {
            broker::start(&cfg.broker)?;
        }
        ingest::mqtt::spawn(cfg.mqtt.clone(), events_tx.clone());
    }
    if args.simulate {
        sim::spawn(cfg.sim.clone(), events_tx.clone());
    }
    if let Some(port) = &args.lidar {
        let port = (!port.is_empty()).then(|| port.clone());
        ingest::serial::spawn(port, args.lidar_id.clone(), events_tx.clone())
            .context("opening the lidar")?;
    }

    if args.log_only {
        return pipeline::log_only(events_rx).await;
    }

    let rerun = if args.rerun {
        viz::rerun::start()?
    } else {
        None
    };
    let hub = Arc::new(Hub::new(cfg.clone(), ingest::mqtt::commander(&cfg.mqtt)));
    let recorder = args
        .record
        .as_deref()
        .map(session::recorder::Recorder::create)
        .transpose()?;
    tokio::spawn(pipeline::run(hub.clone(), events_rx, recorder, rerun));

    api::serve(hub).await
}
