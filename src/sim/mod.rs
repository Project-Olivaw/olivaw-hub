//! `--simulate`: a car drives around a room and emits exactly what the real
//! firmware would (status, telemetry JSON, scan frames) into the pipeline.

pub mod room;

use tokio::sync::mpsc;

use crate::config::SimConfig;
use crate::ingest::CarEvent;

/// Spawn the simulator task.
pub fn spawn(cfg: SimConfig, tx: mpsc::Sender<CarEvent>) {
    tokio::spawn(room::run(cfg, tx));
}
