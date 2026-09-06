//! Feed a recording into the pipeline at real time (or faster).

use std::path::Path;
use std::time::Duration;

use tokio::sync::mpsc;

use crate::ingest::CarEvent;
use crate::session::recorder::read_all;

/// Spawn the replay task. `speed` 0 = no pacing.
pub fn spawn(path: &Path, speed: f64, tx: mpsc::Sender<CarEvent>) -> anyhow::Result<()> {
    let events = read_all(&std::fs::read(path)?)?;
    tracing::info!("replaying {} events from {}", events.len(), path.display());
    tokio::spawn(async move {
        let mut last_ms: Option<u64> = None;
        for event in events {
            if speed > 0.0
                && let Some(prev) = last_ms
                && event.at_ms > prev
            {
                let gap = Duration::from_millis(event.at_ms - prev).div_f64(speed);
                tokio::time::sleep(gap.min(Duration::from_secs(5))).await;
            }
            last_ms = Some(event.at_ms);
            if tx.send(event).await.is_err() {
                return;
            }
        }
        tracing::info!("replay finished");
    });
    Ok(())
}
