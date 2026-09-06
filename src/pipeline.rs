//! The fan-in: every `CarEvent` from any source updates the state, feeds the
//! car's SLAM worker, is recorded, and is broadcast to the dashboard.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::sync::mpsc;

use crate::ingest::{CarEvent, EventKind};
use crate::session::recorder::Recorder;
use crate::slam::worker::{SlamOutput, SlamWorker, WorkerTuning};
use crate::state::{Hub, MapInfo, WsEvent};
use crate::viz::rerun::Sink;

/// `--log-only`: print decoded events and nothing else.
pub async fn log_only(mut rx: mpsc::Receiver<CarEvent>) -> anyhow::Result<()> {
    while let Some(e) = rx.recv().await {
        match &e.kind {
            EventKind::Status(s) => tracing::info!("{}: status {s}", e.car),
            EventKind::Telemetry(t) => tracing::info!("{}: {t:?}", e.car),
            EventKind::Scan(s) => tracing::info!(
                "{}: scan #{} {} points ({} valid, {} dropped)",
                e.car,
                s.seq,
                s.points.len(),
                s.valid_points(),
                s.dropped
            ),
        }
    }
    Ok(())
}

struct CarRate {
    window_start: Instant,
    scans_in_window: u32,
}

/// Run until every event source is gone.
pub async fn run(
    hub: Arc<Hub>,
    mut rx: mpsc::Receiver<CarEvent>,
    mut recorder: Option<Recorder>,
    rerun: Option<Sink>,
) {
    let (slam_tx, mut slam_rx) = mpsc::unbounded_channel::<(String, SlamOutput)>();
    let mut workers: HashMap<String, SlamWorker> = HashMap::new();
    let mut rates: HashMap<String, CarRate> = HashMap::new();
    let tuning = WorkerTuning {
        map_every_scans: hub.cfg.slam.map_every_scans,
        ws_scan_points: hub.cfg.slam.ws_scan_points,
    };
    let trajectory_len = hub.cfg.slam.trajectory_len;
    let mut stats_tick = tokio::time::interval(Duration::from_secs(1));

    loop {
        tokio::select! {
            event = rx.recv() => {
                let Some(event) = event else { break };
                if let Some(rec) = &mut recorder && let Err(e) = rec.write(&event) {
                    tracing::warn!("recorder: {e}");
                }
                on_event(&hub, &event, &mut workers, &mut rates, tuning, &slam_tx, rerun.as_ref());
            }
            Some((car, output)) = slam_rx.recv() => on_slam(&hub, &car, output, trajectory_len),
            _ = stats_tick.tick() => {
                for (car, rate) in &mut rates {
                    let secs = rate.window_start.elapsed().as_secs_f32().max(1e-3);
                    let hz = rate.scans_in_window as f32 / secs;
                    rate.window_start = Instant::now();
                    rate.scans_in_window = 0;
                    let total = hub.update_car(car, |c| { c.scan_hz = hz; c.scans_total });
                    hub.emit(WsEvent::Stats { car: car.clone(), scan_hz: hz, scans_total: total });
                }
            }
        }
    }
    tracing::info!("pipeline: all sources closed");
}

fn on_event(
    hub: &Hub,
    event: &CarEvent,
    workers: &mut HashMap<String, SlamWorker>,
    rates: &mut HashMap<String, CarRate>,
    tuning: WorkerTuning,
    slam_tx: &mpsc::UnboundedSender<(String, SlamOutput)>,
    rerun: Option<&Sink>,
) {
    let car = event.car.as_str();
    hub.update_car(car, |c| c.last_seen_ms = event.at_ms);
    match &event.kind {
        EventKind::Status(status) => {
            let changed = hub.update_car(car, |c| {
                let changed = c.status != *status;
                c.status.clone_from(status);
                changed
            });
            if changed {
                tracing::info!("{car}: {status}");
                hub.emit(WsEvent::Status {
                    car: car.to_owned(),
                    status: status.clone(),
                });
            }
        }
        EventKind::Telemetry(t) => {
            hub.update_car(car, |c| c.telemetry = Some(*t));
            hub.emit(WsEvent::Telemetry {
                car: car.to_owned(),
                telemetry: *t,
                at_ms: event.at_ms,
            });
        }
        EventKind::Scan(frame) => {
            hub.update_car(car, |c| c.scans_total += 1);
            rates
                .entry(car.to_owned())
                .or_insert_with(|| CarRate {
                    window_start: Instant::now(),
                    scans_in_window: 0,
                })
                .scans_in_window += 1;
            if !workers.contains_key(car) {
                match SlamWorker::spawn(car.to_owned(), tuning, slam_tx.clone(), rerun.cloned()) {
                    Ok(w) => {
                        workers.insert(car.to_owned(), w);
                    }
                    Err(e) => {
                        tracing::error!("cannot start SLAM for {car}: {e}");
                        return;
                    }
                }
            }
            if let Some(w) = workers.get(car) {
                w.submit((**frame).clone());
            }
        }
    }
}

fn on_slam(hub: &Hub, car: &str, output: SlamOutput, trajectory_len: usize) {
    match output {
        SlamOutput::Tracked { pose, scan_world } => {
            hub.update_car(car, |c| {
                c.pose = Some(pose);
                c.trajectory.push([pose.x as f32, pose.y as f32]);
                if c.trajectory.len() > trajectory_len {
                    let excess = c.trajectory.len() - trajectory_len;
                    c.trajectory.drain(..excess);
                }
                c.scan.clone_from(&scan_world);
            });
            hub.emit(WsEvent::Pose {
                car: car.to_owned(),
                pose,
            });
            hub.emit(WsEvent::Scan {
                car: car.to_owned(),
                seq: pose.seq,
                points: scan_world,
            });
        }
        SlamOutput::Map(snapshot) => {
            let info = MapInfo {
                version: snapshot.version,
                url: format!("/api/cars/{car}/map.png?v={}", snapshot.version),
                origin: snapshot.origin,
                resolution: snapshot.resolution,
                width: snapshot.width,
                height: snapshot.height,
            };
            hub.maps
                .write()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .insert(car.to_owned(), snapshot);
            hub.update_car(car, |c| c.map = Some(info.clone()));
            hub.emit(WsEvent::Map {
                car: car.to_owned(),
                map: info,
            });
        }
    }
}
