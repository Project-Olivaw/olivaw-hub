//! One SLAM worker per car: a thread that owns a `Slam`, takes the latest
//! rotation (drop-oldest, never queues up), and reports poses and maps.

use std::sync::{Arc, Condvar, Mutex};
use std::time::Instant;

use olivaw_proto::ScanFrame;
use olivaw_slam::{Pose2, Slam, SlamConfig};
use tokio::sync::mpsc;

use crate::slam::map_png::MapSnapshot;
use crate::state::PoseInfo;
use crate::units::scan_frame_to_cloud;

/// What the worker reports back to the pipeline.
#[derive(Debug)]
pub enum SlamOutput {
    /// A processed scan: pose + the scan in the map frame (decimated).
    Tracked {
        /// Pose after this scan.
        pose: PoseInfo,
        /// The scan in the map frame, decimated.
        scan_world: Vec<[f32; 2]>,
    },
    /// A fresh map render.
    Map(Arc<MapSnapshot>),
}

/// Latest-frame mailbox between the async pipeline and the worker thread.
#[derive(Default)]
struct Mailbox {
    slot: Mutex<Option<ScanFrame>>,
    ready: Condvar,
}

/// Handle to a running worker.
pub struct SlamWorker {
    mailbox: Arc<Mailbox>,
}

/// Tunables handed to the worker.
#[derive(Debug, Clone, Copy)]
pub struct WorkerTuning {
    /// Render a map every N processed scans.
    pub map_every_scans: u32,
    /// Points per scan forwarded to the dashboard.
    pub ws_scan_points: usize,
}

impl SlamWorker {
    /// Spawn the thread. `out` receives every result; `rerun` (if any) gets the viz.
    pub fn spawn(
        car: String,
        tuning: WorkerTuning,
        out: mpsc::UnboundedSender<(String, SlamOutput)>,
        rerun: Option<crate::viz::rerun::Sink>,
    ) -> anyhow::Result<Self> {
        let mailbox = Arc::new(Mailbox::default());
        let inbox = mailbox.clone();
        std::thread::Builder::new()
            .name(format!("slam-{car}"))
            .spawn(move || {
                if let Err(e) = run(&car, tuning, &inbox, &out, rerun.as_ref()) {
                    tracing::error!("slam worker {car} stopped: {e}");
                }
            })?;
        Ok(Self { mailbox })
    }

    /// Hand the worker a rotation, replacing any unprocessed one.
    pub fn submit(&self, frame: ScanFrame) {
        let mut slot = self
            .mailbox
            .slot
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if slot.replace(frame).is_some() {
            tracing::trace!("slam: dropped a rotation (worker busy)");
        }
        self.mailbox.ready.notify_one();
    }
}

fn run(
    car: &str,
    tuning: WorkerTuning,
    inbox: &Mailbox,
    out: &mpsc::UnboundedSender<(String, SlamOutput)>,
    rerun: Option<&crate::viz::rerun::Sink>,
) -> anyhow::Result<()> {
    let mut slam = Slam::new(SlamConfig::default())?;
    let mut processed: u32 = 0;
    let mut map_version: u64 = 0;
    let mut trajectory: Vec<(f32, f32)> = Vec::new();
    let started = Instant::now();
    tracing::info!("slam worker {car}: ready");

    loop {
        let frame = {
            let mut slot = inbox
                .slot
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            while slot.is_none() {
                slot = inbox
                    .ready
                    .wait(slot)
                    .unwrap_or_else(std::sync::PoisonError::into_inner);
            }
            slot.take()
        };
        let Some(frame) = frame else { continue };

        let timestamp_ns = u64::from(frame.t_ms) * 1_000_000;
        let cloud = scan_frame_to_cloud(&frame, timestamp_ns);
        if cloud.is_empty() {
            continue;
        }
        let t0 = Instant::now();
        let pose = match slam.process_scan(&cloud) {
            Ok(p) => p,
            Err(e) => {
                tracing::warn!("slam {car}: scan {} rejected: {e}", frame.seq);
                continue;
            }
        };
        let match_ms = t0.elapsed().as_secs_f32() * 1e3;
        processed = processed.wrapping_add(1);
        trajectory.push((pose.x as f32, pose.y as f32));

        let scan_world = decimate(&cloud.points, &pose, tuning.ws_scan_points);
        let info = PoseInfo {
            x: pose.x,
            y: pose.y,
            theta: pose.theta,
            seq: frame.seq,
            match_ms,
            keyframes: slam.keyframes().len(),
            loops: slam.loops_closed(),
        };
        if let Some(sink) = rerun {
            sink.log_tracked(car, processed, &scan_world, &trajectory);
        }
        if out
            .send((
                car.to_owned(),
                SlamOutput::Tracked {
                    pose: info,
                    scan_world,
                },
            ))
            .is_err()
        {
            return Ok(());
        }

        if processed.is_multiple_of(tuning.map_every_scans.max(1)) {
            map_version += 1;
            let snapshot = Arc::new(MapSnapshot::from_grid(slam.grid(), map_version)?);
            if let Some(sink) = rerun {
                sink.log_map(car, slam.grid());
            }
            if out
                .send((car.to_owned(), SlamOutput::Map(snapshot)))
                .is_err()
            {
                return Ok(());
            }
            if map_version.is_multiple_of(30) {
                tracing::info!(
                    "slam {car}: {processed} scans in {:.0} s, {} keyframes, {} loops, last match {match_ms:.1} ms",
                    started.elapsed().as_secs_f32(),
                    slam.keyframes().len(),
                    slam.loops_closed()
                );
            }
        }
    }
}

/// Transform to the map frame and keep at most `max` evenly spaced points.
fn decimate(points: &[olivaw_slam::Point2], pose: &Pose2, max: usize) -> Vec<[f32; 2]> {
    let step = (points.len() / max.max(1)).max(1);
    points
        .iter()
        .step_by(step)
        .map(|p| {
            let w = pose.transform_point(*p);
            [w.x as f32, w.y as f32]
        })
        .collect()
}
