//! `--lidar [PORT]`: an RPLIDAR plugged into this machine, published as a car of its own.
//!
//! Useful before the ESP32 uplink exists (or to build a map by carrying the laptop around):
//! the same pipeline, SLAM worker, API and dashboard as a real car, fed from the serial port.

use std::thread;
use std::time::{Duration, Instant};

use olivaw_lidar::Lidar;
use olivaw_lidar::transport::{auto_detect_port, prefer_callout_device};
use olivaw_proto::{Flags, LinkState, ScanFrame, ScanPoint, Telemetry};
use tokio::sync::mpsc;

use crate::ingest::{CarEvent, EventKind};

/// Spawn a blocking reader thread. `port` `None` auto-detects the USB adapter.
pub fn spawn(
    port: Option<String>,
    car_id: String,
    tx: mpsc::Sender<CarEvent>,
) -> anyhow::Result<()> {
    let port = match port {
        Some(p) => prefer_callout_device(&p),
        None => auto_detect_port().ok_or_else(|| {
            anyhow::anyhow!("no serial port looks like a lidar; plug the C1's USB adapter in or pass --lidar /dev/cu.usbserial-XXXX")
        })?,
    };
    thread::Builder::new()
        .name("lidar-serial".into())
        .spawn(move || {
            if let Err(e) = run(&port, &car_id, &tx) {
                tracing::error!("lidar {port}: {e}");
            }
        })?;
    Ok(())
}

fn run(port: &str, car_id: &str, tx: &mpsc::Sender<CarEvent>) -> anyhow::Result<()> {
    tracing::info!("lidar: opening {port} as {car_id}");
    let mut lidar = Lidar::open(port)?;
    let health = lidar.health()?;
    tracing::info!("lidar: health {health:?}");
    lidar.start_scan()?;
    let started = Instant::now();
    tx.blocking_send(CarEvent::now(car_id, EventKind::Status("online".into())))?;

    let mut seq: u32 = 0;
    let mut last_telemetry: Option<Instant> = None;
    for scan in lidar.scans() {
        let scan = scan?;
        let mut frame = ScanFrame {
            seq,
            t_ms: elapsed_ms(started),
            ..ScanFrame::default()
        };
        for p in scan.points() {
            let point = ScanPoint {
                angle_q6: to_q(p.angle_deg, 64.0),
                dist_q2: to_q(p.distance_mm, 4.0),
                quality: p.quality,
            };
            if frame.points.push(point).is_err() {
                frame.dropped = frame.dropped.saturating_add(1);
            }
        }
        seq = seq.wrapping_add(1);
        tx.blocking_send(CarEvent::now(car_id, EventKind::Scan(Box::new(frame))))?;

        if last_telemetry.is_none_or(|t| t.elapsed() >= Duration::from_secs(1)) {
            last_telemetry = Some(Instant::now());
            let t = Telemetry {
                flags: Flags(0),
                uptime_s: u32::try_from(started.elapsed().as_secs()).unwrap_or(u32::MAX),
                lidar: LinkState::Up,
                ..Telemetry::default()
            };
            tx.blocking_send(CarEvent::now(car_id, EventKind::Telemetry(t)))?;
        }
    }
    Ok(())
}

fn elapsed_ms(since: Instant) -> u32 {
    u32::try_from(since.elapsed().as_millis()).unwrap_or(u32::MAX)
}

/// Fixed-point with saturation; sensor values are non-negative.
fn to_q(v: f32, scale: f32) -> u16 {
    let q = (v * scale).round().clamp(0.0, f32::from(u16::MAX));
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    {
        q as u16
    }
}
