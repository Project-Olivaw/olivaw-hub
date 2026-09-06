//! A 7 × 5 m room with two boxes, a scripted loop, a ray-cast lidar.

use std::f64::consts::{PI, TAU};
use std::time::Duration;

use olivaw_proto::{Flags, LinkState, ScanFrame, ScanPoint, Telemetry};
use tokio::sync::mpsc;

use crate::config::SimConfig;
use crate::ingest::{CarEvent, EventKind};

/// Line segment, metres.
#[derive(Debug, Clone, Copy)]
struct Wall {
    a: (f64, f64),
    b: (f64, f64),
}

fn walls() -> Vec<Wall> {
    let rect = |x0: f64, y0: f64, x1: f64, y1: f64| {
        vec![
            Wall {
                a: (x0, y0),
                b: (x1, y0),
            },
            Wall {
                a: (x1, y0),
                b: (x1, y1),
            },
            Wall {
                a: (x1, y1),
                b: (x0, y1),
            },
            Wall {
                a: (x0, y1),
                b: (x0, y0),
            },
        ]
    };
    let mut w = rect(0.0, 0.0, 7.0, 5.0);
    w.extend(rect(2.0, 1.0, 2.8, 1.8)); // a box
    w.extend(rect(4.5, 3.0, 5.5, 4.2)); // a table
    w.extend(rect(6.2, 0.0, 7.0, 1.5)); // a corner cupboard
    w
}

/// Waypoints of the loop the car drives.
const PATH: [(f64, f64); 6] = [
    (1.0, 0.8),
    (5.5, 0.8),
    (5.8, 2.4),
    (3.5, 4.2),
    (1.0, 4.0),
    (0.8, 2.3),
];

/// Ray from `o` at bearing `theta` against every wall; nearest hit distance.
fn cast(o: (f64, f64), theta: f64, walls: &[Wall], max: f64) -> Option<f64> {
    let (dx, dy) = (theta.cos(), theta.sin());
    let mut best = max;
    for w in walls {
        let (ex, ey) = (w.b.0 - w.a.0, w.b.1 - w.a.1);
        let denom = dx * ey - dy * ex;
        if denom.abs() < 1e-9 {
            continue;
        }
        let (fx, fy) = (w.a.0 - o.0, w.a.1 - o.1);
        let t = (fx * ey - fy * ex) / denom;
        let u = (fx * dy - fy * dx) / denom;
        if t > 0.0 && (0.0..=1.0).contains(&u) && t < best {
            best = t;
        }
    }
    (best < max).then_some(best)
}

/// Tiny deterministic PRNG (no dependency).
struct Lcg(u64);
impl Lcg {
    fn next_f64(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    /// Roughly gaussian via sum of uniforms.
    fn noise(&mut self, sigma: f64) -> f64 {
        let s: f64 = (0..6).map(|_| self.next_f64()).sum::<f64>() - 3.0;
        s * sigma / 0.707
    }
}

/// Drive the loop forever.
pub async fn run(cfg: SimConfig, tx: mpsc::Sender<CarEvent>) {
    let walls = walls();
    let car = cfg.car_id.clone();
    let dt = 1.0 / cfg.scan_hz.max(1.0);
    #[allow(clippy::cast_sign_loss)]
    let dt_ms = (dt * 1000.0) as u32;
    #[allow(clippy::cast_sign_loss)]
    let scans_per_second = cfg.scan_hz.max(1.0) as u32;
    let mut ticker = tokio::time::interval(Duration::from_secs_f64(dt));
    let mut rng = Lcg(0x5EED);

    let (mut x, mut y) = PATH[0];
    let mut theta = 0.0f64;
    let mut target = 1usize;
    let mut seq: u32 = 0;
    let mut t_ms: u32 = 0;
    let mut battery_mv: f64 = 12_400.0;
    let max_range = 12.0;

    let _ = tx
        .send(CarEvent::now(&car, EventKind::Status("online".into())))
        .await;
    tracing::info!("sim: {car} driving the demo room at {} Hz", cfg.scan_hz);

    loop {
        ticker.tick().await;
        // Steer towards the waypoint, differential-drive style.
        let (tx_, ty_) = PATH[target];
        let want = (ty_ - y).atan2(tx_ - x);
        let mut err = want - theta;
        while err > PI {
            err -= TAU;
        }
        while err < -PI {
            err += TAU;
        }
        let omega = err.clamp(-1.2, 1.2);
        let v = if err.abs() > 0.6 { 0.05 } else { cfg.speed_mps };
        theta += omega * dt;
        x += v * theta.cos() * dt;
        y += v * theta.sin() * dt;
        if ((tx_ - x).powi(2) + (ty_ - y).powi(2)).sqrt() < 0.15 {
            target = (target + 1) % PATH.len();
        }

        // Lidar: 500 beams, clockwise from the car's forward axis.
        let mut frame = ScanFrame {
            seq,
            t_ms,
            ..ScanFrame::default()
        };
        for i in 0..500u32 {
            let lidar_deg = f64::from(i) * 360.0 / 500.0;
            let bearing = theta - lidar_deg.to_radians();
            let dist = cast((x, y), bearing, &walls, max_range)
                .map_or(0.0, |d| (d + rng.noise(cfg.noise_m)).max(0.05));
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let point = ScanPoint {
                angle_q6: (lidar_deg * 64.0).round() as u16,
                dist_q2: (dist * 1000.0 * 4.0).round().clamp(0.0, 65_535.0) as u16,
                quality: if dist > 0.0 { 47 } else { 0 },
            };
            let _ = frame.points.push(point);
        }
        seq = seq.wrapping_add(1);
        t_ms = t_ms.wrapping_add(dt_ms);
        if tx
            .send(CarEvent::now(&car, EventKind::Scan(Box::new(frame))))
            .await
            .is_err()
        {
            return;
        }

        if seq.is_multiple_of(scans_per_second) {
            battery_mv -= 0.6;
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let telemetry = Telemetry {
                flags: Flags::BLE_CONNECTED,
                battery_mv: battery_mv as u16,
                battery_pct: olivaw_battery_percent(battery_mv),
                left: ((v - omega * 0.08) / 0.6 * 550.0) as i16,
                right: ((v + omega * 0.08) / 0.6 * 550.0) as i16,
                uptime_s: t_ms / 1000,
                rssi_dbm: -55 - ((x * 3.0) as i8),
                lidar: LinkState::Up,
                wifi: LinkState::Up,
                mqtt: LinkState::Up,
                max_duty_permille: 550,
            };
            if tx
                .send(CarEvent::now(&car, EventKind::Telemetry(telemetry)))
                .await
                .is_err()
            {
                return;
            }
        }
    }
}

/// A coarse 3S curve for the simulated pack.
fn olivaw_battery_percent(pack_mv: f64) -> u8 {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    {
        ((pack_mv - 9_900.0) / (12_600.0 - 9_900.0) * 100.0).clamp(0.0, 100.0) as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rays_hit_the_room_walls() {
        let walls = walls();
        let d = cast((3.5, 2.5), 0.0, &walls, 20.0).unwrap();
        assert!((d - 3.5).abs() < 1e-9, "east wall at 7.0 → {d}");
        let d = cast((3.5, 2.5), PI / 2.0, &walls, 20.0).unwrap();
        assert!((d - 2.5).abs() < 1e-9, "north wall at 5.0 → {d}");
        assert!(
            cast((3.5, 2.5), 0.3, &walls, 0.1).is_none(),
            "beyond max range"
        );
    }
}
