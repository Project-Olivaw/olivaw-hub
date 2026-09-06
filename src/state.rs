//! What the API serves: per-car snapshots, the latest map, and the live
//! event stream every WebSocket client receives.

use std::collections::HashMap;
use std::sync::{Arc, RwLock};

use olivaw_proto::Telemetry;
use serde::Serialize;
use tokio::sync::broadcast;

use crate::config::HubConfig;
use crate::ingest::mqtt::Commander;
use crate::slam::map_png::MapSnapshot;

/// Pose of a car in the map frame (metres, radians).
#[derive(Debug, Clone, Copy, Serialize, Default)]
pub struct PoseInfo {
    /// Metres.
    pub x: f64,
    /// Metres.
    pub y: f64,
    /// Radians, counter-clockwise from +x.
    pub theta: f64,
    /// Rotation counter of the scan that produced it.
    pub seq: u32,
    /// Time the matcher took, milliseconds.
    pub match_ms: f32,
    /// Keyframes in the pose graph.
    pub keyframes: usize,
    /// Loop closures accepted.
    pub loops: usize,
}

/// Map metadata sent to the dashboard (pixels come from `/api/cars/{id}/map.png`).
#[derive(Debug, Clone, Serialize)]
pub struct MapInfo {
    /// Increments on every snapshot.
    pub version: u64,
    /// Where to fetch the PNG.
    pub url: String,
    /// World coordinates of the bottom-left corner, metres.
    pub origin: [f64; 2],
    /// Metres per pixel.
    pub resolution: f64,
    /// Pixels.
    pub width: u32,
    /// Pixels.
    pub height: u32,
}

/// Everything known about one car.
#[derive(Debug, Clone, Serialize)]
pub struct CarSnapshot {
    /// Car id.
    pub id: String,
    /// `online` / `offline` / `unknown`.
    pub status: String,
    /// Last telemetry.
    pub telemetry: Option<Telemetry>,
    /// Last pose.
    pub pose: Option<PoseInfo>,
    /// Recent poses, `[x, y]` metres, oldest first.
    pub trajectory: Vec<[f32; 2]>,
    /// Last decimated scan in the map frame, `[x, y]` metres.
    pub scan: Vec<[f32; 2]>,
    /// Latest map metadata.
    pub map: Option<MapInfo>,
    /// Unix milliseconds of the last event.
    pub last_seen_ms: u64,
    /// Rotations received in total.
    pub scans_total: u64,
    /// Measured scan rate, hertz.
    pub scan_hz: f32,
}

impl CarSnapshot {
    /// A car we have just heard of.
    pub fn new(id: &str) -> Self {
        Self {
            id: id.into(),
            status: "unknown".into(),
            telemetry: None,
            pose: None,
            trajectory: Vec::new(),
            scan: Vec::new(),
            map: None,
            last_seen_ms: 0,
            scans_total: 0,
            scan_hz: 0.0,
        }
    }
}

/// Live events pushed over `/ws`.
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum WsEvent {
    /// First message on connect: everything known.
    Hello {
        /// Every known car.
        cars: Vec<CarSnapshot>,
    },
    /// Online/offline change.
    Status {
        /// Car id.
        car: String,
        /// `online` / `offline`.
        status: String,
    },
    /// New telemetry.
    Telemetry {
        /// Car id.
        car: String,
        /// The sample.
        telemetry: Telemetry,
        /// Unix milliseconds.
        at_ms: u64,
    },
    /// New pose from SLAM.
    Pose {
        /// Car id.
        car: String,
        /// The pose.
        pose: PoseInfo,
    },
    /// Decimated scan in the map frame.
    Scan {
        /// Car id.
        car: String,
        /// Rotation counter.
        seq: u32,
        /// `[x, y]` metres in the map frame.
        points: Vec<[f32; 2]>,
    },
    /// New map snapshot available.
    Map {
        /// Car id.
        car: String,
        /// Where and how big.
        map: MapInfo,
    },
    /// Rates, once a second.
    Stats {
        /// Car id.
        car: String,
        /// Rotations per second over the last second.
        scan_hz: f32,
        /// Rotations since the hub started.
        scans_total: u64,
    },
}

/// Shared hub state.
pub struct Hub {
    /// Configuration.
    pub cfg: HubConfig,
    /// Per-car snapshots.
    pub cars: RwLock<HashMap<String, CarSnapshot>>,
    /// Latest map pixels per car.
    pub maps: RwLock<HashMap<String, Arc<MapSnapshot>>>,
    /// Live event fan-out.
    pub events: broadcast::Sender<WsEvent>,
    /// Command publisher.
    pub commander: Commander,
}

impl Hub {
    /// Empty hub.
    pub fn new(cfg: HubConfig, commander: Commander) -> Self {
        let (events, _) = broadcast::channel(512);
        Self {
            cfg,
            cars: RwLock::new(HashMap::new()),
            maps: RwLock::new(HashMap::new()),
            events,
            commander,
        }
    }

    /// Mutate (creating) a car's snapshot.
    pub fn update_car<R>(&self, id: &str, f: impl FnOnce(&mut CarSnapshot) -> R) -> R {
        let mut cars = self
            .cars
            .write()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let car = cars
            .entry(id.to_owned())
            .or_insert_with(|| CarSnapshot::new(id));
        f(car)
    }

    /// All snapshots, sorted by id.
    pub fn snapshots(&self) -> Vec<CarSnapshot> {
        let cars = self
            .cars
            .read()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let mut v: Vec<_> = cars.values().cloned().collect();
        v.sort_by(|a, b| a.id.cmp(&b.id));
        v
    }

    /// Broadcast to every WebSocket client (no-op without listeners).
    pub fn emit(&self, event: WsEvent) {
        let _ = self.events.send(event);
    }
}
