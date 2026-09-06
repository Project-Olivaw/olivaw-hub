//! `hub.toml` — every knob with a default, so the hub runs with no file.

use std::net::SocketAddr;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// Top-level configuration.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct HubConfig {
    /// Embedded broker.
    pub broker: BrokerConfig,
    /// The broker the hub's own client connects to (the embedded one by default).
    pub mqtt: MqttConfig,
    /// HTTP + WebSocket server.
    pub http: HttpConfig,
    /// SLAM tuning.
    pub slam: SlamTuning,
    /// Simulator.
    pub sim: SimConfig,
}

impl HubConfig {
    /// Load from `path`, or defaults when `None`.
    pub fn load(path: Option<&Path>) -> anyhow::Result<Self> {
        match path {
            Some(p) => Ok(toml::from_str(&std::fs::read_to_string(p)?)?),
            None => Ok(Self::default()),
        }
    }
}

/// Embedded rumqttd listeners.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct BrokerConfig {
    /// MQTT v5 listener (the car and the hub client use this).
    pub v5_listen: SocketAddr,
    /// MQTT v3.1.1 listener for older tools (`mosquitto_sub`, MQTT Explorer).
    pub v4_listen: SocketAddr,
    /// Largest accepted publish payload, bytes.
    pub max_payload_size: usize,
}

impl Default for BrokerConfig {
    fn default() -> Self {
        Self {
            v5_listen: "0.0.0.0:1883"
                .parse()
                .unwrap_or_else(|_| SocketAddr::from(([0, 0, 0, 0], 1883))),
            v4_listen: "0.0.0.0:1884"
                .parse()
                .unwrap_or_else(|_| SocketAddr::from(([0, 0, 0, 0], 1884))),
            max_payload_size: 65_536,
        }
    }
}

/// Where the hub's MQTT client connects.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct MqttConfig {
    /// Broker host.
    pub host: String,
    /// Broker port (MQTT v5).
    pub port: u16,
    /// Client id.
    pub client_id: String,
    /// Optional credentials.
    pub username: Option<String>,
    /// Optional credentials.
    pub password: Option<String>,
}

impl Default for MqttConfig {
    fn default() -> Self {
        Self {
            host: "127.0.0.1".into(),
            port: 1883,
            client_id: "olivaw-hub".into(),
            username: None,
            password: None,
        }
    }
}

/// HTTP server.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct HttpConfig {
    /// Bind address.
    pub bind: SocketAddr,
    /// Built dashboard directory (`olivaw-dashboard/dist`) to serve at `/`.
    pub dashboard_dir: Option<PathBuf>,
    /// Directory for exported maps and recordings.
    pub data_dir: PathBuf,
}

impl Default for HttpConfig {
    fn default() -> Self {
        Self {
            bind: SocketAddr::from(([0, 0, 0, 0], 8080)),
            dashboard_dir: Some(PathBuf::from("../olivaw-dashboard/dist")),
            data_dir: PathBuf::from("data"),
        }
    }
}

/// The few SLAM numbers worth exposing; everything else is `SlamConfig::default()`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct SlamTuning {
    /// Publish a map snapshot every N scans (~1 Hz at 10 Hz scans).
    pub map_every_scans: u32,
    /// Points kept per scan sent to the dashboard (decimated).
    pub ws_scan_points: usize,
    /// Trajectory points kept per car.
    pub trajectory_len: usize,
}

impl Default for SlamTuning {
    fn default() -> Self {
        Self {
            map_every_scans: 10,
            ws_scan_points: 240,
            trajectory_len: 4000,
        }
    }
}

/// Simulator knobs.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct SimConfig {
    /// Car id the simulator reports as.
    pub car_id: String,
    /// Scan rate, hertz.
    pub scan_hz: f64,
    /// Cruise speed, metres per second.
    pub speed_mps: f64,
    /// Range noise, standard deviation, metres.
    pub noise_m: f64,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            car_id: "sim-01".into(),
            scan_hz: 10.0,
            speed_mps: 0.3,
            noise_m: 0.01,
        }
    }
}
