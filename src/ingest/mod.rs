//! Events entering the hub, whatever their source (MQTT, replay, simulator).

pub mod mqtt;
pub mod serial;

use std::time::{SystemTime, UNIX_EPOCH};

use olivaw_proto::topics::{self, Channel};
use olivaw_proto::{ScanFrame, Telemetry};
use serde::{Deserialize, Serialize};

/// One thing a car said.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarEvent {
    /// Car id from the topic.
    pub car: String,
    /// Wall-clock milliseconds since the Unix epoch when the hub saw it.
    pub at_ms: u64,
    /// Payload.
    pub kind: EventKind,
}

/// Decoded payloads.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum EventKind {
    /// `online` / `offline` (retained + last will).
    Status(String),
    /// JSON telemetry.
    Telemetry(Telemetry),
    /// One lidar rotation (boxed: ~3.6 KB inline).
    Scan(Box<ScanFrame>),
}

impl CarEvent {
    /// Stamp `kind` with the current time.
    pub fn now(car: impl Into<String>, kind: EventKind) -> Self {
        Self {
            car: car.into(),
            at_ms: now_ms(),
            kind,
        }
    }

    /// Decode an MQTT message; `None` for foreign topics or undecodable payloads (logged).
    pub fn decode(topic: &str, payload: &[u8]) -> Option<Self> {
        let (car, channel) = topics::parse(topic)?;
        let kind = match channel {
            Channel::Status => EventKind::Status(String::from_utf8_lossy(payload).into_owned()),
            Channel::Telemetry => match serde_json::from_slice::<Telemetry>(payload) {
                Ok(t) => EventKind::Telemetry(t),
                Err(e) => {
                    tracing::warn!("{topic}: bad telemetry json: {e}");
                    return None;
                }
            },
            Channel::Scan => match ScanFrame::decode(payload) {
                Ok(s) => EventKind::Scan(Box::new(s)),
                Err(e) => {
                    tracing::warn!("{topic}: bad scan frame: {e}");
                    return None;
                }
            },
            Channel::Cmd => return None,
        };
        Some(Self::now(car, kind))
    }
}

/// Milliseconds since the Unix epoch.
pub fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_millis()).unwrap_or(u64::MAX))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_status_and_ignores_cmd() {
        let e = CarEvent::decode("olivaw/car-01/status", b"online").unwrap();
        assert_eq!(e.car, "car-01");
        assert!(matches!(e.kind, EventKind::Status(ref s) if s == "online"));
        assert!(CarEvent::decode("olivaw/car-01/cmd", b"\x01").is_none());
        assert!(CarEvent::decode("other/topic", b"x").is_none());
    }

    #[test]
    fn decodes_telemetry_json_and_scan_postcard() {
        let t = Telemetry {
            battery_mv: 11_800,
            battery_pct: 70,
            ..Telemetry::default()
        };
        let json = serde_json::to_vec(&t).unwrap();
        let e = CarEvent::decode("olivaw/car-01/telemetry", &json).unwrap();
        assert!(matches!(e.kind, EventKind::Telemetry(ref x) if x.battery_mv == 11_800));

        let mut frame = ScanFrame {
            seq: 3,
            ..ScanFrame::default()
        };
        frame
            .points
            .push(olivaw_proto::ScanPoint {
                angle_q6: 64,
                dist_q2: 4000,
                quality: 40,
            })
            .unwrap();
        let mut buf = [0u8; ScanFrame::MAX_ENCODED_LEN];
        let bytes = frame.encode(&mut buf).unwrap();
        let e = CarEvent::decode("olivaw/car-01/scan", bytes).unwrap();
        assert!(matches!(e.kind, EventKind::Scan(ref s) if s.seq == 3 && s.points.len() == 1));
    }
}
