//! Append-only session writer.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::ingest::CarEvent;

/// Writes events as they arrive.
pub struct Recorder {
    out: BufWriter<File>,
    records: u64,
}

impl Recorder {
    /// Create (truncate) the file.
    pub fn create(path: &Path) -> anyhow::Result<Self> {
        let out = BufWriter::new(File::create(path)?);
        tracing::info!("recording to {}", path.display());
        Ok(Self { out, records: 0 })
    }

    /// Append one event.
    pub fn write(&mut self, event: &CarEvent) -> anyhow::Result<()> {
        let bytes = postcard::to_stdvec(event)?;
        let len = u32::try_from(bytes.len())?;
        self.out.write_all(&len.to_le_bytes())?;
        self.out.write_all(&bytes)?;
        self.records += 1;
        if self.records.is_multiple_of(100) {
            self.out.flush()?;
        }
        Ok(())
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        let _ = self.out.flush();
        tracing::info!("recorded {} events", self.records);
    }
}

/// Parse a whole recording (used by replay and tests).
pub fn read_all(bytes: &[u8]) -> anyhow::Result<Vec<CarEvent>> {
    let mut events = Vec::new();
    let mut pos = 0;
    while pos + 4 <= bytes.len() {
        let len = u32::from_le_bytes([bytes[pos], bytes[pos + 1], bytes[pos + 2], bytes[pos + 3]])
            as usize;
        pos += 4;
        let Some(end) = pos.checked_add(len).filter(|&e| e <= bytes.len()) else {
            // A recording cut mid-write (hub killed) ends with a partial record; keep what we have.
            tracing::warn!(
                "recording truncated at byte {pos}; kept {} events",
                events.len()
            );
            break;
        };
        events.push(postcard::from_bytes(&bytes[pos..end])?);
        pos = end;
    }
    Ok(events)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ingest::EventKind;

    #[test]
    fn round_trips_through_a_file() {
        let dir = std::env::temp_dir().join(format!("olivaw-rec-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("t.olivawrec");
        {
            let mut rec = Recorder::create(&path).unwrap();
            rec.write(&CarEvent {
                car: "c".into(),
                at_ms: 1,
                kind: EventKind::Status("online".into()),
            })
            .unwrap();
            rec.write(&CarEvent {
                car: "c".into(),
                at_ms: 2,
                kind: EventKind::Telemetry(olivaw_proto::Telemetry::default()),
            })
            .unwrap();
        }
        let events = read_all(&std::fs::read(&path).unwrap()).unwrap();
        assert_eq!(events.len(), 2);
        assert_eq!(events[1].at_ms, 2);
        std::fs::remove_dir_all(dir).ok();
    }
}
