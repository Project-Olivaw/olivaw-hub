# CLAUDE.md — olivaw-hub

> The server side of Project Olivaw: broker + ingest + live SLAM + API. Tokio is fine here (the "no
> tokio" rule belongs to the SLAM library). House standards and lessons live in the vault:
> `../P-Olivaw-Second-Brain/P-Olivaw-Second-Brain/Home.md`.

## Rules

- Every event source (MQTT, replay, simulator) feeds the same `pipeline`; never special-case one.
- Units convert exactly once, in `src/units.rs` (lidar Q6/Q2 → metres/radians). Nowhere else.
- SLAM runs on its own thread per car with a latest-frame mailbox: never let scans queue up.
- Wire types come from `olivaw-proto`; do not redefine them here.
- `cargo test && cargo clippy --all-targets -- -D warnings` must stay green; the simulator is the
  integration test (`--simulate`, then `/api/cars`, `/ws`, `map.png`).
