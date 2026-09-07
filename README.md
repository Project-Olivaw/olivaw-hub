# olivaw-hub

The telemetry hub for Olivaw vehicles. One Rust binary that:

- runs an **MQTT broker** (embedded [rumqttd](https://github.com/bytebeamio/rumqtt), v5 on `:1883`, v3.1.1 on `:1884`) — or connects to an external one (EMQX, Mosquitto) with `--no-broker`;
- **ingests** what the car publishes (`olivaw/<car>/{status,telemetry,scan}`) using the shared `olivaw-proto` types;
- runs **live SLAM** per car with [olivaw-slam](https://github.com/Project-Olivaw/olivaw-slam) (scan-matching only — the car has no encoders) and renders the occupancy grid to PNG/PGM;
- serves an **HTTP + WebSocket API** (`/api/cars`, `/api/cars/{id}/map.png`, `/ws`) and the built [olivaw-dashboard](https://github.com/Project-Olivaw/olivaw-dashboard);
- streams to **rerun** (`--rerun`, feature `rerun`), **records** sessions (`--record`), **replays** them (`--replay`) and can **simulate** a car driving a room (`--simulate`) so everything downstream works without hardware.

```text
car ──MQTT──▶ broker ──▶ ingest ──▶ pipeline ──▶ SLAM worker ──▶ state ──▶ HTTP / WebSocket ──▶ dashboard
                                       │              └──▶ rerun
                                       └──▶ recorder     replay / simulator ──▶ (same pipeline)
```

## Run

```sh
cargo run --release -- --simulate                 # no hardware: a car drives a demo room
cargo run --release                               # real car: broker on :1883, API on :8080
cargo run --release -- --record run1.olivawrec    # keep the session
cargo run --release -- --replay run1.olivawrec    # play it back (--replay-speed 0 = max speed)
cargo run --release --features rerun -- --simulate --rerun
cargo run --release -- --lidar                    # RPLIDAR on this machine's USB, published as car "mac-lidar"
cargo run --release -- --log-only                 # just print decoded MQTT events
cargo run --release --example fake_car -- run1.olivawrec   # publish a recording over MQTT as if it were the car
```

Config: `hub.toml` (see `src/config.rs` for every field and default), `--config`, or `OLIVAW_HUB_CONFIG`.
The dashboard is served from `../olivaw-dashboard/dist` when it exists (`--dashboard-dir` to override).

## API

| Route | What |
| --- | --- |
| `GET /api/health` | liveness + counts |
| `GET /api/cars` · `GET /api/cars/{id}` | snapshots: status, telemetry, pose, trajectory, last scan, map metadata |
| `GET /api/cars/{id}/map.png` · `…/map.pgm` | occupancy grid (gray: occupied dark, free white, unknown mid) |
| `POST /api/cars/{id}/cmd` | `{"op":"e_stop"}` · `{"op":"clear"}` · `{"op":"drive","left":300,"right":300}` · `{"op":"lidar","on":true}` · `{"op":"max_duty","permille":550}` |
| `GET /ws` | `hello` snapshot then `status` / `telemetry` / `pose` / `scan` / `map` / `stats` events (JSON, `type`-tagged) |

## Verify

```sh
cargo test && cargo clippy --all-targets -- -D warnings
cargo run --release -- --simulate    # then open http://localhost:8080/api/cars and curl …/map.png
```
