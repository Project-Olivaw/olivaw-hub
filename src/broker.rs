//! The embedded MQTT broker (rumqttd) on its own thread.

use crate::config::BrokerConfig;

/// Start rumqttd with a v5 and a v4 listener. Returns once the thread is spawned.
#[cfg(feature = "broker")]
pub fn start(cfg: &BrokerConfig) -> anyhow::Result<()> {
    let toml_cfg = format!(
        r#"
id = 0

[router]
id = 0
max_connections = 64
max_outgoing_packet_count = 200
max_segment_size = 10485760
max_segment_count = 10

[v5.1]
name = "v5"
listen = "{v5}"
next_connection_delay_ms = 1
    [v5.1.connections]
    connection_timeout_ms = 60000
    max_payload_size = {payload}
    max_inflight_count = 100
    dynamic_filters = true

[v4.1]
name = "v4"
listen = "{v4}"
next_connection_delay_ms = 1
    [v4.1.connections]
    connection_timeout_ms = 60000
    max_payload_size = {payload}
    max_inflight_count = 100
    dynamic_filters = true
"#,
        v5 = cfg.v5_listen,
        v4 = cfg.v4_listen,
        payload = cfg.max_payload_size
    );
    let config: rumqttd::Config = toml::from_str(&toml_cfg)?;
    let mut broker = rumqttd::Broker::new(config);
    tracing::info!(
        "broker: MQTT v5 on {}, v3.1.1 on {}",
        cfg.v5_listen,
        cfg.v4_listen
    );
    std::thread::Builder::new()
        .name("rumqttd".into())
        .spawn(move || {
            if let Err(e) = broker.start() {
                tracing::error!("broker stopped: {e}");
            }
        })?;
    Ok(())
}

/// Built without the `broker` feature: point `[mqtt]` at an external broker.
#[cfg(not(feature = "broker"))]
pub fn start(_cfg: &BrokerConfig) -> anyhow::Result<()> {
    anyhow::bail!("built without the `broker` feature; run with --no-broker and an external broker")
}
