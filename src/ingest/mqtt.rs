//! The hub's MQTT client: subscribes to every car, publishes commands.

use std::time::Duration;

use rumqttc::v5::mqttbytes::QoS;
use rumqttc::v5::mqttbytes::v5::Packet;
use rumqttc::v5::{AsyncClient, Event, MqttOptions};
use tokio::sync::mpsc;

use crate::config::MqttConfig;
use crate::ingest::CarEvent;

fn options(cfg: &MqttConfig, suffix: &str) -> MqttOptions {
    let mut o = MqttOptions::new(
        format!("{}-{suffix}", cfg.client_id),
        cfg.host.clone(),
        cfg.port,
    );
    o.set_keep_alive(Duration::from_secs(20));
    o.set_max_packet_size(Some(1 << 20));
    if let (Some(u), Some(p)) = (&cfg.username, &cfg.password) {
        o.set_credentials(u.clone(), p.clone());
    }
    o
}

/// Subscribe to `olivaw/+/+` and forward decoded events. Reconnects forever.
pub fn spawn(cfg: MqttConfig, tx: mpsc::Sender<CarEvent>) {
    tokio::spawn(async move {
        let (client, mut eventloop) = AsyncClient::new(options(&cfg, "sub"), 64);
        let mut subscribed = false;
        loop {
            match eventloop.poll().await {
                Ok(Event::Incoming(Packet::ConnAck(_))) => {
                    tracing::info!("mqtt: connected to {}:{}", cfg.host, cfg.port);
                    subscribed = false;
                }
                Ok(Event::Incoming(Packet::Publish(p))) => {
                    let topic = String::from_utf8_lossy(&p.topic);
                    if let Some(event) = CarEvent::decode(&topic, &p.payload)
                        && tx.send(event).await.is_err()
                    {
                        tracing::error!("mqtt: pipeline gone, stopping ingest");
                        return;
                    }
                }
                Ok(_) => {}
                Err(e) => {
                    tracing::warn!("mqtt: {e}; retrying in 3 s");
                    tokio::time::sleep(Duration::from_secs(3)).await;
                    continue;
                }
            }
            if !subscribed {
                match client
                    .subscribe(olivaw_proto::topics::ALL_CARS, QoS::AtMostOnce)
                    .await
                {
                    Ok(()) => subscribed = true,
                    Err(e) => tracing::warn!("mqtt: subscribe failed: {e}"),
                }
            }
        }
    });
}

/// A handle that publishes to `olivaw/<car>/cmd`.
#[derive(Clone)]
pub struct Commander {
    client: AsyncClient,
}

impl Commander {
    /// Publish raw command bytes (a drive frame or a `Control` opcode).
    pub async fn send(&self, car: &str, payload: Vec<u8>) -> anyhow::Result<()> {
        let topic = olivaw_proto::topics::topic(car, olivaw_proto::topics::Channel::Cmd)
            .ok_or_else(|| anyhow::anyhow!("car id too long"))?;
        self.client
            .publish(topic.as_str(), QoS::AtMostOnce, false, payload)
            .await?;
        Ok(())
    }
}

/// Build the command publisher (its event loop runs in the background).
pub fn commander(cfg: &MqttConfig) -> Commander {
    let (client, mut eventloop) = AsyncClient::new(options(cfg, "cmd"), 16);
    tokio::spawn(async move {
        loop {
            if let Err(e) = eventloop.poll().await {
                tracing::debug!("mqtt cmd client: {e}");
                tokio::time::sleep(Duration::from_secs(3)).await;
            }
        }
    });
    Commander { client }
}
