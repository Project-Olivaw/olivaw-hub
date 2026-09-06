//! Publish a recorded session over MQTT as if it were the car — exercises the
//! broker → ingest path without an ESP32.
//!
//! ```sh
//! cargo run --release -- --simulate --record sim.olivawrec   # in one shell, Ctrl-C after a while
//! cargo run --release                                          # hub, listening
//! cargo run --example fake_car -- sim.olivawrec [car-id] [host] [port]
//! ```

use std::time::Duration;

use olivaw_hub::ingest::EventKind;
use olivaw_hub::session::recorder::read_all;
use olivaw_proto::ScanFrame;
use olivaw_proto::topics::{Channel, topic};
use rumqttc::v5::mqttbytes::QoS;
use rumqttc::v5::{AsyncClient, MqttOptions};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let file = args.next().ok_or_else(|| {
        anyhow::anyhow!("usage: fake_car <file.olivawrec> [car-id] [host] [port]")
    })?;
    let car = args.next().unwrap_or_else(|| "car-01".into());
    let host = args.next().unwrap_or_else(|| "127.0.0.1".into());
    let port: u16 = args.next().map_or(Ok(1883), |p| p.parse())?;

    let events = read_all(&std::fs::read(&file)?)?;
    println!(
        "{}: {} events → mqtt://{host}:{port} as {car}",
        file,
        events.len()
    );

    let mut options = MqttOptions::new(format!("fake-{car}"), host, port);
    options.set_keep_alive(Duration::from_secs(20));
    let (client, mut eventloop) = AsyncClient::new(options, 32);
    tokio::spawn(async move {
        loop {
            if let Err(e) = eventloop.poll().await {
                eprintln!("mqtt: {e}");
                tokio::time::sleep(Duration::from_secs(2)).await;
            }
        }
    });

    let t = |c: Channel| {
        topic(&car, c)
            .map(|s| s.as_str().to_owned())
            .ok_or_else(|| anyhow::anyhow!("car id too long"))
    };
    let (status, telemetry, scan) = (
        t(Channel::Status)?,
        t(Channel::Telemetry)?,
        t(Channel::Scan)?,
    );
    let mut buf = vec![0u8; ScanFrame::MAX_ENCODED_LEN];
    let mut last_ms: Option<u64> = None;
    for event in events {
        if let Some(prev) = last_ms
            && event.at_ms > prev
        {
            tokio::time::sleep(Duration::from_millis((event.at_ms - prev).min(2000))).await;
        }
        last_ms = Some(event.at_ms);
        match event.kind {
            EventKind::Status(s) => {
                client
                    .publish(&status, QoS::AtMostOnce, true, s.into_bytes())
                    .await?;
            }
            EventKind::Telemetry(t) => {
                client
                    .publish(&telemetry, QoS::AtMostOnce, false, serde_json::to_vec(&t)?)
                    .await?;
            }
            EventKind::Scan(frame) => {
                let bytes = frame.encode(&mut buf)?.to_vec();
                client.publish(&scan, QoS::AtMostOnce, false, bytes).await?;
            }
        }
    }
    client
        .publish(&status, QoS::AtMostOnce, true, b"offline".to_vec())
        .await?;
    tokio::time::sleep(Duration::from_millis(300)).await;
    println!("done");
    Ok(())
}
