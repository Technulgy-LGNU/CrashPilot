use futures_util::StreamExt;
use prost::Message;
use tokio::sync::mpsc::Sender;
use tokio_tungstenite::connect_async;
use crate::config;
use crate::proto::CpInterface;
use crate::ssl_communication::Event;

pub async fn spawn_websocket(cfg: &config::Config, tx: Sender<Event>) {
  let url = format!("ws://{}:{}", cfg.server.websocket_host, cfg.server.websocket_port);

  let (ws_stream, _) = connect_async(url)
    .await
    .expect("WS connect failed");

  let (_, mut read) = ws_stream.split();

  tokio::spawn(async move {
    while let Some(msg) = read.next().await {
      match msg {
        Ok(msg) if msg.is_binary() => {
          let data = msg.into_data();

          match CpInterface::decode(&*data) {
            Ok(decoded) => {
              tx.send(Event::Websocket(decoded)).await.unwrap_or_else(|e| {
                eprintln!("Failed to send WebSocket event: {}", e);
              });
            }
            Err(e) => {
              eprintln!("Protobuf decode error: {}", e);
            }
          }
        }
        Ok(_) => {}
        Err(e) => {
          eprintln!("WebSocket error: {}", e);
          break;
        }
      }
    }
  });
}
