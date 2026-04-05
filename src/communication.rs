// Combines the SSL, Robot and Websocket communication into one stream

use crate::config;
use crate::interface::spawn_websocket;
use crate::proto::{CpInterface, Referee, TrackerWrapperPacket};
use crate::ssl_communication::get_ssl_data;

#[derive(Debug)]
pub enum Event {
  Referee(Referee),
  SslWrapper(TrackerWrapperPacket),
  Websocket(CpInterface),
}

pub async fn communication_receiver(cfg: &config::Config) -> anyhow::Result<tokio::sync::mpsc::Receiver<Event>> {
  let (tx, rx) = tokio::sync::mpsc::channel::<Event>(1000);

  get_ssl_data(cfg, tx.clone()).await;


  spawn_websocket(cfg, tx.clone()).await;
  Ok(rx)
}

