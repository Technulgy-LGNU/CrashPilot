// Combines the SSL, Robot and Websocket communication into one stream

use std::sync::Arc;
use tokio::sync::Mutex;
use crate::config;
use crate::interface::spawn_websocket;
use crate::proto::{ InterfaceWrapperCp, Referee, TrackerWrapperPacket };
use crate::ssl_communication::get_ssl_data;

#[derive(Debug)]
pub enum Event {
  SslWrapper(TrackerWrapperPacket),
  Referee(Referee),
  Websocket(InterfaceWrapperCp),
}

pub type EventShare = Arc<Mutex<(Option<TrackerWrapperPacket>, Option<InterfaceWrapperCp>, Option<Referee>)>>;

pub async fn communication_receiver(cfg: &config::Config) -> anyhow::Result<EventShare> {
  let tx = Arc::new(Mutex::new((None, None, None)));

  get_ssl_data(cfg, tx.clone()).await;


  spawn_websocket(cfg, tx.clone()).await;
  Ok(tx)
}

