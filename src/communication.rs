use crate::config;
use core_dump::proto::{Referee, SslWrapperPacket, TrackerWrapperPacket};
use std::sync::Arc;
use std::time::Instant;
use tokio::sync::RwLock;

pub mod interface;
pub mod multicast_handler;
pub mod robot_sdr_handler;
pub mod robot_wifi_handler;

pub struct Events {
  pub raw: Option<SslWrapperPacket>,
  pub tracked: Option<TrackerWrapperPacket>,
  pub gc: Option<Referee>,
}
impl Events {
  pub fn new() -> Self {
    Self {
      raw: None,
      tracked: None,
      gc: None,
    }
  }

  pub fn take(&mut self) -> Self {
    Self {
      raw: self.raw.take(),
      tracked: self.tracked.take(),
      gc: self.gc.take(),
    }
  }
}

pub type EventShare = Arc<RwLock<Events>>;

/// Handles returned by [`communication_receiver`].
///
/// - `events`: the latest inbound packets from SSL-Vision / GC / WebSocket (interface -> CP)
/// - `ws_out`: broadcast channel for outbound WebSocket packets (CP -> interface)
#[derive(Clone)]
pub struct CommunicationHandles {
  pub events: EventShare,
  // pub ws_out: WebsocketOut,
}

pub fn communication_receiver(
  cfg: &config::Config,
  process_start: Instant,
) -> anyhow::Result<CommunicationHandles> {
  let events = Arc::new(RwLock::new(Events::new()));

  Ok(CommunicationHandles { events })
}
