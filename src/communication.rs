// Combines the SSL, Robot and Websocket communication into one stream

mod create_multicast_socket;
mod gc_sender;
pub mod interface;
#[cfg(feature = "loki")]
pub mod loki;
mod prometheus;
mod robot_receiver;
pub mod robot_sender;
mod ssl_communication;
mod udp_listener;

use crate::communication::interface::spawn_websocket;
use crate::communication::robot_receiver::robot_receiver;
use crate::communication::ssl_communication::get_ssl_data;
use crate::config;
use core_dump::proto::{
  CpInterfaceWrapper, InterfaceWrapperCp, Referee, RobotCp, SslWrapperPacket, TrackerWrapperPacket,
};
use std::sync::Arc;
use tokio::sync::RwLock;
use tokio::sync::Notify;

#[derive(Debug, Clone, Default)]
pub struct Events {
  pub raw: Option<SslWrapperPacket>,
  pub tracked: Option<TrackerWrapperPacket>,
  pub ws: Option<InterfaceWrapperCp>,
  pub gc: Option<Referee>,
  pub rf: Option<RobotCp>,
}

impl Events {
  pub fn new() -> Self {
    Self {
      raw: None,
      tracked: None,
      ws: None,
      gc: None,
      rf: None,
    }
  }

  pub fn take(&mut self) -> Self {
    Self {
      raw: self.raw.take(),
      tracked: self.tracked.take(),
      ws: self.ws.take(),
      gc: self.gc.take(),
      rf: self.rf.take(),
    }
  }
}

pub type EventShare = Arc<RwLock<Events>>;

#[derive(Default)]
struct WsLatestState {
  seq: u64,
  payload: Option<CpInterfaceWrapper>,
}

/// Outbound WebSocket handle (CP -> interface).
///
/// This is intentionally implemented as an `Arc<RwLock<...>>` holding only the *latest* message.
/// If producers publish faster than a client can send, the client will skip intermediate updates
/// and only transmit the newest snapshot.
#[derive(Clone, Default)]
pub struct WebsocketOut {
  state: Arc<RwLock<WsLatestState>>,
  notify: Arc<Notify>,
}

impl WebsocketOut {
  pub fn new() -> Self {
    Self {
      state: Arc::new(RwLock::new(WsLatestState::default())),
      notify: Arc::new(Notify::new()),
    }
  }

  /// Publish a new binary payload.
  pub async fn publish(&self, payload: CpInterfaceWrapper) {
    let mut lock = self.state.write().await;
    lock.seq = lock.seq.wrapping_add(1);
    lock.payload = Some(payload);
    drop(lock);
    self.notify.notify_waiters();
  }

  /// Wait until a payload newer than `last_seq` is available and return it.
  ///
  /// This is implemented in a race-free way (won't miss notifications): it creates the
  /// notification future *before* checking the current sequence.
  pub async fn wait_latest_after(&self, last_seq: u64) -> (u64, CpInterfaceWrapper) {
    loop {
      let notified = self.notify.notified();

      {
        let lock = self.state.read().await;
        if lock.seq != last_seq
          && let Some(payload) = lock.payload.clone()
        {
          return (lock.seq, payload);
        }
      }

      notified.await;
    }
  }
}

/// Handles returned by [`communication_receiver`].
///
/// - `events`: the latest inbound packets from SSL-Vision / GC / WebSocket (interface -> CP)
/// - `ws_out`: broadcast channel for outbound WebSocket packets (CP -> interface)
#[derive(Clone)]
pub struct CommunicationHandles {
  pub events: EventShare,
  pub ws_out: WebsocketOut,
}

pub fn communication_receiver(cfg: &config::Config) -> anyhow::Result<CommunicationHandles> {
  let events = Arc::new(RwLock::new(Events::new()));
  let ws_out = WebsocketOut::new();

  get_ssl_data(cfg, events.clone());

  spawn_websocket(cfg, events.clone(), ws_out.clone());

  robot_receiver(cfg, events.clone(), |event, mut lock| {
    lock.rf = Some(event);
  });

  Ok(CommunicationHandles { events, ws_out })
}
