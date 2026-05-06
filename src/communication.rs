// Combines the SSL, Robot and Websocket communication into one stream

use std::sync::Arc;
use prost::bytes::Bytes;
use tokio::sync::Mutex;
use tokio::sync::Notify;
use crate::config;
use crate::interface::spawn_websocket;
use crate::proto::{InterfaceWrapperCp, Referee, SslWrapperPacket, TrackerWrapperPacket};
use crate::ssl_communication::get_ssl_data;

#[derive(Debug)]
pub enum Event {
  RawVision(SslWrapperPacket),
  TrackedVision(TrackerWrapperPacket),
  Websocket(InterfaceWrapperCp),
  Referee(Referee),
}

pub type EventShare = Arc<Mutex<(Option<SslWrapperPacket>, Option<TrackerWrapperPacket>, Option<InterfaceWrapperCp>, Option<Referee>)>>;

#[derive(Default)]
struct WsLatestState {
  seq: u64,
  payload: Option<Bytes>,
}

/// Outbound WebSocket handle (CP -> interface).
///
/// This is intentionally implemented as an `Arc<Mutex<...>>` holding only the *latest* message.
/// If producers publish faster than a client can send, the client will skip intermediate updates
/// and only transmit the newest snapshot.
#[derive(Clone, Default)]
pub struct WebsocketOut {
  state: Arc<Mutex<WsLatestState>>,
  notify: Arc<Notify>,
}

impl WebsocketOut {
  pub fn new() -> Self {
    Self {
      state: Arc::new(Mutex::new(WsLatestState::default())),
      notify: Arc::new(Notify::new()),
    }
  }

  /// Publish a new binary payload.
  pub async fn publish(&self, payload: Bytes) {
    let mut lock = self.state.lock().await;
    lock.seq = lock.seq.wrapping_add(1);
    lock.payload = Some(payload);
    drop(lock);
    self.notify.notify_waiters();
  }

  /// Wait until a payload newer than `last_seq` is available and return it.
  ///
  /// This is implemented in a race-free way (won't miss notifications): it creates the
  /// notification future *before* checking the current sequence.
  pub async fn wait_latest_after(&self, last_seq: u64) -> (u64, Bytes) {
    loop {
      let notified = self.notify.notified();

      {
        let lock = self.state.lock().await;
        if lock.seq != last_seq {
          if let Some(payload) = lock.payload.clone() {
            return (lock.seq, payload);
          }
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

pub async fn communication_receiver(cfg: &config::Config) -> anyhow::Result<CommunicationHandles> {
  let events = Arc::new(Mutex::new((None, None, None, None)));
  let ws_out = WebsocketOut::new();

  get_ssl_data(cfg, events.clone()).await;

  spawn_websocket(cfg, events.clone(), ws_out.clone()).await;

  Ok(CommunicationHandles {
    events,
    ws_out,
  })
}

