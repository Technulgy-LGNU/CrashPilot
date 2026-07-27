// Combines the SSL, Robot and Websocket communication into one stream

mod create_multicast_socket;
pub mod interface;
#[cfg(feature = "loki")]
pub mod loki;
mod robot_receiver;
pub mod robot_sender;
mod ssl_communication;
#[cfg(feature = "ssl_game_controller")]
pub mod ssl_gc_handler;
mod udp_listener;

use crate::communication::interface::spawn_websocket;
use crate::communication::robot_receiver::robot_receiver;
use crate::communication::ssl_communication::get_ssl_data;
#[cfg(feature = "ssl_game_controller")]
use crate::communication::ssl_gc_handler::SslGameController;
use crate::config;
use core_dump::proto::{
  ControllerToTeam, CrashpilotInterfaceInput, CrashpilotInterfaceOutput, CrashpilotRobotFeedback,
  Referee, SslWrapperPacket, TrackerWrapperPacket,
};
use prost::Message;
use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use std::time::Instant;
use tokio::sync::Notify;
use tokio::sync::RwLock;

pub type RobotHeartbeat = Arc<Vec<AtomicU64>>;

#[derive(Debug, Clone, Default)]
pub struct Events {
  pub raw: Option<SslWrapperPacket>,
  pub tracked: Option<TrackerWrapperPacket>,
  /// Every raw camera packet received since the controller last drained the inbox.
  /// `raw` is retained for compatibility with simulator callers that construct
  /// `Events` directly.
  pub raw_frames: Vec<SslWrapperPacket>,
  /// Every tracker packet received since the last drain, keyed later by UUID/source.
  pub tracked_frames: Vec<TrackerWrapperPacket>,
  pub ws: Option<CrashpilotInterfaceInput>,
  pub gc: Option<Referee>,
  pub gc_team_messages: Vec<ControllerToTeam>,
  pub rf: Option<CrashpilotRobotFeedback>,
}

impl Events {
  pub fn new() -> Self {
    Self {
      raw: None,
      tracked: None,
      raw_frames: Vec::new(),
      tracked_frames: Vec::new(),
      ws: None,
      gc: None,
      gc_team_messages: Vec::new(),
      rf: None,
    }
  }

  pub fn take(&mut self) -> Self {
    Self {
      raw: self.raw.take(),
      tracked: self.tracked.take(),
      raw_frames: std::mem::take(&mut self.raw_frames),
      tracked_frames: std::mem::take(&mut self.tracked_frames),
      ws: self.ws.take(),
      gc: self.gc.take(),
      gc_team_messages: std::mem::take(&mut self.gc_team_messages),
      rf: self.rf.take(),
    }
  }
}

pub type EventShare = Arc<RwLock<Events>>;

#[derive(Default)]
struct WsLatestState {
  seq: u64,
  payload: Option<CrashpilotInterfaceOutput>,
  encoded_payload: Option<Vec<u8>>,
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
  pub async fn publish(&self, payload: CrashpilotInterfaceOutput) {
    let mut lock = self.state.write().await;
    lock.seq = lock.seq.wrapping_add(1);
    lock.encoded_payload = Some(payload.encode_to_vec());
    lock.payload = Some(payload);
    drop(lock);
    self.notify.notify_waiters();
  }

  /// Synchronous variant for simulator loops that are not async themselves.
  pub fn publish_sync(&self, payload: CrashpilotInterfaceOutput) {
    let mut lock = self.state.blocking_write();
    lock.seq = lock.seq.wrapping_add(1);
    lock.encoded_payload = Some(payload.encode_to_vec());
    lock.payload = Some(payload);
    drop(lock);
    self.notify.notify_waiters();
  }

  /// Wait until a payload newer than `last_seq` is available and return it.
  ///
  /// This is implemented in a race-free way (won't miss notifications): it creates the
  /// notification future *before* checking the current sequence.
  pub async fn wait_latest_after(&self, last_seq: u64) -> (u64, CrashpilotInterfaceOutput) {
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

  /// Publish a wire-compatible superset of CP_InterfaceWrapper. This is used
  /// for diagnostics fields that have not landed in every core_dump branch yet.
  pub async fn publish_encoded(&self, payload: Vec<u8>) {
    let mut lock = self.state.write().await;
    lock.seq = lock.seq.wrapping_add(1);
    lock.encoded_payload = Some(payload);
    drop(lock);
    self.notify.notify_waiters();
  }

  pub async fn wait_latest_encoded_after(&self, last_seq: u64) -> (u64, Vec<u8>) {
    loop {
      let notified = self.notify.notified();
      {
        let lock = self.state.read().await;
        if lock.seq != last_seq
          && let Some(payload) = lock.encoded_payload.clone()
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
/// - `gc`: request handle for the SSL GameController team TCP protocol
/// - `ws_out`: broadcast channel for outbound WebSocket packets (CP -> interface)
#[derive(Clone)]
pub struct CommunicationHandles {
  pub events: EventShare,
  #[cfg(feature = "ssl_game_controller")]
  pub gc: SslGameController,
  pub ws_out: WebsocketOut,
}

pub fn communication_receiver(
  cfg: &config::Config,
  heartbeats: &RobotHeartbeat,
  process_start: Instant,
) -> anyhow::Result<CommunicationHandles> {
  let events = Arc::new(RwLock::new(Events::new()));
  let ws_out = WebsocketOut::new();

  get_ssl_data(cfg, events.clone());

  #[cfg(feature = "ssl_game_controller")]
  let gc = SslGameController::spawn(cfg, events.clone());

  spawn_websocket(cfg, events.clone(), ws_out.clone());

  robot_receiver(
    cfg,
    heartbeats.clone(),
    events.clone(),
    |event, mut lock| {
      lock.rf = Some(event);
    },
    process_start,
  );

  Ok(CommunicationHandles {
    events,
    #[cfg(feature = "ssl_game_controller")]
    gc,
    ws_out,
  })
}
