use crate::communication::multicast_handler::multicast_handler;
use crate::config;
use core_dump::proto::Referee;
#[cfg(feature = "ssl_vision")]
use core_dump::proto::{SslWrapperPacket, TrackerWrapperPacket};
use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use std::time::Instant;
use core_dump::protocol::robot_debug_wire::RobotDebugWire;
use core_dump::protocol::robot_sensor_wire::RobotSensorWire;
use core_dump::protocol::robot_telemetry_wire::RobotTelemetryWire;
use tokio::sync::RwLock;

pub mod interface;
pub mod multicast_handler;
pub mod robot_sdr_handler;
pub mod robot_wifi_handler;

pub type RobotHeartbeat = Arc<Vec<AtomicU64>>;

pub struct Events {
  // SSL Data
  #[cfg(feature = "ssl_vision")]
  pub raw: Option<SslWrapperPacket>,
  #[cfg(feature = "ssl_vision")]
  pub tracked: Option<TrackerWrapperPacket>,
  pub gc: Option<Referee>,
  // Robot data
  pub robot_telemetry: Option<RobotTelemetryWire>,
  pub robot_sensor: Option<RobotSensorWire>,
  pub robot_debug: Option<RobotDebugWire>,
}
impl Events {
  pub fn new() -> Self {
    Self {
      #[cfg(feature = "ssl_vision")]
      raw: None,
      #[cfg(feature = "ssl_vision")]
      tracked: None,
      gc: None,
      robot_telemetry: None,
      robot_sensor: None,
      robot_debug: None,
    }
  }

  pub fn take(&mut self) -> Self {
    Self {
      #[cfg(feature = "ssl_vision")]
      raw: self.raw.take(),
      #[cfg(feature = "ssl_vision")]
      tracked: self.tracked.take(),
      gc: self.gc.take(),
      robot_telemetry: self.robot_telemetry.take(),
      robot_sensor: self.robot_sensor.take(),
      robot_debug: self.robot_debug.take(),
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
  heartbeats: &RobotHeartbeat,
) -> anyhow::Result<CommunicationHandles> {
  let events = Arc::new(RwLock::new(Events::new()));

  // SSL Data
  // Vis Raw
  #[cfg(feature = "ssl_vision")]
  multicast_handler::<SslWrapperPacket>(
    cfg.ssl.vision_raw_ip,
    cfg.ssl.vision_raw_port,
    cfg.ssl.ssl_interface,
    events.clone(),
    |msg, mut lock| {
      lock.raw = Some(msg);
    },
  );
  // Vis Tracked
  #[cfg(feature = "ssl_vision")]
  multicast_handler::<TrackerWrapperPacket>(
    cfg.ssl.vision_tracked_ip,
    cfg.ssl.vision_tracked_port,
    cfg.ssl.ssl_interface,
    events.clone(),
    |msg, mut lock| {
      lock.tracked = Some(msg);
    },
  );
  // GameController
  multicast_handler::<Referee>(
    cfg.ssl.game_controller_ip,
    cfg.ssl.game_controller_port,
    cfg.ssl.ssl_interface,
    events.clone(),
    |msg, mut lock| {
      lock.gc = Some(msg);
    },
  );

  // Robot Data


  // Interface

  Ok(CommunicationHandles { events })
}
