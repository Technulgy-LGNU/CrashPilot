use crate::communication::multicast_handler::multicast_handler;
use crate::communication::robot_wifi_handler::robot_wifi_handler;
use crate::config;
use core_dump::proto::Referee;
#[cfg(feature = "ssl_vision")]
use core_dump::proto::{SslWrapperPacket, TrackerWrapperPacket};
use core_dump::protocol::robot_command_frame::RobotCommandFrame;
use core_dump::protocol::robot_debug_wire::RobotDebugWire;
use core_dump::protocol::robot_sensor_wire::RobotSensorWire;
use core_dump::protocol::robot_telemetry_wire::RobotTelemetryWire;
use std::sync::Arc;
use tokio::sync::RwLock;

pub mod interface;
pub mod multicast_handler;
pub mod robot_sdr_handler;
pub mod robot_wifi_handler;

pub struct Events {
  // SSL Data
  #[cfg(feature = "ssl_vision")]
  pub raw: Option<SslWrapperPacket>,
  #[cfg(feature = "ssl_vision")]
  pub tracked: Option<TrackerWrapperPacket>,
  pub gc: Option<Referee>,
  // Robot data
  pub robot_data: [RobotData; 16],
}
impl Events {
  pub fn new() -> Self {
    Self {
      #[cfg(feature = "ssl_vision")]
      raw: None,
      #[cfg(feature = "ssl_vision")]
      tracked: None,
      gc: None,
      robot_data: std::array::from_fn(|_| RobotData::new()),
    }
  }

  pub fn take(&mut self) -> Self {
    Self {
      #[cfg(feature = "ssl_vision")]
      raw: self.raw.take(),
      #[cfg(feature = "ssl_vision")]
      tracked: self.tracked.take(),
      gc: self.gc.take(),
      robot_data: self
        .robot_data
        .iter_mut()
        .take(16)
        .map(|robot| robot.take())
        .collect::<Vec<_>>()
        .try_into()
        .unwrap_or_else(|v: Vec<RobotData>| {
          panic!("Expected a Vec of length 16 but got {}", v.len())
        }),
    }
  }
}

#[derive(Clone, Debug, Default)]
pub struct RobotData {
  pub robot_telemetry: Option<RobotTelemetryWire>,
  pub robot_sensor: Option<RobotSensorWire>,
  pub robot_debug: Option<RobotDebugWire>,
}

impl RobotData {
  pub fn new() -> Self {
    Self {
      robot_telemetry: None,
      robot_sensor: None,
      robot_debug: None,
    }
  }

  pub fn take(&mut self) -> Self {
    Self {
      robot_telemetry: self.robot_telemetry.take(),
      robot_sensor: self.robot_sensor.take(),
      robot_debug: self.robot_debug.take(),
    }
  }
}

pub type EventShare = Arc<RwLock<Events>>;
pub type RobotsOut = Arc<RwLock<Option<RobotCommandFrame>>>;

/// Handles returned by [`communication_receiver`].
///
/// - `events`: the latest inbound packets from SSL-Vision / GC / WebSocket (interface -> CP)
/// - `robots_out`: the latest command frame to multicast to the robots
#[derive(Clone)]
pub struct CommunicationHandles {
  pub events: EventShare,
  pub robots_out: RobotsOut,
}

pub fn communication_receiver(cfg: &config::Config) -> anyhow::Result<CommunicationHandles> {
  let events = Arc::new(RwLock::new(Events::new()));
  let robots_out = Arc::new(RwLock::new(None));

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
  robot_wifi_handler(cfg, events.clone(), robots_out.clone());

  // Interface

  Ok(CommunicationHandles { events, robots_out })
}
