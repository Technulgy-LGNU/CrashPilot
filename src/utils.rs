#[cfg(feature = "loki")]
use crate::communication::loki::LokiPublisher;
use crate::communication::robot_sender::{NetworkSender, RobotSender};
#[cfg(feature = "prometheus")]
use crate::metrics::PrometheusMetrics;
use crate::{communication, config};
use core_dump::proto::{
  CpInterfaceWrapper, CpRobot, InterfaceCommandCp, Referee, RobotCp, SslGeometryData,
  SslWrapperPacket, TrackerWrapperPacket,
};
use prost::Message;
use std::collections::HashMap;
use std::io::ErrorKind;

pub struct RobotData {
  pub msg: CpRobot,
  pub feedback: RobotCp,
}

#[derive(Debug, Clone)]
pub struct FieldSetup {
  pub width: u32,
  pub height: u32,
  pub goal_width: u32,
  pub penalty_width: u32,
  pub penalty_height: u32,
  pub center_circle_radius: u32,
}
impl Default for FieldSetup {
  fn default() -> Self {
    Self {
      width: 9000,
      height: 6000,
      goal_width: 1000,
      penalty_width: 2000,
      penalty_height: 1000,
      center_circle_radius: 1000,
    }
  }
}

impl From<&SslGeometryData> for FieldSetup {
  fn from(geometry: &SslGeometryData) -> Self {
    Self {
      width: geometry.field.field_length as u32,
      height: geometry.field.field_width as u32,
      goal_width: geometry.field.goal_width as u32,
      penalty_width: geometry.field.penalty_area_width.unwrap_or_default() as u32,
      penalty_height: geometry.field.penalty_area_width.unwrap_or_default() as u32,
      center_circle_radius: geometry.field.center_circle_radius.unwrap_or_default() as u32,
    }
  }
}

#[derive(Debug, Clone, Default)]
pub struct PacketBuffer {
  pub vis_raw: SslWrapperPacket,
  pub vis_tracked: TrackerWrapperPacket,
  pub interface_command: InterfaceCommandCp,
  pub referee: Referee,
  pub packet_id: u32,
}

impl PacketBuffer {
  pub fn clear(&mut self) {
    self.vis_raw = SslWrapperPacket::default();
    self.vis_tracked = TrackerWrapperPacket::default();
    self.interface_command = InterfaceCommandCp::default();
    self.referee = Referee::default();
    self.packet_id = 0;
  }
}

/// Spawns the socket for the robot sender
#[inline]
pub async fn spawn_robot_socket(cfg: &config::Config) -> tokio::net::UdpSocket {
  match tokio::net::UdpSocket::bind(format!(
    "{}:{}",
    cfg.server.robot_socket_host, cfg.server.robot_socket_port
  ))
  .await
  {
    Ok(socket) => socket,
    Err(e) => match e.kind() {
      ErrorKind::AddrNotAvailable => {
        panic!(
          "Failed to bind UDP socket: Address not available. Please check if the IP address and port are correct and not in use."
        );
      }
      _ => panic!("Failed to bind UDP socket: {}", e),
    },
  }
}

/// Sends the latest data to all robots
#[inline]
pub async fn robot_sender(
  cfg: &config::Config,
  robot_socket: &tokio::net::UdpSocket,
  robots: &HashMap<u32, RobotData>,
  #[cfg(feature = "prometheus")] metrics: &PrometheusMetrics,
  #[cfg(feature = "loki")] loki: Option<&LokiPublisher>,
) {
  let network_sender: NetworkSender = NetworkSender {
    socket: robot_socket,
    data: robots,
    #[cfg(feature = "loki")]
    loki,
  };
  let send_report = network_sender.send_to_all_robots(cfg).await;
  if !send_report.failed.is_empty() {
    eprintln!(
      "Robot send: {} ok, {} failed",
      send_report.sent,
      send_report.failed.len()
    );
    for failure in &send_report.failed {
      eprintln!("  robot {}: {:#}", failure.robot_id, failure.error);
    }
  }

  #[cfg(feature = "prometheus")]
  let failed_robot_ids: HashSet<u32> = send_report
    .failed
    .iter()
    .map(|failure| failure.robot_id)
    .collect();
  #[cfg(feature = "prometheus")]
  for robot_id in robots.keys().copied() {
    metrics
      .record_send_result(robot_id, !failed_robot_ids.contains(&robot_id))
      .await;
  }
}

/// Broadcast the latest state to all websocket clients (CP -> interface)
/// Note: if no clients are connected, send() returns an error; that's fine.
#[inline]
pub async fn websocket_sender(
  vis_raw: &SslWrapperPacket,
  vis_tracked: &TrackerWrapperPacket,
  referee: &Referee,
  robots: &HashMap<u32, RobotData>,
  ws_out: &communication::WebsocketOut,
) {
  let ws_packet = CpInterfaceWrapper {
    vision_raw: Some(vis_raw.clone()),
    vision_tracked: Some(vis_tracked.clone()),
    gc_data: if referee.packet_timestamp != 0 {
      Some(referee.clone())
    } else {
      None
    },
    robot_commands: robots.values().map(|robot| robot.msg.clone()).collect(),
  };
  let mut buf = Vec::with_capacity(ws_packet.encoded_len());
  if let Err(e) = ws_packet.encode(&mut buf) {
    eprintln!("Failed to encode websocket packet: {}", e);
  } else {
    ws_out.publish(buf.into()).await;
  }
}
