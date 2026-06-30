#[cfg(feature = "loki")]
use crate::communication::loki::LokiPublisher;
use crate::config;
#[cfg(feature = "prometheus")]
use crate::metrics::PrometheusMetrics;
use core_dump::proto::{
  CpRobot, InterfaceCommandCp, Referee, RobotCp, SslGeometryData, SslWrapperPacket,
  TrackerWrapperPacket,
};
use std::io::ErrorKind;

#[derive(Debug, Clone, Default, PartialEq)]
pub struct RobotData {
  pub msg: CpRobot,
  pub feedback: RobotCp,
}

#[derive(Debug, Clone, Copy)]
pub struct FieldSetup {
  pub width: u32,
  pub height: u32,
  pub goal_width: u32,
  pub penalty_width: u32,
  pub penalty_height: u32,
  pub _center_circle_radius: u32,
  pub run_off_width: u32,
}
impl Default for FieldSetup {
  fn default() -> Self {
    Self {
      width: 9000,
      height: 6000,
      goal_width: 1000,
      penalty_width: 2000,
      penalty_height: 1000,
      _center_circle_radius: 1000,
      run_off_width: 200,
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
      penalty_height: geometry.field.penalty_area_depth.unwrap_or_default() as u32,
      _center_circle_radius: geometry.field.center_circle_radius.unwrap_or_default() as u32,
      run_off_width: geometry.field.goal_substitution_area_width.unwrap_or(200) as u32,
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
  pub fn _clear(&mut self) {
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
