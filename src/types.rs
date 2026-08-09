use crate::communication::RobotData;
use core_dump::proto::Referee;
#[cfg(feature = "ssl_vision")]
use core_dump::proto::{SslWrapperPacket, TrackerWrapperPacket};
use core_dump::types::cp_types::{Ball, FieldData, Robot};

#[derive(Debug, Clone)]
pub struct WorldModel {
  // Fixed data
  pub own_robots: Vec<Robot>,
  pub opp_robots: Vec<Robot>,
  pub ball: Option<Ball>,

  // --
  pub team: Team,
  pub site: Site,
  pub field_data: FieldData,
}

impl WorldModel {
  pub fn default() -> Self {
    Self {
      own_robots: vec![],
      opp_robots: vec![],
      ball: None,
      team: Team::default(),
      site: Site::default(),
      field_data: FieldData::default(),
    }
  }

  pub fn update(&mut self, packets: &PacketBuffer) {}
}

#[derive(Clone, Debug, Default)]
pub enum Team {
  #[default]
  YELLOW,
  BLUE,
}

#[derive(Debug, Default, Clone)]
pub enum Site {
  #[default]
  PositiveX,
  NegativeX,
}

#[derive(Debug, Default, Clone)]
pub struct PacketBuffer {
  #[cfg(feature = "ssl_vision")]
  pub vis_raw: SslWrapperPacket,
  #[cfg(feature = "ssl_vision")]
  pub vis_tracked: TrackerWrapperPacket,
  pub gc: Referee,
  pub robot_data: [RobotData; 16],
  pub packet_id: u32,
}
