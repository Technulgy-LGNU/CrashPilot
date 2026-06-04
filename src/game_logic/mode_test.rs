use std::collections::HashMap;
use core_dump::proto::{CpCommand, InterfaceCommandCp, Referee};
use crate::{config, RobotData};
use crate::game_logic::{BallData, GameState, Robot};

#[inline]
#[allow(clippy::too_many_arguments)]
pub fn mode_test(
  cfg: &config::Config,
  mut robot_data: HashMap<u32, RobotData>,
  state: &mut GameState,
  robots: Vec<Robot>,
  ball: BallData,
  referee: &Referee,
  iface_cmd: &InterfaceCommandCp,
  robots_ws_data: &HashMap<u32, CpCommand>,
) -> HashMap<u32, RobotData> {
  robot_data
}