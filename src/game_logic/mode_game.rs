use crate::game_logic::{BallData, GameState, Robot};
use crate::{RobotData, config};
use core_dump::proto::referee::Command;
use core_dump::proto::{CpCommand, InterfaceCommandCp, Referee};
use std::collections::HashMap;

#[inline]
#[allow(clippy::too_many_arguments)]
pub fn mode_game(
  cfg: &config::Config,
  mut robot_data: HashMap<u32, RobotData>,
  state: &mut GameState,
  robots: Vec<Robot>,
  ball: BallData,
  referee: &Referee,
  iface_cmd: &InterfaceCommandCp,
  robots_ws_data: &HashMap<u32, CpCommand>,
) -> HashMap<u32, RobotData> {
  match Command::try_from(referee.command).unwrap_or(Command::Halt) {
    Command::Halt => {}
    Command::Stop => {}
    Command::NormalStart => {}
    Command::ForceStart => {}
    Command::PrepareKickoffYellow => {}
    Command::PrepareKickoffBlue => {}
    Command::PreparePenaltyYellow => {}
    Command::PreparePenaltyBlue => {}
    Command::DirectFreeYellow => {}
    Command::DirectFreeBlue => {}
    Command::IndirectFreeYellow => {}
    Command::IndirectFreeBlue => {}
    Command::TimeoutYellow => {}
    Command::TimeoutBlue => {}
    Command::GoalYellow => {}
    Command::GoalBlue => {}
    Command::BallPlacementYellow => {}
    Command::BallPlacementBlue => {}
  }
  robot_data
}
