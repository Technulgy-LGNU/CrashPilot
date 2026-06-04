pub mod types;
pub mod ball_placement;
pub mod kick_off;
pub mod defend;

pub(crate) use crate::game_logic::types::{BallData, GameState, Robot};
use crate::{RobotData, config};
use core_dump::proto::{InterfaceCommandCp, Referee};
use std::collections::HashMap;

/// Main Game Logic
/// Checks the game for:
///   - referee command for specific game states, which are not handled by the Game-AI
///   - balls moving towards the robot -> tells them to receive the ball
///   - Interface commands (goalie, field site, etc.)
///   - other game events, which are not handled by the Game-AI
///
/// There will be some scripts located in this repo to
///
/// Also translates AI commands to robot commands (AI commands are more specific, so the AI
/// has an easier time to understand them and apply them
#[inline]
pub async fn game_logic(
  cfg: &config::Config,
  mut robot_data: HashMap<u32, RobotData>,
  state: &mut GameState,
  robots: Vec<Robot>,
  ball: BallData,
  referee: &Referee,
  iface_cmd: &InterfaceCommandCp,
) -> HashMap<u32, RobotData> {
  robot_data
}
