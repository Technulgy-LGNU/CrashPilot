pub mod ball_placement;
pub mod defend;
pub mod kick_off;
pub mod types;

pub(crate) use crate::game_logic::types::{BallData, GameState, Robot};
use crate::{config, RobotData};
use core_dump::proto::{CpCommand, InterfaceCommandCp, Referee};
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
#[allow(clippy::too_many_arguments)]
pub async fn game_logic(
  cfg: &config::Config,
  mut robot_data: HashMap<u32, RobotData>,
  state: &mut GameState,
  robots: Vec<Robot>,
  ball: BallData,
  referee: &Referee,
  iface_cmd: &InterfaceCommandCp,
  robots_ws_data: &HashMap<u32, CpCommand>,
) -> HashMap<u32, RobotData> {
  // Check, weather test mode or game mode is enabled.
  // If test mode is enabled, switch to the websocket messages
  // For game mode, run the game logic algorithm
  if !iface_cmd.game_mode {
    for robot in robot_data.values_mut() {
      robot.msg.cmd = match robots_ws_data.get(&robot.msg.robot_id) {
        Some(cmd) => *cmd,
        None => Default::default(),
      };
      if iface_cmd.gc_data && (referee.command == 0 || referee.command == 1) {
          robot.msg.cmd.state = referee.command;
        }
    }
    return robot_data;
  }

  robot_data
}
