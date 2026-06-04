pub mod ball_placement;
pub mod defend;
pub mod kick_off;
mod mode_game;
mod mode_manual;
mod mode_test;
pub mod types;

use crate::game_logic::mode_game::mode_game;
use crate::game_logic::mode_manual::mode_manual;
use crate::game_logic::mode_test::mode_test;
pub(crate) use crate::game_logic::types::{BallData, GameState, Robot};
use crate::{RobotData, config};
use core_dump::proto::{CpCommand, CpMode, InterfaceCommandCp, Referee};
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
  // Check, which mode is enabled:
  //  - Manual: Use the interface commands to control the robots
  //  - Game: Use the AI and hardcoded game logic
  //  - Test: Run the tests
  match CpMode::try_from(iface_cmd.mode).unwrap_or(CpMode::ModeManual) {
    CpMode::ModeManual => {
      robot_data = mode_manual(
        robot_data,
        robots_ws_data,
        iface_cmd.manual.gc_data,
        referee.command,
      );
    }
    CpMode::ModeGame => {
      robot_data = mode_game(
        cfg,
        robot_data,
        state,
        robots,
        ball,
        referee,
        iface_cmd,
        robots_ws_data,
      );
    }
    CpMode::ModeTest => {
      robot_data = mode_test(
        cfg,
        robot_data,
        state,
        robots,
        ball,
        referee,
        iface_cmd,
        robots_ws_data,
      );
    }
  }

  robot_data
}
