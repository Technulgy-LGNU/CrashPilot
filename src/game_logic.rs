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
use crate::game_logic::types::WorldState;
use crate::{RobotData, config};
use core_dump::proto::{CpCommand, CpMode};
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
pub fn game_logic(
  _cfg: &config::Config,
  robot_data: &mut HashMap<u32, RobotData>,
  state: &mut WorldState,
  robots_ws_data: &HashMap<u32, CpCommand>,
) {
  // Check, which mode is enabled:
  //  - Manual: Use the interface commands to control the robots
  //  - Game: Use the AI and hardcoded game logic
  //  - Test: Run the tests
  match CpMode::try_from(state.iface_cmd.mode).unwrap_or(CpMode::ModeManual) {
    CpMode::ModeManual => {
      mode_manual(
        robot_data,
        robots_ws_data,
        state.iface_cmd.manual.gc_data,
        state.referee.command,
      );
    }
    CpMode::ModeGame => {
      mode_game(robot_data, state);
    }
    CpMode::ModeTest => {
      mode_test(robot_data, state);
    }
  }
}
