mod ai_handler;
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
use crate::{Communication, CrashPilot};
use core_dump::proto::CpMode;
use core_dump::types::Ai;

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
pub fn game_logic<C: Communication, A: Ai + Send>(cp: &mut CrashPilot<C, A>) {
  // Check, which mode is enabled:
  //  - Manual: Use the interface commands to control the robots
  //  - Game: Use the AI and hardcoded game logic
  //  - Test: Run the tests
  match CpMode::try_from(cp.packet_buffer.interface_command.mode).unwrap_or(CpMode::ModeManual) {
    CpMode::ModeManual => {
      mode_manual(
        &mut cp.robots,
        &cp.robots_ws_data,
        cp.packet_buffer.interface_command.manual.gc_data,
        cp.packet_buffer.referee.command,
      );
    }
    CpMode::ModeGame => {
      // If you stop the game in the interface, stop every robot
      if cp.packet_buffer.interface_command.game.running {
        mode_game(cp);
      } else {
        for robot in &mut cp.robots {
          robot.1.msg.cmd.state = 1;
        }
      }
    }
    CpMode::ModeTest => {
      mode_test(&mut cp.robots, &mut cp.state, &cp.field_setup);
    }
  }
}
