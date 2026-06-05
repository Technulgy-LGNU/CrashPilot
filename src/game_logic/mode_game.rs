use crate::game_logic::WorldState;
use crate::RobotData;
use std::collections::HashMap;

#[inline]
pub fn mode_game(
  mut robot_data: HashMap<u32, RobotData>,
  state: &mut WorldState,
) -> HashMap<u32, RobotData> {
  robot_data
}
