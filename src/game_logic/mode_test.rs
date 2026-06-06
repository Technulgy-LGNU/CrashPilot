use crate::game_logic::WorldState;
use crate::RobotData;
use std::collections::HashMap;

#[inline]
pub fn mode_test(
  robot_data: HashMap<u32, RobotData>,
  _state: &mut WorldState,
) -> HashMap<u32, RobotData> {
  robot_data
}
