use simhark::WorldState;
use crate::ai_types::SampledRobotAction;
use crate::{Commands, RobotCommand};

pub fn compute_reward(old: &WorldState, new: &WorldState, commands: Commands) -> f64 {
    1.0
}