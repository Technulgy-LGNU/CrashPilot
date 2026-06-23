use simhark::WorldState;
use crate::{Commands, GameState};



pub fn compute_reward(old_sim: &WorldState, new_sim: &WorldState, old: GameState, new: GameState,  commands: Commands) -> f64 {
    1.0
}